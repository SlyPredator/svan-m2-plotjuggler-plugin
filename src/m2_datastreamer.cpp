#include "m2_datastreamer.h"

#include <dds/dds.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>

#include <iostream>

namespace plotjuggler_m2
{

namespace
{

template <typename T>
class DdsListenerImpl : public ISubListener, public dds::sub::NoOpDataReaderListener<T>
{
public:
  using Callback = std::function<void(const T&)>;

  DdsListenerImpl(int domain_id, const std::string& topic_name, Callback cb)
    : callback_(std::move(cb)),
      participant_(domain_id),
      topic_(participant_, topic_name),
      subscriber_(participant_),
      reader_(dds::core::null)
  {
    dds::sub::qos::DataReaderQos qos;
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(5);

    reader_ = dds::sub::DataReader<T>(subscriber_, topic_, qos, this,
                                      dds::core::status::StatusMask::data_available());
  }

  ~DdsListenerImpl() override
  {
    stop();
  }

  void stop() override
  {
    try
    {
      if (reader_ != dds::core::null)
      {
        reader_.close();
        reader_ = dds::core::null;
      }
    }
    catch (...)
    {
    }
  }

  void on_data_available(dds::sub::DataReader<T>& reader) override
  {
    auto samples = reader.take();
    for (const auto& s : samples)
    {
      if (s.info().valid() && callback_)
      {
        callback_(s.data());
      }
    }
  }

private:
  Callback callback_;
  dds::domain::DomainParticipant participant_;
  dds::topic::Topic<T> topic_;
  dds::sub::Subscriber subscriber_;
  dds::sub::DataReader<T> reader_;
};

} // namespace

M2DataStreamer::M2DataStreamer()
{
  settings_action_ = new QAction(tr("Configure Svan M2 DDS"), this);
  connect(settings_action_, &QAction::triggered, this, &M2DataStreamer::showSettingsDialog);
  actions_.push_back(settings_action_);

  loadDefaultSettings();
}

M2DataStreamer::~M2DataStreamer()
{
  shutdown();
}

const std::vector<QAction*>& M2DataStreamer::availableActions()
{
  return actions_;
}

void M2DataStreamer::loadDefaultSettings()
{
  QSettings settings("XterraRobotics", "PlotJugglerM2");

  config_.domain_id = settings.value("domain_id", 0).toInt();
  config_.network_interface = settings.value("network_interface", "").toString().toStdString();
  config_.clear_existing_data = settings.value("clear_existing_data", true).toBool();

  config_.enhancements.pd_torque_enabled = settings.value("pd_torque_enabled", true).toBool();
  config_.enhancements.joint_power_enabled = settings.value("joint_power_enabled", true).toBool();
  config_.enhancements.tracking_error_enabled = settings.value("tracking_error_enabled", true).toBool();
  config_.enhancements.leg_aliases_enabled = settings.value("leg_aliases_enabled", true).toBool();
  config_.enhancements.metric_first_aliases_enabled = settings.value("metric_first_aliases_enabled", true).toBool();

  config_.topics = {
    {std::string(kRosSensorTopic), "SensorData", true},
    {std::string(kDdsSensorTopic), "SensorData", true},
    {std::string(kRosJointCommandTopic), "JointData", true},
    {std::string(kDdsJointCommandTopic), "JointData", true},
    {std::string(kRosJoystickTopic), "JoyData", true},
    {std::string(kDdsJoystickTopic), "JoyData", true},
    // QuadLog telemetry
    {"/m2_metal/hw/wbc_modified", "QuadLog", true},
    {"rt/m2_metal/hw/wbc_modified", "QuadLog", true},
    {"/m2_metal/hw/estimated", "QuadLog", true},
    {"rt/m2_metal/hw/estimated", "QuadLog", true},
    {"/m2_metal/hw/gt_data", "QuadLog", true},
    {"rt/m2_metal/hw/gt_data", "QuadLog", true},
    {"/m2_metal/hw/reference", "QuadLog", true},
    {"rt/m2_metal/hw/reference", "QuadLog", true},
    // SolverStats telemetry
    {"/m2_metal/hw/solver_stats", "SolverStats", true},
    {"rt/m2_metal/hw/solver_stats", "SolverStats", true},
    // Point3D error telemetry
    {"/m2_metal/hw/base_err", "Point3D", true},
    {"rt/m2_metal/hw/base_err", "Point3D", true},
    // FloatScalar timing telemetry
    {"/m2_metal/hw/mpc_time", "FloatScalar", true},
    {"rt/m2_metal/hw/mpc_time", "FloatScalar", true},
    // PowerData telemetry
    {"/m2_metal/hw/power_data", "PowerData", true},
    {"rt/m2_metal/hw/power_data", "PowerData", true}
  };

  enhancer_.setOptions(config_.enhancements);
}

void M2DataStreamer::saveDefaultSettings() const
{
  QSettings settings("XterraRobotics", "PlotJugglerM2");

  settings.setValue("domain_id", config_.domain_id);
  settings.setValue("network_interface", QString::fromStdString(config_.network_interface));
  settings.setValue("clear_existing_data", config_.clear_existing_data);

  settings.setValue("pd_torque_enabled", config_.enhancements.pd_torque_enabled);
  settings.setValue("joint_power_enabled", config_.enhancements.joint_power_enabled);
  settings.setValue("tracking_error_enabled", config_.enhancements.tracking_error_enabled);
  settings.setValue("leg_aliases_enabled", config_.enhancements.leg_aliases_enabled);
  settings.setValue("metric_first_aliases_enabled", config_.enhancements.metric_first_aliases_enabled);
}

void M2DataStreamer::showSettingsDialog()
{
  QDialog dialog;
  dialog.setWindowTitle(tr("Svan M2 DDS Streamer Settings"));
  auto* layout = new QVBoxLayout(&dialog);

  auto* form_layout = new QFormLayout();

  auto* domain_spin = new QSpinBox(&dialog);
  domain_spin->setRange(0, 232);
  domain_spin->setValue(config_.domain_id);
  form_layout->addRow(tr("DDS Domain ID:"), domain_spin);

  auto* iface_combo = new QComboBox(&dialog);
  iface_combo->addItem(tr("(Auto / All Interfaces)"), "");
  for (const auto& iface : QNetworkInterface::allInterfaces())
  {
    if (iface.flags().testFlag(QNetworkInterface::IsUp) &&
        !iface.flags().testFlag(QNetworkInterface::IsLoopBack))
    {
      iface_combo->addItem(iface.humanReadableName(), iface.name());
    }
  }
  const int iface_idx = iface_combo->findData(QString::fromStdString(config_.network_interface));
  if (iface_idx >= 0)
  {
    iface_combo->setCurrentIndex(iface_idx);
  }
  form_layout->addRow(tr("Network Interface:"), iface_combo);

  layout->addLayout(form_layout);

  auto* enhance_group = new QGroupBox(tr("Telemetry & Math Enhancements"), &dialog);
  auto* enhance_layout = new QVBoxLayout(enhance_group);

  auto* pd_chk = new QCheckBox(tr("Calculate Desired PD Torques (tau_des, tau_p, tau_d)"), enhance_group);
  pd_chk->setChecked(config_.enhancements.pd_torque_enabled);
  enhance_layout->addWidget(pd_chk);

  auto* power_chk = new QCheckBox(tr("Calculate Instantaneous Mechanical Power (P = tau * dq)"), enhance_group);
  power_chk->setChecked(config_.enhancements.joint_power_enabled);
  enhance_layout->addWidget(power_chk);

  auto* error_chk = new QCheckBox(tr("Calculate Tracking Errors (q_cmd - q_act, dq_cmd - dq_act)"), enhance_group);
  error_chk->setChecked(config_.enhancements.tracking_error_enabled);
  enhance_layout->addWidget(error_chk);

  auto* leg_chk = new QCheckBox(tr("Generate Leg Aliases (legs/FR/hip/q, etc.)"), enhance_group);
  leg_chk->setChecked(config_.enhancements.leg_aliases_enabled);
  enhance_layout->addWidget(leg_chk);

  auto* metric_chk = new QCheckBox(tr("Generate 1-Drag Multi-Curve Aliases (joints*/q/00)"), enhance_group);
  metric_chk->setChecked(config_.enhancements.metric_first_aliases_enabled);
  enhance_layout->addWidget(metric_chk);

  layout->addWidget(enhance_group);

  auto* clear_chk = new QCheckBox(tr("Clear existing plots on stream start"), &dialog);
  clear_chk->setChecked(config_.clear_existing_data);
  layout->addWidget(clear_chk);

  auto* btn_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  connect(btn_box, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(btn_box, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(btn_box);

  if (dialog.exec() == QDialog::Accepted)
  {
    config_.domain_id = domain_spin->value();
    config_.network_interface = iface_combo->currentData().toString().toStdString();
    config_.clear_existing_data = clear_chk->isChecked();

    config_.enhancements.pd_torque_enabled = pd_chk->isChecked();
    config_.enhancements.joint_power_enabled = power_chk->isChecked();
    config_.enhancements.tracking_error_enabled = error_chk->isChecked();
    config_.enhancements.leg_aliases_enabled = leg_chk->isChecked();
    config_.enhancements.metric_first_aliases_enabled = metric_chk->isChecked();

    enhancer_.setOptions(config_.enhancements);
    saveDefaultSettings();
  }
}

bool M2DataStreamer::xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const
{
  QDomElement elem = doc.createElement("SvanM2_Streamer");
  elem.setAttribute("domain_id", config_.domain_id);
  elem.setAttribute("network_interface", QString::fromStdString(config_.network_interface));
  elem.setAttribute("clear_existing_data", config_.clear_existing_data ? "true" : "false");
  elem.setAttribute("pd_torque", config_.enhancements.pd_torque_enabled ? "true" : "false");
  elem.setAttribute("joint_power", config_.enhancements.joint_power_enabled ? "true" : "false");
  elem.setAttribute("tracking_error", config_.enhancements.tracking_error_enabled ? "true" : "false");
  elem.setAttribute("leg_aliases", config_.enhancements.leg_aliases_enabled ? "true" : "false");
  elem.setAttribute("metric_first", config_.enhancements.metric_first_aliases_enabled ? "true" : "false");
  parent_element.appendChild(elem);
  return true;
}

bool M2DataStreamer::xmlLoadState(const QDomElement& parent_element)
{
  const QDomElement elem = parent_element.firstChildElement("SvanM2_Streamer");
  if (!elem.isNull())
  {
    config_.domain_id = elem.attribute("domain_id", "0").toInt();
    config_.network_interface = elem.attribute("network_interface", "").toStdString();
    config_.clear_existing_data = (elem.attribute("clear_existing_data", "true") == "true");

    config_.enhancements.pd_torque_enabled = (elem.attribute("pd_torque", "true") == "true");
    config_.enhancements.joint_power_enabled = (elem.attribute("joint_power", "true") == "true");
    config_.enhancements.tracking_error_enabled = (elem.attribute("tracking_error", "true") == "true");
    config_.enhancements.leg_aliases_enabled = (elem.attribute("leg_aliases", "true") == "true");
    config_.enhancements.metric_first_aliases_enabled = (elem.attribute("metric_first", "true") == "true");

    enhancer_.setOptions(config_.enhancements);
  }
  return true;
}

double M2DataStreamer::elapsedSeconds() const
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start_time_).count();
}

void M2DataStreamer::appendSampleUnlocked(const std::string& series_name, double stamp, double value)
{
  dataMap().getOrCreateNumeric(series_name).pushBack(PJ::PlotData::Point{stamp, value});
}

void M2DataStreamer::clearState()
{
  subscribers_.clear();
  enhancer_.reset();
}

bool M2DataStreamer::start(QStringList* /*selected_datasources*/)
{
  if (running_)
  {
    return true;
  }

  if (config_.clear_existing_data)
  {
    {
      std::lock_guard<std::mutex> lock(mutex());
      dataMap().clear();
    }
    Q_EMIT clearBuffers();
  }

  clearState();
  start_time_ = std::chrono::steady_clock::now();

  const SampleSink sink = [this](const std::string& name, double stamp, double value) {
    std::lock_guard<std::mutex> lock(mutex());
    appendSampleUnlocked(name, stamp, value);
  };

  try
  {
    for (const auto& topic_cfg : config_.topics)
    {
      if (!topic_cfg.enabled) continue;
      const std::string& topic_name = topic_cfg.name;
      const std::string& type_name = topic_cfg.type_name;

      if (type_name == "SensorData")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::SensorData_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::SensorData_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onSensorData(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "JointData")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::JointData_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::JointData_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onJointData(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "JoyData")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::JoyData_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::JoyData_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onJoyData(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "QuadLog")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::QuadLog_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::QuadLog_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onQuadLog(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "SolverStats")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::SolverStats_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::SolverStats_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onSolverStats(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "Point3D")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::Point3D_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::Point3D_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onPoint3D(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "FloatScalar")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::FloatScalar_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::FloatScalar_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onFloatScalar(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
      else if (type_name == "PowerData")
      {
        auto sub = std::make_unique<DdsListenerImpl<xterra::msg::dds_::PowerData_>>(
            config_.domain_id, topic_name,
            [this, sink, topic_name](const xterra::msg::dds_::PowerData_& msg) {
              std::lock_guard<std::mutex> callback_lock(callback_mutex_);
              if (!running_) return;
              enhancer_.onPowerData(topic_name, msg, elapsedSeconds(), sink);
              Q_EMIT dataReceived();
            });
        subscribers_.push_back(std::move(sub));
      }
    }

    running_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    QMessageBox::critical(nullptr, tr("DDS Error"),
                          tr("Failed to start CycloneDDS subscribers: %1").arg(e.what()));
    clearState();
    running_ = false;
    return false;
  }
}

void M2DataStreamer::shutdown()
{
  if (!running_)
  {
    return;
  }
  running_ = false;
  clearState();
}

bool M2DataStreamer::isRunning() const
{
  return running_;
}

} // namespace plotjuggler_m2
