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

struct EnhanceField
{
  const char* xml_name;
  const char* settings_name;
  const char* label;
  bool EnhancementOptions::* member;
};

constexpr EnhanceField kEnhanceFields[] = {
  {"pd_torque", "pd_torque_enabled", "Calculate Desired PD Torques (tau_des, tau_p, tau_d)", &EnhancementOptions::pd_torque_enabled},
  {"joint_power", "joint_power_enabled", "Calculate Instantaneous Mechanical Power (P = tau * dq)", &EnhancementOptions::joint_power_enabled},
  {"tracking_error", "tracking_error_enabled", "Calculate Tracking Errors (q_cmd - q_act, dq_cmd - dq_act)", &EnhancementOptions::tracking_error_enabled},
  {"leg_aliases", "leg_aliases_enabled", "Generate Leg Aliases (legs/FR/hip/q, etc.)", &EnhancementOptions::leg_aliases_enabled},
  {"metric_first", "metric_first_aliases_enabled", "Generate 1-Drag Multi-Curve Aliases (joints*/q/00)", &EnhancementOptions::metric_first_aliases_enabled}
};

} // namespace

template <typename T, typename Method>
void M2DataStreamer::addSubscriber(const std::string& topic_name, Method method, const SampleSink& sink)
{
  subscribers_.push_back(std::make_unique<DdsListenerImpl<T>>(
      config_.domain_id, topic_name,
      [this, sink, topic_name, method](const T& msg) {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        if (!running_) return;
        (enhancer_.*method)(topic_name, msg, elapsedSeconds(), sink);
        Q_EMIT dataReceived();
      }));
}

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

  for (const auto& f : kEnhanceFields)
  {
    config_.enhancements.*(f.member) = settings.value(f.settings_name, true).toBool();
  }

  config_.topics = {
    {std::string(kRosSensorTopic), "SensorData", true},
    {std::string(kDdsSensorTopic), "SensorData", true},
    {std::string(kRosJointCommandTopic), "JointData", true},
    {std::string(kDdsJointCommandTopic), "JointData", true},
    {std::string(kRosJoystickTopic), "JoyData", true},
    {std::string(kDdsJoystickTopic), "JoyData", true}
  };

  struct NamedType { const char* name; const char* type; };
  for (const auto& t : {
    NamedType{"wbc_modified", "QuadLog"}, {"estimated", "QuadLog"},
    NamedType{"gt_data", "QuadLog"}, {"reference", "QuadLog"},
    NamedType{"solver_stats", "SolverStats"}, {"base_err", "Point3D"},
    NamedType{"mpc_time", "FloatScalar"}, {"power_data", "PowerData"}
  })
  {
    config_.topics.push_back({std::string("/m2_metal/hw/") + t.name, t.type, true});
    config_.topics.push_back({std::string("rt/m2_metal/hw/") + t.name, t.type, true});
  }

  enhancer_.setOptions(config_.enhancements);
}

void M2DataStreamer::saveDefaultSettings() const
{
  QSettings settings("XterraRobotics", "PlotJugglerM2");

  settings.setValue("domain_id", config_.domain_id);
  settings.setValue("network_interface", QString::fromStdString(config_.network_interface));
  settings.setValue("clear_existing_data", config_.clear_existing_data);

  for (const auto& f : kEnhanceFields)
  {
    settings.setValue(f.settings_name, config_.enhancements.*(f.member));
  }
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

  std::vector<QCheckBox*> check_boxes;
  for (const auto& f : kEnhanceFields)
  {
    auto* chk = new QCheckBox(tr(f.label), enhance_group);
    chk->setChecked(config_.enhancements.*(f.member));
    enhance_layout->addWidget(chk);
    check_boxes.push_back(chk);
  }
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

    for (std::size_t i = 0; i < check_boxes.size(); ++i)
    {
      config_.enhancements.*(kEnhanceFields[i].member) = check_boxes[i]->isChecked();
    }

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
  for (const auto& f : kEnhanceFields)
  {
    elem.setAttribute(f.xml_name, (config_.enhancements.*(f.member)) ? "true" : "false");
  }
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

    for (const auto& f : kEnhanceFields)
    {
      config_.enhancements.*(f.member) = (elem.attribute(f.xml_name, "true") == "true");
    }

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

      if (type_name == "SensorData") addSubscriber<xterra::msg::dds_::SensorData_>(topic_name, &M2EnhancementEngine::onSensorData, sink);
      else if (type_name == "JointData") addSubscriber<xterra::msg::dds_::JointData_>(topic_name, &M2EnhancementEngine::onJointData, sink);
      else if (type_name == "JoyData") addSubscriber<xterra::msg::dds_::JoyData_>(topic_name, &M2EnhancementEngine::onJoyData, sink);
      else if (type_name == "QuadLog") addSubscriber<xterra::msg::dds_::QuadLog_>(topic_name, &M2EnhancementEngine::onQuadLog, sink);
      else if (type_name == "SolverStats") addSubscriber<xterra::msg::dds_::SolverStats_>(topic_name, &M2EnhancementEngine::onSolverStats, sink);
      else if (type_name == "Point3D") addSubscriber<xterra::msg::dds_::Point3D_>(topic_name, &M2EnhancementEngine::onPoint3D, sink);
      else if (type_name == "FloatScalar") addSubscriber<xterra::msg::dds_::FloatScalar_>(topic_name, &M2EnhancementEngine::onFloatScalar, sink);
      else if (type_name == "PowerData") addSubscriber<xterra::msg::dds_::PowerData_>(topic_name, &M2EnhancementEngine::onPowerData, sink);
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
