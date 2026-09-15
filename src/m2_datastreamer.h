#pragma once

#include "plotjuggler_m2/m2_data_enhancement.h"

#include <PlotJuggler/datastreamer_base.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QAction>
#include <QDialog>
#include <QObject>

class QEvent;

namespace plotjuggler_m2
{

class ISubListener
{
public:
  virtual ~ISubListener() = default;
  virtual void stop() = 0;
};

struct StreamTopicConfig
{
  std::string name;
  std::string type_name; // "SensorData", "JointData", "JoyData"
  bool enabled = true;
};

struct M2StreamConfig
{
  std::string network_interface;
  int domain_id = 0;
  bool clear_existing_data = true;
  EnhancementOptions enhancements;
  std::vector<StreamTopicConfig> topics;
};

class M2DataStreamer : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataStreamer")
  Q_INTERFACES(PJ::DataStreamer)

public:
  M2DataStreamer();
  ~M2DataStreamer() override;

  const char* name() const override { return "Svan M2 DDS"; }
  const std::vector<QAction*>& availableActions() override;
  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;
  bool xmlLoadState(const QDomElement& parent_element) override;
  bool start(QStringList* selected_datasources) override;
  void shutdown() override;
  bool isRunning() const override;

private:
  void showSettingsDialog();
  void loadDefaultSettings();
  void saveDefaultSettings() const;
  double elapsedSeconds() const;
  void appendSampleUnlocked(const std::string& series_name, double stamp, double value);
  void clearState();

  std::atomic_bool running_{false};
  std::chrono::steady_clock::time_point start_time_;
  std::mutex callback_mutex_;
  M2StreamConfig config_;
  M2EnhancementEngine enhancer_;
  std::vector<std::unique_ptr<ISubListener>> subscribers_;
  QAction* settings_action_ = nullptr;
  std::vector<QAction*> actions_;
};

} // namespace plotjuggler_m2
