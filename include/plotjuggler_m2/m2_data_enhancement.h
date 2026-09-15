#pragma once

#include "plotjuggler_m2/m2_canonical_names.h"

#include <functional>
#include <mutex>
#include <optional>
#include <string>

// Include the generated CycloneDDS types
#include "SensorData.hpp"
#include "JointData.hpp"
#include "JoyData.hpp"

namespace plotjuggler_m2
{

using SampleSink = std::function<void(const std::string& name, double stamp, double value)>;

struct EnhancementOptions
{
  bool pd_torque_enabled = true;
  bool joint_power_enabled = true;
  bool tracking_error_enabled = true;
  bool leg_aliases_enabled = true;
  bool metric_first_aliases_enabled = true;
};

class M2EnhancementEngine
{
public:
  explicit M2EnhancementEngine(EnhancementOptions options = {});

  void setOptions(const EnhancementOptions& options);
  const EnhancementOptions& options() const;

  // Process incoming SensorData (feedback from robot)
  void onSensorData(const std::string& topic_prefix,
                    const xterra::msg::dds_::SensorData_& msg,
                    double stamp,
                    const SampleSink& sink);

  // Process incoming JointData (actuator commands sent to robot)
  void onJointData(const std::string& topic_prefix,
                   const xterra::msg::dds_::JointData_& msg,
                   double stamp,
                   const SampleSink& sink);

  // Process incoming JoyData (joystick teleoperation)
  void onJoyData(const std::string& topic_prefix,
                 const xterra::msg::dds_::JoyData_& msg,
                 double stamp,
                 const SampleSink& sink);

  void reset();

private:
  EnhancementOptions options_;
  std::mutex mutex_;

  struct StampedJointData
  {
    xterra::msg::dds_::JointData_ msg;
    double stamp = 0.0;
  };

  struct StampedSensorData
  {
    xterra::msg::dds_::SensorData_ msg;
    double stamp = 0.0;
  };

  std::optional<StampedJointData> latest_cmd_;
  std::optional<StampedSensorData> latest_sensor_;

  void emitDesiredAndErrorMetrics(const std::string& topic_prefix,
                                  const xterra::msg::dds_::JointData_& cmd,
                                  const xterra::msg::dds_::SensorData_& sensor,
                                  double stamp,
                                  const SampleSink& sink);
};

} // namespace plotjuggler_m2
