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
#include "QuadLog.hpp"
#include "Point3D.hpp"
#include "FloatScalar.hpp"
#include "SolverStats.hpp"
#include "PowerData.hpp"

namespace plotjuggler_m2
{

using SampleSink = std::function<void(const std::string& name, double stamp, double value)>;

struct EnhancementOptions
{
  bool enhanced_mode = false;
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

  // Process incoming QuadLog (state estimation / reference / wbc telemetry)
  void onQuadLog(const std::string& topic_prefix,
                 const xterra::msg::dds_::QuadLog_& msg,
                 double stamp,
                 const SampleSink& sink);

  // Process incoming Point3D (3D vector / tracking error telemetry)
  void onPoint3D(const std::string& topic_prefix,
                 const xterra::msg::dds_::Point3D_& msg,
                 double stamp,
                 const SampleSink& sink);

  // Process incoming FloatScalar (MPC timing / scalar telemetry)
  void onFloatScalar(const std::string& topic_prefix,
                     const xterra::msg::dds_::FloatScalar_& msg,
                     double stamp,
                     const SampleSink& sink);

  // Process incoming SolverStats (QP / NLP solver telemetry)
  void onSolverStats(const std::string& topic_prefix,
                     const xterra::msg::dds_::SolverStats_& msg,
                     double stamp,
                     const SampleSink& sink);

  // Process incoming PowerData (Battery and BMS telemetry)
  void onPowerData(const std::string& topic_prefix,
                   const xterra::msg::dds_::PowerData_& msg,
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

  void emitDesiredAndErrorMetrics(const xterra::msg::dds_::JointData_& cmd,
                                  const xterra::msg::dds_::SensorData_& sensor,
                                  double stamp,
                                  const SampleSink& sink);
};

} // namespace plotjuggler_m2
