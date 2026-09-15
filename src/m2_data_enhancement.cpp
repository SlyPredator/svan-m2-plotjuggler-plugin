#include "plotjuggler_m2/m2_data_enhancement.h"

#include <cmath>
#include <iostream>

namespace plotjuggler_m2
{

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kMaxPairingAgeSec = 1.0; // Watchdog: discard cmd/state pairs older than 1.0s
}

M2EnhancementEngine::M2EnhancementEngine(EnhancementOptions options)
  : options_(options)
{
}

void M2EnhancementEngine::setOptions(const EnhancementOptions& options)
{
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
}

const EnhancementOptions& M2EnhancementEngine::options() const
{
  return options_;
}

void M2EnhancementEngine::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_cmd_.reset();
  latest_sensor_.reset();
}

void M2EnhancementEngine::onSensorData(const std::string& topic_prefix,
                                      const xterra::msg::dds_::SensorData_& msg,
                                      double stamp,
                                      const SampleSink& sink)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_sensor_ = StampedSensorData{msg, stamp};

  const std::string prefix = topic_prefix.empty() ? "sensor_data" : topic_prefix;

  double total_elec_power = 0.0;
  double total_mech_power_est = 0.0;

  // Unpack per-joint kinematics and diagnostics
  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string joint_path = prefix + "/joint/" + idx_str;

    const double q_val = static_cast<double>(msg.q()[i]);
    const double dq_val = static_cast<double>(msg.dq()[i]);
    const double ddq_val = static_cast<double>(msg.ddq()[i]);
    const double tau_val = static_cast<double>(msg.tau_est()[i]);
    const double q_curr_val = static_cast<double>(msg.q_current()[i]);

    const double fault_val = static_cast<double>(msg.driver_fault()[i]);
    const double volt_val = static_cast<double>(msg.driver_voltage()[i]);
    const double power_val = static_cast<double>(msg.driver_power()[i]);
    const double fet_t_val = static_cast<double>(msg.fet_temp()[i]);
    const double mot_t_val = static_cast<double>(msg.motor_temp()[i]);

    // Primary hierarchy
    sink(joint_path + "/q", stamp, q_val);
    sink(joint_path + "/dq", stamp, dq_val);
    sink(joint_path + "/ddq", stamp, ddq_val);
    sink(joint_path + "/tau_est", stamp, tau_val);
    sink(joint_path + "/q_current", stamp, q_curr_val);

    sink(joint_path + "/driver_fault", stamp, fault_val);
    sink(joint_path + "/driver_voltage", stamp, volt_val);
    sink(joint_path + "/driver_power", stamp, power_val);
    sink(joint_path + "/fet_temp", stamp, fet_t_val);
    sink(joint_path + "/motor_temp", stamp, mot_t_val);

    // Mechanical joint power: P = tau * dq
    const double p_mech_est = tau_val * dq_val;
    sink(joint_path + "/power_mech_est", stamp, p_mech_est);

    total_elec_power += power_val;
    total_mech_power_est += p_mech_est;

    // Optional Leg Aliases: legs/FR/hip/q, etc.
    if (options_.leg_aliases_enabled)
    {
      sink(legJointAlias(i, "q"), stamp, q_val);
      sink(legJointAlias(i, "dq"), stamp, dq_val);
      sink(legJointAlias(i, "tau_est"), stamp, tau_val);
      sink(legJointAlias(i, "driver_power"), stamp, power_val);
    }

    // Optional Metric-First Aliases for 1-drag multi-curve plotting: joints*/q/00
    if (options_.metric_first_aliases_enabled)
    {
      sink("joints*/q/" + idx_str, stamp, q_val);
      sink("joints*/dq/" + idx_str, stamp, dq_val);
      sink("joints*/tau_est/" + idx_str, stamp, tau_val);
      sink("joints*/motor_temp/" + idx_str, stamp, mot_t_val);
      sink("joints*/driver_power/" + idx_str, stamp, power_val);
    }
  }

  // Summary Totals
  sink(prefix + "/summary/total_electrical_power", stamp, total_elec_power);
  sink(prefix + "/summary/total_mechanical_power_est", stamp, total_mech_power_est);
  sink(prefix + "/summary/electrical_mechanical_delta", stamp, total_elec_power - total_mech_power_est);

  // IMU orientation quaternion [x, y, z, w]
  sink(prefix + "/imu/quat/x", stamp, static_cast<double>(msg.quat()[0]));
  sink(prefix + "/imu/quat/y", stamp, static_cast<double>(msg.quat()[1]));
  sink(prefix + "/imu/quat/z", stamp, static_cast<double>(msg.quat()[2]));
  sink(prefix + "/imu/quat/w", stamp, static_cast<double>(msg.quat()[3]));

  // IMU angular velocity (gyro) [x, y, z] in rad/s
  sink(prefix + "/imu/gyro/x", stamp, static_cast<double>(msg.gyro()[0]));
  sink(prefix + "/imu/gyro/y", stamp, static_cast<double>(msg.gyro()[1]));
  sink(prefix + "/imu/gyro/z", stamp, static_cast<double>(msg.gyro()[2]));

  // IMU linear acceleration [x, y, z] in m/s^2
  sink(prefix + "/imu/accel/x", stamp, static_cast<double>(msg.accel()[0]));
  sink(prefix + "/imu/accel/y", stamp, static_cast<double>(msg.accel()[1]));
  sink(prefix + "/imu/accel/z", stamp, static_cast<double>(msg.accel()[2]));

  // IMU Roll/Pitch/Yaw (rad and deg)
  const double roll_rad = static_cast<double>(msg.rpy()[0]);
  const double pitch_rad = static_cast<double>(msg.rpy()[1]);
  const double yaw_rad = static_cast<double>(msg.rpy()[2]);

  sink(prefix + "/imu/rpy/roll_rad", stamp, roll_rad);
  sink(prefix + "/imu/rpy/pitch_rad", stamp, pitch_rad);
  sink(prefix + "/imu/rpy/yaw_rad", stamp, yaw_rad);

  sink(prefix + "/imu/rpy/roll_deg", stamp, roll_rad * kRadToDeg);
  sink(prefix + "/imu/rpy/pitch_deg", stamp, pitch_rad * kRadToDeg);
  sink(prefix + "/imu/rpy/yaw_deg", stamp, yaw_rad * kRadToDeg);

  // Pair with latest command if available and fresh
  if (latest_cmd_.has_value() && std::abs(stamp - latest_cmd_->stamp) <= kMaxPairingAgeSec)
  {
    emitDesiredAndErrorMetrics(prefix, latest_cmd_->msg, msg, stamp, sink);
  }
}

void M2EnhancementEngine::onJointData(const std::string& topic_prefix,
                                     const xterra::msg::dds_::JointData_& msg,
                                     double stamp,
                                     const SampleSink& sink)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_cmd_ = StampedJointData{msg, stamp};

  const std::string prefix = topic_prefix.empty() ? "joint_command" : topic_prefix;

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string joint_path = prefix + "/joint/" + idx_str;

    const double q_cmd = static_cast<double>(msg.q()[i]);
    const double dq_cmd = static_cast<double>(msg.dq()[i]);
    const double kp_val = static_cast<double>(msg.kp()[i]);
    const double kd_val = static_cast<double>(msg.kd()[i]);
    const double tau_ff = static_cast<double>(msg.tau()[i]);

    sink(joint_path + "/q", stamp, q_cmd);
    sink(joint_path + "/dq", stamp, dq_cmd);
    sink(joint_path + "/kp", stamp, kp_val);
    sink(joint_path + "/kd", stamp, kd_val);
    sink(joint_path + "/tau_ff", stamp, tau_ff);

    if (options_.metric_first_aliases_enabled)
    {
      sink("joint_targets*/q/" + idx_str, stamp, q_cmd);
      sink("joint_targets*/tau_ff/" + idx_str, stamp, tau_ff);
    }
  }

  // Pair with latest state if available and fresh
  if (latest_sensor_.has_value() && std::abs(stamp - latest_sensor_->stamp) <= kMaxPairingAgeSec)
  {
    emitDesiredAndErrorMetrics(prefix, msg, latest_sensor_->msg, stamp, sink);
  }
}

void M2EnhancementEngine::onJoyData(const std::string& topic_prefix,
                                   const xterra::msg::dds_::JoyData_& msg,
                                   double stamp,
                                   const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "joystick" : topic_prefix;

  sink(prefix + "/priority", stamp, static_cast<double>(msg.priority()));

  // Unpack raw and semantic axes
  for (std::size_t i = 0; i < kM2AxesCount; ++i)
  {
    const double val = static_cast<double>(msg.axes()[i]);
    sink(prefix + "/axes_raw/" + std::to_string(i), stamp, val);
    sink(prefix + "/axes/" + std::string(kJoyAxisNames[i]), stamp, val);
  }

  // Unpack buttons
  for (std::size_t i = 0; i < kM2ButtonCount; ++i)
  {
    const double val = static_cast<double>(msg.buttons()[i]);
    sink(prefix + "/buttons_raw/" + formatIndex(i), stamp, val);
    sink(prefix + "/buttons/" + std::string(kJoyButtonNames[i]), stamp, val);
  }
}

void M2EnhancementEngine::emitDesiredAndErrorMetrics(const std::string& /*topic_prefix*/,
                                                     const xterra::msg::dds_::JointData_& cmd,
                                                     const xterra::msg::dds_::SensorData_& sensor,
                                                     double stamp,
                                                     const SampleSink& sink)
{
  double total_p_mech_des = 0.0;

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string prefix = "enhanced/" + idx_str;

    const double q_cmd = static_cast<double>(cmd.q()[i]);
    const double dq_cmd = static_cast<double>(cmd.dq()[i]);
    const double kp = static_cast<double>(cmd.kp()[i]);
    const double kd = static_cast<double>(cmd.kd()[i]);
    const double tau_ff = static_cast<double>(cmd.tau()[i]);

    const double q_act = static_cast<double>(sensor.q()[i]);
    const double dq_act = static_cast<double>(sensor.dq()[i]);

    const double err_q = q_cmd - q_act;
    const double err_dq = dq_cmd - dq_act;

    // Driver equation: tau = tau_ff + kp*(q_cmd - q_act) + kd*(dq_cmd - dq_act)
    const double tau_p = kp * err_q;
    const double tau_d = kd * err_dq;
    const double tau_des = tau_ff + tau_p + tau_d;
    const double p_mech_des = tau_des * dq_act;

    total_p_mech_des += p_mech_des;

    if (options_.pd_torque_enabled)
    {
      sink(prefix + "/tau_des_p", stamp, tau_p);
      sink(prefix + "/tau_des_d", stamp, tau_d);
      sink(prefix + "/tau_des", stamp, tau_des);
    }

    if (options_.tracking_error_enabled)
    {
      sink(prefix + "/error_q", stamp, err_q);
      sink(prefix + "/error_dq", stamp, err_dq);
    }

    if (options_.joint_power_enabled)
    {
      sink(prefix + "/power_mech_des", stamp, p_mech_des);
    }

    if (options_.metric_first_aliases_enabled)
    {
      sink("tracking_error*/q/" + idx_str, stamp, err_q);
      sink("tracking_error*/dq/" + idx_str, stamp, err_dq);
      sink("tau_des*/total/" + idx_str, stamp, tau_des);
    }
  }

  sink("enhanced/summary/total_mechanical_power_des", stamp, total_p_mech_des);
}

} // namespace plotjuggler_m2
