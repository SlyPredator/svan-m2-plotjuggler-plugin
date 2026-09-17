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

void M2EnhancementEngine::onQuadLog(const std::string& topic_prefix,
                                    const xterra::msg::dds_::QuadLog_& msg,
                                    double stamp,
                                    const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "quad_log" : topic_prefix;

  // Contact states & probabilities
  for (std::size_t i = 0; i < kM2LegCount; ++i)
  {
    const std::string leg(kLegNames[i]);
    sink(prefix + "/contact_state/" + leg, stamp, static_cast<double>(msg.contact_state()[i]));
    sink(prefix + "/contact_prob/" + leg, stamp, static_cast<double>(msg.contact_prob()[i]));
  }

  // Contact force (4 legs x 3 xyz)
  for (std::size_t leg_idx = 0; leg_idx < kM2LegCount; ++leg_idx)
  {
    const std::string leg(kLegNames[leg_idx]);
    sink(prefix + "/contact_force/" + leg + "/x", stamp, static_cast<double>(msg.contact_force()[leg_idx * 3 + 0]));
    sink(prefix + "/contact_force/" + leg + "/y", stamp, static_cast<double>(msg.contact_force()[leg_idx * 3 + 1]));
    sink(prefix + "/contact_force/" + leg + "/z", stamp, static_cast<double>(msg.contact_force()[leg_idx * 3 + 2]));
  }

  // Base Position & Orientation
  sink(prefix + "/base_pos/x", stamp, static_cast<double>(msg.base_position().x()));
  sink(prefix + "/base_pos/y", stamp, static_cast<double>(msg.base_position().y()));
  sink(prefix + "/base_pos/z", stamp, static_cast<double>(msg.base_position().z()));

  sink(prefix + "/base_quat/x", stamp, static_cast<double>(msg.base_orientation().x()));
  sink(prefix + "/base_quat/y", stamp, static_cast<double>(msg.base_orientation().y()));
  sink(prefix + "/base_quat/z", stamp, static_cast<double>(msg.base_orientation().z()));
  sink(prefix + "/base_quat/w", stamp, static_cast<double>(msg.base_orientation().w()));

  // Velocities
  sink(prefix + "/linear_vel/x", stamp, static_cast<double>(msg.linear_velocity().x()));
  sink(prefix + "/linear_vel/y", stamp, static_cast<double>(msg.linear_velocity().y()));
  sink(prefix + "/linear_vel/z", stamp, static_cast<double>(msg.linear_velocity().z()));

  sink(prefix + "/angular_vel/x", stamp, static_cast<double>(msg.angular_velocity().x()));
  sink(prefix + "/angular_vel/y", stamp, static_cast<double>(msg.angular_velocity().y()));
  sink(prefix + "/angular_vel/z", stamp, static_cast<double>(msg.angular_velocity().z()));

  sink(prefix + "/plane_normal/x", stamp, static_cast<double>(msg.plane_normal().x()));
  sink(prefix + "/plane_normal/y", stamp, static_cast<double>(msg.plane_normal().y()));
  sink(prefix + "/plane_normal/z", stamp, static_cast<double>(msg.plane_normal().z()));

  // Base Wrench (fx, fy, fz, tx, ty, tz)
  static constexpr const char* kWrenchLabels[] = {"fx", "fy", "fz", "tx", "ty", "tz"};
  for (std::size_t i = 0; i < 6; ++i)
  {
    sink(prefix + "/base_wrench/" + kWrenchLabels[i], stamp, static_cast<double>(msg.base_wrench()[i]));
  }

  // Joints (position, velocity, torque)
  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const double q_val = static_cast<double>(msg.joint_position()[i]);
    const double dq_val = static_cast<double>(msg.joint_velocity()[i]);
    const double tau_val = static_cast<double>(msg.joint_torque()[i]);

    sink(prefix + "/joint_position/" + idx_str, stamp, q_val);
    sink(prefix + "/joint_velocity/" + idx_str, stamp, dq_val);
    sink(prefix + "/joint_torque/" + idx_str, stamp, tau_val);

    if (options_.leg_aliases_enabled)
    {
      sink(prefix + "/" + legJointAlias(i, "pos"), stamp, q_val);
      sink(prefix + "/" + legJointAlias(i, "vel"), stamp, dq_val);
      sink(prefix + "/" + legJointAlias(i, "tau"), stamp, tau_val);
    }
  }

  // Foot position and velocity (4 legs x 3 xyz)
  for (std::size_t leg_idx = 0; leg_idx < kM2LegCount; ++leg_idx)
  {
    const std::string leg(kLegNames[leg_idx]);
    sink(prefix + "/foot_pos/" + leg + "/x", stamp, static_cast<double>(msg.foot_position()[leg_idx * 3 + 0]));
    sink(prefix + "/foot_pos/" + leg + "/y", stamp, static_cast<double>(msg.foot_position()[leg_idx * 3 + 1]));
    sink(prefix + "/foot_pos/" + leg + "/z", stamp, static_cast<double>(msg.foot_position()[leg_idx * 3 + 2]));

    sink(prefix + "/foot_vel/" + leg + "/x", stamp, static_cast<double>(msg.foot_velocity()[leg_idx * 3 + 0]));
    sink(prefix + "/foot_vel/" + leg + "/y", stamp, static_cast<double>(msg.foot_velocity()[leg_idx * 3 + 1]));
    sink(prefix + "/foot_vel/" + leg + "/z", stamp, static_cast<double>(msg.foot_velocity()[leg_idx * 3 + 2]));
  }
}

void M2EnhancementEngine::onPoint3D(const std::string& topic_prefix,
                                   const xterra::msg::dds_::Point3D_& msg,
                                   double stamp,
                                   const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "point3d" : topic_prefix;
  sink(prefix + "/x", stamp, static_cast<double>(msg.x()));
  sink(prefix + "/y", stamp, static_cast<double>(msg.y()));
  sink(prefix + "/z", stamp, static_cast<double>(msg.z()));
  const double norm = std::sqrt(msg.x() * msg.x() + msg.y() * msg.y() + msg.z() * msg.z());
  sink(prefix + "/norm", stamp, norm);
}

void M2EnhancementEngine::onFloatScalar(const std::string& topic_prefix,
                                       const xterra::msg::dds_::FloatScalar_& msg,
                                       double stamp,
                                       const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "scalar" : topic_prefix;
  sink(prefix + "/data", stamp, static_cast<double>(msg.data()));
}

void M2EnhancementEngine::onSolverStats(const std::string& topic_prefix,
                                       const xterra::msg::dds_::SolverStats_& msg,
                                       double stamp,
                                       const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "solver_stats" : topic_prefix;
  sink(prefix + "/iters", stamp, static_cast<double>(msg.iters()));
  sink(prefix + "/max_iters", stamp, static_cast<double>(msg.max_iters()));
  sink(prefix + "/time_ms", stamp, static_cast<double>(msg.time_ms()));

  for (std::size_t i = 0; i < 6; ++i)
  {
    sink(prefix + "/residual/" + std::to_string(i), stamp, static_cast<double>(msg.residual()[i]));
  }
  for (std::size_t i = 0; i < 4; ++i)
  {
    sink(prefix + "/constraint_violation/" + std::to_string(i), stamp, static_cast<double>(msg.constraint_violation()[i]));
  }
}

void M2EnhancementEngine::onPowerData(const std::string& topic_prefix,
                                     const xterra::msg::dds_::PowerData_& msg,
                                     double stamp,
                                     const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "power" : topic_prefix;
  sink(prefix + "/voltage", stamp, static_cast<double>(msg.voltage()));
  sink(prefix + "/current", stamp, static_cast<double>(msg.current()));
  sink(prefix + "/temperature", stamp, static_cast<double>(msg.temperature()));
  sink(prefix + "/energy", stamp, static_cast<double>(msg.energy()));
  sink(prefix + "/power_calc", stamp, static_cast<double>(msg.voltage() * msg.current()));
}

} // namespace plotjuggler_m2
