#include "plotjuggler_m2/m2_data_enhancement.h"

#include <cmath>
#include <iostream>

namespace plotjuggler_m2
{

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kMaxPairingAgeSec = 1.0; // Watchdog: discard cmd/state pairs older than 1.0s

template <typename V>
void sinkVec3(const SampleSink& sink, const std::string& p, double stamp, const V& v)
{
  sink(p + "/x", stamp, static_cast<double>(v.x()));
  sink(p + "/y", stamp, static_cast<double>(v.y()));
  sink(p + "/z", stamp, static_cast<double>(v.z()));
}

void sinkRawVec3(const SampleSink& sink, const std::string& p, double stamp, const float* v)
{
  sink(p + "/x", stamp, static_cast<double>(v[0]));
  sink(p + "/y", stamp, static_cast<double>(v[1]));
  sink(p + "/z", stamp, static_cast<double>(v[2]));
}

template <typename Q>
void sinkQuat(const SampleSink& sink, const std::string& p, double stamp, const Q& q)
{
  sink(p + "/x", stamp, static_cast<double>(q.x()));
  sink(p + "/y", stamp, static_cast<double>(q.y()));
  sink(p + "/z", stamp, static_cast<double>(q.z()));
  sink(p + "/w", stamp, static_cast<double>(q.w()));
}

template <typename Container>
void sinkIndexed(const SampleSink& sink, const std::string& p, double stamp, const Container& c, std::size_t n)
{
  for (std::size_t i = 0; i < n; ++i)
  {
    sink(p + "/" + std::to_string(i), stamp, static_cast<double>(c[i]));
  }
}
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

  if (!options_.enhanced_mode)
  {
    sinkIndexed(sink, prefix + "/driver_fault", stamp, msg.driver_fault(), kM2JointCount);
    sinkIndexed(sink, prefix + "/driver_voltage", stamp, msg.driver_voltage(), kM2JointCount);
    sinkIndexed(sink, prefix + "/driver_power", stamp, msg.driver_power(), kM2JointCount);
    sinkIndexed(sink, prefix + "/fet_temp", stamp, msg.fet_temp(), kM2JointCount);
    sinkIndexed(sink, prefix + "/motor_temp", stamp, msg.motor_temp(), kM2JointCount);
    sinkIndexed(sink, prefix + "/q", stamp, msg.q(), kM2JointCount);
    sinkIndexed(sink, prefix + "/dq", stamp, msg.dq(), kM2JointCount);
    sinkIndexed(sink, prefix + "/q_current", stamp, msg.q_current(), kM2JointCount);
    sinkIndexed(sink, prefix + "/ddq", stamp, msg.ddq(), kM2JointCount);
    sinkIndexed(sink, prefix + "/tau_est", stamp, msg.tau_est(), kM2JointCount);
    sinkIndexed(sink, prefix + "/quat", stamp, msg.quat(), 4);
    sinkIndexed(sink, prefix + "/gyro", stamp, msg.gyro(), 3);
    sinkIndexed(sink, prefix + "/accel", stamp, msg.accel(), 3);
    sinkIndexed(sink, prefix + "/rpy", stamp, msg.rpy(), 3);
    return;
  }

  double total_elec_power = 0.0;
  double total_mech_power_est = 0.0;

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string jp = prefix + "/joint/" + idx_str;

    const double q_val = msg.q()[i], dq_val = msg.dq()[i], ddq_val = msg.ddq()[i];
    const double tau_val = msg.tau_est()[i], q_curr_val = msg.q_current()[i];
    const double fault_val = msg.driver_fault()[i], volt_val = msg.driver_voltage()[i];
    const double power_val = msg.driver_power()[i], fet_t_val = msg.fet_temp()[i], mot_t_val = msg.motor_temp()[i];
    const double p_mech_est = tau_val * dq_val;

    total_elec_power += power_val;
    total_mech_power_est += p_mech_est;

    struct Field { const char* name; double val; };
    for (const auto& f : {
      Field{"/q", q_val}, {"/dq", dq_val}, {"/ddq", ddq_val},
      {"/tau_est", tau_val}, {"/q_current", q_curr_val},
      {"/driver_fault", fault_val}, {"/driver_voltage", volt_val},
      {"/driver_power", power_val}, {"/fet_temp", fet_t_val},
      {"/motor_temp", mot_t_val}, {"/power_mech_est", p_mech_est}
    }) {
      sink(jp + f.name, stamp, f.val);
    }

    if (options_.leg_aliases_enabled)
    {
      sink(legJointAlias(i, "q"), stamp, q_val);
      sink(legJointAlias(i, "dq"), stamp, dq_val);
      sink(legJointAlias(i, "tau_est"), stamp, tau_val);
      sink(legJointAlias(i, "driver_power"), stamp, power_val);
    }

    if (options_.metric_first_aliases_enabled)
    {
      sink("joints*/q/" + idx_str, stamp, q_val);
      sink("joints*/dq/" + idx_str, stamp, dq_val);
      sink("joints*/tau_est/" + idx_str, stamp, tau_val);
      sink("joints*/motor_temp/" + idx_str, stamp, mot_t_val);
      sink("joints*/driver_power/" + idx_str, stamp, power_val);
    }
  }

  sink(prefix + "/summary/total_electrical_power", stamp, total_elec_power);
  sink(prefix + "/summary/total_mechanical_power_est", stamp, total_mech_power_est);
  sink(prefix + "/summary/electrical_mechanical_delta", stamp, total_elec_power - total_mech_power_est);

  sink(prefix + "/imu/quat/x", stamp, static_cast<double>(msg.quat()[0]));
  sink(prefix + "/imu/quat/y", stamp, static_cast<double>(msg.quat()[1]));
  sink(prefix + "/imu/quat/z", stamp, static_cast<double>(msg.quat()[2]));
  sink(prefix + "/imu/quat/w", stamp, static_cast<double>(msg.quat()[3]));

  sinkRawVec3(sink, prefix + "/imu/gyro", stamp, msg.gyro().data());
  sinkRawVec3(sink, prefix + "/imu/accel", stamp, msg.accel().data());

  const double roll_rad = msg.rpy()[0], pitch_rad = msg.rpy()[1], yaw_rad = msg.rpy()[2];
  sink(prefix + "/imu/rpy/roll_rad", stamp, roll_rad);
  sink(prefix + "/imu/rpy/pitch_rad", stamp, pitch_rad);
  sink(prefix + "/imu/rpy/yaw_rad", stamp, yaw_rad);
  sink(prefix + "/imu/rpy/roll_deg", stamp, roll_rad * kRadToDeg);
  sink(prefix + "/imu/rpy/pitch_deg", stamp, pitch_rad * kRadToDeg);
  sink(prefix + "/imu/rpy/yaw_deg", stamp, yaw_rad * kRadToDeg);

  if (latest_cmd_.has_value() && std::abs(stamp - latest_cmd_->stamp) <= kMaxPairingAgeSec)
  {
    emitDesiredAndErrorMetrics(latest_cmd_->msg, msg, stamp, sink);
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

  if (!options_.enhanced_mode)
  {
    sinkIndexed(sink, prefix + "/q", stamp, msg.q(), kM2JointCount);
    sinkIndexed(sink, prefix + "/dq", stamp, msg.dq(), kM2JointCount);
    sinkIndexed(sink, prefix + "/kp", stamp, msg.kp(), kM2JointCount);
    sinkIndexed(sink, prefix + "/kd", stamp, msg.kd(), kM2JointCount);
    sinkIndexed(sink, prefix + "/tau", stamp, msg.tau(), kM2JointCount);
    return;
  }

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string jp = prefix + "/joint/" + idx_str;

    const double q_cmd = msg.q()[i], dq_cmd = msg.dq()[i];
    const double kp_val = msg.kp()[i], kd_val = msg.kd()[i], tau_ff = msg.tau()[i];

    sink(jp + "/q", stamp, q_cmd);
    sink(jp + "/dq", stamp, dq_cmd);
    sink(jp + "/kp", stamp, kp_val);
    sink(jp + "/kd", stamp, kd_val);
    sink(jp + "/tau_ff", stamp, tau_ff);

    if (options_.metric_first_aliases_enabled)
    {
      sink("joint_targets*/q/" + idx_str, stamp, q_cmd);
      sink("joint_targets*/tau_ff/" + idx_str, stamp, tau_ff);
    }
  }

  if (latest_sensor_.has_value() && std::abs(stamp - latest_sensor_->stamp) <= kMaxPairingAgeSec)
  {
    emitDesiredAndErrorMetrics(msg, latest_sensor_->msg, stamp, sink);
  }
}

void M2EnhancementEngine::onJoyData(const std::string& topic_prefix,
                                   const xterra::msg::dds_::JoyData_& msg,
                                   double stamp,
                                   const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "joystick" : topic_prefix;

  if (!options_.enhanced_mode)
  {
    sink(prefix + "/priority", stamp, static_cast<double>(msg.priority()));
    sinkIndexed(sink, prefix + "/axes", stamp, msg.axes(), kM2AxesCount);
    sinkIndexed(sink, prefix + "/buttons", stamp, msg.buttons(), kM2ButtonCount);
    return;
  }

  sink(prefix + "/priority", stamp, static_cast<double>(msg.priority()));

  for (std::size_t i = 0; i < kM2AxesCount; ++i)
  {
    const double val = static_cast<double>(msg.axes()[i]);
    sink(prefix + "/axes_raw/" + std::to_string(i), stamp, val);
    sink(prefix + "/axes/" + std::string(kJoyAxisNames[i]), stamp, val);
  }

  for (std::size_t i = 0; i < kM2ButtonCount; ++i)
  {
    const double val = static_cast<double>(msg.buttons()[i]);
    sink(prefix + "/buttons_raw/" + formatIndex(i), stamp, val);
    sink(prefix + "/buttons/" + std::string(kJoyButtonNames[i]), stamp, val);
  }
}

void M2EnhancementEngine::emitDesiredAndErrorMetrics(const xterra::msg::dds_::JointData_& cmd,
                                                     const xterra::msg::dds_::SensorData_& sensor,
                                                     double stamp,
                                                     const SampleSink& sink)
{
  if (!options_.enhanced_mode)
  {
    return;
  }

  double total_p_mech_des = 0.0;

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const std::string prefix = "enhanced/" + idx_str;

    const double q_cmd = cmd.q()[i], dq_cmd = cmd.dq()[i];
    const double kp = cmd.kp()[i], kd = cmd.kd()[i], tau_ff = cmd.tau()[i];
    const double q_act = sensor.q()[i], dq_act = sensor.dq()[i];

    const double err_q = q_cmd - q_act, err_dq = dq_cmd - dq_act;
    const double tau_p = kp * err_q, tau_d = kd * err_dq;
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

  if (options_.joint_power_enabled)
  {
    sink("enhanced/summary/total_mechanical_power_des", stamp, total_p_mech_des);
  }
}

void M2EnhancementEngine::onQuadLog(const std::string& topic_prefix,
                                    const xterra::msg::dds_::QuadLog_& msg,
                                    double stamp,
                                    const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "quad_log" : topic_prefix;

  for (std::size_t i = 0; i < kM2LegCount; ++i)
  {
    const std::string leg(kLegNames[i]);
    sink(prefix + "/contact_state/" + leg, stamp, static_cast<double>(msg.contact_state()[i]));
    sink(prefix + "/contact_prob/" + leg, stamp, static_cast<double>(msg.contact_prob()[i]));
    sinkRawVec3(sink, prefix + "/contact_force/" + leg, stamp, &msg.contact_force()[i * 3]);
    sinkRawVec3(sink, prefix + "/foot_pos/" + leg, stamp, &msg.foot_position()[i * 3]);
    sinkRawVec3(sink, prefix + "/foot_vel/" + leg, stamp, &msg.foot_velocity()[i * 3]);
  }

  sinkVec3(sink, prefix + "/base_pos", stamp, msg.base_position());
  sinkQuat(sink, prefix + "/base_quat", stamp, msg.base_orientation());
  sinkVec3(sink, prefix + "/linear_vel", stamp, msg.linear_velocity());
  sinkVec3(sink, prefix + "/angular_vel", stamp, msg.angular_velocity());
  sinkVec3(sink, prefix + "/plane_normal", stamp, msg.plane_normal());

  static constexpr const char* kWrenchLabels[] = {"fx", "fy", "fz", "tx", "ty", "tz"};
  for (std::size_t i = 0; i < 6; ++i)
  {
    sink(prefix + "/base_wrench/" + kWrenchLabels[i], stamp, static_cast<double>(msg.base_wrench()[i]));
  }

  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    const std::string idx_str = formatIndex(i);
    const double q_val = msg.joint_position()[i], dq_val = msg.joint_velocity()[i], tau_val = msg.joint_torque()[i];
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
}

void M2EnhancementEngine::onPoint3D(const std::string& topic_prefix,
                                   const xterra::msg::dds_::Point3D_& msg,
                                   double stamp,
                                   const SampleSink& sink)
{
  const std::string prefix = topic_prefix.empty() ? "point3d" : topic_prefix;
  sinkVec3(sink, prefix, stamp, msg);
  if (options_.enhanced_mode)
  {
    sink(prefix + "/norm", stamp, std::sqrt(msg.x() * msg.x() + msg.y() * msg.y() + msg.z() * msg.z()));
  }
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
  sinkIndexed(sink, prefix + "/residual", stamp, msg.residual(), 6);
  sinkIndexed(sink, prefix + "/constraint_violation", stamp, msg.constraint_violation(), 4);
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
  if (options_.enhanced_mode)
  {
    sink(prefix + "/power_calc", stamp, static_cast<double>(msg.voltage() * msg.current()));
  }
}

} // namespace plotjuggler_m2
