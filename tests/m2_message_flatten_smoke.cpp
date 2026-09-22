#include "plotjuggler_m2/m2_canonical_names.h"
#include "plotjuggler_m2/m2_data_enhancement.h"

#include <cassert>
#include <iostream>
#include <map>
#include <string>

int main()
{
  using namespace plotjuggler_m2;

  std::map<std::string, double> captured_samples;
  SampleSink sink = [&](const std::string& name, double /*stamp*/, double value) {
    captured_samples[name] = value;
  };

  EnhancementOptions opts;
  opts.enhanced_mode = true;
  opts.pd_torque_enabled = true;
  opts.joint_power_enabled = true;
  opts.tracking_error_enabled = true;
  opts.leg_aliases_enabled = true;
  opts.metric_first_aliases_enabled = true;

  M2EnhancementEngine engine(opts);

  // 1. Create a dummy SensorData_ message
  xterra::msg::dds_::SensorData_ sensor_msg;
  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    sensor_msg.q()[i] = static_cast<float>(i) * 0.1f;
    sensor_msg.dq()[i] = 1.5f;
    sensor_msg.ddq()[i] = 0.2f;
    sensor_msg.tau_est()[i] = 10.0f + static_cast<float>(i);
    sensor_msg.q_current()[i] = 2.0f;

    sensor_msg.driver_fault()[i] = 0.0f;
    sensor_msg.driver_voltage()[i] = 24.0f;
    sensor_msg.driver_power()[i] = 50.0f;
    sensor_msg.fet_temp()[i] = 35.0f;
    sensor_msg.motor_temp()[i] = 40.0f;
  }
  sensor_msg.quat() = {0.0f, 0.0f, 0.0f, 1.0f};
  sensor_msg.gyro() = {0.01f, 0.02f, 0.03f};
  sensor_msg.accel() = {0.0f, 0.0f, 9.81f};
  sensor_msg.rpy() = {0.05f, 0.10f, 0.15f};

  engine.onSensorData("sensor_data", sensor_msg, 1.0, sink);

  // Verify joint 0 values
  assert(captured_samples.count("sensor_data/joint/00/q") == 1);
  assert(std::abs(captured_samples["sensor_data/joint/00/q"] - 0.0) < 1e-5);
  assert(captured_samples.count("sensor_data/joint/01/q") == 1);
  assert(std::abs(captured_samples["sensor_data/joint/01/q"] - 0.1) < 1e-5);

  // Verify mechanical power: P = tau * dq = 10.0 * 1.5 = 15.0
  assert(captured_samples.count("sensor_data/joint/00/power_mech_est") == 1);
  assert(std::abs(captured_samples["sensor_data/joint/00/power_mech_est"] - 15.0) < 1e-5);

  // Verify IMU data
  assert(captured_samples.count("sensor_data/imu/quat/w") == 1);
  assert(std::abs(captured_samples["sensor_data/imu/quat/w"] - 1.0) < 1e-5);
  assert(captured_samples.count("sensor_data/imu/accel/z") == 1);
  assert(std::abs(captured_samples["sensor_data/imu/accel/z"] - 9.81) < 1e-4);

  // Verify leg alias
  assert(captured_samples.count("legs/FR/hip/q") == 1);
  assert(captured_samples.count("legs/RL/calf/q") == 1);

  // Verify 1-drag multi-curve alias
  assert(captured_samples.count("joints*/q/00") == 1);
  assert(captured_samples.count("joints*/q/11") == 1);

  std::cout << "SUCCESS: All SensorData flattening tests passed! Captured "
            << captured_samples.size() << " unique time-series curves." << std::endl;

  // 2. Create and feed JointData_ to test pairing and PD torque calculations
  captured_samples.clear();
  xterra::msg::dds_::JointData_ cmd_msg;
  for (std::size_t i = 0; i < kM2JointCount; ++i)
  {
    cmd_msg.q()[i] = 0.5f;
    cmd_msg.dq()[i] = 0.0f;
    cmd_msg.kp()[i] = 40.0f;
    cmd_msg.kd()[i] = 2.0f;
    cmd_msg.tau()[i] = 5.0f;
  }

  engine.onJointData("joint_command", cmd_msg, 1.01, sink);

  // Check commands unpacked
  assert(captured_samples.count("joint_command/joint/00/kp") == 1);
  assert(std::abs(captured_samples["joint_command/joint/00/kp"] - 40.0) < 1e-5);

  // Check paired PD calculations for joint 0:
  // q_cmd = 0.5, q_act = 0.0 -> err_q = 0.5
  // dq_cmd = 0.0, dq_act = 1.5 -> err_dq = -1.5
  // tau_p = kp * err_q = 40 * 0.5 = 20.0
  // tau_d = kd * err_dq = 2 * (-1.5) = -3.0
  // tau_des = tau_ff + tau_p + tau_d = 5.0 + 20.0 - 3.0 = 22.0
  assert(captured_samples.count("enhanced/00/error_q") == 1);
  assert(std::abs(captured_samples["enhanced/00/error_q"] - 0.5) < 1e-5);

  assert(captured_samples.count("enhanced/00/tau_des_p") == 1);
  assert(std::abs(captured_samples["enhanced/00/tau_des_p"] - 20.0) < 1e-5);

  assert(captured_samples.count("enhanced/00/tau_des_d") == 1);
  assert(std::abs(captured_samples["enhanced/00/tau_des_d"] - (-3.0)) < 1e-5);

  assert(captured_samples.count("enhanced/00/tau_des") == 1);
  assert(std::abs(captured_samples["enhanced/00/tau_des"] - 22.0) < 1e-5);

  std::cout << "SUCCESS: All Command/Feedback pairing and PD torque tests passed!" << std::endl;

  // 3. Create and feed JoyData_
  captured_samples.clear();
  xterra::msg::dds_::JoyData_ joy_msg;
  joy_msg.priority() = 1;
  joy_msg.axes() = {0.2f, -0.8f, 0.0f, 0.5f, 0.1f, 0.9f};
  joy_msg.buttons()[0] = 1; // Stand trigger
  joy_msg.buttons()[2] = 1; // Move trigger

  engine.onJoyData("joystick", joy_msg, 1.02, sink);

  assert(captured_samples.count("joystick/axes/longitudinal") == 1);
  assert(std::abs(captured_samples["joystick/axes/longitudinal"] - (-0.8)) < 1e-5);
  assert(captured_samples.count("joystick/buttons/fixed_stand") == 1);
  assert(std::abs(captured_samples["joystick/buttons/fixed_stand"] - 1.0) < 1e-5);

  std::cout << "SUCCESS: All JoyData tests passed!" << std::endl;

  // 4. Test QuadLog_
  captured_samples.clear();
  xterra::msg::dds_::QuadLog_ quad_msg;
  quad_msg.base_position().x(1.23f);
  quad_msg.base_position().y(-0.45f);
  quad_msg.base_position().z(0.67f);
  quad_msg.contact_force()[0] = 120.0f; // FR x
  quad_msg.joint_torque()[0] = 15.5f;

  engine.onQuadLog("quad_log", quad_msg, 1.03, sink);
  assert(captured_samples.count("quad_log/base_pos/x") == 1);
  assert(std::abs(captured_samples["quad_log/base_pos/x"] - 1.23) < 1e-4);
  assert(captured_samples.count("quad_log/contact_force/FR/x") == 1);
  assert(std::abs(captured_samples["quad_log/contact_force/FR/x"] - 120.0) < 1e-4);
  assert(captured_samples.count("quad_log/joint_torque/00") == 1);
  assert(std::abs(captured_samples["quad_log/joint_torque/00"] - 15.5) < 1e-4);
  std::cout << "SUCCESS: All QuadLog tests passed!" << std::endl;

  // 5. Test Point3D_
  captured_samples.clear();
  xterra::msg::dds_::Point3D_ pt_msg;
  pt_msg.x(3.0f);
  pt_msg.y(4.0f);
  pt_msg.z(0.0f);
  engine.onPoint3D("base_err", pt_msg, 1.04, sink);
  assert(captured_samples.count("base_err/x") == 1);
  assert(captured_samples.count("base_err/norm") == 1);
  assert(std::abs(captured_samples["base_err/norm"] - 5.0) < 1e-5);
  std::cout << "SUCCESS: All Point3D tests passed!" << std::endl;

  // 6. Test SolverStats_
  captured_samples.clear();
  xterra::msg::dds_::SolverStats_ stats_msg;
  stats_msg.iters(12);
  stats_msg.time_ms(3.45f);
  engine.onSolverStats("solver_stats", stats_msg, 1.05, sink);
  assert(captured_samples.count("solver_stats/iters") == 1);
  assert(std::abs(captured_samples["solver_stats/iters"] - 12.0) < 1e-5);
  assert(captured_samples.count("solver_stats/time_ms") == 1);
  assert(std::abs(captured_samples["solver_stats/time_ms"] - 3.45) < 1e-4);
  std::cout << "SUCCESS: All SolverStats tests passed!" << std::endl;

  // 7. Test FloatScalar_
  captured_samples.clear();
  xterra::msg::dds_::FloatScalar_ fs_msg;
  fs_msg.data(42.0f);
  engine.onFloatScalar("mpc_time", fs_msg, 1.06, sink);
  assert(captured_samples.count("mpc_time/data") == 1);
  assert(std::abs(captured_samples["mpc_time/data"] - 42.0) < 1e-5);
  std::cout << "SUCCESS: All FloatScalar tests passed!" << std::endl;

  // 8. Test PowerData_
  captured_samples.clear();
  xterra::msg::dds_::PowerData_ pwr_msg;
  pwr_msg.voltage(24.0f);
  pwr_msg.current(5.0f);
  engine.onPowerData("power_data", pwr_msg, 1.07, sink);
  assert(captured_samples.count("power_data/power_calc") == 1);
  assert(std::abs(captured_samples["power_data/power_calc"] - 120.0) < 1e-4);
  std::cout << "SUCCESS: All PowerData tests passed!" << std::endl;

  // 9. Test Default / Canonical Mode (enhanced_mode = false)
  captured_samples.clear();
  EnhancementOptions canonical_opts;
  canonical_opts.enhanced_mode = false;
  M2EnhancementEngine canonical_engine(canonical_opts);

  canonical_engine.onSensorData("rt/m2_metal/hw/sensor_data", sensor_msg, 2.0, sink);
  canonical_engine.onJointData("rt/m2_metal/hw/joint_command", cmd_msg, 2.01, sink);
  canonical_engine.onJoyData("rt/joystick_data", joy_msg, 2.02, sink);
  canonical_engine.onPoint3D("rt/m2_metal/hw/base_err", pt_msg, 2.03, sink);
  canonical_engine.onPowerData("rt/m2_metal/hw/power_data", pwr_msg, 2.04, sink);

  // Must have canonical raw IDL paths
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/q/0") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/q/11") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/tau_est/0") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/driver_power/11") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/quat/3") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/gyro/2") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/accel/2") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/sensor_data/rpy/2") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/joint_command/q/0") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/joint_command/tau/11") == 1);
  assert(captured_samples.count("rt/joystick_data/priority") == 1);
  assert(captured_samples.count("rt/joystick_data/axes/0") == 1);
  assert(captured_samples.count("rt/joystick_data/buttons/0") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/base_err/x") == 1);
  assert(captured_samples.count("rt/m2_metal/hw/power_data/voltage") == 1);

  // Must NOT have any enhanced, derived, summary, or alias curves
  for (const auto& [name, _] : captured_samples)
  {
    assert(name.rfind("enhanced", 0) != 0 && "Found enhanced topic in canonical mode!");
    assert(name.find("/summary/") == std::string::npos && "Found summary topic in canonical mode!");
    assert(name.find("/joint/") == std::string::npos && "Found joint/00 topic in canonical mode!");
    assert(name.rfind("legs/", 0) != 0 && "Found legs/ alias in canonical mode!");
    assert(name.rfind("joints*/", 0) != 0 && "Found joints*/ alias in canonical mode!");
    assert(name.rfind("joint_targets*/", 0) != 0 && "Found joint_targets*/ alias in canonical mode!");
    assert(name.rfind("tracking_error*/", 0) != 0 && "Found tracking_error*/ in canonical mode!");
    assert(name.rfind("tau_des*/", 0) != 0 && "Found tau_des*/ in canonical mode!");
    assert(name.find("power_mech") == std::string::npos && "Found power_mech in canonical mode!");
    assert(name.find("power_calc") == std::string::npos && "Found power_calc in canonical mode!");
    assert(name.find("/norm") == std::string::npos && "Found /norm in canonical mode!");
  }
  std::cout << "SUCCESS: All Canonical Mode verification tests passed! Total canonical curves: "
            << captured_samples.size() << std::endl;

  return 0;
}
