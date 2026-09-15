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
  return 0;
}
