#include <dds/dds.hpp>
#include "SensorData.hpp"
#include "JointData.hpp"
#include "JoyData.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

int main(int argc, char* argv[])
{
  int domain_id = 0;
  if (argc > 1)
  {
    domain_id = std::atoi(argv[1]);
  }

  std::cout << "Starting Svan M2 Mock DDS Publisher on Domain " << domain_id << "..." << std::endl;

  try
  {
    dds::domain::DomainParticipant participant(domain_id);

    dds::topic::Topic<xterra::msg::dds_::SensorData_> sensor_topic(
        participant, "rt/m2_metal/hw/sensor_data");
    dds::pub::Publisher sensor_pub(participant);
    dds::pub::DataWriter<xterra::msg::dds_::SensorData_> sensor_writer(sensor_pub, sensor_topic);

    dds::topic::Topic<xterra::msg::dds_::JointData_> cmd_topic(
        participant, "rt/m2_metal/hw/joint_command");
    dds::pub::Publisher cmd_pub(participant);
    dds::pub::DataWriter<xterra::msg::dds_::JointData_> cmd_writer(cmd_pub, cmd_topic);

    dds::topic::Topic<xterra::msg::dds_::JoyData_> joy_topic(
        participant, "rt/mission/joystick_data");
    dds::pub::Publisher joy_pub(participant);
    dds::pub::DataWriter<xterra::msg::dds_::JoyData_> joy_writer(joy_pub, joy_topic);

    std::cout << "Publishing mock telemetry:\n"
              << "  - rt/m2_metal/hw/sensor_data (500 Hz)\n"
              << "  - rt/m2_metal/hw/joint_command (200 Hz)\n"
              << "  - rt/mission/joystick_data (20 Hz)\n"
              << "Press Ctrl+C to stop." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    uint64_t counter = 0;

    while (true)
    {
      const auto now = std::chrono::steady_clock::now();
      const double t = std::chrono::duration<double>(now - start_time).count();

      // 500 Hz loop
      xterra::msg::dds_::SensorData_ sensor;
      for (int i = 0; i < 12; ++i)
      {
        const double phase = (i % 3) * 0.5 + (i / 3) * 1.2;
        sensor.q()[i] = static_cast<float>(0.3 * std::sin(2.0 * 3.14159 * 1.5 * t + phase));
        sensor.dq()[i] = static_cast<float>(0.3 * 2.0 * 3.14159 * 1.5 * std::cos(2.0 * 3.14159 * 1.5 * t + phase));
        sensor.ddq()[i] = 0.0f;
        sensor.tau_est()[i] = static_cast<float>(12.0 * std::sin(2.0 * 3.14159 * 1.5 * t + phase));
        sensor.q_current()[i] = sensor.tau_est()[i] * 0.2f;

        sensor.driver_fault()[i] = 0.0f;
        sensor.driver_voltage()[i] = static_cast<float>(24.0 + 0.5 * std::sin(0.2 * t));
        sensor.driver_power()[i] = static_cast<float>(30.0 + 15.0 * std::abs(std::sin(t)));
        sensor.fet_temp()[i] = static_cast<float>(36.0 + 0.05 * (i % 4));
        sensor.motor_temp()[i] = static_cast<float>(42.0 + 0.08 * (i % 4));
      }

      // IMU data
      sensor.quat()[0] = 0.0f;
      sensor.quat()[1] = 0.0f;
      sensor.quat()[2] = 0.0f;
      sensor.quat()[3] = 1.0f;
      sensor.gyro()[0] = static_cast<float>(0.05 * std::sin(t));
      sensor.gyro()[1] = static_cast<float>(0.05 * std::cos(t));
      sensor.gyro()[2] = 0.0f;
      sensor.accel()[0] = 0.0f;
      sensor.accel()[1] = 0.0f;
      sensor.accel()[2] = 9.81f;
      sensor.rpy()[0] = static_cast<float>(0.05 * std::sin(t));
      sensor.rpy()[1] = static_cast<float>(0.03 * std::cos(t));
      sensor.rpy()[2] = 0.0f;

      sensor_writer.write(sensor);

      // 200 Hz for joint commands
      if (counter % 2 == 0)
      {
        xterra::msg::dds_::JointData_ cmd;
        for (int i = 0; i < 12; ++i)
        {
          const double phase = (i % 3) * 0.5 + (i / 3) * 1.2;
          cmd.q()[i] = static_cast<float>(0.3 * std::sin(2.0 * 3.14159 * 1.5 * t + phase));
          cmd.dq()[i] = 0.0f;
          cmd.kp()[i] = 45.0f;
          cmd.kd()[i] = 2.5f;
          cmd.tau()[i] = 0.0f;
        }
        cmd_writer.write(cmd);
      }

      // 20 Hz for joystick
      if (counter % 25 == 0)
      {
        xterra::msg::dds_::JoyData_ joy;
        joy.priority() = 1;
        joy.axes()[0] = static_cast<float>(0.1 * std::sin(t));
        joy.axes()[1] = -0.5f; // Moving forward
        joy.axes()[2] = 0.0f;
        joy.axes()[3] = 0.0f;
        joy.axes()[4] = 0.0f;
        joy.axes()[5] = 0.8f; // Step height
        joy.buttons()[2] = 1;  // Locomotion move state
        joy_writer.write(joy);
      }

      ++counter;
      std::this_thread::sleep_for(std::chrono::microseconds(2000)); // 500 Hz
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "DDS Publisher Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
