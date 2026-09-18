#include <dds/dds.hpp>
#include "SensorData.hpp"
#include "third_party/mcap/mcap.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

int main()
{
  constexpr int kTestDomain = 43;
  std::cout << "Testing direct MCAP bag player on Domain " << kTestDomain << "..." << std::endl;

  std::atomic_bool received_sensor{false};
  float captured_q0 = 0.0f;
  float captured_quat3 = 0.0f;

  dds::domain::DomainParticipant participant(kTestDomain);

  // 1. Setup Subscriber
  dds::topic::Topic<xterra::msg::dds_::SensorData_> topic(
      participant, "rt/m2_metal/hw/sensor_data");
  dds::sub::Subscriber sub(participant);

  class Listener : public dds::sub::NoOpDataReaderListener<xterra::msg::dds_::SensorData_>
  {
  public:
    Listener(std::atomic_bool& flag, float& q0, float& quat3)
      : flag_(flag), q0_(q0), quat3_(quat3) {}

    void on_data_available(dds::sub::DataReader<xterra::msg::dds_::SensorData_>& reader) override
    {
      auto samples = reader.take();
      for (const auto& s : samples)
      {
        if (s.info().valid())
        {
          q0_ = s.data().q()[0];
          quat3_ = s.data().quat()[3];
          flag_ = true;
        }
      }
    }
  private:
    std::atomic_bool& flag_;
    float& q0_;
    float& quat3_;
  };

  Listener listener(received_sensor, captured_q0, captured_quat3);
  dds::sub::qos::DataReaderQos qos;
  qos << dds::core::policy::Reliability::BestEffort();

  dds::sub::DataReader<xterra::msg::dds_::SensorData_> reader(
      sub, topic, qos, &listener, dds::core::status::StatusMask::data_available());

  // 2. Setup Publisher
  dds::pub::Publisher pub(participant);
  dds::pub::DataWriter<xterra::msg::dds_::SensorData_> writer(pub, topic);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // 3. Read MCAP file and publish first few samples
  mcap::McapReader mcap_reader;
  auto status = mcap_reader.open("xtr_rosbags/testbag4/testbag4_0.mcap");
  assert(status.ok() && "Failed to open xtr_rosbags/testbag4/testbag4_0.mcap");

  int published_count = 0;
  for (const auto& msgView : mcap_reader.readMessages())
  {
    if (msgView.channel->topic == "/m2_metal/hw/sensor_data" && msgView.message.dataSize >= 536)
    {
      xterra::msg::dds_::SensorData_ sensor;
      const float* f = reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(msgView.message.data) + 4);
      auto readF = [&](auto& dst, std::size_t n) { std::memcpy(dst.data(), f, n * sizeof(float)); f += n; };
      readF(sensor.driver_fault(), 12); readF(sensor.driver_voltage(), 12);
      readF(sensor.driver_power(), 12); readF(sensor.fet_temp(), 12);
      readF(sensor.motor_temp(), 12); readF(sensor.q(), 12);
      readF(sensor.dq(), 12); readF(sensor.q_current(), 12);
      readF(sensor.ddq(), 12); readF(sensor.tau_est(), 12);
      readF(sensor.quat(), 4); readF(sensor.gyro(), 3);
      readF(sensor.accel(), 3); readF(sensor.rpy(), 3);

      writer.write(sensor);
      published_count++;

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if (received_sensor.load())
      {
        break;
      }
    }
  }

  assert(received_sensor.load() && "Failed to receive live sample from direct MCAP player!");
  std::cout << "SUCCESS: Direct MCAP player received real telemetry! q[0] = "
            << captured_q0 << ", quat[3] = " << captured_quat3
            << " after publishing " << published_count << " samples." << std::endl;

  return 0;
}
