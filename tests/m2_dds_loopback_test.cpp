#include <dds/dds.hpp>
#include "plotjuggler_m2/m2_data_enhancement.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>

int main()
{
  constexpr int kTestDomain = 42;
  std::cout << "Running live CycloneDDS Loopback Integration Test on Domain " << kTestDomain << "..." << std::endl;

  std::atomic_bool received_sensor{false};
  std::atomic_bool received_cmd{false};
  std::atomic_bool received_joy{false};

  std::map<std::string, double> captured;
  std::mutex mutex;

  auto sink = [&](const std::string& name, double /*stamp*/, double val) {
    std::lock_guard<std::mutex> lock(mutex);
    captured[name] = val;
  };

  plotjuggler_m2::M2EnhancementEngine engine;

  // 1. Setup Subscriber
  dds::domain::DomainParticipant participant(kTestDomain);

  dds::topic::Topic<xterra::msg::dds_::SensorData_> sensor_topic(participant, "rt/m2_metal/hw/sensor_data");
  dds::sub::Subscriber subscriber(participant);

  dds::sub::qos::DataReaderQos qos;
  qos << dds::core::policy::Reliability::BestEffort();

  class SensorListener : public dds::sub::NoOpDataReaderListener<xterra::msg::dds_::SensorData_>
  {
  public:
    SensorListener(plotjuggler_m2::M2EnhancementEngine& eng,
                   plotjuggler_m2::SampleSink s,
                   std::atomic_bool& flag)
      : engine_(eng), sink_(s), flag_(flag) {}

    void on_data_available(dds::sub::DataReader<xterra::msg::dds_::SensorData_>& reader) override
    {
      auto samples = reader.take();
      for (const auto& s : samples)
      {
        if (s.info().valid())
        {
          engine_.onSensorData("rt/m2_metal/hw/sensor_data", s.data(), 1.0, sink_);
          flag_ = true;
        }
      }
    }
  private:
    plotjuggler_m2::M2EnhancementEngine& engine_;
    plotjuggler_m2::SampleSink sink_;
    std::atomic_bool& flag_;
  };

  SensorListener listener(engine, sink, received_sensor);
  dds::sub::DataReader<xterra::msg::dds_::SensorData_> reader(
      subscriber, sensor_topic, qos, &listener, dds::core::status::StatusMask::data_available());

  // 2. Setup Publisher
  dds::pub::Publisher publisher(participant);
  dds::pub::DataWriter<xterra::msg::dds_::SensorData_> writer(publisher, sensor_topic);

  // Wait for DDS discovery
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 3. Publish test message
  xterra::msg::dds_::SensorData_ msg;
  for (int i = 0; i < 12; ++i)
  {
    msg.q()[i] = 0.42f + i * 0.01f;
    msg.dq()[i] = 2.0f;
    msg.tau_est()[i] = 15.0f;
    msg.driver_power()[i] = 45.0f;
  }
  msg.quat() = {0.0f, 0.0f, 0.0f, 1.0f};

  for (int retry = 0; retry < 10; ++retry)
  {
    writer.write(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (received_sensor.load()) break;
  }

  assert(received_sensor.load() && "Failed to receive live DDS SensorData over loopback!");

  {
    std::lock_guard<std::mutex> lock(mutex);
    assert(captured.count("rt/m2_metal/hw/sensor_data/joint/00/q") == 1);
    assert(std::abs(captured["rt/m2_metal/hw/sensor_data/joint/00/q"] - 0.42) < 1e-5);
    std::cout << "Captured live sample from DDS: q[0] = "
              << captured["rt/m2_metal/hw/sensor_data/joint/00/q"] << " rad." << std::endl;
  }

  std::cout << "SUCCESS: Full CycloneDDS live network loopback test passed!" << std::endl;
  return 0;
}
