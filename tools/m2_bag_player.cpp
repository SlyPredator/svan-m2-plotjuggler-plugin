#include <dds/dds.hpp>
#include "SensorData.hpp"
#include "JointData.hpp"
#include "JoyData.hpp"
#include "QuadLog.hpp"
#include "Point3D.hpp"
#include "FloatScalar.hpp"
#include "SolverStats.hpp"
#include "PowerData.hpp"
#include "third_party/mcap/mcap.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

static std::atomic<bool> g_stop_requested{false};

void signalHandler(int signum)
{
  (void)signum;
  g_stop_requested = true;
}

std::string findMcapFile(const std::string& input_path)
{
  if (!fs::exists(input_path))
  {
    return "";
  }
  if (fs::is_regular_file(input_path) && input_path.size() >= 5 &&
      input_path.substr(input_path.size() - 5) == ".mcap")
  {
    return input_path;
  }
  if (fs::is_directory(input_path))
  {
    for (const auto& entry : fs::directory_iterator(input_path))
    {
      if (entry.is_regular_file() && entry.path().extension() == ".mcap")
      {
        return entry.path().string();
      }
    }
  }
  return "";
}

void printUsage(const char* prog_name)
{
  std::cout << "Usage: " << prog_name << " <bag_dir_or_mcap_file> [OPTIONS]\n"
            << "\nOptions:\n"
            << "  --rate <float>     Playback rate multiplier (default: 1.0, 0 for max speed)\n"
            << "  --domain <int>     DDS Domain ID (default: 0)\n"
            << "  --loop             Loop playback when reaching end of file\n"
            << "  --help, -h         Show this help message\n"
            << std::endl;
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    printUsage(argv[0]);
    return 1;
  }

  std::string input_path;
  double playback_rate = 1.0;
  int domain_id = 0;
  bool loop_playback = false;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      printUsage(argv[0]);
      return 0;
    }
    else if (arg == "--rate" && i + 1 < argc)
    {
      playback_rate = std::stod(argv[++i]);
    }
    else if (arg == "--domain" && i + 1 < argc)
    {
      domain_id = std::stoi(argv[++i]);
    }
    else if (arg == "--loop")
    {
      loop_playback = true;
    }
    else if (input_path.empty() && arg[0] != '-')
    {
      input_path = arg;
    }
  }

  if (input_path.empty())
  {
    std::cerr << "Error: No bag path provided." << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  std::string mcap_path = findMcapFile(input_path);
  if (mcap_path.empty())
  {
    std::cerr << "Error: Could not find .mcap file in path: " << input_path << std::endl;
    return 1;
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::cout << "==================================================\n"
            << " Svan M2 Direct MCAP -> CycloneDDS Player\n"
            << " File:      " << mcap_path << "\n"
            << " Domain ID: " << domain_id << "\n"
            << " Rate:      " << playback_rate << "x\n"
            << " Mode:      " << (loop_playback ? "Looping" : "Single run") << "\n"
            << "==================================================" << std::endl;

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

    // QuadLog topics
    dds::topic::Topic<xterra::msg::dds_::QuadLog_> wbc_topic(participant, "rt/m2_metal/hw/wbc_modified");
    dds::pub::DataWriter<xterra::msg::dds_::QuadLog_> wbc_writer(sensor_pub, wbc_topic);

    dds::topic::Topic<xterra::msg::dds_::QuadLog_> est_topic(participant, "rt/m2_metal/hw/estimated");
    dds::pub::DataWriter<xterra::msg::dds_::QuadLog_> est_writer(sensor_pub, est_topic);

    dds::topic::Topic<xterra::msg::dds_::QuadLog_> gt_topic(participant, "rt/m2_metal/hw/gt_data");
    dds::pub::DataWriter<xterra::msg::dds_::QuadLog_> gt_writer(sensor_pub, gt_topic);

    dds::topic::Topic<xterra::msg::dds_::QuadLog_> ref_topic(participant, "rt/m2_metal/hw/reference");
    dds::pub::DataWriter<xterra::msg::dds_::QuadLog_> ref_writer(sensor_pub, ref_topic);

    // SolverStats topic
    dds::topic::Topic<xterra::msg::dds_::SolverStats_> solver_topic(participant, "rt/m2_metal/hw/solver_stats");
    dds::pub::DataWriter<xterra::msg::dds_::SolverStats_> solver_writer(sensor_pub, solver_topic);

    // Point3D base_err topic
    dds::topic::Topic<xterra::msg::dds_::Point3D_> base_err_topic(participant, "rt/m2_metal/hw/base_err");
    dds::pub::DataWriter<xterra::msg::dds_::Point3D_> base_err_writer(sensor_pub, base_err_topic);

    // FloatScalar mpc_time topic
    dds::topic::Topic<xterra::msg::dds_::FloatScalar_> mpc_time_topic(participant, "rt/m2_metal/hw/mpc_time");
    dds::pub::DataWriter<xterra::msg::dds_::FloatScalar_> mpc_time_writer(sensor_pub, mpc_time_topic);

    // PowerData topic
    dds::topic::Topic<xterra::msg::dds_::PowerData_> power_topic(participant, "rt/m2_metal/hw/power_data");
    dds::pub::DataWriter<xterra::msg::dds_::PowerData_> power_writer(sensor_pub, power_topic);

    auto decode_quad_log = [](const uint8_t* raw, xterra::msg::dds_::QuadLog_& quad) {
      const float* f = reinterpret_cast<const float*>(raw + 4);
      std::memcpy(quad.contact_state().data(), f, 4 * sizeof(float)); f += 4;
      std::memcpy(quad.contact_prob().data(), f, 4 * sizeof(float)); f += 4;
      std::memcpy(quad.contact_force().data(), f, 12 * sizeof(float)); f += 12;

      quad.base_position().x(f[0]);
      quad.base_position().y(f[1]);
      quad.base_position().z(f[2]);
      f += 3;

      quad.base_orientation().x(f[0]);
      quad.base_orientation().y(f[1]);
      quad.base_orientation().z(f[2]);
      quad.base_orientation().w(f[3]);
      f += 4;

      quad.linear_velocity().x(f[0]);
      quad.linear_velocity().y(f[1]);
      quad.linear_velocity().z(f[2]);
      f += 3;

      quad.angular_velocity().x(f[0]);
      quad.angular_velocity().y(f[1]);
      quad.angular_velocity().z(f[2]);
      f += 3;

      quad.plane_normal().x(f[0]);
      quad.plane_normal().y(f[1]);
      quad.plane_normal().z(f[2]);
      f += 3;

      std::memcpy(quad.base_wrench().data(), f, 6 * sizeof(float)); f += 6;
      std::memcpy(quad.joint_position().data(), f, 12 * sizeof(float)); f += 12;
      std::memcpy(quad.joint_velocity().data(), f, 12 * sizeof(float)); f += 12;
      std::memcpy(quad.joint_torque().data(), f, 12 * sizeof(float)); f += 12;
      std::memcpy(quad.foot_position().data(), f, 12 * sizeof(float)); f += 12;
      std::memcpy(quad.foot_velocity().data(), f, 12 * sizeof(float));
    };

    // Wait briefly for DDS endpoint announcement
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    uint64_t loop_count = 0;

    do
    {
      if (loop_count > 0)
      {
        std::cout << "\n[Loop " << loop_count << "] Restarting playback..." << std::endl;
      }

      mcap::McapReader reader;
      auto status = reader.open(mcap_path);
      if (!status.ok())
      {
        std::cerr << "Failed to open MCAP: " << status.message << std::endl;
        return 1;
      }

      uint64_t start_sim_time_ns = 0;
      auto start_real_time = std::chrono::steady_clock::now();
      bool first_msg = true;

      uint64_t count_sensor = 0;
      uint64_t count_cmd = 0;
      uint64_t count_joy = 0;
      uint64_t count_quad = 0;
      uint64_t count_solver = 0;
      uint64_t count_other = 0;

      for (const auto& msgView : reader.readMessages())
      {
        if (g_stop_requested)
        {
          break;
        }

        const std::string& topic = msgView.channel->topic;
        const auto* raw = reinterpret_cast<const uint8_t*>(msgView.message.data);
        const auto size = msgView.message.dataSize;

        // Topic matching
        const bool is_sensor = (topic == "/m2_metal/hw/sensor_data" && size >= 536);
        const bool is_cmd = (topic == "/m2_metal/hw/joint_command" && size >= 244);
        const bool is_joy = ((topic == "/joystick_data" || topic == "/bt_usb/joystick_data" ||
                              topic == "/m2_metal/hw/nav2/joystick_data") && size >= 44);
        const bool is_quad_wbc = (topic == "/m2_metal/hw/wbc_modified" && size >= 412);
        const bool is_quad_est = (topic == "/m2_metal/hw/estimated" && size >= 412);
        const bool is_quad_gt = (topic == "/m2_metal/hw/gt_data" && size >= 412);
        const bool is_quad_ref = (topic == "/m2_metal/hw/reference" && size >= 412);
        const bool is_solver = (topic == "/m2_metal/hw/solver_stats" && size >= 52);
        const bool is_base_err = (topic == "/m2_metal/hw/base_err" && size >= 16);
        const bool is_mpc_time = (topic == "/m2_metal/hw/mpc_time" && size >= 8);
        const bool is_power = (topic == "/m2_metal/hw/power_data" && size >= 20);

        if (!is_sensor && !is_cmd && !is_joy &&
            !is_quad_wbc && !is_quad_est && !is_quad_gt && !is_quad_ref &&
            !is_solver && !is_base_err && !is_mpc_time && !is_power)
        {
          continue;
        }

        // Pacing
        const uint64_t log_time_ns = msgView.message.logTime;
        if (first_msg)
        {
          start_sim_time_ns = log_time_ns;
          start_real_time = std::chrono::steady_clock::now();
          first_msg = false;
        }
        else if (playback_rate > 0.0)
        {
          const uint64_t elapsed_sim_ns = log_time_ns - start_sim_time_ns;
          const double target_elapsed_sec = (static_cast<double>(elapsed_sim_ns) * 1e-9) / playback_rate;
          const auto target_time = start_real_time + std::chrono::duration<double>(target_elapsed_sec);

          const auto now = std::chrono::steady_clock::now();
          if (target_time > now)
          {
            std::this_thread::sleep_until(target_time);
          }
        }

        // Publish to CycloneDDS
        if (is_sensor)
        {
          xterra::msg::dds_::SensorData_ sensor;
          const float* f = reinterpret_cast<const float*>(raw + 4);

          std::memcpy(sensor.driver_fault().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.driver_voltage().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.driver_power().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.fet_temp().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.motor_temp().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.q().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.dq().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.q_current().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.ddq().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.tau_est().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(sensor.quat().data(), f, 4 * sizeof(float)); f += 4;
          std::memcpy(sensor.gyro().data(), f, 3 * sizeof(float)); f += 3;
          std::memcpy(sensor.accel().data(), f, 3 * sizeof(float)); f += 3;
          std::memcpy(sensor.rpy().data(), f, 3 * sizeof(float));

          sensor_writer.write(sensor);
          count_sensor++;
        }
        else if (is_cmd)
        {
          xterra::msg::dds_::JointData_ cmd;
          const float* f = reinterpret_cast<const float*>(raw + 4);

          std::memcpy(cmd.q().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(cmd.dq().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(cmd.kp().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(cmd.kd().data(), f, 12 * sizeof(float)); f += 12;
          std::memcpy(cmd.tau().data(), f, 12 * sizeof(float));

          cmd_writer.write(cmd);
          count_cmd++;
        }
        else if (is_joy)
        {
          xterra::msg::dds_::JoyData_ joy;
          joy.priority(raw[4]);

          const float* axes = reinterpret_cast<const float*>(raw + 8);
          for (std::size_t a = 0; a < 6; ++a)
          {
            joy.axes()[a] = axes[a];
          }

          const uint8_t* btns = raw + 32;
          for (std::size_t b = 0; b < 12; ++b)
          {
            joy.buttons()[b] = btns[b];
          }

          joy_writer.write(joy);
          count_joy++;
        }
        else if (is_quad_wbc || is_quad_est || is_quad_gt || is_quad_ref)
        {
          xterra::msg::dds_::QuadLog_ quad;
          decode_quad_log(raw, quad);

          if (is_quad_wbc) wbc_writer.write(quad);
          else if (is_quad_est) est_writer.write(quad);
          else if (is_quad_gt) gt_writer.write(quad);
          else if (is_quad_ref) ref_writer.write(quad);

          count_quad++;
        }
        else if (is_solver)
        {
          xterra::msg::dds_::SolverStats_ stats;
          stats.iters(*reinterpret_cast<const uint16_t*>(raw + 4));
          stats.max_iters(*reinterpret_cast<const uint16_t*>(raw + 6));
          const float* f = reinterpret_cast<const float*>(raw + 8);
          std::memcpy(stats.residual().data(), f, 6 * sizeof(float)); f += 6;
          std::memcpy(stats.constraint_violation().data(), f, 4 * sizeof(float)); f += 4;
          stats.time_ms(*f);

          solver_writer.write(stats);
          count_solver++;
        }
        else if (is_base_err)
        {
          xterra::msg::dds_::Point3D_ pt;
          const float* f = reinterpret_cast<const float*>(raw + 4);
          pt.x(f[0]);
          pt.y(f[1]);
          pt.z(f[2]);

          base_err_writer.write(pt);
          count_other++;
        }
        else if (is_mpc_time)
        {
          xterra::msg::dds_::FloatScalar_ fs_msg;
          const float* f = reinterpret_cast<const float*>(raw + 4);
          fs_msg.data(*f);

          mpc_time_writer.write(fs_msg);
          count_other++;
        }
        else if (is_power)
        {
          xterra::msg::dds_::PowerData_ pwr;
          const float* f = reinterpret_cast<const float*>(raw + 4);
          pwr.voltage(f[0]);
          pwr.current(f[1]);
          pwr.temperature(f[2]);
          pwr.energy(f[3]);

          power_writer.write(pwr);
          count_other++;
        }

        if ((count_sensor + count_cmd + count_joy + count_quad + count_solver + count_other) % 1000 == 0)
        {
          const auto now = std::chrono::steady_clock::now();
          const double elapsed = std::chrono::duration<double>(now - start_real_time).count();
          std::cout << "\r[Playback] Elapsed: " << std::fixed << std::setprecision(1) << elapsed
                    << "s | Sensor: " << count_sensor
                    << " | Cmd: " << count_cmd
                    << " | Joy: " << count_joy
                    << " | QuadLog: " << count_quad
                    << " | Solver: " << count_solver
                    << " | Other: " << count_other << std::flush;
        }
      }

      std::cout << "\nFinished reading file: " << count_sensor << " SensorData, "
                << count_cmd << " JointCmd, " << count_joy << " Joy, "
                << count_quad << " QuadLog, " << count_solver << " SolverStats, "
                << count_other << " Other messages." << std::endl;

      loop_count++;
    } while (loop_playback && !g_stop_requested);

    std::cout << "Playback complete. Exiting." << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception in player: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
