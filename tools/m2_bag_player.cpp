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

template <typename T>
inline void readFloats(T& dst, const float*& src, std::size_t n)
{
  std::memcpy(dst.data(), src, n * sizeof(float));
  src += n;
}

inline void decodeSensorData(const uint8_t* raw, xterra::msg::dds_::SensorData_& sensor)
{
  const float* f = reinterpret_cast<const float*>(raw + 4);
  readFloats(sensor.driver_fault(), f, 12);
  readFloats(sensor.driver_voltage(), f, 12);
  readFloats(sensor.driver_power(), f, 12);
  readFloats(sensor.fet_temp(), f, 12);
  readFloats(sensor.motor_temp(), f, 12);
  readFloats(sensor.q(), f, 12);
  readFloats(sensor.dq(), f, 12);
  readFloats(sensor.q_current(), f, 12);
  readFloats(sensor.ddq(), f, 12);
  readFloats(sensor.tau_est(), f, 12);
  readFloats(sensor.quat(), f, 4);
  readFloats(sensor.gyro(), f, 3);
  readFloats(sensor.accel(), f, 3);
  readFloats(sensor.rpy(), f, 3);
}

inline void decodeJointCmd(const uint8_t* raw, xterra::msg::dds_::JointData_& cmd)
{
  const float* f = reinterpret_cast<const float*>(raw + 4);
  readFloats(cmd.q(), f, 12);
  readFloats(cmd.dq(), f, 12);
  readFloats(cmd.kp(), f, 12);
  readFloats(cmd.kd(), f, 12);
  readFloats(cmd.tau(), f, 12);
}

inline void decodeQuadLog(const uint8_t* raw, xterra::msg::dds_::QuadLog_& quad)
{
  const float* f = reinterpret_cast<const float*>(raw + 4);
  readFloats(quad.contact_state(), f, 4);
  readFloats(quad.contact_prob(), f, 4);
  readFloats(quad.contact_force(), f, 12);

  quad.base_position().x(f[0]); quad.base_position().y(f[1]); quad.base_position().z(f[2]); f += 3;
  quad.base_orientation().x(f[0]); quad.base_orientation().y(f[1]);
  quad.base_orientation().z(f[2]); quad.base_orientation().w(f[3]); f += 4;
  quad.linear_velocity().x(f[0]); quad.linear_velocity().y(f[1]); quad.linear_velocity().z(f[2]); f += 3;
  quad.angular_velocity().x(f[0]); quad.angular_velocity().y(f[1]); quad.angular_velocity().z(f[2]); f += 3;
  quad.plane_normal().x(f[0]); quad.plane_normal().y(f[1]); quad.plane_normal().z(f[2]); f += 3;

  readFloats(quad.base_wrench(), f, 6);
  readFloats(quad.joint_position(), f, 12);
  readFloats(quad.joint_velocity(), f, 12);
  readFloats(quad.joint_torque(), f, 12);
  readFloats(quad.foot_position(), f, 12);
  readFloats(quad.foot_velocity(), f, 12);
}

template <typename T>
inline dds::pub::DataWriter<T> makeWriter(dds::domain::DomainParticipant& dp, dds::pub::Publisher& pub, const char* topic)
{
  return dds::pub::DataWriter<T>(pub, dds::topic::Topic<T>(dp, topic));
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
    dds::pub::Publisher pub(participant);

    auto sensor_writer = makeWriter<xterra::msg::dds_::SensorData_>(participant, pub, "rt/m2_metal/hw/sensor_data");
    auto cmd_writer = makeWriter<xterra::msg::dds_::JointData_>(participant, pub, "rt/m2_metal/hw/joint_command");
    auto joy_writer = makeWriter<xterra::msg::dds_::JoyData_>(participant, pub, "rt/mission/joystick_data");
    auto wbc_writer = makeWriter<xterra::msg::dds_::QuadLog_>(participant, pub, "rt/m2_metal/hw/wbc_modified");
    auto est_writer = makeWriter<xterra::msg::dds_::QuadLog_>(participant, pub, "rt/m2_metal/hw/estimated");
    auto gt_writer = makeWriter<xterra::msg::dds_::QuadLog_>(participant, pub, "rt/m2_metal/hw/gt_data");
    auto ref_writer = makeWriter<xterra::msg::dds_::QuadLog_>(participant, pub, "rt/m2_metal/hw/reference");
    auto solver_writer = makeWriter<xterra::msg::dds_::SolverStats_>(participant, pub, "rt/m2_metal/hw/solver_stats");
    auto base_err_writer = makeWriter<xterra::msg::dds_::Point3D_>(participant, pub, "rt/m2_metal/hw/base_err");
    auto mpc_time_writer = makeWriter<xterra::msg::dds_::FloatScalar_>(participant, pub, "rt/m2_metal/hw/mpc_time");
    auto power_writer = makeWriter<xterra::msg::dds_::PowerData_>(participant, pub, "rt/m2_metal/hw/power_data");

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

      uint64_t count_sensor = 0, count_cmd = 0, count_joy = 0;
      uint64_t count_quad = 0, count_solver = 0, count_other = 0;

      for (const auto& msgView : reader.readMessages())
      {
        if (g_stop_requested)
        {
          break;
        }

        const std::string& topic = msgView.channel->topic;
        const auto* raw = reinterpret_cast<const uint8_t*>(msgView.message.data);
        const auto size = msgView.message.dataSize;

        enum class MsgKind { None, Sensor, Cmd, Joy, QuadWbc, QuadEst, QuadGt, QuadRef, Solver, BaseErr, MpcTime, Power };
        MsgKind kind = MsgKind::None;
        if (topic == "/m2_metal/hw/sensor_data" && size >= 536) kind = MsgKind::Sensor;
        else if (topic == "/m2_metal/hw/joint_command" && size >= 244) kind = MsgKind::Cmd;
        else if ((topic == "/joystick_data" || topic == "/bt_usb/joystick_data" ||
                  topic == "/m2_metal/hw/nav2/joystick_data") && size >= 44) kind = MsgKind::Joy;
        else if (size >= 412) {
          if (topic == "/m2_metal/hw/wbc_modified") kind = MsgKind::QuadWbc;
          else if (topic == "/m2_metal/hw/estimated") kind = MsgKind::QuadEst;
          else if (topic == "/m2_metal/hw/gt_data") kind = MsgKind::QuadGt;
          else if (topic == "/m2_metal/hw/reference") kind = MsgKind::QuadRef;
        }
        else if (topic == "/m2_metal/hw/solver_stats" && size >= 52) kind = MsgKind::Solver;
        else if (topic == "/m2_metal/hw/base_err" && size >= 16) kind = MsgKind::BaseErr;
        else if (topic == "/m2_metal/hw/mpc_time" && size >= 8) kind = MsgKind::MpcTime;
        else if (topic == "/m2_metal/hw/power_data" && size >= 20) kind = MsgKind::Power;

        if (kind == MsgKind::None) continue;

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
        switch (kind)
        {
          case MsgKind::Sensor: {
            xterra::msg::dds_::SensorData_ sensor;
            decodeSensorData(raw, sensor);
            sensor_writer.write(sensor);
            count_sensor++;
            break;
          }
          case MsgKind::Cmd: {
            xterra::msg::dds_::JointData_ cmd;
            decodeJointCmd(raw, cmd);
            cmd_writer.write(cmd);
            count_cmd++;
            break;
          }
          case MsgKind::Joy: {
            xterra::msg::dds_::JoyData_ joy;
            joy.priority(raw[4]);
            const float* axes = reinterpret_cast<const float*>(raw + 8);
            for (std::size_t a = 0; a < 6; ++a) joy.axes()[a] = axes[a];
            const uint8_t* btns = raw + 32;
            for (std::size_t b = 0; b < 12; ++b) joy.buttons()[b] = btns[b];
            joy_writer.write(joy);
            count_joy++;
            break;
          }
          case MsgKind::QuadWbc:
          case MsgKind::QuadEst:
          case MsgKind::QuadGt:
          case MsgKind::QuadRef: {
            xterra::msg::dds_::QuadLog_ quad;
            decodeQuadLog(raw, quad);
            if (kind == MsgKind::QuadWbc) wbc_writer.write(quad);
            else if (kind == MsgKind::QuadEst) est_writer.write(quad);
            else if (kind == MsgKind::QuadGt) gt_writer.write(quad);
            else if (kind == MsgKind::QuadRef) ref_writer.write(quad);
            count_quad++;
            break;
          }
          case MsgKind::Solver: {
            xterra::msg::dds_::SolverStats_ stats;
            stats.iters(*reinterpret_cast<const uint16_t*>(raw + 4));
            stats.max_iters(*reinterpret_cast<const uint16_t*>(raw + 6));
            const float* f = reinterpret_cast<const float*>(raw + 8);
            readFloats(stats.residual(), f, 6);
            readFloats(stats.constraint_violation(), f, 4);
            stats.time_ms(*f);
            solver_writer.write(stats);
            count_solver++;
            break;
          }
          case MsgKind::BaseErr: {
            xterra::msg::dds_::Point3D_ pt;
            const float* f = reinterpret_cast<const float*>(raw + 4);
            pt.x(f[0]); pt.y(f[1]); pt.z(f[2]);
            base_err_writer.write(pt);
            count_other++;
            break;
          }
          case MsgKind::MpcTime: {
            xterra::msg::dds_::FloatScalar_ fs_msg;
            fs_msg.data(*reinterpret_cast<const float*>(raw + 4));
            mpc_time_writer.write(fs_msg);
            count_other++;
            break;
          }
          case MsgKind::Power: {
            xterra::msg::dds_::PowerData_ pwr;
            const float* f = reinterpret_cast<const float*>(raw + 4);
            pwr.voltage(f[0]); pwr.current(f[1]); pwr.temperature(f[2]); pwr.energy(f[3]);
            power_writer.write(pwr);
            count_other++;
            break;
          }
          default: break;
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
