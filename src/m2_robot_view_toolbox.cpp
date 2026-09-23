#include "m2_robot_view_toolbox.h"
#include "plotjuggler_m2/m2_canonical_names.h"

#include <PlotJuggler/plotdata.h>

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHash>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QOpenGLFunctions_2_0>
#include <QOpenGLWidget>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

#ifndef PJ_M2_ASSETS_DIR
#define PJ_M2_ASSETS_DIR ""
#endif

namespace plotjuggler_m2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double length() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const
  {
    const double len = length();
    return len > 1e-8 ? Vec3{x / len, y / len, z / len} : Vec3{0, 0, 1};
  }
  static Vec3 cross(const Vec3& a, const Vec3& b)
  {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  }
};

struct Mat4
{
  std::array<double, 16> m = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  static Mat4 identity() { return Mat4(); }

  static Mat4 translation(const Vec3& t)
  {
    Mat4 res;
    res.m[12] = t.x; res.m[13] = t.y; res.m[14] = t.z;
    return res;
  }

  static Mat4 fromRpy(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Mat4 res;
    res.m[0] = cy * cp;
    res.m[1] = sy * cp;
    res.m[2] = -sp;
    res.m[4] = cy * sp * sr - sy * cr;
    res.m[5] = sy * sp * sr + cy * cr;
    res.m[6] = cp * sr;
    res.m[8] = cy * sp * cr + sy * sr;
    res.m[9] = sy * sp * cr - cy * sr;
    res.m[10] = cp * cr;
    return res;
  }

  static Mat4 fromQuat(double x, double y, double z, double w)
  {
    Mat4 res;
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (n < 1e-8) return res;
    x /= n; y /= n; z /= n; w /= n;
    res.m[0] = 1.0 - 2.0 * (y * y + z * z);
    res.m[1] = 2.0 * (x * y + z * w);
    res.m[2] = 2.0 * (x * z - y * w);
    res.m[4] = 2.0 * (x * y - z * w);
    res.m[5] = 1.0 - 2.0 * (x * x + z * z);
    res.m[6] = 2.0 * (y * z + x * w);
    res.m[8] = 2.0 * (x * z + y * w);
    res.m[9] = 2.0 * (y * z - x * w);
    res.m[10] = 1.0 - 2.0 * (x * x + y * y);
    return res;
  }

  static Mat4 axisAngle(const Vec3& axis, double angle)
  {
    const Vec3 a = axis.normalized();
    const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
    Mat4 res;
    res.m[0] = t * a.x * a.x + c;
    res.m[1] = t * a.x * a.y + s * a.z;
    res.m[2] = t * a.x * a.z - s * a.y;
    res.m[4] = t * a.x * a.y - s * a.z;
    res.m[5] = t * a.y * a.y + c;
    res.m[6] = t * a.y * a.z + s * a.x;
    res.m[8] = t * a.x * a.z + s * a.y;
    res.m[9] = t * a.y * a.z - s * a.x;
    res.m[10] = t * a.z * a.z + c;
    return res;
  }

  Mat4 operator*(const Mat4& o) const
  {
    Mat4 res;
    for (int col = 0; col < 4; ++col)
    {
      for (int row = 0; row < 4; ++row)
      {
        res.m[col * 4 + row] =
            m[0 * 4 + row] * o.m[col * 4 + 0] +
            m[1 * 4 + row] * o.m[col * 4 + 1] +
            m[2 * 4 + row] * o.m[col * 4 + 2] +
            m[3 * 4 + row] * o.m[col * 4 + 3];
      }
    }
    return res;
  }
};

struct Triangle
{
  Vec3 a, b, c;
  Vec3 normal;
};

struct StlMesh
{
  std::vector<Triangle> triangles;
  bool valid = false;
  GLuint display_list = 0;
};

bool loadBinaryStl(const QByteArray& data, StlMesh* mesh)
{
  if (data.size() < 84) return false;
  uint32_t count = 0;
  std::memcpy(&count, data.constData() + 80, sizeof(uint32_t));
  if (data.size() < static_cast<qsizetype>(84 + count * 50)) return false;

  mesh->triangles.reserve(count);
  const char* ptr = data.constData() + 84;
  for (uint32_t i = 0; i < count; ++i)
  {
    const float* f = reinterpret_cast<const float*>(ptr);
    Vec3 norm{f[0], f[1], f[2]}, v1{f[3], f[4], f[5]}, v2{f[6], f[7], f[8]}, v3{f[9], f[10], f[11]};
    ptr += 50;
    Vec3 computed = Vec3::cross(v2 - v1, v3 - v1).normalized();
    Vec3 final_norm = (norm.x != 0.0 || norm.y != 0.0 || norm.z != 0.0) ? norm.normalized() : computed;
    mesh->triangles.push_back({v1, v2, v3, final_norm});
  }
  mesh->valid = !mesh->triangles.empty();
  return mesh->valid;
}

struct JointModel
{
  QString name;
  QString parent_link;
  QString child_link;
  Vec3 origin_xyz;
  Vec3 origin_rpy;
  Vec3 axis;
  int motor_index = -1;
  Mat4 local_origin = Mat4::identity();
};

struct LinkModel
{
  QString name;
  StlMesh mesh;
  QColor color = QColor(110, 115, 125);
};

class GlobalShortcutFilter : public QObject
{
public:
  explicit GlobalShortcutFilter(M2RobotViewToolbox* tb, QObject* parent = nullptr)
    : QObject(parent), tb_(tb)
  {
  }

  bool eventFilter(QObject* obj, QEvent* event) override
  {
    if (event->type() == QEvent::KeyPress)
    {
      auto* ke = static_cast<QKeyEvent*>(event);
      if ((ke->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) == (Qt::ControlModifier | Qt::ShiftModifier))
      {
        if (ke->key() == Qt::Key_R)
        {
          tb_->toggleView();
          return true;
        }
        else if (ke->key() == Qt::Key_P || ke->key() == Qt::Key_D)
        {
          tb_->togglePopOut();
          return true;
        }
      }
    }
    return QObject::eventFilter(obj, event);
  }

private:
  M2RobotViewToolbox* tb_;
};

inline QSpinBox* findStreamingSpinBox()
{
  for (auto* top : QApplication::topLevelWidgets())
  {
    if (auto* spin = top->findChild<QSpinBox*>("streamingSpinBox"))
    {
      return spin;
    }
  }
  return nullptr;
}

inline bool isBagActive(PJ::PlotDataMapRef* plot_data = nullptr)
{
  if (const char* env_mode = std::getenv("PLOTJUGGLER_M2_MODE"))
  {
    if (std::strcmp(env_mode, "bag") == 0) return true;
    if (std::strcmp(env_mode, "live") == 0) return false;
  }

  static auto last_proc_check = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  static bool cached_proc = false;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_proc_check).count() > 800)
  {
    last_proc_check = now;
    cached_proc = (system("pgrep -f 'm2_bag_player|ros2 bag play' >/dev/null 2>&1") == 0);
  }
  if (cached_proc) return true;

  if (plot_data)
  {
    for (const auto& kv : plot_data->numeric)
    {
      if (kv.first.find("playback_active") != std::string::npos && kv.second.size() > 0)
      {
        if (kv.second.back().y > 0.5) return true;
      }
    }
  }

  return false;
}

} // namespace

class M2RobotViewCanvas : public QOpenGLWidget, protected QOpenGLFunctions_2_0
{
public:
  struct TimeRange
  {
    double min_time = 0.0;
    double max_time = 0.0;
    bool valid = false;
  };

  explicit M2RobotViewCanvas(QWidget* parent = nullptr)
    : QOpenGLWidget(parent)
  {
    setFocusPolicy(Qt::StrongFocus);
    findAndLoadRobotAssets();

    auto* repaint_timer = new QTimer(this);
    connect(repaint_timer, &QTimer::timeout, this, [this]() {
      if (isVisible())
      {
        updatePoseFromPlotData();
        update();
      }
    });
    repaint_timer->start(33);
  }

  ~M2RobotViewCanvas() override
  {
    makeCurrent();
    for (auto& link : links_)
    {
      if (link.mesh.display_list != 0)
      {
        glDeleteLists(link.mesh.display_list, 1);
        link.mesh.display_list = 0;
      }
    }
    doneCurrent();
  }

  void setPlotDataMap(PJ::PlotDataMapRef* plot_data) { plot_data_ = plot_data; }
  PJ::PlotDataMapRef* plotDataMap() const { return plot_data_; }
  void setUseImu(bool val) { use_imu_ = val; update(); }
  bool useImu() const { return use_imu_; }
  void setDrawMesh(bool val) { draw_mesh_ = val; update(); }
  void setLiveMode(bool val) { live_mode_ = val; update(); }
  bool liveMode() const { return live_mode_; }
  void setSelectedTime(double t) { selected_time_ = t; update(); }
  double selectedTime() const { return selected_time_; }

  bool hasOrientation() const { return has_orientation_; }
  double rollDeg() const { return roll_deg_; }
  double pitchDeg() const { return pitch_deg_; }
  double yawDeg() const { return yaw_deg_; }

  void resetCamera()
  {
    yaw_ = 0.6;
    pitch_ = 0.4;
    camera_dist_ = 1.4;
    camera_target_ = {0.0, 0.0, 0.0};
    update();
  }

  TimeRange dataTimeRange() const
  {
    TimeRange r;
    if (!plot_data_) return r;

    auto updateRange = [&](const auto& series) {
      if (series.size() == 0) return;
      const double t0 = series.front().x, t1 = series.back().x;
      if (std::isfinite(t0) && std::isfinite(t1))
      {
        r.min_time = r.valid ? std::min({r.min_time, t0, t1}) : std::min(t0, t1);
        r.max_time = r.valid ? std::max({r.max_time, t0, t1}) : std::max(t0, t1);
        r.valid = true;
      }
    };

    // Scan for any joint or sensor_data series in the data map
    for (const auto& kv : plot_data_->numeric)
    {
      if (kv.first.find("/q") != std::string::npos || kv.first.find("sensor_data") != std::string::npos)
      {
        updateRange(kv.second);
      }
    }

    // General fallback: if no joint series matched, scan any non-empty numeric curve
    if (!r.valid)
    {
      for (const auto& kv : plot_data_->numeric)
      {
        updateRange(kv.second);
      }
    }

    return r;
  }

  void updatePoseFromPlotData()
  {
    if (!plot_data_) return;

    auto getVal = [&](const std::string& name, double& val) -> bool {
      auto it = plot_data_->numeric.find(name);
      if (it == plot_data_->numeric.end() || it->second.size() == 0) return false;
      if (live_mode_)
      {
        val = it->second.back().y;
        return std::isfinite(val);
      }
      auto opt = it->second.getYfromX(selected_time_);
      if (!opt || !std::isfinite(*opt))
      {
        if (selected_time_ <= it->second.front().x) val = it->second.front().y;
        else if (selected_time_ >= it->second.back().x) val = it->second.back().y;
        else return false;
      }
      else
      {
        val = *opt;
      }
      return std::isfinite(val);
    };

    auto getSeriesVec = [&](const std::string& prefix, const char* suffixes[], double* vals, int n) -> bool {
      for (int k = 0; k < n; ++k)
      {
        if (!getVal(prefix + suffixes[k], vals[k])) return false;
      }
      return true;
    };

    // 1. Read joint angles
    std::array<double, 12> q_vals = {};
    bool has_q = false;
    static int cached_pattern = -1;

    auto makeCandidate = [](int pattern, int i) -> std::string {
      const std::string idx = formatIndex(i);
      const std::string s_idx = std::to_string(i);
      switch (pattern)
      {
        case 0: return "rt/m2_metal/hw/sensor_data/joint/" + idx + "/q";
        case 1: return "/m2_metal/hw/sensor_data/joint/" + idx + "/q";
        case 2: return "sensor_data/joint/" + idx + "/q";
        case 3: return "rt/m2_metal/hw/sensor_data/q/" + s_idx;
        case 4: return "/m2_metal/hw/sensor_data/q/" + s_idx;
        case 5: return "sensor_data/q/" + s_idx;
        case 6: return "rt/m2_metal/hw/sensor_data/q/" + idx;
        case 7: return "/m2_metal/hw/sensor_data/q/" + idx;
        case 8: return "sensor_data/q/" + idx;
        case 9: return "joints*/q/" + idx;
        case 10: return "rt/m2_metal/hw/joint_command/joint/" + idx + "/q";
        case 11: return "/m2_metal/hw/joint_command/joint/" + idx + "/q";
        case 12: return "joint_command/joint/" + idx + "/q";
        case 13: return "rt/m2_metal/hw/joint_command/q/" + s_idx;
        case 14: return "/m2_metal/hw/joint_command/q/" + s_idx;
        case 15: return "joint_command/q/" + s_idx;
        case 16: return "rt/m2_metal/hw/joint_command/q/" + idx;
        case 17: return "/m2_metal/hw/joint_command/q/" + idx;
        case 18: return "joint_command/q/" + idx;
        case 19: return "joint_targets*/q/" + idx;
        default: return "";
      }
    };

    constexpr int kNumPatterns = 20;

    if (cached_pattern >= 0 && cached_pattern < kNumPatterns)
    {
      bool all_ok = true;
      for (int i = 0; i < 12; ++i)
      {
        if (!getVal(makeCandidate(cached_pattern, i), q_vals[i]))
        {
          all_ok = false;
          break;
        }
      }
      if (all_ok) has_q = true;
      else cached_pattern = -1;
    }

    if (!has_q)
    {
      for (int p = 0; p < kNumPatterns; ++p)
      {
        if (getVal(makeCandidate(p, 0), q_vals[0]))
        {
          bool all_matched = true;
          for (int i = 1; i < 12; ++i)
          {
            if (!getVal(makeCandidate(p, i), q_vals[i])) { all_matched = false; break; }
          }
          if (all_matched)
          {
            has_q = true;
            cached_pattern = p;
            break;
          }
        }
      }
    }

    // Dynamic scan fallback if no static pattern matched
    if (!has_q && plot_data_)
    {
      for (const auto& kv : plot_data_->numeric)
      {
        const std::string& key = kv.first;
        std::string prefix;
        bool is_joint_fmt = false;

        if (key.size() >= 4 && key.rfind("/q/0") == key.size() - 4)
        {
          prefix = key.substr(0, key.size() - 4);
        }
        else if (key.size() >= 12 && key.rfind("/joint/00/q") == key.size() - 12)
        {
          prefix = key.substr(0, key.size() - 12);
          is_joint_fmt = true;
        }

        if (!prefix.empty())
        {
          bool ok = true;
          for (int i = 0; i < 12; ++i)
          {
            std::string c = is_joint_fmt
                ? (prefix + "/joint/" + formatIndex(i) + "/q")
                : (prefix + "/q/" + std::to_string(i));
            if (!getVal(c, q_vals[i]))
            {
              if (!is_joint_fmt && getVal(prefix + "/q/" + formatIndex(i), q_vals[i]))
              {
                continue;
              }
              ok = false;
              break;
            }
          }
          if (ok)
          {
            has_q = true;
            break;
          }
        }
      }
    }
    if (has_q) current_q_ = q_vals;

    // 2. Read IMU orientation
    bool found_quat = false;
    double quat_vals[4] = {0.0, 0.0, 0.0, 1.0};
    const char* quat_xyz[] = {"/x", "/y", "/z", "/w"};
    const char* quat_num[] = {"/0", "/1", "/2", "/3"};
    const char* quat_idx[] = {"/quat.[0]", "/quat.[1]", "/quat.[2]", "/quat.[3]"};
    const char* rpy_suffixes[] = {"/roll", "/pitch", "/yaw"};

    for (const auto& p : {"rt/m2_metal/hw/sensor_data", "/m2_metal/hw/sensor_data", "sensor_data"})
    {
      if (getSeriesVec(std::string(p) + "/imu/quat", quat_xyz, quat_vals, 4) ||
          getSeriesVec(std::string(p) + "/quat", quat_num, quat_vals, 4) ||
          getSeriesVec(p, quat_idx, quat_vals, 4))
      {
        found_quat = true;
        break;
      }
    }

    bool found_rpy = false;
    double rpy_vals[3] = {0.0, 0.0, 0.0};
    const char* rpy_num[] = {"/0", "/1", "/2"};
    if (!found_quat)
    {
      for (const auto& p : {"rt/m2_metal/hw/sensor_data/rpy", "/m2_metal/hw/sensor_data/rpy", "sensor_data/rpy",
                            "rt/m2_metal/hw/sensor_data/imu/rpy", "/m2_metal/hw/sensor_data/imu/rpy", "sensor_data/imu/rpy"})
      {
        if (getSeriesVec(p, rpy_suffixes, rpy_vals, 3) || getSeriesVec(p, rpy_num, rpy_vals, 3))
        {
          found_rpy = true;
          break;
        }
      }
    }

    if (found_quat)
    {
      double qx = quat_vals[0], qy = quat_vals[1], qz = quat_vals[2], qw = quat_vals[3];
      const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      if (norm > 1e-6)
      {
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;
        has_orientation_ = true;
        current_root_tf_ = Mat4::fromQuat(qx, qy, qz, qw);

        const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        const double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

        roll_deg_ = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / kPi;
        pitch_deg_ = std::asin(sinp) * 180.0 / kPi;
        yaw_deg_ = std::atan2(siny_cosp, cosy_cosp) * 180.0 / kPi;
      }
    }
    else if (found_rpy)
    {
      has_orientation_ = true;
      current_root_tf_ = Mat4::fromRpy(rpy_vals[0], rpy_vals[1], rpy_vals[2]);
      roll_deg_ = rpy_vals[0] * 180.0 / kPi;
      pitch_deg_ = rpy_vals[1] * 180.0 / kPi;
      yaw_deg_ = rpy_vals[2] * 180.0 / kPi;
    }

    update();
  }

protected:
  void initializeGL() override
  {
    initializeOpenGLFunctions();

    // Invalidate cached display lists so they are recompiled in the newly created context
    for (auto it = links_.begin(); it != links_.end(); ++it)
    {
      it.value().mesh.display_list = 0;
    }

    glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat light_pos[] = {2.0f, -3.0f, 4.0f, 1.0f};
    GLfloat light_ambient[] = {0.35f, 0.35f, 0.4f, 1.0f};
    GLfloat light_diffuse[] = {0.8f, 0.8f, 0.85f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  }

  void resizeGL(int w, int h) override
  {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspectiveCustom(45.0, static_cast<double>(w) / std::max(1, h), 0.05, 50.0);
    glMatrixMode(GL_MODELVIEW);
  }

  void paintGL() override
  {
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    const double cx = camera_target_.x + camera_dist_ * std::cos(pitch_) * std::sin(yaw_);
    const double cy = camera_target_.y - camera_dist_ * std::cos(pitch_) * std::cos(yaw_);
    const double cz = camera_target_.z + camera_dist_ * std::sin(pitch_);
    gluLookAtCustom(cx, cy, cz, camera_target_.x, camera_target_.y, camera_target_.z, 0.0, 0.0, 1.0);

    drawGroundGrid();
    renderRobotModel();
  }

  void mousePressEvent(QMouseEvent* event) override { last_mouse_pos_ = event->pos(); }

  void mouseMoveEvent(QMouseEvent* event) override
  {
    const int dx = event->pos().x() - last_mouse_pos_.x();
    const int dy = event->pos().y() - last_mouse_pos_.y();
    last_mouse_pos_ = event->pos();

    if (event->buttons() & Qt::LeftButton)
    {
      yaw_ += dx * 0.01;
      pitch_ = std::clamp(pitch_ + dy * 0.01, -kPi * 0.45, kPi * 0.45);
      update();
    }
    else if (event->buttons() & (Qt::RightButton | Qt::MiddleButton))
    {
      camera_target_.x += (-std::cos(yaw_) * dx - std::sin(yaw_) * dy) * 0.002 * camera_dist_;
      camera_target_.y += (-std::sin(yaw_) * dx + std::cos(yaw_) * dy) * 0.002 * camera_dist_;
      update();
    }
  }

  void wheelEvent(QWheelEvent* event) override
  {
    camera_dist_ = std::clamp(camera_dist_ * (1.0 - (event->angleDelta().y() / 8.0) * 0.01), 0.2, 10.0);
    update();
  }

private:
  PJ::PlotDataMapRef* plot_data_ = nullptr;
  std::array<double, 12> current_q_ = {};

  bool use_imu_ = true;
  bool draw_mesh_ = true;
  bool live_mode_ = true;
  double selected_time_ = 0.0;
  bool has_orientation_ = false;
  Mat4 current_root_tf_ = Mat4::identity();
  double roll_deg_ = 0.0, pitch_deg_ = 0.0, yaw_deg_ = 0.0;

  QHash<QString, LinkModel> links_;
  QVector<JointModel> joints_;
  QHash<QString, QVector<int>> children_by_parent_;
  QString root_link_ = "base_link";

  double yaw_ = 0.6, pitch_ = 0.4, camera_dist_ = 1.4;
  Vec3 camera_target_ = {0.0, 0.0, 0.0};
  QPoint last_mouse_pos_;

  void gluPerspectiveCustom(double fovy, double aspect, double zNear, double zFar)
  {
    const double f = 1.0 / std::tan((fovy * kPi / 180.0) / 2.0);
    GLdouble m[16] = {f / aspect, 0, 0, 0,  0, f, 0, 0,  0, 0, (zFar + zNear) / (zNear - zFar), -1.0,  0, 0, (2.0 * zFar * zNear) / (zNear - zFar), 0};
    glMultMatrixd(m);
  }

  void gluLookAtCustom(double eyex, double eyey, double eyez, double centerx, double centery, double centerz, double upx, double upy, double upz)
  {
    Vec3 f = Vec3{centerx - eyex, centery - eyey, centerz - eyez}.normalized();
    Vec3 s = Vec3::cross(f, Vec3{upx, upy, upz}.normalized()).normalized();
    Vec3 u = Vec3::cross(s, f);
    GLdouble m[16] = {s.x, u.x, -f.x, 0,  s.y, u.y, -f.y, 0,  s.z, u.z, -f.z, 0,  0, 0, 0, 1.0};
    glMultMatrixd(m);
    glTranslated(-eyex, -eyey, -eyez);
  }

  void drawGroundGrid()
  {
    glDisable(GL_LIGHTING);
    glColor4f(0.25f, 0.28f, 0.35f, 0.5f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    constexpr double kSize = 1.5, kStep = 0.15;
    for (double i = -kSize; i <= kSize + 1e-4; i += kStep)
    {
      glVertex3d(i, -kSize, -0.28); glVertex3d(i, kSize, -0.28);
      glVertex3d(-kSize, i, -0.28); glVertex3d(kSize, i, -0.28);
    }
    glEnd();
    glEnable(GL_LIGHTING);
  }

  void renderRobotModel()
  {
    QHash<QString, Mat4> link_transforms;
    Mat4 root_tf = (use_imu_ && has_orientation_) ? current_root_tf_ : Mat4::identity();
    link_transforms.insert(root_link_, root_tf);
    applyFkRecursive(root_link_, root_tf, &link_transforms);

    // Skeleton bone lines
    if (!draw_mesh_)
    {
      glDisable(GL_LIGHTING);
      glLineWidth(3.0f);
      glBegin(GL_LINES);
      glColor4f(0.2f, 0.8f, 1.0f, 1.0f);
      for (const auto& joint : joints_)
      {
        if (link_transforms.contains(joint.parent_link) && link_transforms.contains(joint.child_link))
        {
          const auto& p0 = link_transforms[joint.parent_link].m;
          const auto& p1 = link_transforms[joint.child_link].m;
          glVertex3d(p0[12], p0[13], p0[14]);
          glVertex3d(p1[12], p1[13], p1[14]);
        }
      }
      glEnd();
      glEnable(GL_LIGHTING);
    }

    for (auto it = link_transforms.begin(); it != link_transforms.end(); ++it)
    {
      const QString& link_name = it.key();
      const Mat4& tf = it.value();
      if (!links_.contains(link_name)) continue;
      LinkModel& link = links_[link_name];

      glPushMatrix();
      glMultMatrixd(tf.m.data());

      auto drawTriangles = [this](const std::vector<Triangle>& tris) {
        glBegin(GL_TRIANGLES);
        for (const auto& tri : tris)
        {
          glNormal3d(tri.normal.x, tri.normal.y, tri.normal.z);
          glVertex3d(tri.a.x, tri.a.y, tri.a.z);
          glVertex3d(tri.b.x, tri.b.y, tri.b.z);
          glVertex3d(tri.c.x, tri.c.y, tri.c.z);
        }
        glEnd();
      };

      if (draw_mesh_ && link.mesh.valid)
      {
        if (link.mesh.display_list == 0 || !glIsList(link.mesh.display_list))
        {
          link.mesh.display_list = glGenLists(1);
          if (link.mesh.display_list != 0)
          {
            glNewList(link.mesh.display_list, GL_COMPILE);
            drawTriangles(link.mesh.triangles);
            glEndList();
          }
        }

        glColor4f(link.color.redF(), link.color.greenF(), link.color.blueF(), 1.0f);
        if (link.mesh.display_list != 0 && glIsList(link.mesh.display_list))
        {
          glCallList(link.mesh.display_list);
        }
        else
        {
          // Direct fallback rendering ensures mesh renders even if display list generation is unsupported or deferred
          drawTriangles(link.mesh.triangles);
        }
      }
      else
      {
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        glColor4f(link.color.redF(), link.color.greenF(), link.color.blueF(), 1.0f);
        glBegin(GL_LINES);
        glVertex3d(0, 0, 0); glVertex3d(0, 0, 0.02);
        glEnd();
        glEnable(GL_LIGHTING);
      }

      glPopMatrix();
    }
  }

  void applyFkRecursive(const QString& parent_link, const Mat4& parent_tf, QHash<QString, Mat4>* transforms)
  {
    const QVector<int>& child_joints = children_by_parent_.value(parent_link);
    for (int joint_idx : child_joints)
    {
      const JointModel& joint = joints_[joint_idx];
      const double q = (joint.motor_index >= 0 && joint.motor_index < 12) ? current_q_[joint.motor_index] : 0.0;
      const Mat4 rot = Mat4::axisAngle(joint.axis, q);
      const Mat4 child_tf = parent_tf * joint.local_origin * rot;
      transforms->insert(joint.child_link, child_tf);
      applyFkRecursive(joint.child_link, child_tf, transforms);
    }
  }

  void findAndLoadRobotAssets()
  {
    QString base_dir;
    for (const auto& dir : {
      PJ_M2_ASSETS_DIR,
      QDir(QCoreApplication::applicationDirPath()).filePath("../assets/m2_metal_description").toUtf8().constData(),
      "third_party/xterra_m2_assets/m2_metal_description"
    })
    {
      if (!QString(dir).isEmpty() && QDir(dir).exists("urdf/m2_metal_description.urdf"))
      {
        base_dir = dir;
        break;
      }
    }
    if (base_dir.isEmpty()) return;

    QFile file(QDir(base_dir).filePath("urdf/m2_metal_description.urdf"));
    if (!file.open(QIODevice::ReadOnly)) return;

    QDomDocument doc;
    if (!doc.setContent(&file)) return;

    QHash<QString, int> motor_indices;
    for (std::size_t i = 0; i < kM2JointCount; ++i)
      motor_indices.insert(QString::fromUtf8(kJointNames[i].data(), kJointNames[i].size()), static_cast<int>(i));

    auto parseVec = [](const QString& s) -> Vec3 {
      const auto p = s.split(' ', Qt::SkipEmptyParts);
      return p.size() == 3 ? Vec3{p[0].toDouble(), p[1].toDouble(), p[2].toDouble()} : Vec3{};
    };

    const QString meshes_dir = QDir(base_dir).filePath("meshes");
    const QDomNodeList link_nodes = doc.elementsByTagName("link");
    for (int i = 0; i < link_nodes.size(); ++i)
    {
      const QDomElement elem = link_nodes.at(i).toElement();
      LinkModel link;
      link.name = elem.attribute("name");

      const QDomElement mesh_elem = elem.firstChildElement("visual").firstChildElement("geometry").firstChildElement("mesh");
      if (!mesh_elem.isNull())
      {
        QFile mesh_file(QDir(meshes_dir).filePath(QFileInfo(mesh_elem.attribute("filename")).fileName()));
        if (mesh_file.open(QIODevice::ReadOnly)) loadBinaryStl(mesh_file.readAll(), &link.mesh);
      }

      // Official Svan M2 scheme: svan_green torso, svan_grey legs
      if (link.name == "base_link" || link.name.contains("torso"))
      {
        link.color = QColor::fromRgbF(0.035f, 0.28f, 0.19f); // svan_green (Forest Green torso)
      }
      else if (link.name.contains("foot"))
      {
        link.color = QColor(22, 24, 26); // Dark rubber foot
      }
      else
      {
        link.color = QColor::fromRgbF(0.50f, 0.50f, 0.50f); // svan_grey (Medium Grey legs)
      }

      links_.insert(link.name, link);
    }

    const QDomNodeList joint_nodes = doc.elementsByTagName("joint");
    for (int i = 0; i < joint_nodes.size(); ++i)
    {
      const QDomElement elem = joint_nodes.at(i).toElement();
      JointModel joint;
      joint.name = elem.attribute("name");
      joint.parent_link = elem.firstChildElement("parent").attribute("link");
      joint.child_link = elem.firstChildElement("child").attribute("link");

      const QDomElement origin_elem = elem.firstChildElement("origin");
      if (!origin_elem.isNull())
      {
        joint.origin_xyz = parseVec(origin_elem.attribute("xyz"));
        joint.origin_rpy = parseVec(origin_elem.attribute("rpy"));
        joint.local_origin = Mat4::translation(joint.origin_xyz) *
                             Mat4::fromRpy(joint.origin_rpy.x, joint.origin_rpy.y, joint.origin_rpy.z);
      }

      const QDomElement axis_elem = elem.firstChildElement("axis");
      joint.axis = (!axis_elem.isNull()) ? parseVec(axis_elem.attribute("xyz")).normalized() : Vec3{0, 0, 1};
      joint.motor_index = motor_indices.value(joint.name, -1);

      children_by_parent_[joint.parent_link].push_back(joints_.size());
      joints_.push_back(joint);
    }
  }
};

class M2RobotViewWidget : public QWidget
{
public:
  explicit M2RobotViewWidget(QWidget* parent = nullptr)
    : QWidget(parent)
  {
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(6, 6, 6, 6);
    main_layout->setSpacing(4);

    auto* toolbar = new QHBoxLayout();
    toolbar->setSpacing(8);

    imu_chk_ = new QCheckBox(tr("IMU"), this);
    imu_chk_->setChecked(true);
    imu_chk_->setToolTip(tr("Rotate 3D robot body according to onboard IMU orientation"));
    toolbar->addWidget(imu_chk_);

    mesh_chk_ = new QCheckBox(tr("Mesh"), this);
    mesh_chk_->setChecked(true);
    mesh_chk_->setToolTip(tr("Toggle 3D STL meshes vs skeleton kinematics"));
    toolbar->addWidget(mesh_chk_);

    live_chk_ = new QCheckBox(tr("Live"), this);
    live_chk_->setChecked(true);
    live_chk_->setToolTip(tr("Follow incoming real-time telemetry stream"));
    toolbar->addWidget(live_chk_);

    reset_btn_ = new QPushButton(tr("Reset Camera"), this);
    reset_btn_->setToolTip(tr("Reset 3D camera to default orbit position and angle"));
    reset_btn_->setCursor(Qt::PointingHandCursor);
    toolbar->addWidget(reset_btn_);

    status_label_ = new QLabel(tr("IMU: Searching..."), this);
    toolbar->addWidget(status_label_);

    toolbar->addStretch(1);

    popout_btn_ = new QPushButton(tr("⧉ Pop Out (PiP)"), this);
    popout_btn_->setToolTip(tr("Detach 3D view into a floating Picture-in-Picture window (Ctrl+Shift+P / Ctrl+Shift+D)"));
    popout_btn_->setCursor(Qt::PointingHandCursor);
    toolbar->addWidget(popout_btn_);

    close_btn_ = new QPushButton(tr("✕ Close View (Esc)"), this);
    close_btn_->setToolTip(tr("Close 3D View and return to Plots (Esc / Ctrl+Shift+R)"));
    close_btn_->setCursor(Qt::PointingHandCursor);
    toolbar->addWidget(close_btn_);

    main_layout->addLayout(toolbar);

    auto* esc_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(esc_shortcut, &QShortcut::activated, close_btn_, &QPushButton::click);

    connect(popout_btn_, &QPushButton::clicked, this, [this]() {
      if (popout_cb_) popout_cb_();
    });

    canvas_ = new M2RobotViewCanvas(this);
    main_layout->addWidget(canvas_, 1);

    auto* timeline = new QHBoxLayout();
    timeline->setSpacing(6);
    time_min_ = new QLabel("--", this);
    time_slider_ = new QSlider(Qt::Horizontal, this);
    time_slider_->setRange(0, 10000);
    time_slider_->setEnabled(false);
    time_max_ = new QLabel("--", this);
    time_current_ = new QLabel("--", this);
    time_current_->setStyleSheet("font-weight: bold; min-width: 55px;");

    buffer_btn_ = new QPushButton(this);
    buffer_btn_->setCursor(Qt::PointingHandCursor);
    updateBufferButton();

    connect(buffer_btn_, &QPushButton::clicked, this, [this]() {
      auto* spin = findStreamingSpinBox();
      if (spin)
      {
        manual_buffer_override_ = true;
        if (spin->value() == spin->maximum())
        {
          spin->setValue(30);
        }
        else
        {
          spin->setValue(spin->maximum());
        }
      }
      updateBufferButton();
    });

    timeline->addWidget(time_min_);
    timeline->addWidget(time_slider_, 1);
    timeline->addWidget(time_max_);
    timeline->addWidget(time_current_);
    timeline->addWidget(buffer_btn_);
    main_layout->addLayout(timeline);

    connect(imu_chk_, &QCheckBox::toggled, canvas_, [this](bool checked) { canvas_->setUseImu(checked); });
    connect(mesh_chk_, &QCheckBox::toggled, canvas_, [this](bool checked) { canvas_->setDrawMesh(checked); });

    connect(live_chk_, &QCheckBox::toggled, this, [this](bool checked) {
      canvas_->setLiveMode(checked);
      if (checked)
      {
        const auto r = canvas_->dataTimeRange();
        if (r.valid)
        {
          time_slider_->blockSignals(true);
          time_slider_->setValue(10000);
          time_slider_->blockSignals(false);
        }
      }
      updateTimeline();
    });

    connect(time_slider_, &QSlider::valueChanged, this, [this](int val) {
      const auto r = canvas_->dataTimeRange();
      if (!r.valid || r.max_time <= r.min_time) return;
      live_chk_->setChecked(false);
      const double t = r.min_time + (static_cast<double>(val) / 10000.0) * (r.max_time - r.min_time);
      canvas_->setSelectedTime(t);
      canvas_->updatePoseFromPlotData();
      time_current_->setText(QString("%1s").arg(t, 0, 'f', 2));
    });

    connect(reset_btn_, &QPushButton::clicked, canvas_, [this]() { canvas_->resetCamera(); });
  }

  QPushButton* closeButton() const { return close_btn_; }
  void setPopoutCallback(std::function<void()> cb) { popout_cb_ = std::move(cb); }

  void setPoppedOutState(bool popped_out)
  {
    is_popped_out_ = popped_out;
    if (is_popped_out_)
    {
      popout_btn_->setText(tr("⇲ Dock View"));
      popout_btn_->setToolTip(tr("Dock 3D view back into the main PlotJuggler window (Ctrl+Shift+P / Ctrl+Shift+D)"));
    }
    else
    {
      popout_btn_->setText(tr("⧉ Pop Out (PiP)"));
      popout_btn_->setToolTip(tr("Detach 3D view into a floating Picture-in-Picture window (Ctrl+Shift+P / Ctrl+Shift+D)"));
    }
  }

  void setPlotDataMap(PJ::PlotDataMapRef* plot_data)
  {
    if (canvas_) canvas_->setPlotDataMap(plot_data);
  }

  void updateBufferButton()
  {
    if (!buffer_btn_) return;
    auto* spin = findStreamingSpinBox();
    if (!spin) return;

    static bool spin_signal_connected = false;
    if (!spin_signal_connected)
    {
      spin_signal_connected = true;
      connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) {
        updateBufferButton();
      });
    }

    // Dynamic bag detection: auto-adjust if user hasn't manually overridden
    if (!manual_buffer_override_)
    {
      const bool bag_detected = isBagActive(canvas_ ? canvas_->plotDataMap() : nullptr);
      if (bag_detected && spin->value() != spin->maximum())
      {
        spin->setValue(spin->maximum());
      }
      else if (!bag_detected && spin->value() > 30)
      {
        spin->setValue(30);
      }
    }

    const bool is_inf = (spin->value() == spin->maximum());
    if (is_inf)
    {
      buffer_btn_->setText(manual_buffer_override_ ? tr("Buffer: ∞ Bag") : tr("Buffer: ∞ Bag (Auto)"));
      buffer_btn_->setToolTip(tr("Retaining full bag history. Click to toggle to 30s rolling live buffer."));
      buffer_btn_->setStyleSheet(
        "QPushButton { background: #1a3826; color: #4cd964; border: 1px solid #2d6641; border-radius: 4px; padding: 2px 7px; font-weight: bold; font-size: 11px; } "
        "QPushButton:hover { background: #245035; color: #6ee885; } "
        "QPushButton:pressed { background: #142a1d; }");
    }
    else
    {
      const int val = spin->value();
      buffer_btn_->setText(manual_buffer_override_ ? QString("Buffer: %1s").arg(val) : QString("Buffer: %1s (Auto)").arg(val));
      buffer_btn_->setToolTip(tr("Retaining %1s rolling buffer. Click to toggle to unlimited full bag buffer.").arg(val));
      buffer_btn_->setStyleSheet(
        "QPushButton { background: #252830; color: #61afef; border: 1px solid #3d4352; border-radius: 4px; padding: 2px 7px; font-weight: bold; font-size: 11px; } "
        "QPushButton:hover { background: #323642; color: #82c0f4; } "
        "QPushButton:pressed { background: #1e2027; }");
    }
  }

  void updateTimeline()
  {
    if (++auto_detect_counter_ % 25 == 0)
    {
      updateBufferButton();
    }
    if (!canvas_) return;
    const auto r = canvas_->dataTimeRange();
    time_slider_->setEnabled(r.valid && r.max_time > r.min_time);
    if (!r.valid)
    {
      time_min_->setText("--");
      time_max_->setText("--");
      time_current_->setText("--");
      return;
    }
    time_min_->setText(QString("%1s").arg(r.min_time, 0, 'f', 2));
    time_max_->setText(QString("%1s").arg(r.max_time, 0, 'f', 2));
    if (canvas_->liveMode())
    {
      time_slider_->blockSignals(true);
      time_slider_->setValue(10000);
      time_slider_->blockSignals(false);
      time_current_->setText(QString("%1s").arg(r.max_time, 0, 'f', 2));
    }
    else
    {
      time_current_->setText(QString("%1s").arg(canvas_->selectedTime(), 0, 'f', 2));
    }
  }

  void updatePoseFromPlotData()
  {
    if (!canvas_) return;
    canvas_->updatePoseFromPlotData();
    updateTimeline();

    if (!canvas_->useImu())
    {
      status_label_->setText(tr("IMU: Disabled"));
      status_label_->setStyleSheet("color: #e5a50a;");
    }
    else if (canvas_->hasOrientation())
    {
      status_label_->setText(tr("IMU: Roll %1° | Pitch %2° | Yaw %3°")
                                 .arg(canvas_->rollDeg(), 5, 'f', 1)
                                 .arg(canvas_->pitchDeg(), 5, 'f', 1)
                                 .arg(canvas_->yawDeg(), 5, 'f', 1));
      status_label_->setStyleSheet("color: #4cd964; font-weight: bold;");
    }
    else
    {
      status_label_->setText(tr("IMU: Inactive"));
      status_label_->setStyleSheet("color: #8899a6;");
    }
  }

protected:
  void showEvent(QShowEvent* event) override
  {
    QWidget::showEvent(event);
    updatePoseFromPlotData();
  }

  void resizeEvent(QResizeEvent* event) override
  {
    QWidget::resizeEvent(event);
    if (canvas_) canvas_->update();
  }

  void closeEvent(QCloseEvent* event) override
  {
    if (is_popped_out_)
    {
      event->ignore();
      if (popout_cb_) popout_cb_();
      return;
    }
    QWidget::closeEvent(event);
  }

private:
  M2RobotViewCanvas* canvas_ = nullptr;
  QCheckBox* imu_chk_ = nullptr;
  QCheckBox* mesh_chk_ = nullptr;
  QCheckBox* live_chk_ = nullptr;
  QPushButton* reset_btn_ = nullptr;
  QPushButton* close_btn_ = nullptr;
  QPushButton* popout_btn_ = nullptr;
  QLabel* status_label_ = nullptr;
  QLabel* time_min_ = nullptr;
  QLabel* time_max_ = nullptr;
  QLabel* time_current_ = nullptr;
  QPushButton* buffer_btn_ = nullptr;
  QSlider* time_slider_ = nullptr;
  bool is_popped_out_ = false;
  std::function<void()> popout_cb_;
  bool manual_buffer_override_ = false;
  int auto_detect_counter_ = 0;
};

M2RobotViewToolbox::M2RobotViewToolbox() = default;
M2RobotViewToolbox::~M2RobotViewToolbox() = default;

void M2RobotViewToolbox::init(PJ::PlotDataMapRef& src_data, PJ::TransformsMap& /*transform_map*/)
{
  if (!widget_)
  {
    widget_ = new M2RobotViewWidget();
    connect(widget_->closeButton(), &QPushButton::clicked, this, [this]() {
      if (is_popped_out_)
      {
        togglePopOut();
      }
      Q_EMIT closed();
    });
    widget_->setPopoutCallback([this]() {
      togglePopOut();
    });
  }
  widget_->setPlotDataMap(&src_data);

  if (auto* spin = findStreamingSpinBox())
  {
    if (isBagActive(&src_data))
    {
      spin->setValue(spin->maximum());
    }
    else if (spin->value() < 30)
    {
      spin->setValue(30);
    }
  }

  if (!filter_)
  {
    filter_ = new GlobalShortcutFilter(this, this);
    qApp->installEventFilter(filter_);
  }

}

std::pair<QWidget*, PJ::ToolboxPlugin::WidgetType> M2RobotViewToolbox::providedWidget() const
{
  return {widget_, PJ::ToolboxPlugin::FIXED};
}

bool M2RobotViewToolbox::onShowWidget()
{
  if (is_popped_out_ && pip_dialog_)
  {
    pip_dialog_->show();
    pip_dialog_->raise();
    pip_dialog_->activateWindow();
    return true;
  }
  if (widget_)
  {
    widget_->show();
    widget_->raise();
    return true;
  }
  return false;
}

void M2RobotViewToolbox::toggleView()
{
  if (!widget_) return;

  if (is_popped_out_ && pip_dialog_)
  {
    if (pip_dialog_->isVisible())
    {
      pip_dialog_->hide();
    }
    else
    {
      pip_dialog_->show();
      pip_dialog_->raise();
      pip_dialog_->activateWindow();
    }
    return;
  }

  auto* stacked = qobject_cast<QStackedWidget*>(widget_->parentWidget());
  if (stacked)
  {
    if (stacked->currentWidget() == widget_)
    {
      Q_EMIT closed();
    }
    else
    {
      onShowWidget();
      stacked->setCurrentWidget(widget_);
    }
    return;
  }

  if (widget_->isVisible())
  {
    widget_->hide();
    Q_EMIT closed();
  }
  else
  {
    onShowWidget();
    widget_->show();
    widget_->raise();
  }
}

void M2RobotViewToolbox::togglePopOut()
{
  if (!widget_) return;

  if (!is_popped_out_)
  {
    if (!saved_parent_ && widget_->parentWidget())
    {
      saved_parent_ = widget_->parentWidget();
    }
    if (auto* stacked = qobject_cast<QStackedWidget*>(saved_parent_))
    {
      saved_stacked_index_ = stacked->indexOf(widget_);
    }

    if (!pip_dialog_)
    {
      pip_dialog_ = new QDialog(nullptr, Qt::Window | Qt::WindowTitleHint | Qt::WindowCloseButtonHint | Qt::WindowMinMaxButtonsHint | Qt::WindowStaysOnTopHint);
      pip_dialog_->setWindowTitle(tr("Svan M2 Robot View — Picture-in-Picture"));
      pip_dialog_->resize(680, 500);
      auto* layout = new QVBoxLayout(pip_dialog_);
      layout->setContentsMargins(0, 0, 0, 0);
      layout->setSpacing(0);
      connect(pip_dialog_, &QDialog::finished, this, [this](int) {
        if (is_popped_out_)
        {
          togglePopOut();
        }
      });
    }

    pip_dialog_->layout()->addWidget(widget_);
    widget_->setPoppedOutState(true);
    is_popped_out_ = true;

    pip_dialog_->show();
    pip_dialog_->raise();
    pip_dialog_->activateWindow();
    widget_->show();
    widget_->updatePoseFromPlotData();

    // Switch main PlotJuggler window back to plots so user has simultaneous PiP and plot view
    Q_EMIT closed();
  }
  else
  {
    widget_->setPoppedOutState(false);
    is_popped_out_ = false;

    if (pip_dialog_)
    {
      pip_dialog_->hide();
    }

    if (auto* stacked = qobject_cast<QStackedWidget*>(saved_parent_))
    {
      if (saved_stacked_index_ >= 0 && saved_stacked_index_ <= stacked->count())
      {
        stacked->insertWidget(saved_stacked_index_, widget_);
      }
      else
      {
        stacked->addWidget(widget_);
      }
      stacked->setCurrentWidget(widget_);
    }
    else if (saved_parent_)
    {
      widget_->setParent(saved_parent_);
      if (saved_parent_->layout())
      {
        saved_parent_->layout()->addWidget(widget_);
      }
    }

    widget_->show();
    widget_->raise();
    widget_->updatePoseFromPlotData();
  }
}

} // namespace plotjuggler_m2
