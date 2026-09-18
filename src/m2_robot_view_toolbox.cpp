#include "m2_robot_view_toolbox.h"
#include "plotjuggler_m2/m2_canonical_names.h"

#include <PlotJuggler/plotdata.h>

#include <QCheckBox>
#include <QCoreApplication>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHash>
#include <QLabel>
#include <QMouseEvent>
#include <QOpenGLFunctions_2_0>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
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
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

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
    return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    };
  }

  static double dot(const Vec3& a, const Vec3& b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }
};

struct Mat4
{
  std::array<double, 16> m = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

  static Mat4 identity() { return Mat4(); }

  static Mat4 translation(const Vec3& t)
  {
    Mat4 res;
    res.m[12] = t.x;
    res.m[13] = t.y;
    res.m[14] = t.z;
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
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

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
  QColor color = QColor(140, 145, 150); // Metallic gray default
};

} // namespace

class M2RobotViewCanvas : public QOpenGLWidget, protected QOpenGLFunctions_2_0
{
public:
  explicit M2RobotViewCanvas(QWidget* parent = nullptr)
    : QOpenGLWidget(parent)
  {
    setFocusPolicy(Qt::StrongFocus);
    findAndLoadRobotAssets();
  }

  void setPlotDataMap(PJ::PlotDataMapRef* plot_data)
  {
    plot_data_ = plot_data;
  }

  void setUseImu(bool val)
  {
    use_imu_ = val;
    update();
  }

  bool useImu() const { return use_imu_; }
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

  void updatePoseFromPlotData()
  {
    if (!plot_data_) return;

    auto getVal = [&](const std::string& name, double& val) -> bool {
      auto it = plot_data_->numeric.find(name);
      if (it != plot_data_->numeric.end() && it->second.size() > 0)
      {
        val = it->second.back().y;
        return true;
      }
      return false;
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

    for (int i = 0; i < 12; ++i)
    {
      const std::string idx = formatIndex(i);
      const std::string s_idx = std::to_string(i);
      for (const std::string& candidate : {
        "rt/m2_metal/hw/sensor_data/joint/" + idx + "/q",
        "/m2_metal/hw/sensor_data/joint/" + idx + "/q",
        "sensor_data/joint/" + idx + "/q",
        "/m2_metal/hw/sensor_data/q." + s_idx,
        "/m2_metal/hw/sensor_data/q.[" + s_idx + "]",
        "sensor_data/q/" + idx,
        "joints*/q/" + idx
      })
      {
        if (getVal(candidate, q_vals[i]))
        {
          has_q = true;
          break;
        }
      }
    }
    if (has_q) current_q_ = q_vals;

    // 2. Read IMU orientation quaternion (or RPY fallback)
    bool found_quat = false;
    double quat_vals[4] = {0.0, 0.0, 0.0, 1.0};
    const char* quat_xyz[] = {"/x", "/y", "/z", "/w"};
    const char* quat_idx[] = {"/quat.[0]", "/quat.[1]", "/quat.[2]", "/quat.[3]"};
    const char* rpy_suffixes[] = {"/roll", "/pitch", "/yaw"};

    for (const auto& p : {"rt/m2_metal/hw/sensor_data", "/m2_metal/hw/sensor_data", "sensor_data"})
    {
      if (getSeriesVec(std::string(p) + "/imu/quat", quat_xyz, quat_vals, 4) ||
          getSeriesVec(p, quat_idx, quat_vals, 4))
      {
        found_quat = true;
        break;
      }
    }

    bool found_rpy = false;
    double rpy_vals[3] = {0.0, 0.0, 0.0};
    if (!found_quat)
    {
      for (const auto& p : {"rt/m2_metal/hw/sensor_data/imu/rpy", "/m2_metal/hw/sensor_data/imu/rpy", "sensor_data/imu/rpy"})
      {
        if (getSeriesVec(p, rpy_suffixes, rpy_vals, 3))
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

    if (has_q || has_orientation_)
    {
      update();
    }
  }

protected:
  void initializeGL() override
  {
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.14f, 0.18f, 1.0f); // Sleek dark slate
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
    const double aspect = static_cast<double>(w) / std::max(1, h);
    gluPerspectiveCustom(45.0, aspect, 0.05, 50.0);
    glMatrixMode(GL_MODELVIEW);
  }

  void paintGL() override
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Camera orbit
    const double cx = camera_target_.x + camera_dist_ * std::cos(pitch_) * std::sin(yaw_);
    const double cy = camera_target_.y - camera_dist_ * std::cos(pitch_) * std::cos(yaw_);
    const double cz = camera_target_.z + camera_dist_ * std::sin(pitch_);

    gluLookAtCustom(cx, cy, cz, camera_target_.x, camera_target_.y, camera_target_.z, 0.0, 0.0, 1.0);

    drawGroundGrid();
    renderRobotModel();
  }

  void mousePressEvent(QMouseEvent* event) override
  {
    last_mouse_pos_ = event->pos();
  }

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
    const double num_degrees = event->angleDelta().y() / 8.0;
    camera_dist_ = std::clamp(camera_dist_ * (1.0 - num_degrees * 0.01), 0.2, 10.0);
    update();
  }

private:
  PJ::PlotDataMapRef* plot_data_ = nullptr;
  std::array<double, 12> current_q_ = {};

  bool use_imu_ = true;
  bool has_orientation_ = false;
  Mat4 current_root_tf_ = Mat4::identity();
  double roll_deg_ = 0.0;
  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;

  QHash<QString, LinkModel> links_;
  QVector<JointModel> joints_;
  QHash<QString, QVector<int>> children_by_parent_;
  QString root_link_ = "base_link";

  // Camera parameters
  double yaw_ = 0.6;
  double pitch_ = 0.4;
  double camera_dist_ = 1.4;
  Vec3 camera_target_ = {0.0, 0.0, 0.0};
  QPoint last_mouse_pos_;

  void gluPerspectiveCustom(double fovy, double aspect, double zNear, double zFar)
  {
    const double f = 1.0 / std::tan((fovy * kPi / 180.0) / 2.0);
    GLdouble m[16] = {f / aspect, 0, 0, 0,  0, f, 0, 0,  0, 0, (zFar + zNear) / (zNear - zFar), -1.0,  0, 0, (2.0 * zFar * zNear) / (zNear - zFar), 0};
    glMultMatrixd(m);
  }

  void gluLookAtCustom(double eyex, double eyey, double eyez,
                       double centerx, double centery, double centerz,
                       double upx, double upy, double upz)
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
    constexpr double kSize = 1.5;
    constexpr double kStep = 0.15;
    for (double i = -kSize; i <= kSize + 1e-4; i += kStep)
    {
      glVertex3d(i, -kSize, -0.28);
      glVertex3d(i, kSize, -0.28);
      glVertex3d(-kSize, i, -0.28);
      glVertex3d(kSize, i, -0.28);
    }
    glEnd();
    glEnable(GL_LIGHTING);
  }

  void renderRobotModel()
  {
    QHash<QString, Mat4> link_transforms;
    Mat4 root_tf = Mat4::identity();
    if (use_imu_ && has_orientation_)
    {
      root_tf = current_root_tf_;
    }

    link_transforms.insert(root_link_, root_tf);

    applyFkRecursive(root_link_, root_tf, &link_transforms);

    for (auto it = link_transforms.begin(); it != link_transforms.end(); ++it)
    {
      const QString& link_name = it.key();
      const Mat4& tf = it.value();

      if (!links_.contains(link_name)) continue;
      const LinkModel& link = links_[link_name];
      if (!link.mesh.valid) continue;

      glPushMatrix();
      glMultMatrixd(tf.m.data());

      glColor4f(link.color.redF(), link.color.greenF(), link.color.blueF(), 1.0f);
      glBegin(GL_TRIANGLES);
      for (const auto& tri : link.mesh.triangles)
      {
        glNormal3d(tri.normal.x, tri.normal.y, tri.normal.z);
        glVertex3d(tri.a.x, tri.a.y, tri.a.z);
        glVertex3d(tri.b.x, tri.b.y, tri.b.z);
        glVertex3d(tri.c.x, tri.c.y, tri.c.z);
      }
      glEnd();

      glPopMatrix();
    }
  }

  void applyFkRecursive(const QString& parent_link, const Mat4& parent_tf, QHash<QString, Mat4>* transforms)
  {
    const QVector<int>& child_joints = children_by_parent_.value(parent_link);
    for (int joint_idx : child_joints)
    {
      const JointModel& joint = joints_[joint_idx];
      double q = 0.0;
      if (joint.motor_index >= 0 && joint.motor_index < 12)
      {
        q = current_q_[joint.motor_index];
      }

      const Mat4 rot = Mat4::axisAngle(joint.axis, q);
      const Mat4 child_tf = parent_tf * joint.local_origin * rot;
      transforms->insert(joint.child_link, child_tf);

      applyFkRecursive(joint.child_link, child_tf, transforms);
    }
  }

  void findAndLoadRobotAssets()
  {
    QString base_dir;
    const QStringList search_dirs = {
      PJ_M2_ASSETS_DIR,
      QDir(QCoreApplication::applicationDirPath()).filePath("../assets/m2_metal_description"),
      QDir::current().filePath("third_party/xterra_m2_assets/m2_metal_description"),
      "/home/robotics/navneeth/pj-plugin/m2-pj/third_party/xterra_m2_assets/m2_metal_description"
    };

    for (const auto& dir : search_dirs)
    {
      if (!dir.isEmpty() && QDir(dir).exists("urdf/m2_metal_description.urdf"))
      {
        base_dir = dir;
        break;
      }
    }

    if (base_dir.isEmpty()) return;

    const QString urdf_path = QDir(base_dir).filePath("urdf/m2_metal_description.urdf");
    const QString meshes_dir = QDir(base_dir).filePath("meshes");

    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return;

    QDomDocument doc;
    if (!doc.setContent(&file)) return;

    // Build motor name to index mapping
    QHash<QString, int> motor_indices;
    for (std::size_t i = 0; i < kM2JointCount; ++i)
    {
      motor_indices.insert(QString::fromUtf8(kJointNames[i].data(), kJointNames[i].size()), static_cast<int>(i));
    }

    auto parseVec = [](const QString& s) -> Vec3 {
      const auto parts = s.split(' ', Qt::SkipEmptyParts);
      if (parts.size() == 3) return {parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()};
      return {};
    };

    // Parse links and meshes
    const QDomNodeList link_nodes = doc.elementsByTagName("link");
    for (int i = 0; i < link_nodes.size(); ++i)
    {
      const QDomElement elem = link_nodes.at(i).toElement();
      LinkModel link;
      link.name = elem.attribute("name");

      const QDomElement mesh_elem = elem.firstChildElement("visual").firstChildElement("geometry").firstChildElement("mesh");
      if (!mesh_elem.isNull())
      {
        const QString mesh_full_path = QDir(meshes_dir).filePath(QFileInfo(mesh_elem.attribute("filename")).fileName());
        QFile mesh_file(mesh_full_path);
        if (mesh_file.open(QIODevice::ReadOnly))
        {
          loadBinaryStl(mesh_file.readAll(), &link.mesh);
        }
      }

      if (link.name.contains("foot")) link.color = QColor(40, 42, 45); // Dark rubber
      else if (link.name == "base_link") link.color = QColor(160, 165, 170); // Metal body
      else link.color = QColor(110, 115, 125); // Leg links

      links_.insert(link.name, link);
    }

    // Parse joints
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
    toolbar->setSpacing(10);

    imu_chk_ = new QCheckBox(tr("Torso IMU Orientation"), this);
    imu_chk_->setChecked(true);
    imu_chk_->setToolTip(tr("Rotate 3D robot body according to onboard IMU orientation"));
    toolbar->addWidget(imu_chk_);

    reset_btn_ = new QPushButton(tr("Reset Camera"), this);
    reset_btn_->setFixedWidth(100);
    toolbar->addWidget(reset_btn_);

    status_label_ = new QLabel(tr("IMU: Searching..."), this);
    status_label_->setStyleSheet("color: #8899a6;");
    toolbar->addWidget(status_label_);

    toolbar->addStretch(1);

    close_btn_ = new QPushButton(tr("✕ Close View"), this);
    close_btn_->setToolTip(tr("Close 3D View and return to Plots (Esc)"));
    close_btn_->setCursor(Qt::PointingHandCursor);
    close_btn_->setStyleSheet(
      "QPushButton { background: #3b2020; color: #ff8080; border: 1px solid #772b2b; border-radius: 4px; padding: 4px 12px; font-weight: bold; } "
      "QPushButton:hover { background: #5b2828; color: #ffffff; border-color: #aa3b3b; } "
      "QPushButton:pressed { background: #772b2b; }");
    toolbar->addWidget(close_btn_);

    main_layout->addLayout(toolbar);

    auto* esc_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(esc_shortcut, &QShortcut::activated, close_btn_, &QPushButton::click);

    canvas_ = new M2RobotViewCanvas(this);
    main_layout->addWidget(canvas_, 1);

    connect(imu_chk_, &QCheckBox::toggled, canvas_, [this](bool checked) {
      canvas_->setUseImu(checked);
    });

    connect(reset_btn_, &QPushButton::clicked, canvas_, [this]() {
      canvas_->resetCamera();
    });
  }

  QPushButton* closeButton() const { return close_btn_; }

  void setPlotDataMap(PJ::PlotDataMapRef* plot_data)
  {
    if (canvas_) canvas_->setPlotDataMap(plot_data);
  }

  void updatePoseFromPlotData()
  {
    if (!canvas_) return;
    canvas_->updatePoseFromPlotData();

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
      status_label_->setText(tr("IMU: Inactive (waiting for data)"));
      status_label_->setStyleSheet("color: #8899a6;");
    }
  }

private:
  M2RobotViewCanvas* canvas_ = nullptr;
  QCheckBox* imu_chk_ = nullptr;
  QPushButton* reset_btn_ = nullptr;
  QPushButton* close_btn_ = nullptr;
  QLabel* status_label_ = nullptr;
};

M2RobotViewToolbox::M2RobotViewToolbox() = default;
M2RobotViewToolbox::~M2RobotViewToolbox() = default;

void M2RobotViewToolbox::init(PJ::PlotDataMapRef& src_data, PJ::TransformsMap& /*transform_map*/)
{
  if (!widget_)
  {
    widget_ = new M2RobotViewWidget();
    connect(widget_->closeButton(), &QPushButton::clicked, this, [this]() {
      Q_EMIT closed();
    });
  }
  widget_->setPlotDataMap(&src_data);

  // Poll timer for timeline playback and scrubbing updates
  auto* timer = new QTimer(widget_);
  connect(timer, &QTimer::timeout, widget_, [this]() {
    if (widget_ && widget_->isVisible())
    {
      widget_->updatePoseFromPlotData();
    }
  });
  timer->start(30); // ~33 Hz refresh
}

std::pair<QWidget*, PJ::ToolboxPlugin::WidgetType> M2RobotViewToolbox::providedWidget() const
{
  return {widget_, PJ::ToolboxPlugin::FIXED};
}

bool M2RobotViewToolbox::onShowWidget()
{
  if (widget_)
  {
    widget_->show();
    widget_->raise();
    return true;
  }
  return false;
}

} // namespace plotjuggler_m2
