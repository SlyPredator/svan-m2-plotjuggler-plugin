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

  Vec3 transformPoint(const Vec3& p) const
  {
    return {
      m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
      m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
      m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]
    };
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
  Vec3 bounds_min;
  Vec3 bounds_max;
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
    float norm[3], v1[3], v2[3], v3[3];
    std::memcpy(norm, ptr, 12); ptr += 12;
    std::memcpy(v1, ptr, 12); ptr += 12;
    std::memcpy(v2, ptr, 12); ptr += 12;
    std::memcpy(v3, ptr, 12); ptr += 12;
    ptr += 2; // attribute byte count

    Triangle tri;
    tri.a = {v1[0], v1[1], v1[2]};
    tri.b = {v2[0], v2[1], v2[2]};
    tri.c = {v3[0], v3[1], v3[2]};

    Vec3 computed_norm = Vec3::cross(tri.b - tri.a, tri.c - tri.a).normalized();
    tri.normal = (norm[0] != 0.0f || norm[1] != 0.0f || norm[2] != 0.0f) ?
                 Vec3{norm[0], norm[1], norm[2]}.normalized() : computed_norm;

    mesh->triangles.push_back(tri);
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
  QString mesh_filename;
  StlMesh mesh;
  QColor color = QColor(140, 145, 150); // Metallic gray default
};

} // namespace

class M2RobotViewWidget : public QOpenGLWidget, protected QOpenGLFunctions_2_0
{
public:
  explicit M2RobotViewWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent)
  {
    setFocusPolicy(Qt::StrongFocus);
    findAndLoadRobotAssets();
  }

  void setPlotDataMap(PJ::PlotDataMapRef* plot_data)
  {
    plot_data_ = plot_data;
  }

  void updatePoseFromPlotData()
  {
    if (!plot_data_) return;

    // Read current time and joint positions
    // Check various common topic names
    std::array<double, 12> q_vals = {};
    bool has_q = false;

    for (int i = 0; i < 12; ++i)
    {
      const QString idx = QString("%1").arg(i, 2, 10, QLatin1Char('0'));
      const QStringList candidates = {
        QString("sensor_data/joint/%1/q").arg(idx),
        QString("/m2_metal/hw/sensor_data/joint/%1/q").arg(idx),
        QString("rt/m2_metal/hw/sensor_data/joint/%1/q").arg(idx),
        QString("sensor_data/q/%1").arg(idx),
        QString("joints*/q/%1").arg(idx)
      };

      for (const auto& curve_name : candidates)
      {
        auto it = plot_data_->numeric.find(curve_name.toStdString());
        if (it != plot_data_->numeric.end() && it->second.size() > 0)
        {
          // Get the latest value
          q_vals[i] = it->second.back().y;
          has_q = true;
          break;
        }
      }
    }

    if (has_q)
    {
      current_q_ = q_vals;
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
    GLdouble m[16] = {};
    m[0] = f / aspect;
    m[5] = f;
    m[10] = (zFar + zNear) / (zNear - zFar);
    m[11] = -1.0;
    m[14] = (2.0 * zFar * zNear) / (zNear - zFar);
    glMultMatrixd(m);
  }

  void gluLookAtCustom(double eyex, double eyey, double eyez,
                       double centerx, double centery, double centerz,
                       double upx, double upy, double upz)
  {
    Vec3 forward = Vec3{centerx - eyex, centery - eyey, centerz - eyez}.normalized();
    Vec3 up = Vec3{upx, upy, upz}.normalized();
    Vec3 side = Vec3::cross(forward, up).normalized();
    up = Vec3::cross(side, forward);

    GLdouble m[16] = {};
    m[0] = side.x;     m[4] = side.y;     m[8]  = side.z;
    m[1] = up.x;       m[5] = up.y;       m[9]  = up.z;
    m[2] = -forward.x; m[6] = -forward.y; m[10] = -forward.z;
    m[15] = 1.0;
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
    link_transforms.insert(root_link_, Mat4::identity());

    applyFkRecursive(root_link_, Mat4::identity(), &link_transforms);

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
        QString filename = mesh_elem.attribute("filename");
        filename = QFileInfo(filename).fileName(); // e.g. base_link.STL
        const QString mesh_full_path = QDir(meshes_dir).filePath(filename);

        QFile mesh_file(mesh_full_path);
        if (mesh_file.open(QIODevice::ReadOnly))
        {
          loadBinaryStl(mesh_file.readAll(), &link.mesh);
        }
      }

      if (link.name.contains("foot"))
      {
        link.color = QColor(40, 42, 45); // Dark rubber
      }
      else if (link.name == "base_link")
      {
        link.color = QColor(160, 165, 170); // Metal body
      }
      else
      {
        link.color = QColor(110, 115, 125); // Leg links
      }

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
        const QString xyz_str = origin_elem.attribute("xyz");
        const QString rpy_str = origin_elem.attribute("rpy");

        auto parseVec = [](const QString& s) -> Vec3 {
          const auto parts = s.split(' ', Qt::SkipEmptyParts);
          if (parts.size() == 3)
          {
            return {parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()};
          }
          return {};
        };

        joint.origin_xyz = parseVec(xyz_str);
        joint.origin_rpy = parseVec(rpy_str);

        joint.local_origin = Mat4::translation(joint.origin_xyz) *
                             Mat4::fromRpy(joint.origin_rpy.x, joint.origin_rpy.y, joint.origin_rpy.z);
      }

      const QDomElement axis_elem = elem.firstChildElement("axis");
      if (!axis_elem.isNull())
      {
        const auto parts = axis_elem.attribute("xyz").split(' ', Qt::SkipEmptyParts);
        if (parts.size() == 3)
        {
          joint.axis = Vec3{parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()}.normalized();
        }
      }
      else
      {
        joint.axis = {0, 0, 1};
      }

      joint.motor_index = motor_indices.value(joint.name, -1);

      children_by_parent_[joint.parent_link].push_back(joints_.size());
      joints_.push_back(joint);
    }
  }
};

M2RobotViewToolbox::M2RobotViewToolbox() = default;
M2RobotViewToolbox::~M2RobotViewToolbox() = default;

void M2RobotViewToolbox::init(PJ::PlotDataMapRef& src_data, PJ::TransformsMap& /*transform_map*/)
{
  if (!widget_)
  {
    widget_ = new M2RobotViewWidget();
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
