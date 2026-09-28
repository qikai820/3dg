#undef QT_USE_QSTRINGBUILDER
#include "MissionController.h"
#include "QuadrotorModel.h"
#include "mainwindow.h"
#include "ccDBRoot.h"
#include <FileIOFilter.h>
#include <QAbstractVideoSurface>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMediaPlayer>
#include <QQuaternion>
#include <QStandardPaths>
#include <QVideoFrame>
#include <QVideoSurfaceFormat>
#include <QtWidgets>
#include <ccGLUtils.h>
#include <ccGLWindowInterface.h>
#include <ccGLWindowSignalEmitter.h>
#include <ccPickingHub.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <cmath>
#include <tuple>
#include <google/protobuf/util/json_util.h>

// Paint decoded frames as ordinary widgets. QVideoWidget's native video window
// promotes sibling cards to native surfaces and breaks alpha overlays on GL.
class VideoSurface : public QAbstractVideoSurface {
public:
  explicit VideoSurface(QWidget *target)
      : QAbstractVideoSurface(target), target_(target) {}
  QList<QVideoFrame::PixelFormat>
  supportedPixelFormats(QAbstractVideoBuffer::HandleType type) const override {
    if (type != QAbstractVideoBuffer::NoHandle)
      return {};
    return {QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
            QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
            QVideoFrame::Format_RGB565};
  }
  bool present(const QVideoFrame &input) override {
    QVideoFrame frame(input);
    if (!frame.map(QAbstractVideoBuffer::ReadOnly))
      return false;
    auto format = QVideoFrame::imageFormatFromPixelFormat(frame.pixelFormat());
    if (format == QImage::Format_Invalid) {
      frame.unmap();
      return false;
    }
    image = QImage(frame.bits(), frame.width(), frame.height(),
                   frame.bytesPerLine(), format)
                .copy();
    if (surfaceFormat().scanLineDirection() == QVideoSurfaceFormat::BottomToTop)
      image = image.mirrored();
    frame.unmap();
    target_->update();
    return true;
  }
  void stop() override {
    image = QImage();
    target_->update();
    QAbstractVideoSurface::stop();
  }
  QImage image;

private:
  QWidget *target_;
};
class VideoCanvas : public QWidget {
public:
  explicit VideoCanvas(QWidget *p)
      : QWidget(p), surface(new VideoSurface(this)) {
    setMinimumSize(160, 90);
  }
  VideoSurface *surface;
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(8, 17, 22));
    if (!surface->image.isNull()) {
      QSize size =
          surface->image.size().scaled(this->size(), Qt::KeepAspectRatio);
      QRect area(
          QPoint((width() - size.width()) / 2, (height() - size.height()) / 2),
          size);
      painter.drawImage(area, surface->image);
    } else {
      painter.setPen(QColor(220, 231, 236));
      painter.drawText(rect().adjusted(12, 12, -12, -12),
                       Qt::AlignCenter | Qt::TextWordWrap,
                       QStringLiteral("暂无视频画面\n请在 3DG 配置中设置视频地址，或启动机载视频"));
    }
  }
};
class AttitudeWidget : public QWidget {
public:
  double roll = 0, pitch = 0;
  bool valid = false;
  explicit AttitudeWidget(QWidget *p) : QWidget(p) { setFixedSize(76, 76); }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.translate(38, 38);
    QPainterPath clip;
    clip.addEllipse(QPointF(0, 0), 34, 34);
    p.setClipPath(clip);
    p.save();
    p.rotate(-roll);
    p.translate(0, pitch * 0.65);
    p.fillRect(-100, -140, 200, 140, QColor(53, 102, 132));
    p.fillRect(-100, 0, 200, 140, QColor(112, 83, 60));
    p.setPen(QPen(Qt::white, 1));
    p.drawLine(-80, 0, 80, 0);
    for (int y : {-20, -10, 10, 20})
      p.drawLine(-12, y, 12, y);
    p.restore();
    p.setPen(QPen(QColor(255, 213, 100), 3));
    p.drawLine(-22, 0, -6, 0);
    p.drawLine(6, 0, 22, 0);
    p.drawLine(-6, 0, 0, 5);
    p.drawLine(0, 5, 6, 0);
    if (!valid) {
      p.fillRect(-38, -38, 76, 76, QColor(10, 20, 30, 180));
      p.setPen(Qt::white);
      p.drawText(QRect(-35, -15, 70, 30), Qt::AlignCenter, "无姿态");
    }
  }
};
// A screen-space handle for moving a selected waypoint. The arrows are
// projected from the actual world axes, so dragging one changes only that
// coordinate regardless of the current camera orientation.
class WaypointGizmo : public QWidget {
public:
  WaypointGizmo(QWidget *parent, ccGLWindowInterface *gl,
                std::function<void(int, double)> moved,
                std::function<void()> finished)
      : QWidget(parent), gl_(gl), moved_(std::move(moved)),
        finished_(std::move(finished)) {
    setFixedSize(kSize, kSize);
    setMouseTracking(true);
    setAttribute(Qt::WA_TranslucentBackground);
    hide();
  }

  bool isDragging() const { return activeAxis_ >= 0; }

  void sync(const QVector3D &point) {
    if (!gl_ || !parentWidget()) {
      hide();
      return;
    }
    ccGLCameraParameters camera;
    gl_->getGLCameraParameters(camera);
    CCVector3d center3d;
    bool centerVisible = false;
    if (!camera.project(CCVector3d(point.x(), point.y(), point.z()), center3d,
                        &centerVisible) ||
        !centerVisible) {
      hide();
      return;
    }
    const QPointF center(center3d.x, camera.viewport[3] - center3d.y);
    const double worldLength = qBound(0.25, camera.pixelSize * 72.0, 30.0);
    const QVector<QVector3D> axes{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int axis = 0; axis < 3; ++axis) {
      const QVector3D tip = point + axes[axis] * worldLength;
      CCVector3d tip3d;
      bool tipVisible = false;
      if (!camera.project(CCVector3d(tip.x(), tip.y(), tip.z()), tip3d,
                          &tipVisible) ||
          !tipVisible) {
        ends_[axis] = QPointF(kHalf, kHalf);
        enabled_[axis] = false;
        continue;
      }
      ends_[axis] = QPointF(tip3d.x, camera.viewport[3] - tip3d.y) - center +
                    QPointF(kHalf, kHalf);
      enabled_[axis] = QLineF(QPointF(kHalf, kHalf), ends_[axis]).length() > 14;
      scales_[axis] = worldLength;
    }
    move(qRound(center.x()) - kHalf, qRound(center.y()) - kHalf);
    show();
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const QPointF origin(kHalf, kHalf);
    const QColor colors[] = {QColor(239, 83, 80), QColor(70, 180, 105),
                             QColor(66, 142, 225)};
    const QString labels[] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
      if (!enabled_[axis])
        continue;
      QColor color = colors[axis];
      if (axis == activeAxis_)
        color = color.lighter(135);
      painter.setPen(QPen(color, axis == activeAxis_ ? 4.5 : 3.0,
                          Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      painter.drawLine(origin, ends_[axis]);
      const QLineF shaft(origin, ends_[axis]);
      const double angle = std::atan2(-shaft.dy(), shaft.dx());
      QPolygonF arrow;
      arrow << shaft.p2()
            << shaft.p2() - QPointF(std::cos(angle - 0.48) * 12,
                                    -std::sin(angle - 0.48) * 12)
            << shaft.p2() - QPointF(std::cos(angle + 0.48) * 12,
                                    -std::sin(angle + 0.48) * 12);
      painter.setBrush(color);
      painter.setPen(Qt::NoPen);
      painter.drawPolygon(arrow);
      painter.setPen(QPen(color, 1));
      painter.drawText(QRectF(shaft.p2() + QPointF(5, -15), QSizeF(24, 20)),
                       Qt::AlignCenter, labels[axis]);
    }
    painter.setPen(QPen(QColor(247, 250, 252), 2));
    painter.setBrush(QColor(36, 52, 64, 220));
    painter.drawEllipse(origin, 5, 5);
  }

  void mousePressEvent(QMouseEvent *event) override {
    if (event->button() != Qt::LeftButton)
      return;
    activeAxis_ = hitAxis(event->pos());
    if (activeAxis_ < 0) {
      event->ignore();
      return;
    }
    lastMouse_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    update();
    event->accept();
  }

  void mouseMoveEvent(QMouseEvent *event) override {
    if (activeAxis_ < 0) {
      const int hit = hitAxis(event->pos());
      setCursor(hit >= 0 ? Qt::OpenHandCursor : Qt::ArrowCursor);
      event->ignore();
      return;
    }
    const QPointF direction = ends_[activeAxis_] - QPointF(kHalf, kHalf);
    const QPointF motion = event->pos() - lastMouse_;
    const double length2 = QPointF::dotProduct(direction, direction);
    if (length2 > 0) {
      const double distance = QPointF::dotProduct(motion, direction) /
                              length2 * scales_[activeAxis_];
      if (std::abs(distance) > 1e-8)
        moved_(activeAxis_, distance);
    }
    lastMouse_ = event->pos();
    event->accept();
  }

  void mouseReleaseEvent(QMouseEvent *event) override {
    if (event->button() != Qt::LeftButton || activeAxis_ < 0)
      return;
    activeAxis_ = -1;
    unsetCursor();
    update();
    finished_();
    event->accept();
  }

private:
  int hitAxis(const QPointF &point) const {
    const QPointF origin(kHalf, kHalf);
    int best = -1;
    double bestDistance = 12.0;
    for (int axis = 0; axis < 3; ++axis) {
      if (!enabled_[axis])
        continue;
      const QPointF direction = ends_[axis] - origin;
      const double length2 = QPointF::dotProduct(direction, direction);
      const double along = QPointF::dotProduct(point - origin, direction) / length2;
      if (along < 0.2 || along > 1.3)
        continue;
      const QPointF closest = origin + direction * along;
      const double distance = QLineF(point, closest).length();
      if (distance < bestDistance) {
        best = axis;
        bestDistance = distance;
      }
    }
    return best;
  }

  static constexpr int kSize = 180;
  static constexpr int kHalf = kSize / 2;
  ccGLWindowInterface *gl_ = nullptr;
  std::function<void(int, double)> moved_;
  std::function<void()> finished_;
  QPointF ends_[3];
  double scales_[3]{};
  bool enabled_[3]{};
  int activeAxis_ = -1;
  QPointF lastMouse_;
};
namespace {
QPushButton *button(QBoxLayout *l, const QString &t, std::function<void()> f) {
  auto *b = new QPushButton(t);
  l->addWidget(b);
  QObject::connect(b, &QPushButton::clicked, b, std::move(f));
  return b;
}
QFrame *card(QWidget *p, const QString &title) {
  auto *c = new QFrame(p);
  c->setObjectName("missionCard");
  auto *l = new QVBoxLayout(c);
  l->setContentsMargins(14, 12, 14, 12);
  l->setSpacing(9);
  if (!title.isEmpty()) {
    auto *h = new QLabel(title);
    h->setObjectName("cardTitle");
    l->addWidget(h);
  }
  return c;
}
QVBoxLayout *box(QWidget *w) {
  return qobject_cast<QVBoxLayout *>(w->layout());
}
QVector3D vec(const mission::Vec3 &p) { return QVector3D(p.x(), p.y(), p.z()); }
void setvec(mission::Vec3 *p, const QVector3D &v) {
  p->set_x(v.x());
  p->set_y(v.y());
  p->set_z(v.z());
}
QString safe(const std::string &s) {
  return QString::fromStdString(s).toHtmlEscaped();
}
} // namespace
void MissionController::updateVehicle(const mission::VehicleState &s) {
  lastVehicle_ = s;
  vehicleAge_.restart();
  attitude_->valid = s.pose_valid();
  QString position = "—", angles = "—", speed = "—";
  if (s.pose_valid()) {
    auto p = vec(s.position());
    if (history_.isEmpty() || (history_.last() - p).length() > .05f) {
      history_ << p;
      trailDirty_ = dirty_ = true;
    }
    if (history_.size() > 20000)
      history_.remove(0, 1000);
    auto &q = s.orientation();
    double roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                             1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    double pitch =
        std::asin(qBound(-1., 2 * (q.w() * q.y() - q.z() * q.x()), 1.));
    double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                            1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    attitude_->roll = roll * 180 / M_PI;
    attitude_->pitch = pitch * 180 / M_PI;
    angles = QString("R%1° P%2° Y%3°")
                 .arg(roll * 180 / M_PI, 0, 'f', 0)
                 .arg(pitch * 180 / M_PI, 0, 'f', 0)
                 .arg(yaw * 180 / M_PI, 0, 'f', 0);
    position = QString("%1 / %2 / %3 m")
                   .arg(p.x(), 0, 'f', 1)
                   .arg(p.y(), 0, 'f', 1)
                   .arg(p.z(), 0, 'f', 1);
  }
  if (s.velocity_valid())
    speed = QString::number(vec(s.velocity()).length(), 'f', 1) + " m/s";
  hud_->setText(
      QString("%1\n速度 %2\nXYZ %3\n电量 %4")
          .arg(angles, speed, position,
               s.battery_valid()
                   ? QString::number(s.battery_percent(), 'f', 0) + "%"
                   : "—"));
  batteryBar_->setValue(
      s.battery_valid() ? qBound(0, int(s.battery_percent()), 100) : 0);
  attitude_->update();
  // Pose is latency-sensitive and cheap: do not wait for the 10 Hz cloud pass.
  updateAircraftPose();
  if (gl_)
    gl_->redraw(); // Qt coalesces paint requests; geometry remains cached.
}
MissionController::MissionController(MainWindow *w)
    : QObject(w), window_(w), gl_(w->getActiveGLWindow()), client_(this) {
  auto *sub = w->getMDISubWindow(gl_);
  if (sub) {
    sub->installEventFilter(this);
    sub->setWindowFlags(Qt::SubWindow | Qt::FramelessWindowHint);
    sub->showMaximized();
  }
  canvas_ = sub ? sub->widget() : w->centralWidget();
  root_ = new ccHObject("3DG scene");
  gl_->addToOwnDB(root_);
  auto group = [this](const QString &n) {
    auto *g = new ccHObject(n);
    root_->addChild(g);
    return g;
  };
  live_ = group("实时点云");
  map_ = group("参考地图");
  route_ = group("本地任务航线");
  remoteRoute_ = group("任务机航点（只读）");
  ego_ = group("EGO 规划");
  grid_ = group("EGO 栅格地图");
  trail_ = group("实际轨迹");
  aircraft_ = group("无人机");
  aircraftModel_ = createQuadrotorModel();
  if (aircraftModel_) {
    aircraftModel_->setDisplay(gl_);
    aircraftModel_->setVisible(false);
    aircraft_->addChild(aircraftModel_);
  }
  mission_.set_mission_id(
      QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString());
  QSettings s("3DG", "Mission");
  baseUrl_ = s.value("url", baseUrl_).toString();
  videoUrl_ = s.value("video").toString();
  frame_ = s.value("frame", frame_).toString();
  mapId_ = s.value("map", mapId_).toString();
  voxel_ = s.value("voxel", voxel_).toDouble();
  cloudHz_ = s.value("hz", cloudHz_).toDouble();
  clearance_ = s.value("clearance", clearance_).toDouble();
  const QColor savedGridColor(s.value("gridColor", gridColor_.name()).toString());
  if (savedGridColor.isValid())
    gridColor_ = savedGridColor;
  const double savedGridOpacity = s.value("gridOpacity", gridOpacity_).toDouble();
  if (std::isfinite(savedGridOpacity) && savedGridOpacity >= 0.0 &&
      savedGridOpacity <= 1.0)
    gridOpacity_ = savedGridOpacity;
  if (!std::isfinite(voxel_) || voxel_ < .02 || voxel_ > 5)
    voxel_ = .15;
  const auto logs =
      QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) +
      "/3dg/logs";
  QDir().mkpath(logs);
  logFile_.setFileName(
      logs + "/" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss") +
      ".log");
  logFile_.open(QIODevice::WriteOnly | QIODevice::Append);
  buildUi();
  restoreAccumulatedMap();
  // The standard CloudCompare tree only owns its main DB. Mission overlays live
  // in the GL window's own DB and some are replaced on every telemetry frame.
  // Sending their pointers to the hidden tree creates persistent QModelIndex
  // entries for soon-to-be-deleted objects; a later click/drag dereferences
  // those stale entries in ccHObject::getIndex(). Mission point picking uses
  // ccPickingHub and remains connected independently.
  const bool singleSelectionDetached =
      QObject::disconnect(gl_->signalEmitter(),
                          &ccGLWindowSignalEmitter::entitySelectionChanged,
                          window_, nullptr);
  const bool multiSelectionDetached =
      QObject::disconnect(gl_->signalEmitter(),
                          &ccGLWindowSignalEmitter::entitiesSelectionChanged,
                          window_, nullptr);
  if (!singleSelectionDetached || !multiSelectionDetached)
    log("无法隔离旧对象树的选择连接；请不要点击实时对象");
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_SIZE")) {
    const auto parts = qEnvironmentVariable("THREEDG_SMOKE_SIZE").split('x');
    if (parts.size() == 2 && parts[0].toInt() >= 850 && parts[1].toInt() >= 650) {
      const QSize smokeSize(parts[0].toInt(), parts[1].toInt());
      QTimer::singleShot(0, window_, [this, smokeSize] {
        window_->showNormal();
        window_->resize(smokeSize);
      });
    }
  }
  canvas_->installEventFilter(this);
  connect(&client_, &ProtocolClient::message, this,
          &MissionController::receive);
  connect(&client_, &ProtocolClient::diagnostic, this,
          [this](const QString &t) { log(t, "协议"); });
  connect(&client_, &ProtocolClient::sessionChanged, this,
          &MissionController::resetSession);
  connect(&client_, &ProtocolClient::stateChanged, this,
          [this](const QString &t, bool r) {
            ready_ = r;
            if (r)
              requestRemoteRoute();
            else
              clearRemoteRoute();
            mode_->setText(demo_ ? "● 模拟演示 · 非真实遥测"
                                 : (r ? "● " + t : "○ " + t));
            connectButton_->setText(r ? "断开" : "连接");
            if (!r) {
              statusAge_.invalidate();
              pendingKinds_.clear();
              mediaOperations_.clear();
              mediaErrors_.clear();
              refreshStartupButtons();
              lastVehicle_.set_pose_valid(false);
              if (aircraftModel_)
                aircraftModel_->setVisible(false);
              ego_->setVisible(false);
              expireGrid();
              dirty_ = true;
              vehicleAge_.invalidate();
              hud_->setText("姿态 / 速度 / 位置 / 电量：—");
              batteryBar_->setValue(0);
              attitude_->valid = false;
              attitude_->update();
              status_->setText(
                  "任务机网络　　离线\nOdin1　　　　　 —\nEGO- Planner　　 "
                  "—\n定位状态　　　　—\n视频 / 记录　　　—");
              player_->stop();
              video_->hide();
              if (download_) {
                download_.reset();
                log("连接中断，未完成文件已取消");
              }
            }
            log(t, "连接");
            protocolHint_->setText(demo_ ? "Protobuf v1  ·  模拟数据"
                                         : (r ? "Protobuf v1  ·  任务机在线"
                                              : "Protobuf v1  ·  未连接"));
          });
  connect(&demoTimer_, &QTimer::timeout, this, &MissionController::demoTick);
  demoTimer_.setTimerType(Qt::PreciseTimer);
  demoTimer_.setInterval(20);
  connect(&redraw_, &QTimer::timeout, this, &MissionController::updateScene);
  redraw_.start(100);
  connect(&autosaveTimer_, &QTimer::timeout, this, [this] {
    if (accumulationDirty_)
      saveAccumulatedMap();
  });
  autosaveTimer_.start(30000);
  connect(&staleTimer_, &QTimer::timeout, this, [this] {
    if (!gl_)
      return;
    refreshStartupButtons();
    if (!demo_ && statusAge_.isValid() && statusAge_.elapsed() > 3000)
      status_->setText(
          "任务机网络　　状态已过期\nOdin1　　　　　 "
          "—\nEGO-Planner　　—\n定位状态　　　　—\n视频 / 记录　　　—");
    if (download_ && downloadAge_.isValid() && downloadAge_.elapsed() > 30000) {
      download_.reset();
      log("文件下载超时，未完成文件已取消");
    }
    if (!demo_ && vehicleAge_.isValid() && vehicleAge_.elapsed() > 3000) {
      hud_->setText("遥测已过期（超过 3 秒）\n姿态 / 速度 / 位置 / 电量：—");
      attitude_->valid = false;
      attitude_->update();
      if (aircraftModel_)
        aircraftModel_->setVisible(false);
      lastVehicle_.set_pose_valid(false);
      gl_->redraw();
    }
    if (!demo_ && plannerAge_.isValid() && plannerAge_.elapsed() > 3000) {
      ego_->setVisible(false);
      gl_->redraw();
    }
    if (!demo_ && gridAge_.isValid() && gridAge_.elapsed() > 3000)
      expireGrid();
  });
  staleTimer_.start(1000);
  connect(gl_->signalEmitter(), &ccGLWindowSignalEmitter::aboutToClose, this,
          [this](ccGLWindowInterface *) {
            gl_ = nullptr;
            root_ = nullptr;
            aircraftModel_ = nullptr;
            demoTimer_.stop();
            redraw_.stop();
          });
  arrange();
  if (!mapPoints_.isEmpty()) {
    updateScene();
    gl_->zoomGlobal();
  }
  log("3DG 已就绪。可加载地图、编辑航线，或开启模拟演示。");
  if (!aircraftModel_)
    log("X500 模型资源加载失败，无法显示无人机模型");
  if (qEnvironmentVariableIsSet("THREEDG_DEMO"))
    setDemo(true);
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_VIDEO")) {
    videoUrl_ = qEnvironmentVariable("THREEDG_SMOKE_VIDEO");
    showVideo();
  } else if (!videoUrl_.isEmpty())
    showVideo();
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_DIR"))
    QTimer::singleShot(4500, this, &MissionController::smoke);
}
MissionController::~MissionController() {
  if (accumulationDirty_)
    saveAccumulatedMap();
  client_.stop();
  demoTimer_.stop();
  redraw_.stop();
  if (window_->pickingHub())
    window_->pickingHub()->removeListener(this);
  if (gl_ && root_) {
    gl_->removeFromOwnDB(root_);
    delete root_;
  }
  delete left_;
  delete mapPopover_;
  delete right_;
  delete top_;
  delete editor_;
  delete routeDialog_;
  delete mapBadge_;
  delete video_;
  delete logPopup_;
}
void MissionController::buildUi() {
  window_->setWindowTitle("3DG Mission · Odin1 / EGO 地面站");
  window_->resize(1360, 860);
  QFont uiFont("Noto Sans CJK SC", 10);
  window_->setFont(uiFont);
  for (auto *action : window_->findChildren<QAction *>())
    action->setEnabled(false);
  for (auto *d : window_->findChildren<QDockWidget *>()) {
    d->installEventFilter(this);
    d->hide();
  }
  for (auto *t : window_->findChildren<QToolBar *>())
    t->hide();
  const QString css =
      "QMainWindow{background:#173341;}"
      "QFrame#missionCard,QFrame#missionToolbar{background:rgba(242,248,251,"
      "216);"
      "border:1px solid rgba(255,255,255,220);border-radius:12px;}"
      "QFrame#missionToolbar{background:rgba(235,244,249,220);border-radius:"
      "10px;}"
      "QFrame#videoOverlay{background:#081116;border:0;border-radius:0;}"
      "QLabel{color:#304756;background:transparent;}"
      "QLabel#cardTitle{font-size:13px;font-weight:650;color:#263d4c;}"
      "QLabel#sectionTitle{font-size:11px;font-weight:650;color:#657d8d;}"
      "QLabel#mutedText{color:#657d8d;font-size:11px;}"
      "QLabel#mapBadge{background:rgba(240,247,250,238);border:1px solid white;"
      "border-radius:9px;padding:8px;color:#304756;font-size:11px;}"
      "QPushButton,QToolButton{color:#304959;background:rgba(255,255,255,210);"
      "border:1px solid #c8d9e2;border-radius:7px;padding:5px "
      "9px;min-height:21px;}"
      "QPushButton:hover,QToolButton:hover{background:#e0f7f2;border-color:#"
      "70ccb8;}"
      "QPushButton:pressed,QToolButton:pressed{background:#c9eee6;}"
      "QPushButton[serviceRunning=true]{background:#218653;color:white;"
      "border-color:#197044;}"
      "QPushButton[serviceRunning=true]:hover{background:#197044;}"
      "QPushButton[serviceRunning=true]:pressed{background:#125735;}"
      "QPushButton:disabled{background:#e4ebef;color:#7b8d98;border-color:#d0dce3;}"
      "QPushButton:checked,QToolButton:checked{background:#c9f2e9;border-color:"
      "#52bfa9;"
      "color:#147a68;}"
      "QPushButton[role=toolbar]{background:transparent;border:0;padding:5px "
      "7px;}"
      "QPushButton[role=toolbar]:hover{background:rgba(199,231,237,190);border-"
      "radius:6px;}"
      "QPushButton[role=toolbar]:checked{background:#d0f1e9;color:#147b68;}"
      "QToolButton#layerTool{font-size:20px;font-weight:500;padding:3px;"
      "min-width:34px;min-height:34px;}"
      "QToolButton#mapDetailsTool{font-size:18px;padding:3px;"
      "min-width:34px;min-height:34px;}"
      "QPushButton#routeChip{font-size:11px;min-width:31px;min-height:22px;"
      "padding:2px 5px;}"
      "QLabel#routeConnector{color:#96b5c0;}"
      "QPushButton#dangerButton{background:#fff4ee;color:#a75d45;border-color:#"
      "efc9b7;}"
      "QPushButton#latestLog{background:rgba(255,255,255,226);border:1px solid "
      "#deebf0;"
      "border-radius:7px;color:#2b4654;font-size:11px;padding:"
      "1px 10px;}"
      "QCheckBox{color:#2d4655;spacing:7px;padding:5px 3px;}"
      "QCheckBox::indicator{width:11px;height:11px;border:1px solid #90a8b7;"
      "border-radius:6px;background:#f5fafc;}"
      "QCheckBox::indicator:checked{background:#58cdb5;border-color:#58cdb5;}"
      "QCheckBox#layerMap::indicator:checked,QCheckBox#layerTrail::indicator:"
      "checked"
      "{background:#69aff0;border-color:#69aff0;}"
      "QCheckBox#layerRoute::indicator:checked{background:#f9ae5a;border-color:"
      "#f9ae5a;}"
      "QTableWidget,QListWidget,QLineEdit,QDoubleSpinBox,QComboBox{color:#"
      "304756;"
      "background:rgba(255,255,255,225);border:1px solid #ccdae3;"
      "border-radius:6px;selection-background-color:#d4f3ec;"
      "selection-color:#204356;}"
      "QTableWidget{gridline-color:#e0eaf0;}"
      "QHeaderView::section{background:#e6eff4;color:#586f7d;border:0;padding:"
      "4px;}"
      "QMenuBar{background:rgba(235,244,249,248);color:#344d5c;"
      "}"
      "QMenuBar::item{padding:3px 8px;margin:1px 1px;border-radius:5px;}"
      "QMenuBar::item:selected{background:#d7e9ee;}"
      "QMenu{background:#f2f8fa;color:#304756;border:1px solid #c8d9e2;}"
      "QMenu::item:selected{background:#d5f1ea;}"
      "QStatusBar{background:rgba(236,246,250,246);color:#5a7180;"
      "border-top:1px solid #d6e5ec;min-height:23px;}"
      "QStatusBar::item{border:0;}"
      "QDialog{background:#edf5f9;color:#304756;}"
      "QProgressBar{background:#d7e6eb;border:0;border-radius:3px;min-height:"
      "6px;"
      "max-height:6px;text-align:center;}"
      "QProgressBar::chunk{background:#54cbb3;border-radius:3px;}";
  window_->setStyleSheet(css);
  auto *bar = window_->menuBar();
  bar->setFixedHeight(34);
  for (auto *a : bar->actions())
    a->setVisible(false);
  auto *missionMenu = bar->addMenu("3DG Mission ▾");
  missionMenu->setObjectName("missionWorkspaceMenu");
  auto *workspaceGroup = new QActionGroup(missionMenu);
  workspaceGroup->setExclusive(true);
  const QVector<QPair<QString, Workspace>> workspaces = {
      {"任务监控", Workspace::Monitor}, {"点云处理", Workspace::Processing},
      {"对比分析", Workspace::Comparison}};
  for (const auto &entry : workspaces) {
    auto *action = missionMenu->addAction(entry.first);
    action->setCheckable(true);
    action->setChecked(entry.second == Workspace::Monitor);
    action->setActionGroup(workspaceGroup);
    workspaceActions_[static_cast<int>(entry.second)] = action;
    connect(action, &QAction::triggered, this,
            [this, workspace = entry.second] { setWorkspace(workspace); });
  }
  auto *file = bar->addMenu("文件");
  file->addAction("加载 PCD / PLY 地图", this, &MissionController::loadMap);
  file->addAction("导入处理对象…", this, &MissionController::importProcessingFiles);
  file->addAction("创建当前地图快照", this, &MissionController::snapshotMap);
  file->addAction("保存点云 PCD", this, &MissionController::saveMap);
  file->addAction("打开航线任务", this, &MissionController::openMission);
  file->addAction("保存航线任务", this, &MissionController::saveMission);
  file->addAction("下载任务机文件", this, &MissionController::requestFile);
  auto addProcessingAction = [this](QMenu *menu, const QString &label,
                                    const char *actionName) {
    menu->addAction(label, this, [this, actionName] {
      runCloudCompareAction(actionName);
    });
  };
  auto *editMenu = bar->addMenu("编辑");
  addProcessingAction(editMenu, "复制选中对象…", "actionClone");
  addProcessingAction(editMenu, "抽稀…", "actionSubsample");
  addProcessingAction(editMenu, "分割…", "actionSegment");
  auto *processMenu = bar->addMenu("处理");
  addProcessingAction(processMenu, "计算法向量…", "actionComputeNormals");
  addProcessingAction(processMenu, "栅格化…", "actionRasterize");
  addProcessingAction(processMenu, "ICP 配准…", "actionRegister");
  addProcessingAction(processMenu, "对应点配准…", "actionPointPairsAlign");
  auto *analysisMenu = bar->addMenu("分析");
  addProcessingAction(analysisMenu, "点云到点云距离…", "actionCloudCloudDist");
  addProcessingAction(analysisMenu, "点云到网格距离…", "actionCloudMeshDist");
  auto *taskMenu = bar->addMenu("任务机");
  taskMenu->addAction("任务机 YAML", this, &MissionController::yamlSettings);
  taskMenu->addAction("日志记录", this, [this] { latest_->click(); });
  taskMenu->addAction("清空累积地图", this, [this] { clearAccumulatedMap(); });
  auto *startup = taskMenu->addMenu("任务启动");
  startup->addAction("启动 Odin", this,
                     [this] { command(mission::Command::START_ODIN); });
  startup->addAction("启动 EGO", this,
                     [this] { command(mission::Command::START_EGO); });
  auto *view = bar->addMenu("视图");
  view->addAction("等轴测", this, [this] { gl_->setView(CC_ISO_VIEW_1); });
  view->addAction("俯视", this, [this] { gl_->setView(CC_TOP_VIEW); });
  view->addAction("定位无人机", this, &MissionController::focusAircraft);
  view->addAction("悬浮视频窗口", this, &MissionController::showVideo);
  auto *demo = view->addAction("模拟演示（非真实遥测）");
  demo->setCheckable(true);
  connect(demo, &QAction::toggled, this, &MissionController::setDemo);
  auto *settingsMenu = bar->addMenu("设置");
  settingsMenu->addAction("3DG 配置", this, &MissionController::settings);
  latest_ = new QPushButton("● 3DG 已就绪 ▾", bar);
  latest_->setFixedSize(350, 26);
  latest_->setObjectName("latestLog");
  bar->installEventFilter(this);
  latest_->move(bar->width() - 360, (bar->height() - latest_->height()) / 2);
  latest_->show();
  latest_->raise();
  logPopup_ = card(window_, "运行日志");
  logPopup_->setWindowFlags(Qt::Popup);
  logPopup_->setStyleSheet(css);
  logPopup_->resize(640, 284);
  logList_ = new QListWidget;
  logList_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  logList_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  logList_->setFixedHeight(6 * 27 + 2);
  box(logPopup_)->addWidget(logList_);
  logCount_ = new QLabel("自动跟随最新日志");
  box(logPopup_)->addWidget(logCount_);
  auto *logActions = new QHBoxLayout;
  box(logPopup_)->addLayout(logActions);
  logRecordButton_ = button(logActions, "开始记录", [this] {
    toggleMedia(mission::Command::RECORD_START);
  });
  logRecordState_ = new QLabel;
  logRecordState_->setObjectName("mutedText");
  logActions->addWidget(logRecordState_);
  button(logActions, "回到最新", [this] {
    logList_->scrollToBottom();
    unread_ = 0;
    logCount_->setText("自动跟随最新日志");
  });
  connect(latest_, &QPushButton::clicked, this, [this] {
    QPoint pos = latest_->mapToGlobal(QPoint(0, latest_->height() + 4));
    const QRect screen = latest_->screen()->availableGeometry();
    pos.setX(qBound(screen.left() + 6, pos.x(),
                    screen.right() - logPopup_->width() - 6));
    logPopup_->move(pos);
    logList_->scrollToBottom();
    unread_ = 0;
    logPopup_->show();
  });
  top_ = card(canvas_, "");
  top_->setObjectName("missionToolbar");
  box(top_)->setContentsMargins(7, 4, 7, 4);
  auto *toolbarRow = new QHBoxLayout;
  toolbarRow->setContentsMargins(0, 0, 0, 0);
  toolbarRow->setSpacing(2);
  box(top_)->addLayout(toolbarRow);
  toolbarFold_ = button(toolbarRow, "◀", [this] {
    setToolbarCollapsed(!toolbarCollapsed_);
  });
  toolbarFold_->setProperty("role", "toolbar");
  toolbarFold_->setFixedWidth(30);
  toolbarFold_->setToolTip("向左收起工具栏");
  toolbarFold_->setAccessibleName("向左收起工具栏");
  toolbarContents_ = new QWidget(top_);
  toolbarRow->addWidget(toolbarContents_, 1);
  auto *tools = new QHBoxLayout(toolbarContents_);
  tools->setContentsMargins(0, 0, 0, 0);
  tools->setSpacing(2);
  auto addTool = [&](const QString &label, std::function<void()> action) {
    auto *b = button(tools, label, std::move(action));
    b->setProperty("role", "toolbar");
    return b;
  };
  accumulate_ = new QCheckBox("实时累积");
  accumulate_->setChecked(true);
  accumulate_->setToolTip("将收到的实时点云累积成地图；取消勾选后保留已有地图，暂停累积");
  tools->addWidget(accumulate_);
  addTool("＋ 航点", [this] {
    const int row = addWaypointAtAircraft();
    if (row < 0)
      return;
    waypoints_->selectRow(row);
    setEditorCollapsed(false);
    editWaypoint();
  });
  picking_ = new QCheckBox("点云拾取");
  tools->addWidget(picking_);
  addTool("适配", [this] { gl_->zoomGlobal(); });
  tools->addStretch(1);
  auto *viewButton =
      addTool("自由视角", [this] { gl_->setView(CC_ISO_VIEW_1); });
  viewButton->setToolTip("返回等轴测视角；鼠标仍可旋转三维视图");
  mapBadge_ = new QLabel("任务地图 · 等待数据",
                         canvas_);
  mapBadge_->setObjectName("mapBadge");
  mapBadge_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

  left_ = card(canvas_, "");
  auto *l = box(left_);
  l->setContentsMargins(5, 7, 5, 7);
  l->setSpacing(5);
  const QVector<std::tuple<QString, QString, ccHObject *>> layerTools = {
      {"☁", "实时彩色点云", live_}, {"▧", "累积 / 参考地图", map_},
      {"⌁", "本地任务航线", route_},
      {"◇", "任务机航点（只读）", remoteRoute_},
      {"∿", "EGO 实时规划", ego_},
      {"▦", "EGO 栅格地图", grid_},
      {"◎", "飞机实际轨迹", trail_}};
  for (const auto &entry : layerTools) {
    auto *tool = new QToolButton(left_);
    tool->setObjectName("layerTool");
    tool->setText(std::get<0>(entry));
    tool->setToolTip(std::get<1>(entry));
    tool->setAccessibleName(std::get<1>(entry));
    tool->setCheckable(true);
    tool->setChecked(true);
    tool->setFixedSize(42, 42);
    if (std::get<2>(entry) == grid_) {
      gridButton_ = tool;
      tool->setToolTip("EGO 栅格地图 · 等待任务机数据");
      // Draw the grid icon explicitly so it does not depend on font coverage.
      QPixmap icon(24, 24);
      icon.fill(Qt::transparent);
      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing);
      painter.setPen(QPen(QColor("#13877e"), 1.5));
      for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x)
          painter.drawRoundedRect(QRectF(2 + x * 7, 2 + y * 7, 5, 5), .7, .7);
      painter.end();
      tool->setText({});
      tool->setIcon(QIcon(icon));
      tool->setIconSize(QSize(24, 24));
    }
    l->addWidget(tool);
    connect(tool, &QToolButton::toggled, this,
            [this, g = std::get<2>(entry)](bool visible) {
              g->setEnabled(visible);
              gl_->redraw();
            });
  }
  auto *divider = new QFrame(left_);
  divider->setFrameShape(QFrame::HLine);
  l->addWidget(divider);
  mapDetailsButton_ = new QToolButton(left_);
  mapDetailsButton_->setObjectName("mapDetailsTool");
  mapDetailsButton_->setText("⌖");
  mapDetailsButton_->setToolTip("当前地图与任务机连接");
  mapDetailsButton_->setAccessibleName("当前地图与任务机连接");
  mapDetailsButton_->setFixedSize(42, 42);
  l->addWidget(mapDetailsButton_);
  mapPopover_ = card(canvas_, "当前地图");
  auto *mapBox = box(mapPopover_);
  mapInfo_ = new QLabel(
      QString("坐标系  %1\n地图 ID  %2\n累积点  0").arg(frame_, mapId_),
      mapPopover_);
  mapInfo_->setObjectName("mutedText");
  mapInfo_->setWordWrap(true);
  mapBox->addWidget(mapInfo_);
  mode_ = new QLabel("● 离线 · 等待任务机", mapPopover_);
  mode_->setObjectName("mutedText");
  mode_->setWordWrap(true);
  mapBox->addWidget(mode_);
  auto *leftButtons = new QHBoxLayout;
  mapBox->addLayout(leftButtons);
  connectButton_ = button(leftButtons, "连接", [this] {
    if (ready_) {
      client_.stop();
      ready_ = false;
      mode_->setText("未连接");
      connectButton_->setText("连接");
    } else {
      setDemo(false);
      client_.open(QUrl(baseUrl_));
    }
  });
  button(leftButtons, "加载点云", [this] { loadMap(); });
  mapPopover_->hide();
  connect(mapDetailsButton_, &QToolButton::clicked, this, [this] {
    mapPopover_->setVisible(!mapPopover_->isVisible());
    if (mapPopover_->isVisible())
      mapPopover_->raise();
  });
  editor_ = card(canvas_, "");
  auto *edit = box(editor_);
  auto *routeHeading = new QHBoxLayout;
  edit->addLayout(routeHeading);
  auto *routeTitle = new QLabel("航点任务", editor_);
  routeTitle->setObjectName("cardTitle");
  routeHeading->addWidget(routeTitle);
  routeSummary_ = new QLabel("0 个航点 · 本地编排", editor_);
  routeSummary_->setObjectName("mutedText");
  routeHeading->addWidget(routeSummary_);
  routeHeading->addStretch();
  auto *remoteRouteButton = button(routeHeading, "机载航点", [this] {
    requestRemoteRoute();
    setEditorCollapsed(false);
  });
  remoteRouteButton->setToolTip("从任务机重新读取当前 EGO 航点（只读）");
  routeFold_ = button(routeHeading, "查看 ↗", [this] {
    setEditorCollapsed(!editorCollapsed_);
  });
  routeFold_->setAccessibleName("查看航点任务弹窗");
  routeStrip_ = new QWidget(editor_);
  auto *strip = new QHBoxLayout(routeStrip_);
  strip->setContentsMargins(0, 0, 0, 0);
  strip->setSpacing(3);
  routeStrip_->setFixedHeight(31);
  edit->addWidget(routeStrip_);
  routeDialog_ = new QDialog(window_);
  routeDialog_->setWindowTitle("航点任务 · 本地与机载");
  routeDialog_->setModal(false);
  routeDialog_->resize(660, 440);
  routeDialog_->setMinimumSize(540, 340);
  auto *popup = new QVBoxLayout(routeDialog_);
  popup->setContentsMargins(12, 10, 12, 12);
  routeDetails_ = new QWidget(routeDialog_);
  auto *details = new QVBoxLayout(routeDetails_);
  details->setContentsMargins(0, 0, 0, 0);
  details->setSpacing(6);
  popup->addWidget(routeDetails_);
  connect(routeDialog_, &QDialog::finished, this,
          [this] { setEditorCollapsed(true); });
  auto *instruction = new QLabel("添加默认取飞机当前 XYZ；本地航点可双击编辑；机载航点只读", routeDetails_);
  instruction->setObjectName("mutedText");
  details->addWidget(instruction);
  auto *routeOptions = new QHBoxLayout;
  details->addLayout(routeOptions);
  follow_ = new QCheckBox("跟随飞机");
  routeOptions->addWidget(follow_);
  connect(picking_, &QCheckBox::toggled, this, [this](bool on) {
    if (on) {
      if (!window_->pickingHub()->addListener(this, true)) {
        picking_->setChecked(false);
        log("拾取功能正被其他工具使用");
      }
    } else
      window_->pickingHub()->removeListener(this);
  });
  altitude_ = new QDoubleSpinBox;
  altitude_->setRange(-1000, 10000);
  altitude_->setValue(3);
  altitude_->setSuffix(" m · 拾取点 Z");
  altitude_->setToolTip("点云拾取航点时使用；添加按钮取飞机当前 XYZ");
  altitude_->setMaximumWidth(180);
  routeOptions->addWidget(altitude_);
  routeOptions->addStretch();
  waypoints_ = new QTableWidget(0, 7, routeDetails_);
  waypoints_->setHorizontalHeaderLabels(
      {"来源", "点", "X", "Y", "Z", "类型", "停留"});
  waypoints_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  waypoints_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  waypoints_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  waypoints_->verticalHeader()->hide();
  waypoints_->setSelectionBehavior(QAbstractItemView::SelectRows);
  waypoints_->setSelectionMode(QAbstractItemView::SingleSelection);
  waypoints_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  details->addWidget(waypoints_, 1);
  remoteRouteInfo_ = new QLabel("任务机航点尚未读取 · 只读", routeDetails_);
  remoteRouteInfo_->setObjectName("mutedText");
  remoteRouteInfo_->setWordWrap(true);
  details->addWidget(remoteRouteInfo_);
  gizmo_ = new WaypointGizmo(
      canvas_, gl_, [this](int axis, double distance) {
        moveWaypointAlongAxis(axis, distance);
      },
      [this] { updateWaypointGizmo(); });
  connect(waypoints_, &QTableWidget::currentCellChanged, this,
          [this](int, int, int, int) {
            updateWaypointGizmo();
            refreshRouteStrip();
          });
  connect(waypoints_, &QTableWidget::cellDoubleClicked, this,
          [this](int, int) { editWaypoint(); });
  auto *row = new QHBoxLayout;
  details->addLayout(row);
  button(row, "添加本地", [this] {
    const int row = addWaypointAtAircraft();
    if (row < 0)
      return;
    waypoints_->selectRow(row);
    editWaypoint();
  });
  button(row, "删除本地", [this] {
    int n = waypoints_->currentRow();
    if (n >= 0 && n < mission_.waypoints_size()) {
      mission_.mutable_waypoints()->DeleteSubrange(n, 1);
      rebuildRoute();
    }
  });
  button(row, "本地上移", [this] {
    int n = waypoints_->currentRow();
    if (n > 0 && n < mission_.waypoints_size()) {
      mission_.mutable_waypoints()->SwapElements(n, n - 1);
      rebuildRoute();
      waypoints_->selectRow(n - 1);
    }
  });
  button(details, "保存本地航线", [this] { saveMission(); });
  button(details, "清空飞机轨迹", [this] {
    history_.clear();
    trailDirty_ = dirty_ = true;
  });
  routeDialog_->hide();
  editor_->show();
  refreshRouteStrip();
  right_ = new QFrame(canvas_);
  auto *r = new QVBoxLayout(right_);
  r->setContentsMargins(0, 0, 0, 0);
  r->setSpacing(12);
  auto *h = card(right_, "无人机 HUD");
  r->addWidget(h);
  auto *hr = new QHBoxLayout;
  attitude_ = new AttitudeWidget(h);
  hr->addWidget(attitude_);
  hud_ = new QLabel("R— P— Y—\n速度 —\nXYZ —\n电量 —");
  hud_->setStyleSheet("font-size:11px;");
  hud_->setWordWrap(true);
  hr->addWidget(hud_, 1);
  box(h)->addLayout(hr);
  batteryBar_ = new QProgressBar(h);
  batteryBar_->setRange(0, 100);
  batteryBar_->setValue(0);
  batteryBar_->setTextVisible(false);
  box(h)->addWidget(batteryBar_);
  auto *s = card(right_, "任务机状态信息");
  r->addWidget(s);
  status_ = new QLabel("任务机网络　离线\nOdin1　　　—\nEGO　　　　—\n定位　　"
                       "　　—\n视频 / 记录　—");
  status_->setStyleSheet("font-size:11px;");
  status_->setWordWrap(true);
  box(s)->addWidget(status_);
  auto *o = card(right_, "任务机操作");
  r->addWidget(o);
  auto *grid = new QGridLayout;
  box(o)->addLayout(grid);
  QVector<QPair<QString, mission::Command::Kind>> ops = {
      {"启动 Odin1", mission::Command::START_ODIN},
      {"启动 EGO", mission::Command::START_EGO},
      {"开启视频", mission::Command::VIDEO_START},
      {"开始记录", mission::Command::RECORD_START},
      {"保存机载地图", mission::Command::SAVE_MAP},
      {"上传航线", mission::Command::UPLOAD_MISSION},
      {"结束任务", mission::Command::END_MISSION}};
  int i = 0;
  for (auto op : ops) {
    auto *b = new QPushButton(op.first);
    if (op.second == mission::Command::END_MISSION)
      b->setObjectName("dangerButton");
    commands_[op.second] = b;
    if (op.second == mission::Command::VIDEO_START ||
        op.second == mission::Command::RECORD_START) {
      auto *cell = new QVBoxLayout;
      cell->setSpacing(3);
      cell->addWidget(b);
      auto *state = new QLabel;
      state->setObjectName("mutedText");
      state->setAlignment(Qt::AlignCenter);
      state->setWordWrap(true);
      cell->addWidget(state);
      mediaStates_[op.second] = state;
      grid->addLayout(cell, i / 2, i % 2);
      connect(b, &QPushButton::clicked, this,
              [this, k = op.second] { toggleMedia(k); });
    } else {
      if (op.second == mission::Command::END_MISSION)
        grid->addWidget(b, i / 2, 0, 1, 2);
      else
        grid->addWidget(b, i / 2, i % 2);
      connect(b, &QPushButton::clicked, this,
              [this, k = op.second] { command(k); });
    }
    ++i;
  }
  refreshStartupButtons();
  video_ = new QFrame(canvas_);
  video_->setObjectName("videoOverlay");
  auto *videoLayout = new QVBoxLayout(video_);
  videoLayout->setContentsMargins(0, 0, 0, 0);
  videoLayout->setSpacing(0);
  video_->setMinimumSize(160, 90);
  video_->resize(320, 180);
  video_->installEventFilter(this);
  videoWidget_ = new VideoCanvas(video_);
  videoWidget_->setMouseTracking(true);
  videoWidget_->setToolTip("拖动视频移动；拖动右上角等比例缩放");
  videoWidget_->installEventFilter(this);
  videoLayout->addWidget(videoWidget_);
  player_ = new QMediaPlayer(this);
  player_->setVideoOutput(videoWidget_->surface);
  connect(player_, QOverload<QMediaPlayer::Error>::of(&QMediaPlayer::error),
          this, [this](QMediaPlayer::Error error) {
            if (error == QMediaPlayer::NoError)
              return;
            log(player_->errorString(), "视频");
            videoWidget_->update();
          });
  connect(player_, &QMediaPlayer::mediaStatusChanged, this,
          [this](QMediaPlayer::MediaStatus s) {
            if (s == QMediaPlayer::StalledMedia)
              log("视频缓冲中或流中断", "视频");
            else if (s == QMediaPlayer::EndOfMedia)
              videoWidget_->update();
          });
  video_->hide();
  processingToolbar_ = card(canvas_, "");
  processingToolbar_->setObjectName("missionToolbar");
  auto *processingTools = new QHBoxLayout;
  processingTools->setContentsMargins(0, 0, 0, 0);
  processingTools->setSpacing(3);
  box(processingToolbar_)->setContentsMargins(7, 4, 7, 4);
  box(processingToolbar_)->addLayout(processingTools);
  button(processingTools, "＋ 导入", [this] { importProcessingFiles(); });
  button(processingTools, "地图快照", [this] { snapshotMap(); });
  button(processingTools, "适配视图", [this] { gl_->zoomGlobal(); });
  button(processingTools, "导出选中", [this] {
    runCloudCompareAction("actionSave");
  });
  objectsCard_ = card(canvas_, "处理对象");
  box(objectsCard_)->setContentsMargins(9, 7, 9, 9);
  auto *objectsHint = new QLabel("独立对象 · 多选可用于配准和比较", objectsCard_);
  objectsHint->setObjectName("mutedText");
  objectsHint->setWordWrap(true);
  box(objectsCard_)->addWidget(objectsHint);
  processingObjects_ = new QListWidget(objectsCard_);
  processingObjects_->setObjectName("processingObjects");
  processingObjects_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  processingObjects_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  processingObjects_->setTextElideMode(Qt::ElideMiddle);
  processingObjects_->setMinimumHeight(160);
  box(objectsCard_)->addWidget(processingObjects_);
  connect(processingObjects_, &QListWidget::itemSelectionChanged, this, [this] {
    auto *root = window_->dbRootObject();
    if (!root)
      return;
    QSet<unsigned> selectedIds;
    for (auto *item : processingObjects_->selectedItems())
      selectedIds.insert(item->data(Qt::UserRole).toUInt());
    ccHObject::Container selectedObjects;
    for (int row = 0; row < processingObjects_->count(); ++row) {
      const unsigned id = processingObjects_->item(row)->data(Qt::UserRole).toUInt();
      if (selectedIds.contains(id)) {
        if (auto *object = root->find(id))
          selectedObjects.push_back(object);
      }
    }
    window_->db()->selectEntities(selectedObjects);
  });
  button(box(objectsCard_), "＋ 导入文件", [this] { importProcessingFiles(); });
  processingCard_ = card(canvas_, "处理工具");
  box(processingCard_)->setContentsMargins(9, 7, 9, 9);
  processingHint_ = new QLabel("选择点云或网格，再运行已编译的工具。", processingCard_);
  processingHint_->setObjectName("mutedText");
  processingHint_->setWordWrap(true);
  box(processingCard_)->addWidget(processingHint_);
  processingOnly_ = new QWidget(processingCard_);
  auto *processingActions = new QVBoxLayout(processingOnly_);
  processingActions->setContentsMargins(0, 0, 0, 0);
  processingActions->setSpacing(4);
  box(processingCard_)->addWidget(processingOnly_);
  auto addCardTool = [this](QVBoxLayout *layout, const QString &label,
                            const char *name) {
    button(layout, label, [this, name] {
      runCloudCompareAction(name);
    });
  };
  addCardTool(processingActions, "复制对象", "actionClone");
  addCardTool(processingActions, "抽稀", "actionSubsample");
  addCardTool(processingActions, "计算法向量", "actionComputeNormals");
  addCardTool(processingActions, "栅格化", "actionRasterize");
  comparisonOnly_ = new QWidget(processingCard_);
  auto *comparisonActions = new QVBoxLayout(comparisonOnly_);
  comparisonActions->setContentsMargins(0, 0, 0, 0);
  comparisonActions->setSpacing(4);
  auto *comparisonInputs = new QFormLayout;
  referenceObject_ = new QComboBox(comparisonOnly_);
  targetObject_ = new QComboBox(comparisonOnly_);
  comparisonInputs->addRow("参考", referenceObject_);
  comparisonInputs->addRow("待测", targetObject_);
  comparisonActions->addLayout(comparisonInputs);
  connect(referenceObject_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this] { selectComparisonPair(); });
  connect(targetObject_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this] { selectComparisonPair(); });
  addCardTool(comparisonActions, "ICP 配准…", "actionRegister");
  addCardTool(comparisonActions, "点云到点云距离…", "actionCloudCloudDist");
  addCardTool(comparisonActions, "点云到网格距离…", "actionCloudMeshDist");
  box(processingCard_)->addWidget(comparisonOnly_);
  comparisonOnly_->hide();
  processingToolbar_->hide();
  objectsCard_->hide();
  processingCard_->hide();
  left_->show();
  right_->show();
  top_->show();
  mapBadge_->show();
  gl_->setView(CC_ISO_VIEW_1);
  window_->statusBar()->showMessage("地图坐标：" + frame_ + "　·　单位：m");
  protocolHint_ = new QLabel("Protobuf v1  ·  未连接", window_->statusBar());
  protocolHint_->setObjectName("mutedText");
  window_->statusBar()->addPermanentWidget(protocolHint_);
  for (auto *cardWidget : {left_, mapPopover_, top_, editor_, h, s, o,
                           processingToolbar_, objectsCard_, processingCard_}) {
    auto *shadow = new QGraphicsDropShadowEffect(cardWidget);
    shadow->setBlurRadius(22);
    shadow->setOffset(0, 4);
    shadow->setColor(QColor(7, 26, 37, 53));
    cardWidget->setGraphicsEffect(shadow);
  }
  auto *objectRefresh = new QTimer(this);
  objectRefresh->setInterval(900);
  connect(objectRefresh, &QTimer::timeout, this,
          &MissionController::refreshProcessingObjects);
  objectRefresh->start();
}
void MissionController::setWorkspace(Workspace workspace) {
  workspace_ = workspace;
  const bool monitor = workspace == Workspace::Monitor;
  if (!monitor && picking_ && picking_->isChecked())
    picking_->setChecked(false);
  if (!monitor && routeDialog_)
    routeDialog_->hide();
  if (mapPopover_)
    mapPopover_->hide();
  for (QWidget *widget : QVector<QWidget *>{left_, right_, top_, editor_, mapBadge_})
    widget->setVisible(monitor);
  for (auto *widget : {objectsCard_, processingCard_, processingToolbar_})
    widget->setVisible(!monitor);
  processingOnly_->setVisible(workspace == Workspace::Processing);
  comparisonOnly_->setVisible(workspace == Workspace::Comparison);
  if (auto *action = workspaceActions_.value(static_cast<int>(workspace)))
    action->setChecked(true);
  if (!monitor) {
    auto *title = processingCard_->findChild<QLabel *>("cardTitle");
    if (title)
      title->setText(workspace == Workspace::Comparison ? "对比分析" : "处理工具");
    processingHint_->setText(
        workspace == Workspace::Comparison
            ? "选择两个对象，使用 ICP 配准或计算点云距离。"
            : "选择处理对象；工具使用 CloudCompare 原有算法与对话框。");
    refreshProcessingObjects();
    if (workspace == Workspace::Comparison)
      selectComparisonPair();
  }
  arrange();
}
void MissionController::refreshProcessingObjects() {
  if (!processingObjects_ || workspace_ == Workspace::Monitor)
    return;
  auto *root = window_->dbRootObject();
  if (!root)
    return;
  QVector<QPair<unsigned, QString>> objects;
  std::function<void(ccHObject *, int)> collect = [&](ccHObject *parent,
                                                       int depth) {
    for (unsigned i = 0; i < parent->getChildrenNumber(); ++i) {
      auto *child = parent->getChild(i);
      const bool pointCloud = child->isKindOf(CC_TYPES::POINT_CLOUD);
      const bool mesh = child->isKindOf(CC_TYPES::MESH);
      if (pointCloud || mesh) {
        objects.append({child->getUniqueID(),
                        QString(depth * 2, QChar(' ')) +
                            (mesh ? "▧ " : "☁ ") + child->getName()});
      }
      if (!mesh)
        collect(child, depth + 1);
    }
  };
  collect(root, 0);
  bool changed = objects.size() != processingObjects_->count();
  if (!changed) {
    for (int i = 0; i < objects.size(); ++i) {
      auto *item = processingObjects_->item(i);
      if (item->data(Qt::UserRole).toUInt() != objects[i].first ||
          item->text() != objects[i].second) {
        changed = true;
        break;
      }
    }
  }
  QSet<unsigned> selectedIds;
  for (auto *object : window_->getSelectedEntities())
    selectedIds.insert(object->getUniqueID());
  QSignalBlocker blocked(processingObjects_);
  if (changed) {
    processingObjects_->clear();
    for (const auto &object : objects) {
      auto *item = new QListWidgetItem(object.second, processingObjects_);
      item->setData(Qt::UserRole, object.first);
      item->setToolTip(object.second.trimmed());
    }
  }
  if (changed) {
    const unsigned previousReference = referenceObject_->currentData().toUInt();
    const unsigned previousTarget = targetObject_->currentData().toUInt();
    const QSignalBlocker referenceBlocked(referenceObject_);
    const QSignalBlocker targetBlocked(targetObject_);
    referenceObject_->clear();
    targetObject_->clear();
    for (const auto &object : objects) {
      const QString name = object.second.trimmed();
      const QString compact = name.size() > 18
                                  ? name.left(6) + "…" + name.right(10)
                                  : name;
      referenceObject_->addItem(compact, object.first);
      targetObject_->addItem(compact, object.first);
      referenceObject_->setItemData(referenceObject_->count() - 1, name,
                                    Qt::ToolTipRole);
      targetObject_->setItemData(targetObject_->count() - 1, name,
                                 Qt::ToolTipRole);
    }
    if (referenceObject_->count()) {
      referenceObject_->setCurrentIndex(
          qMax(0, referenceObject_->findData(previousReference)));
      targetObject_->setCurrentIndex(
          qMax(0, targetObject_->findData(previousTarget)));
      if (objects.size() > 1 &&
          targetObject_->currentData() == referenceObject_->currentData())
        targetObject_->setCurrentIndex(referenceObject_->currentIndex() == 0 ? 1 : 0);
    }
  }
  for (int i = 0; i < processingObjects_->count(); ++i) {
    auto *item = processingObjects_->item(i);
    item->setSelected(selectedIds.contains(item->data(Qt::UserRole).toUInt()));
  }
  if (objects.isEmpty())
    processingHint_->setText("尚无处理对象。导入文件，或为实时地图创建快照。");
}
void MissionController::selectComparisonPair() {
  if (workspace_ != Workspace::Comparison || !processingObjects_)
    return;
  const unsigned reference = referenceObject_->currentData().toUInt();
  const unsigned target = targetObject_->currentData().toUInt();
  if (!reference || !target || reference == target) {
    processingHint_->setText("参考和待测必须是两个不同对象。");
    return;
  }
  auto *root = window_->dbRootObject();
  if (!root)
    return;
  ccHObject::Container pair;
  if (auto *object = root->find(reference))
    pair.push_back(object);
  if (auto *object = root->find(target))
    pair.push_back(object);
  window_->db()->selectEntities(pair);
  refreshProcessingObjects();
  processingHint_->setText("已选择参考与待测对象；距离方向可在原工具对话框确认。");
}
void MissionController::importProcessingFiles() {
  QSettings settings("3DG", "Mission");
  const QStringList paths = QFileDialog::getOpenFileNames(
      window_, "导入处理对象", settings.value("processingDir").toString(),
      FileIOFilter::ImportFilterList().join(";;"));
  if (paths.isEmpty())
    return;
  settings.setValue("processingDir", QFileInfo(paths.first()).absolutePath());
  window_->addToDB(paths, QString(), gl_);
  setWorkspace(Workspace::Processing);
  refreshProcessingObjects();
  log(QString("已请求导入 %1 个处理文件").arg(paths.size()), "点云处理");
}
void MissionController::snapshotMap() {
  const auto &source = !mapPoints_.isEmpty() ? mapPoints_ : livePoints_;
  if (source.isEmpty()) {
    log("当前没有可快照的地图或实时点云", "点云处理");
    return;
  }
  auto *cloud = new ccPointCloud(
      QString("地图快照_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
  if (!cloud->reserve(source.size()) || !cloud->reserveTheRGBTable()) {
    delete cloud;
    log("地图快照内存分配失败", "点云处理");
    return;
  }
  for (const auto &point : source) {
    cloud->addPoint(CCVector3(point.x, point.y, point.z));
    cloud->addColor(point.r, point.g, point.b);
  }
  cloud->showColors(true);
  cloud->setPointSize(2);
  cloud->setMetaData("3dg.source", mapPoints_.isEmpty() ? "live" : "map");
  cloud->setMetaData("3dg.mapId", mapId_);
  cloud->setMetaData("3dg.frame", frame_);
  cloud->setMetaData("3dg.unit", "m");
  cloud->setMetaData("3dg.capturedAt", QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
  cloud->setDisplay(gl_);
  window_->addToDB(cloud, false, true, false, true);
  setWorkspace(Workspace::Processing);
  refreshProcessingObjects();
  log(QString("已创建 %1 点地图快照").arg(source.size()), "点云处理");
}
void MissionController::runCloudCompareAction(const char *objectName) {
  auto *action = window_->findChild<QAction *>(QString::fromLatin1(objectName));
  if (!action) {
    log(QString("当前构建不包含工具 %1").arg(objectName), "点云处理");
    return;
  }
  if (!action->isEnabled()) {
    const QString message = QString("%1：请检查选中对象的数量与类型")
                                .arg(action->text().remove('&'));
    processingHint_->setText(message);
    log(message, "点云处理");
    return;
  }
  if (picking_ && picking_->isChecked())
    picking_->setChecked(false);
  action->trigger();
  QTimer::singleShot(0, this, &MissionController::refreshProcessingObjects);
}
void MissionController::refreshMapInfo() {
  if (mapInfo_)
    mapInfo_->setText(QString("坐标系  %1\n地图 ID  %2\n实时点  %3\n%4  %5")
                          .arg(frame_, mapId_)
                          .arg(livePoints_.size())
                          .arg(referenceMap_ ? "参考地图点" : "累积点")
                          .arg(mapPoints_.size()));
  if (mapBadge_)
    mapBadge_->setText(
        QString("航点 本地 %1 · 机载 %2  |  EGO · 轨迹")
            .arg(mission_.waypoints_size()).arg(remotePoints_.size()));
  if (window_)
    window_->statusBar()->showMessage("地图坐标：" + frame_ + "　·　单位：m");
}
void MissionController::refreshRouteStrip() {
  if (!routeStrip_ || !routeSummary_)
    return;
  const int localCount = mission_.waypoints_size();
  const int remoteCount = remotePoints_.size();
  const int count = localCount + remoteCount;
  const int selected = waypoints_ ? waypoints_->currentRow() : -1;
  routeSummary_->setText(QString("%1 个 · 本地 %2 / 机载 %3")
                             .arg(count).arg(localCount).arg(remoteCount));
  auto *strip = qobject_cast<QHBoxLayout *>(routeStrip_->layout());
  while (auto *item = strip->takeAt(0)) {
    if (auto *widget = item->widget()) {
      widget->hide();
      widget->deleteLater();
    }
    delete item;
  }
  if (count == 0) {
    auto *empty = new QLabel("暂无航点 · 点击查看打开弹窗", routeStrip_);
    empty->setObjectName("mutedText");
    strip->addWidget(empty);
    strip->addStretch();
    return;
  }
  constexpr int kVisible = 4;
  const int first = qBound(0, selected - 1, qMax(0, count - kVisible));
  const int last = qMin(count, first + kVisible);
  if (first > 0)
    strip->addWidget(new QLabel("…", routeStrip_));
  for (int i = first; i < last; ++i) {
    const bool local = i < localCount;
    auto *chip = new QPushButton(
        QString("%1%2").arg(local ? "本" : "机")
            .arg(local ? i + 1 : i - localCount + 1), routeStrip_);
    chip->setObjectName("routeChip");
    chip->setCheckable(true);
    chip->setChecked(i == selected);
    chip->setToolTip(local ? QString("本地航点 %1 · 可编辑").arg(i + 1)
                           : QString("机载航点 %1 · 只读").arg(i - localCount + 1));
    strip->addWidget(chip);
    connect(chip, &QPushButton::clicked, this, [this, i] {
      waypoints_->selectRow(i);
      setEditorCollapsed(false);
    });
  }
  if (last < count)
    strip->addWidget(new QLabel(QString("… +%1").arg(count - last), routeStrip_));
  strip->addStretch();
}
void MissionController::setEditorCollapsed(bool collapsed) {
  if (!editor_ || !routeDialog_ || !routeStrip_)
    return;
  editorCollapsed_ = collapsed;
  routeFold_->setText(collapsed ? "查看 ↗" : "关闭 ×");
  routeFold_->setAccessibleName(collapsed ? "查看航点任务弹窗" : "关闭航点任务弹窗");
  if (collapsed) {
    routeDialog_->hide();
  } else {
    if (!routeDialog_->isVisible())
      routeDialog_->move(window_->frameGeometry().center() -
                         QPoint(routeDialog_->width() / 2,
                                routeDialog_->height() / 2));
    routeDialog_->show();
    routeDialog_->raise();
    routeDialog_->activateWindow();
  }
  updateWaypointGizmo();
}
void MissionController::setToolbarCollapsed(bool collapsed) {
  if (!top_ || !toolbarContents_ || !toolbarFold_)
    return;
  toolbarCollapsed_ = collapsed;
  toolbarContents_->setVisible(!collapsed);
  toolbarFold_->setText(collapsed ? "▶" : "◀");
  toolbarFold_->setToolTip(collapsed ? "展开工具栏" : "向左收起工具栏");
  toolbarFold_->setAccessibleName(collapsed ? "展开工具栏" : "向左收起工具栏");
  arrange();
}
void MissionController::arrange() {
  if (!canvas_)
    return;
  int w = canvas_->width(), h = canvas_->height();
  int margin = 12, leftWidth = 54,
      rightWidth = w < 1150 ? 228 : 270;
  int centerX = leftWidth + margin * 2,
      centerWidth = qMax(320, w - leftWidth - rightWidth - margin * 4);
  left_->setGeometry(margin, margin, leftWidth,
                     qMin(h - margin * 2, left_->sizeHint().height()));
  mapPopover_->setGeometry(margin + leftWidth + 8, margin + 220, 248,
                           mapPopover_->sizeHint().height());
  right_->setGeometry(w - rightWidth - margin, margin, rightWidth,
                      right_->sizeHint().height());
  top_->setGeometry(centerX, margin,
                    toolbarCollapsed_ ? 44 : qMin(centerWidth, 450), 46);
  objectsCard_->setGeometry(margin, margin + 58, 190,
                            qMin(h - margin * 2 - 64, 320));
  processingCard_->setGeometry(w - margin - 234, margin + 58, 234,
                               qMin(h - margin * 2 - 64,
                                    processingCard_->sizeHint().height()));
  processingToolbar_->setGeometry(
      qMax(210, (w - 450) / 2), margin, qMin(450, w - 440), 46);
  const int editorWidth = qMin(440, centerWidth - 4);
  const int editorHeight = qMin(h - margin * 2, editor_->sizeHint().height());
  const int editorX = centerX + (centerWidth - editorWidth) / 2;
  const int editorY = h - editorHeight - margin;
  editor_->setGeometry(editorX, editorY, editorWidth, editorHeight);
  mapBadge_->setGeometry(centerX,
                         editorX < centerX + 302 ? editorY - 44 : h - margin - 34,
                         302, 34);
  if (!dragging_ && !videoPlaced_) {
    video_->move(centerX + 2,
                 qMax(margin + 120,
                      qMin(h - video_->height() - margin,
                           editorY - video_->height() - 10)));
  } else if (videoPlaced_) {
    const int maxWidth = qMin(w, (h * 16) / 9);
    const int width = qBound(video_->minimumWidth(), video_->width(),
                             qMax(video_->minimumWidth(), maxWidth));
    video_->resize(width, qRound(width * 9.0 / 16.0));
    video_->move(qBound(0, video_->x(), qMax(0, w - video_->width())),
                 qBound(0, video_->y(), qMax(0, h - video_->height())));
  }
  top_->raise();
  mapBadge_->raise();
  left_->raise();
  right_->raise();
  editor_->raise();
  if (workspace_ != Workspace::Monitor) {
    objectsCard_->raise();
    processingCard_->raise();
    processingToolbar_->raise();
  }
  if (video_->isVisible())
    video_->raise();
  if (mapPopover_->isVisible())
    mapPopover_->raise();
}
bool MissionController::eventFilter(QObject *o, QEvent *e) {
  if (e->type() == QEvent::Close && qobject_cast<QMdiSubWindow *>(o)) {
    e->ignore();
    return true;
  }
  if (e->type() == QEvent::Show) {
    if (auto *d = qobject_cast<QDockWidget *>(o))
      QTimer::singleShot(0, d, &QWidget::hide);
  }
  if (o == canvas_ && e->type() == QEvent::Resize)
    arrange();
  if (o == canvas_ &&
      (e->type() == QEvent::MouseMove || e->type() == QEvent::Wheel ||
       e->type() == QEvent::MouseButtonRelease || e->type() == QEvent::Resize))
    QTimer::singleShot(0, this, [this] { updateWaypointGizmo(); });
  if (o == window_->menuBar() && e->type() == QEvent::Resize && latest_) {
    latest_->move(qMax(500, window_->menuBar()->width() - 360),
                  (window_->menuBar()->height() - latest_->height()) / 2);
    latest_->raise();
  }
  if (o == video_ || o == videoWidget_) {
    QMouseEvent *m = nullptr;
    if (e->type() == QEvent::MouseButtonPress ||
        e->type() == QEvent::MouseButtonRelease ||
        e->type() == QEvent::MouseMove)
      m = static_cast<QMouseEvent *>(e);
    const QPoint local = m ? video_->mapFromGlobal(m->globalPos()) : QPoint();
    constexpr int resizeCorner = 18;
    const bool atResizeCorner =
        local.x() >= video_->width() - resizeCorner &&
        local.x() < video_->width() && local.y() >= 0 &&
        local.y() < resizeCorner;
    if (e->type() == QEvent::MouseButtonPress) {
      if (m->button() == Qt::LeftButton) {
        if (atResizeCorner) {
          videoResizeMouse_ = m->globalPos();
          videoResizeStart_ = video_->size();
          videoResizeBottomLeft_ = video_->geometry().bottomLeft();
          resizingVideo_ = true;
        } else {
          videoDrag_ = local;
          dragging_ = true;
        }
        video_->raise();
        return true;
      }
    }
    if (e->type() == QEvent::MouseMove && resizingVideo_) {
      const QPoint delta = m->globalPos() - videoResizeMouse_;
      // Project mouse movement onto the upper-right diagonal. The lower-left
      // corner stays fixed while the window keeps its 16:9 aspect ratio.
      constexpr double heightPerWidth = 9.0 / 16.0;
      const double widthDelta =
          (delta.x() - heightPerWidth * delta.y()) /
          (1.0 + heightPerWidth * heightPerWidth);
      const int maxWidth = qMax(
          video_->minimumWidth(),
          qMin(canvas_->width() - videoResizeBottomLeft_.x(),
               (videoResizeBottomLeft_.y() + 1) * 16 / 9));
      const int width = qBound(video_->minimumWidth(),
                               qRound(videoResizeStart_.width() + widthDelta),
                               maxWidth);
      const int height = qRound(width * heightPerWidth);
      video_->setGeometry(videoResizeBottomLeft_.x(),
                          videoResizeBottomLeft_.y() - height + 1,
                          width, height);
      videoPlaced_ = true;
      return true;
    }
    if (e->type() == QEvent::MouseMove && dragging_) {
      QPoint p = canvas_->mapFromGlobal(m->globalPos() - videoDrag_);
      p.setX(qBound(0, p.x(), qMax(0, canvas_->width() - video_->width())));
      p.setY(qBound(0, p.y(), qMax(0, canvas_->height() - video_->height())));
      video_->move(p);
      videoPlaced_ = true;
      return true;
    }
    if (e->type() == QEvent::MouseMove) {
      videoWidget_->setCursor(atResizeCorner ? Qt::SizeBDiagCursor
                                              : Qt::SizeAllCursor);
    }
    if (e->type() == QEvent::MouseButtonRelease &&
        m->button() == Qt::LeftButton) {
      if (dragging_ || resizingVideo_)
        videoPlaced_ = true;
      dragging_ = false;
      resizingVideo_ = false;
      return true;
    }
  }
  return QObject::eventFilter(o, e);
}
void MissionController::log(const QString &t, const QString &source) {
  const QString line = QDateTime::currentDateTime().toString("HH:mm:ss") +
                       " [" + source + "] " + t.left(4000).replace('\n', ' ');
  if (logFile_.isOpen()) {
    logFile_.write((line + "\n").toUtf8());
    logFile_.flush();
  }
  auto *bar = logList_->verticalScrollBar();
  bool bottom = bar->value() >= bar->maximum() - 2;
  auto *item = new QListWidgetItem(line, logList_);
  item->setSizeHint(QSize(600, 27));
  item->setToolTip(line);
  while (logList_->count() > 2000)
    delete logList_->takeItem(0);
  if (bottom) {
    logList_->scrollToBottom();
    unread_ = 0;
  } else
    ++unread_;
  logCount_->setText(unread_ ? QString("新增 %1 条 · 当前保留 %2 条")
                                   .arg(unread_)
                                   .arg(logList_->count())
                             : "自动跟随最新日志");
  latest_->setText(
      QFontMetrics(latest_->font()).elidedText(line, Qt::ElideRight, 300) +
      " ▾");
  latest_->setToolTip(line);
}
void MissionController::makeCloud(ccHObject *g,
                                  const QVector<MissionPoint> &pts) {
  g->removeAllChildren();
  if (pts.isEmpty())
    return;
  auto *c = new ccPointCloud("XYZRGB");
  if (!c->reserve(pts.size()) || !c->reserveTheRGBTable()) {
    delete c;
    log("点云显存数据分配失败");
    return;
  }
  for (const auto &p : pts) {
    c->addPoint(CCVector3(p.x, p.y, p.z));
    c->addColor(p.r, p.g, p.b);
  }
  c->showColors(true);
  c->setPointSize(2);
  c->setDisplay(gl_);
  g->addChild(c);
}
void MissionController::makeLine(ccHObject *g, const QVector<QVector3D> &pts,
                                 unsigned char r, unsigned char green,
                                 unsigned char b, float width) {
  g->removeAllChildren();
  if (pts.isEmpty())
    return;
  auto *c = new ccPointCloud("vertices");
  if (!c->reserve(pts.size())) {
    delete c;
    return;
  }
  for (const auto &p : pts)
    c->addPoint(CCVector3(p.x(), p.y(), p.z()));
  auto *line = new ccPolyline(c);
  line->addChild(c);
  c->setEnabled(false);
  line->addPointIndex(0, pts.size());
  line->set2DMode(false);
  line->setColor(ccColor::Rgb(r, green, b));
  line->showColors(true);
  line->setWidth(width);
  line->setDisplay(gl_);
  g->addChild(line);
}
void MissionController::expireGrid() {
  gridAge_.invalidate();
  gridCenters_.clear();
  gridDirty_ = false;
  if (grid_) {
    grid_->removeAllChildren();
    grid_->setVisible(false);
  }
  if (gridButton_)
    gridButton_->setToolTip("EGO 栅格地图 · 数据已失效，等待任务机更新");
  if (gl_)
    gl_->redraw();
}
void MissionController::makeVoxelGrid() {
  grid_->removeAllChildren();
  // An alpha-zero mesh can still write to the depth buffer and hide other
  // layers, so skip its geometry entirely.
  if (gridCenters_.isEmpty() || gridOpacity_ <= 0.0)
    return;
  auto *vertices = new ccPointCloud("EGO voxel vertices");
  auto *mesh = new ccMesh(vertices);
  mesh->addChild(vertices);
  vertices->setEnabled(false);
  const unsigned count = unsigned(gridCenters_.size());
  if (!vertices->reserve(count * 8) || !vertices->reserveTheRGBTable() ||
      !mesh->reserve(count * 12)) {
    delete mesh;
    log("EGO 栅格显示内存分配失败");
    return;
  }
  const float half = float(gridResolution_ * .40); // Visual gaps reveal cells over dense clouds.
  const auto alpha = static_cast<unsigned char>(std::lround(gridOpacity_ * 255.0));
  static const unsigned faces[12][3] = {
      {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
      {0, 1, 5}, {0, 5, 4}, {2, 6, 7}, {2, 7, 3},
      {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
  for (unsigned i = 0; i < count; ++i) {
    const auto &p = gridCenters_[int(i)];
    for (int corner = 0; corner < 8; ++corner) {
      vertices->addPoint(CCVector3(p.x() + ((corner & 1) ? half : -half),
                                  p.y() + ((corner & 2) ? half : -half),
                                  p.z() + ((corner & 4) ? half : -half)));
      vertices->addColor(gridColor_.red(), gridColor_.green(),
                         gridColor_.blue(), alpha);
    }
    for (const auto &f : faces)
      mesh->addTriangle(i * 8 + f[0], i * 8 + f[1], i * 8 + f[2]);
  }
  mesh->setName("EGO occupancy voxels");
  vertices->showColors(true);
  mesh->showColors(true);
  mesh->showNormals(mesh->computePerTriangleNormals());
  mesh->setDisplay(gl_);
  grid_->addChild(mesh);
}
void MissionController::updateScene() {
  if (!gl_ || !dirty_)
    return;
  dirty_ = false;
  if (liveDirty_) {
    makeCloud(live_, livePoints_);
    liveDirty_ = false;
  }
  if (mapDirty_) {
    makeCloud(map_, mapPoints_);
    mapDirty_ = false;
  }
  if (gridDirty_) {
    makeVoxelGrid();
    gridDirty_ = false;
  }
  if (trailDirty_) {
    makeLine(trail_, history_, 255, 0, 0, 2);
    trailDirty_ = false;
  }
  if (egoDirty_) {
    makeLine(ego_, egoPoints_, 56, 224, 211, 3);
    egoDirty_ = false;
  }
  gl_->redraw();
}
void MissionController::updateAircraftPose() {
  if (!gl_)
    return;
  if (aircraftModel_)
    aircraftModel_->setVisible(lastVehicle_.pose_valid());
  if (lastVehicle_.pose_valid()) {
    const auto p = vec(lastVehicle_.position());
    const auto &q = lastVehicle_.orientation();
    // Keep local geometry immutable: absolute body-to-map transform each frame.
    // Normalization also tolerates the protocol's small quaternion norm error.
    QMatrix4x4 transform;
    transform.translate(p);
    transform.rotate(QQuaternion(q.w(), q.x(), q.y(), q.z()).normalized());
    if (aircraftModel_)
      aircraftModel_->setGLTransformation(ccGLMatrix(transform.constData()));
    if (follow_->isChecked())
      gl_->setPivotPoint(CCVector3d(p.x(), p.y(), p.z()));
  }
}
void MissionController::focusAircraft() {
  if (!gl_ || !lastVehicle_.pose_valid()) {
    log("当前没有有效的无人机位置");
    return;
  }
  const auto p = vec(lastVehicle_.position());
  const CCVector3 center(p.x(), p.y(), p.z());
  const CCVector3 margin(.55f, .55f, .55f);
  ccBBox bounds;
  bounds.add(center - margin);
  bounds.add(center + margin);
  gl_->updateConstellationCenterAndZoom(&bounds);
  gl_->redraw();
}
void MissionController::updateWaypointGizmo() {
  if (!gizmo_ || !waypoints_ || !gl_ || gizmo_->isDragging())
    return;
  const int row = waypoints_->currentRow();
  if (row < 0 || row >= mission_.waypoints_size()) {
    gizmo_->hide();
    return;
  }
  gizmo_->sync(vec(mission_.waypoints(row).position()));
}
