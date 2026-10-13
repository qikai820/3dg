#undef QT_USE_QSTRINGBUILDER
#include "MissionController.h"
#include "BagReplayToolbar.h"
#include "FlightReplayDialog.h"
#include "OdinDataManagerDialog.h"
#include "QuadrotorModel.h"
#include "mainwindow.h"
#include "ccDBRoot.h"
#include <FileIOFilter.h>
#include <QAbstractVideoSurface>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMediaPlayer>
#include <QNetworkAccessManager>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QQuaternion>
#include <QStandardPaths>
#include <QVideoFrame>
#include <QVideoSurfaceFormat>
#include <QtWidgets>
#include <ccGLUtils.h>
#include <ccGLWindowInterface.h>
#include <ccGLWindowSignalEmitter.h>
#include <ccGenericPrimitive.h>
#include <ccHObjectCaster.h>
#include <ccPickingHub.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccPluginManager.h>
#include <ccPolyline.h>
#include <ccStdPluginInterface.h>
#include <algorithm>
#include <cmath>
#include <CloudSamplingTools.h>
#include <ReferenceCloud.h>
#include <set>
#include <tuple>
#include <vector>
#include <google/protobuf/util/json_util.h>

class ProcessingResizeHandle final : public QWidget {
public:
  explicit ProcessingResizeHandle(QWidget *parent) : QWidget(parent) {
    setFixedSize(12, 12);
    setCursor(Qt::SizeFDiagCursor);
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(QColor(67, 100, 119), 1.3, Qt::SolidLine, Qt::RoundCap));
    for (int i = 0; i < 3; ++i)
      painter.drawLine(QPoint(width() - 5 - i * 3, height() - 2),
                       QPoint(width() - 2, height() - 5 - i * 3));
  }
};

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
    if (nativeResolution() != image.size())
      setNativeResolution(image.size());
    ++framesPresented;
    target_->update();
    return true;
  }
  void stop() override {
    image = QImage();
    target_->update();
    QAbstractVideoSurface::stop();
  }
  QImage image;
  quint64 framesPresented = 0;

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
  QString armed = "未知", flightMode = "未知", angles = "— / — / —",
          speed = "—", position = "— / — / —", battery = "—",
          flightTime = "00:00:00", flightDistance = "0.0 m", notice, context;
  explicit AttitudeWidget(QWidget *p) : QWidget(p) {
    setMinimumHeight(230);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setToolTip("数值顺序：横滚 / 俯仰 / 航向；位置 X / Y / Z。"
               "飞行统计按当前实时会话中已解锁且定位有效的遥测累计。");
  }
  QSize sizeHint() const override { return {270, 244}; }
  void reset(const QString &message = {}) {
    roll = pitch = 0;
    valid = false;
    armed = flightMode = "未知";
    angles = position = "— / — / —";
    speed = battery = "—";
    notice = message;
    context.clear();
    update();
  }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF area = rect();
    QPainterPath clip;
    clip.addRoundedRect(area, 12, 12);
    p.setClipPath(clip);
    p.fillRect(area, QColor(242, 248, 251, 216));
    if (valid) {
      p.save();
      p.translate(area.center());
      p.rotate(-roll);
      p.translate(0, pitch * 1.4);
      const qreal span = qMax(width(), height()) * 2.0;
      p.fillRect(QRectF(-span, -span, span * 2, span), QColor(90, 166, 202, 55));
      p.fillRect(QRectF(-span, 0, span * 2, span), QColor(119, 160, 150, 42));
      p.setPen(QPen(QColor(83, 128, 146, 160), 1.2));
      p.drawLine(QPointF(-span, 0), QPointF(span, 0));
      for (int y : {-40, -20, 20, 40})
        p.drawLine(QPointF(-18, y), QPointF(18, y));
      p.restore();

      // Fade the instrument beneath the values without separate dark panels.
      QLinearGradient fade(0, 0, 0, height());
      fade.setColorAt(0, QColor(242, 248, 251, 160));
      fade.setColorAt(0.3, QColor(242, 248, 251, 0));
      fade.setColorAt(0.58, QColor(242, 248, 251, 0));
      fade.setColorAt(0.76, QColor(242, 248, 251, 185));
      fade.setColorAt(1, QColor(242, 248, 251, 185));
      p.fillRect(area, fade);
      p.setPen(QPen(QColor(20, 125, 110), 2.5,
                    Qt::SolidLine, Qt::RoundCap));
      const qreal cx = width() / 2.0, cy = height() / 2.0;
      p.drawLine(QPointF(cx - 31, cy), QPointF(cx - 7, cy));
      p.drawLine(QPointF(cx + 7, cy), QPointF(cx + 31, cy));
      p.drawLine(QPointF(cx - 7, cy), QPointF(cx, cy + 6));
      p.drawLine(QPointF(cx, cy + 6), QPointF(cx + 7, cy));
    }

    p.setPen(QColor(48, 71, 86));
    QFont font = p.font();
    font.setPixelSize(13);
    font.setWeight(QFont::DemiBold);
    p.setFont(font);
    p.drawText(QRect(12, 5, width() - 112, 28),
               Qt::AlignVCenter | Qt::AlignLeft, armed + "  " + flightTime);
    p.drawText(QRect(width() - 100, 5, 88, 28),
               Qt::AlignVCenter | Qt::AlignRight, flightMode);
    font.setPixelSize(12);
    font.setWeight(QFont::Normal);
    p.setFont(font);
    p.drawText(QRect(width() - 100, 31, 88, 21),
               Qt::AlignVCenter | Qt::AlignRight, flightDistance);
    if (!context.isEmpty()) {
      font.setPixelSize(11);
      p.setFont(font);
      p.drawText(QRect(12, 31, width() - 112, 21),
                 Qt::AlignVCenter | Qt::AlignLeft, context);
    }
    font.setPixelSize(14);
    p.setFont(font);
    const int bottom = height() - 82;
    p.drawText(QRect(8, bottom + 2, width() - 16, 25), Qt::AlignCenter,
               angles);
    p.drawText(QRect(12, bottom + 28, width() / 2 - 12, 23),
               Qt::AlignVCenter | Qt::AlignLeft, speed);
    p.drawText(QRect(width() / 2, bottom + 28, width() / 2 - 12, 23),
               Qt::AlignVCenter | Qt::AlignRight, battery);
    font.setPixelSize(12);
    p.setFont(font);
    p.drawText(QRect(8, bottom + 53, width() - 16, 24), Qt::AlignCenter,
               position);
    if (!valid) {
      p.setPen(QColor(101, 125, 141));
      p.drawText(QRect(8, 53, width() - 16, bottom - 51),
                 Qt::AlignCenter | Qt::TextWordWrap,
                 notice.isEmpty() ? QStringLiteral("无姿态") : notice);
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
// The task-machine preview defaults to 0.2 Hz and can be set as low as 0.1 Hz.
// Keep a received snapshot through one delayed update; disconnect clears it.
constexpr int kGridStaleMs = 20000;
struct OverlayColor {
  unsigned char red, green, blue;
};
constexpr OverlayColor kCameraFovColor{84, 216, 243};
constexpr OverlayColor kLidarFovColor{255, 190, 100};
constexpr OverlayColor kAircraftInflationColor{202, 166, 250};
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
bool canCopyProcessingObject(const ccHObject *object) {
  return object && (object->isKindOf(CC_TYPES::POINT_CLOUD) ||
                    object->isKindOf(CC_TYPES::PRIMITIVE) ||
                    object->isA(CC_TYPES::MESH));
}
std::unique_ptr<ccHObject> cloneProcessingObject(ccHObject *source) {
  ccHObject *clone = nullptr;
  if (source->isKindOf(CC_TYPES::POINT_CLOUD)) {
    if (auto *cloud = ccHObjectCaster::ToGenericPointCloud(source))
      clone = cloud->clone();
  } else if (source->isKindOf(CC_TYPES::PRIMITIVE)) {
    if (auto *primitive = dynamic_cast<ccGenericPrimitive *>(source))
      clone = primitive->clone();
  } else if (source->isA(CC_TYPES::MESH)) {
    if (auto *mesh = ccHObjectCaster::ToMesh(source))
      clone = mesh->cloneMesh();
  }
  if (clone) {
    clone->setGLTransformationHistory(source->getGLTransformationHistory());
    clone->setName(source->getName());
  }
  return std::unique_ptr<ccHObject>(clone);
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
QString flightTimeText(double totalSeconds) {
  const qint64 seconds = qint64(totalSeconds);
  return QString("%1:%2:%3")
      .arg(seconds / 3600, 2, 10, QChar('0'))
      .arg((seconds / 60) % 60, 2, 10, QChar('0'))
      .arg(seconds % 60, 2, 10, QChar('0'));
}
QString flightDistanceText(double distanceM) {
  return distanceM >= 1000
      ? QString::number(distanceM / 1000, 'f', 2) + " km"
      : QString::number(distanceM, 'f', 1) + " m";
}
} // namespace
void MissionController::setStatusCells(const QStringList &values) {
  for (int i = 0; i < statusCells_.size(); ++i)
    statusCells_[i]->setText(values.value(i));
}
void MissionController::refreshLinkStats() {
  if (!bitError_)
    return;
  if (demo_ || !ready_) {
    bitError_->setText(demo_ ? "通讯误码率  —（模拟）"
                             : "通讯误码率  —（未连接）");
    protocolError_->setText("协议消息异常率  —");
    networkDown_->setText("网络下行  —");
    networkUp_->setText("网络上行  —");
  } else {
    bitError_->setText("通讯误码率  —（链路未提供）");
    const quint64 total = client_.receivedMessages();
    const quint64 invalid = client_.invalidMessages();
    protocolError_->setText(
        total ? QString("协议消息异常率  %1%（%2/%3 条）")
                    .arg(double(invalid) * 100.0 / double(total), 0, 'f', 2)
                    .arg(invalid)
                    .arg(total)
              : "协议消息异常率  —（等待数据）");
    const auto rate = [](double mbps) {
      return mbps >= 1.0 ? QString("%1 Mbps").arg(mbps, 0, 'f', 2)
                         : QString("%1 kbps").arg(mbps * 1000.0, 0, 'f', 1);
    };
    networkDown_->setText("网络下行  " + rate(client_.receiveMbps()));
    networkUp_->setText("网络上行  " + rate(client_.transmitMbps()));
  }
  // The numeric rate and message counts can wrap to another line while the
  // popover is open, so keep its height in sync with the labels.
  mapPopover_->layout()->activate();
  const int height = mapPopover_->layout()->heightForWidth(mapPopover_->width());
  mapPopover_->resize(mapPopover_->width(),
                      height > 0 ? height : mapPopover_->sizeHint().height());
}
void MissionController::resetHud(const QString &message,
                                 const QString &context) {
  if (!attitude_)
    return;
  attitude_->reset(message);
  attitude_->context = context;
  if (context.startsWith("历史"))
    attitude_->flightTime = attitude_->flightDistance = "—";
  else
    showFlightTotals();
  attitude_->update();
}
void MissionController::showFlightTotals() {
  if (!attitude_)
    return;
  attitude_->flightTime = flightTimeText(flightSeconds_);
  attitude_->flightDistance = flightDistanceText(flightDistanceM_);
}
void MissionController::updateFlightTotals(const mission::VehicleState &state) {
  if (bagReplayActive_ || !state.fcu_state_valid() || !state.armed() ||
      !state.pose_valid()) {
    flightSampleAge_.invalidate();
    return;
  }
  const QVector3D position = vec(state.position());
  if (flightSampleAge_.isValid()) {
    const qint64 elapsedMs = flightSampleAge_.elapsed();
    if (elapsedMs > 0 && elapsedMs <= 1000) {
      const double dt = elapsedMs / 1000.0;
      flightSeconds_ += dt;
      if (flightSampleFrame_ == QString::fromStdString(state.frame_id()) &&
          flightSampleMap_ == QString::fromStdString(state.map_id())) {
        const double step = double((position - flightSamplePosition_).length());
        if (step <= 40.0 * dt + 0.5)
          flightDistanceM_ += step;
      }
    }
  }
  flightSamplePosition_ = position;
  flightSampleFrame_ = QString::fromStdString(state.frame_id());
  flightSampleMap_ = QString::fromStdString(state.map_id());
  flightSampleAge_.start();
}
void MissionController::updateVehicle(const mission::VehicleState &s,
                                      bool countFlight) {
  if (countFlight)
    updateFlightTotals(s);
  lastVehicle_ = s;
  vehicleAge_.restart();
  if (s.pose_valid()) {
    const auto p = vec(s.position());
    if (history_.isEmpty() || (history_.last() - p).length() > .05f) {
      history_ << p;
      trailDirty_ = dirty_ = true;
    }
    if (history_.size() > 20000)
      history_.remove(0, 1000);
  }
  if (replayDialog_)
    return;
  attitude_->valid = s.pose_valid();
  QString position = "— / — / —", angles = "— / — / —", speed = "—";
  if (s.pose_valid()) {
    auto p = vec(s.position());
    auto &q = s.orientation();
    double roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                             1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    double pitch =
        std::asin(qBound(-1., 2 * (q.w() * q.y() - q.z() * q.x()), 1.));
    double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                            1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    attitude_->roll = roll * 180 / M_PI;
    attitude_->pitch = pitch * 180 / M_PI;
    angles = QString("%1°  /  %2°  /  %3°")
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
  const QString armed = s.fcu_state_valid()
                            ? (s.armed() ? "已解锁" : "未解锁") : "未知";
  const QString flightMode = s.fcu_state_valid() && !s.flight_mode().empty() &&
                                     s.flight_mode() != "UNKNOWN"
                                 ? QString::fromStdString(s.flight_mode()) : "未知";
  attitude_->armed = armed;
  attitude_->flightMode = flightMode;
  attitude_->angles = angles;
  attitude_->speed = speed;
  attitude_->position = position;
  attitude_->battery = s.battery_valid()
                           ? QString::number(s.battery_percent(), 'f', 0) + "%"
                           : "—";
  attitude_->notice.clear();
  attitude_->context = bagReplayActive_ ? "历史 bag 回放" : QString();
  if (bagReplayActive_) {
    attitude_->flightTime = attitude_->flightDistance = "—";
  } else {
    showFlightTotals();
  }
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
  live_ = group("实时彩色点云");
  map_ = group("普通累积 / 参考地图");
  colorMap_ = group("彩色累积点云");
  colorMap_->setEnabled(false);
  route_ = group("本地任务航线");
  remoteRoute_ = group("任务机航点");
  ego_ = group("EGO 规划");
  grid_ = group("EGO 栅格地图");
  trail_ = group("实际轨迹");
  aircraft_ = group("无人机");
  cameraFov_ = group("Odin1 RGB 相机视场");
  lidarFov_ = group("Odin1 激光雷达视场");
  imageProjection_ = group("Odin1 图像投影 · 点云贴图");
  aircraftInflation_ = group("飞机等效膨胀区");
  replay_ = group("飞行记录回放 · 历史数据");
  replay_->setVisible(false);
  auto replayLayer = [this](const QString &name) {
    auto *layer = new ccHObject(name);
    replay_->addChild(layer);
    return layer;
  };
  replayTrail_ = replayLayer("历史实际轨迹");
  replaySetpoints_ = replayLayer("历史指令位置");
  replayGrid_ = replayLayer("历史 EGO 栅格地图");
  replayCloud_ = replayLayer("历史 Odin 点云");
  replayAircraft_ = createQuadrotorModel();
  if (replayAircraft_) {
    replayAircraft_->setDisplay(gl_);
    replayAircraft_->setVisible(false);
    replay_->addChild(replayAircraft_);
  }
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
  const int savedGridSizePercent =
      s.value("gridSizePercent", gridSizePercent_).toInt();
  if (savedGridSizePercent >= 10 && savedGridSizePercent <= 100)
    gridSizePercent_ = savedGridSizePercent;
  const int savedNoiseKnn = s.value("noiseKnn", noiseKnn_).toInt();
  if (savedNoiseKnn >= 3 && savedNoiseKnn <= 256)
    noiseKnn_ = savedNoiseKnn;
  const double savedNoiseSigma = s.value("noiseSigma", noiseSigma_).toDouble();
  if (std::isfinite(savedNoiseSigma) && savedNoiseSigma >= 0.1 &&
      savedNoiseSigma <= 10.0)
    noiseSigma_ = savedNoiseSigma;
  const int savedNoisePointSize = s.value("noisePointSize", noisePointSize_).toInt();
  if (savedNoisePointSize >= 2 && savedNoisePointSize <= 12)
    noisePointSize_ = savedNoisePointSize;
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
  connect(&client_, &ProtocolClient::linkStatsChanged, this,
          &MissionController::refreshLinkStats);
  connect(&client_, &ProtocolClient::stateChanged, this,
          [this](const QString &t, bool r) {
            ready_ = r;
            connectionState_ = t;
            refreshLinkStats();
            if (r) {
              refreshOdinOverlayStatus();
              importRemoteOnReceive_ = mission_.waypoints_size() == 0;
              importRemoteAsUploaded_ = false;
              routeImportMissionRevision_ = mission_.revision();
              requestRemoteRoute();
              if (!videoUrl_.trimmed().isEmpty())
                showVideo();
            } else
              clearRemoteRoute();
            mode_->setText(replayDialog_
                               ? "◷ 飞行记录回放 · 历史数据 · " + t
                               : bagReplayActive_ ? "◷ bag 回放 · 历史数据 · " + t
                               : (demo_ ? "● 模拟演示 · 非真实遥测"
                                        : (r ? "● " + t : "○ " + t)));
            connectButton_->setText(r ? "断开" : "连接");
            if (!r) {
              odinModesSupported_ = false;
              taskModeSupported_ = false;
              taskMode_ = mission::MODE_UNKNOWN;
              modeSwitchRequest_.clear();
              refreshTaskModeControls();
              odinMapsRequest_.clear();
              if (odinMapList_)
                odinMapList_->clear();
              if (odinMapHint_)
                odinMapHint_->setText("任务机连接已断开；请重新连接并刷新地图。");
              refreshOdinStartDialog();
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
              clearOdinOverlays();
              dirty_ = true;
              vehicleAge_.invalidate();
              flightSampleAge_.invalidate();
              if (!replayDialog_) {
                resetHud("等待遥测");
              }
              setStatusCells({"任务机 离线", "Odin1 —", "EGO —", "定位 —",
                              "点云 —", "网络 —", "CPU —", "磁盘 —",
                              "视频 —", "记录 —"});
              videoRetry_.stop();
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
    if (accumulationDirty_ || colorAccumulationDirty_)
      saveAccumulatedMap();
  });
  autosaveTimer_.start(30000);
  connect(&staleTimer_, &QTimer::timeout, this, [this] {
    if (!gl_)
      return;
    refreshStartupButtons();
    refreshTaskModeControls();
    if (!demo_ && statusAge_.isValid() && statusAge_.elapsed() > 3000)
      setStatusCells({"任务机 状态已过期", "Odin1 —", "EGO —", "定位 —",
                      "点云 —", "网络 —", "CPU —", "磁盘 —",
                      "视频 —", "记录 —"});
    if (download_ && downloadAge_.isValid() && downloadAge_.elapsed() > 30000) {
      download_.reset();
      log("文件下载超时，未完成文件已取消");
    }
    if (!demo_ && bagReplayState_ != mission::ReplayStatus::PAUSED &&
        bagReplayState_ != mission::ReplayStatus::ENDED &&
        vehicleAge_.isValid() && vehicleAge_.elapsed() > 3000) {
      flightSampleAge_.invalidate();
      if (!replayDialog_) {
        resetHud("遥测已过期（超过 3 秒）");
      }
      if (aircraftModel_)
        aircraftModel_->setVisible(false);
      lastVehicle_.set_pose_valid(false);
      gl_->redraw();
    }
    if (!demo_ && plannerAge_.isValid() && plannerAge_.elapsed() > 3000) {
      ego_->setVisible(false);
      gl_->redraw();
    }
    if (!demo_ && gridAge_.isValid() && gridAge_.elapsed() > kGridStaleMs)
      expireGrid();
    if (!demo_ && sensorAge_.isValid() && sensorAge_.elapsed() > 1500) {
      sensorAge_.invalidate();
      lastSensorPose_.Clear();
      cameraFov_->removeAllChildren();
      lidarFov_->removeAllChildren();
      aircraftInflation_->removeAllChildren();
      refreshOdinOverlayStatus();
      gl_->redraw();
    }
    if (!demo_ && projectionAge_.isValid() &&
        projectionAge_.elapsed() > 3000) {
      projectionPoints_.clear();
      imageProjection_->removeAllChildren();
      projectionAge_.invalidate();
      refreshOdinOverlayStatus();
      gl_->redraw();
    }
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
  if (!mapPoints_.isEmpty() || !colorMapPoints_.isEmpty()) {
    updateScene();
    gl_->zoomGlobal();
  }
  log("3DG 已就绪。可加载地图、编辑航线，或开启模拟演示。");
  if (!aircraftModel_)
    log("X500 模型资源加载失败，无法显示无人机模型");
  if (qEnvironmentVariableIsSet("THREEDG_DEMO"))
    setDemo(true);
  else {
    client_.open(QUrl(baseUrl_));
    log("正在连接任务机：" + baseUrl_, "连接");
  }
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_VIDEO")) {
    videoUrl_ = qEnvironmentVariable("THREEDG_SMOKE_VIDEO");
    showVideo();
  } else if (!videoUrl_.isEmpty())
    showVideo();
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_DIR"))
    QTimer::singleShot(4500, this, &MissionController::smoke);
}
MissionController::~MissionController() {
  if (accumulationDirty_ || colorAccumulationDirty_)
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
void MissionController::placeLatestLogButton() {
  if (!latest_ || !settingsMenuAction_)
    return;
  auto *bar = window_->menuBar();
  const QRect settings = bar->actionGeometry(settingsMenuAction_);
  if (!settings.isValid())
    return;
  const int x = settings.right() + 8;
  const int width = qMin(350, qMax(0, bar->width() - x - 8));
  latest_->setVisible(width >= 90);
  if (width < 90)
    return;
  latest_->setGeometry(x, (bar->height() - latest_->height()) / 2, width,
                       latest_->height());
  latest_->raise();
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
      "QFrame#hudCard{background:transparent;border:0;border-radius:0;}"
      "QFrame#videoOverlay{background:#081116;border:0;border-radius:0;}"
      "QLabel{color:#304756;background:transparent;}"
      "QLabel#cardTitle{font-size:13px;font-weight:650;color:#263d4c;}"
      "QLabel#statusCell{font-size:12px;color:#304756;}"
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
      "QPushButton[compactButton=true]{padding:3px 7px;min-height:20px;}"
      "QPushButton[role=toolbar]{background:transparent;border:0;padding:5px "
      "7px;}"
      "QPushButton[role=toolbar]:hover{background:rgba(199,231,237,190);border-"
      "radius:6px;}"
      "QPushButton[role=toolbar]:checked{background:#d0f1e9;color:#147b68;}"
      "QPushButton#openCloudTools{background:#e1f5f0;color:#146d61;"
      "border:1px solid #9bd6c8;border-radius:7px;font-weight:600;"
      "padding:4px 10px;}"
      "QPushButton#openCloudTools:hover{background:#c8eee4;border-color:#58bda8;}"
      "QPushButton#openCloudTools:pressed{background:#ace0d2;}"
      "QPushButton#cloudActionButton{background:#f8fbfd;color:#315062;"
      "border:1px solid #c9dce5;border-radius:8px;text-align:left;"
      "padding:3px 8px;min-height:20px;font-size:13px;}"
      "QPushButton#cloudActionButton:hover{background:#e5f5f3;"
      "border-color:#70c8b7;color:#176f63;}"
      "QPushButton#cloudActionButton:pressed{background:#ceeae4;}"
      "QPushButton#cloudPrimaryButton{background:#168778;color:white;"
      "border:1px solid #117467;border-radius:8px;font-weight:600;"
      "text-align:left;padding:3px 8px;min-height:20px;font-size:13px;}"
      "QPushButton#cloudPrimaryButton:hover{background:#0e7569;border-color:#0c655a;}"
      "QPushButton#cloudPrimaryButton:pressed{background:#095b52;}"
      "QPushButton#cloudActionButton:disabled,QPushButton#cloudPrimaryButton:disabled{"
      "background:#e6edf0;color:#8397a2;border-color:#d2dfe5;}"
      "QTabWidget#missionToolsTabs::pane{border:1px solid #c9dce5;"
      "border-radius:8px;background:rgba(255,255,255,145);padding:4px;}"
      "QTabWidget#missionToolsTabs QTabBar::tab{background:#e9f1f5;"
      "color:#5b7280;border:1px solid #c9dce5;border-bottom:0;"
      "border-top-left-radius:7px;border-top-right-radius:7px;"
      "padding:4px 12px;margin-right:3px;}"
      "QTabWidget#missionToolsTabs QTabBar::tab:hover{background:#dff2ed;"
      "color:#176f63;}"
      "QTabWidget#missionToolsTabs QTabBar::tab:selected{background:#f9fcfd;"
      "color:#146f62;border-color:#8dcdbf;font-weight:600;}"
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
      "QCheckBox#sensorToggle{padding:1px 2px;spacing:5px;}"
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
      "QToolBar#bagReplayDock{background:transparent;border:0;padding:2px 0 3px 0;}"
      "QFrame#bagReplayToolbar{"
      "background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
      "stop:0 rgba(250,253,254,249),stop:1 rgba(229,241,246,246));"
      "border:1px solid #c7dce4;border-radius:9px;}"
      "QLabel#replayTimeLabel{background:transparent;"
      "color:#315866;border:0;}"
      "QToolButton#replayPlaybackButton{background:transparent;"
      "color:#176f63;border:0;border-radius:6px;padding:0;}"
      "QToolButton#replayPlaybackButton:hover{background:#d7efeb;}"
      "QToolButton#replayPlaybackButton:pressed{background:#c8eee4;}"
      "QToolButton#replayPlaybackButton::menu-button{width:16px;"
      "border:0;background:transparent;}"
      "QToolButton#replayPlaybackButton:disabled{background:transparent;"
      "color:#75a19a;border:0;}"
      "QToolButton#replayPlaybackButton:disabled::menu-button{"
      "background:transparent;border:0;}"
      "QToolButton#replaySpeedButton{background:transparent;"
      "color:#315062;border:0;border-radius:6px;padding:0;"
      "font-weight:700;}"
      "QToolButton#replaySpeedButton:hover{background:#e5f5f3;}"
      "QToolButton#replaySpeedButton:disabled{background:transparent;"
      "color:#8397a2;border:0;}"
      "QPushButton#bagReplayStart{background:#e1f5f0;color:#146d61;"
      "border:1px solid #9bd6c8;border-radius:7px;font-weight:600;}"
      "QPushButton#bagReplayStart:hover{background:#c8eee4;border-color:#58bda8;}"
      "QPushButton#bagReplayStart:pressed{background:#ace0d2;}"
      "QPushButton#bagReplayStart:disabled{background:#e6edf0;color:#8397a2;"
      "border-color:#d2dfe5;}"
      "QDialog{background:#edf5f9;color:#304756;}"
      "QProgressBar{background:#d7e6eb;border:0;border-radius:3px;min-height:"
      "6px;"
      "max-height:6px;text-align:center;}"
      "QProgressBar::chunk{background:#54cbb3;border-radius:3px;}";
  window_->setStyleSheet(css);
  auto *replayDock = new QToolBar(window_);
  replayDock->setObjectName("bagReplayDock");
  replayDock->setWindowTitle("bag 回放");
  replayDock->setMovable(false);
  replayDock->setFloatable(false);
  replayDock->setAllowedAreas(Qt::BottomToolBarArea);
  replayDock->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  auto *replayHolder = new QWidget(replayDock);
  replayHolder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  auto *replayLayout = new QHBoxLayout(replayHolder);
  replayLayout->setContentsMargins(0, 0, 0, 0);
  bagReplayToolbar_ = new BagReplayToolbar(&client_, replayHolder);
  replayLayout->addWidget(bagReplayToolbar_, 1);
  replayDock->addWidget(replayHolder);
  window_->addToolBar(Qt::BottomToolBarArea, replayDock);
  window_->installEventFilter(this);
  auto *bar = window_->menuBar();
  bar->setFixedHeight(34);
  for (auto *a : bar->actions())
    a->setVisible(false);
  auto *missionMenu = bar->addMenu("3DG Mission ▾");
  missionMenu->setObjectName("missionMainMenu");
  auto *showObjects = missionMenu->addAction("显示对象列表");
  showObjects->setCheckable(true);
  showObjects->setChecked(true);
  connect(showObjects, &QAction::toggled, this,
          [this](bool visible) { objectsButton_->setChecked(visible); });
  auto *showTools = missionMenu->addAction("显示点云工具");
  showTools->setCheckable(true);
  showTools->setChecked(true);
  connect(showTools, &QAction::toggled, this,
          [this](bool visible) {
            processingCard_->setVisible(visible);
            if (visible)
              detailsScroll_->verticalScrollBar()->setValue(processingCard_->y());
          });
  missionMenu->addSeparator();
  missionMenu->addAction("使用当前点云", this,
                         [this] { selectCurrentCloud(); });
  missionMenu->addAction("创建当前地图快照", this,
                         &MissionController::snapshotMap);
  auto *file = bar->addMenu("文件");
  file->addAction("加载 PCD / PLY 地图", this, &MissionController::loadMap);
  file->addAction("导入文件（处理对象）…", this,
                  &MissionController::importProcessingFiles);
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
  processMenu->addAction("一键生成数模（泊松）", this, &MissionController::quickReconstruct);
  addProcessingAction(processMenu, "栅格化…", "actionRasterize");
  addProcessingAction(processMenu, "ICP 配准…", "actionRegister");
  addProcessingAction(processMenu, "对应点配准…", "actionPointPairsAlign");
  auto *analysisMenu = bar->addMenu("分析");
  addProcessingAction(analysisMenu, "点云到点云距离…", "actionCloudCloudDist");
  addProcessingAction(analysisMenu, "点云到网格距离…", "actionCloudMeshDist");
  auto *toolsMenu = bar->addMenu("工具");
  toolsMenu->setObjectName("missionToolsMenu");
  auto *detectNoiseAction = toolsMenu->addAction("自动筛选点云噪点", this,
                                                &MissionController::detectCloudNoise);
  detectNoiseAction->setToolTip("使用下方设置的 SOR 参数检测当前点云的离群点");
  toolsMenu->addAction("噪点检测参数…", this,
                       &MissionController::configureNoiseDetection);
  toolsMenu->addSeparator();
  noiseOverlayAction_ = toolsMenu->addAction("显示原图和噪点（红色）", this,
                                             [this] { showCloudNoise(false); });
  noiseOnlyAction_ = toolsMenu->addAction("只显示噪点", this,
                                          [this] { showCloudNoise(true); });
  noiseRestoreAction_ = toolsMenu->addAction("恢复原显示", this,
                                             &MissionController::restoreCloudNoiseView);
  for (auto *action : {noiseOverlayAction_, noiseOnlyAction_, noiseRestoreAction_})
    action->setEnabled(false);
  auto *taskMenu = bar->addMenu("任务机");
  auto *modeMenu = taskMenu->addMenu("运行模式");
  taskModeStatus_ = modeMenu->addAction("模式状态：等待任务机");
  taskModeStatus_->setEnabled(false);
  modeMenu->addSeparator();
  auto *modeGroup = new QActionGroup(modeMenu);
  modeGroup->setExclusive(true);
  realModeAction_ = modeMenu->addAction("实物模式");
  simModeAction_ = modeMenu->addAction("仿真模式");
  for (auto *action : {realModeAction_, simModeAction_}) {
    action->setCheckable(true);
    modeGroup->addAction(action);
  }
  connect(realModeAction_, &QAction::triggered, this,
          [this] { switchTaskMode(mission::REAL); });
  connect(simModeAction_, &QAction::triggered, this,
          [this] { switchTaskMode(mission::SIMULATION); });
  realModeAction_->setToolTip("选择任务机实物流程；切换不自动启动设备");
  simModeAction_->setToolTip("选择任务机仿真流程；仿真主机和 simproxy 需按顺序单独启动");
  refreshTaskModeControls();
  taskMenu->addSeparator();
  taskMenu->addAction("任务机 YAML", this, &MissionController::yamlSettings);
  taskMenu->addAction("保存机载地图", this,
                      [this] { command(mission::Command::SAVE_MAP); });
  const auto openFlightReplay = [this](const QString &recordId) {
    if (bagReplayActive_) {
      log("请先停止 bag 回放，再打开飞行记录回放");
      return;
    }
    if (replayDialog_) {
      if (!recordId.isEmpty())
        replayDialog_->selectRemoteRecord(recordId);
      replayDialog_->raise();
      replayDialog_->activateWindow();
      return;
    }
    for (auto *layer : {live_, map_, colorMap_, route_, remoteRoute_, ego_, grid_, trail_, aircraft_}) {
      replayVisibility_.insert(layer, layer->isEnabled());
      layer->setEnabled(false);
    }
    if (gl_)
      gl_->redraw();
    mode_->setText("◷ 飞行记录回放 · 历史数据 · " + connectionState_);
    resetHud("等待记录", "历史回放");
    if (demoAction_)
      demoAction_->setEnabled(false);
    auto *dialog = new FlightReplayDialog(QUrl(baseUrl_),
        [this](const FlightReplayFrame &frame) { renderFlightReplay(frame); },
        window_, recordId);
    replayDialog_ = dialog;
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    connect(dialog, &QDialog::finished, this, [this] {
      clearFlightReplay();
      replayDialog_.clear();
      if (vehicleAge_.isValid() && vehicleAge_.elapsed() <= 3000)
        updateVehicle(lastVehicle_, false);
      else {
        resetHud("等待遥测");
      }
      mode_->setText(demo_ ? "● 模拟演示 · 非真实遥测"
                           : (ready_ ? "● " + connectionState_
                                     : "○ " + connectionState_));
      if (demoAction_)
        demoAction_->setEnabled(true);
    });
    dialog->show();
  };
  taskMenu->addAction("数据管理…", this, [this, openFlightReplay] {
    OdinDataManagerDialog dialog(QUrl(baseUrl_), window_,
                                 OdinDataManagerDialog::InitialAction::None, &client_);
    dialog.exec();
    if (!dialog.selectedFlightReplayId().isEmpty())
      openFlightReplay(dialog.selectedFlightReplayId());
  });
  taskMenu->addAction("飞行记录回放…", this,
                      [openFlightReplay] { openFlightReplay({}); });
  auto *recordAction = taskMenu->addAction("正在查询录制状态…");
  recordAction->setEnabled(false);
  auto *recordNetwork = new QNetworkAccessManager(taskMenu);
  recordNetwork->setProxy(QNetworkProxy(QNetworkProxy::NoProxy));
  auto recordStatusReply = std::make_shared<QPointer<QNetworkReply>>();
  connect(taskMenu, &QMenu::aboutToShow, this,
          [this, recordAction, recordNetwork, recordStatusReply] {
    if (*recordStatusReply) {
      auto *previous = recordStatusReply->data();
      recordStatusReply->clear();
      previous->abort();
    }
    recordAction->setText("正在查询录制状态…");
    recordAction->setEnabled(false);
    const QUrl taskSocket(baseUrl_);
    if (taskSocket.host().isEmpty()) {
      recordAction->setText("录制状态不可用");
      return;
    }
    QUrl url;
    url.setScheme("http");
    url.setHost(taskSocket.host());
    url.setPort(8000);
    url.setPath("/api/odin/record/status");
    auto *reply = recordNetwork->get(QNetworkRequest(url));
    *recordStatusReply = reply;
    QTimer::singleShot(8000, reply, [reply] {
      if (reply->isRunning())
        reply->abort();
    });
    connect(reply, &QNetworkReply::finished, this,
            [recordAction, recordStatusReply, reply] {
      if (*recordStatusReply != reply) {
        reply->deleteLater();
        return;
      }
      recordStatusReply->clear();
      const auto status = QJsonDocument::fromJson(reply->readAll()).object();
      if (reply->error() == QNetworkReply::NoError &&
          reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() == 200 &&
          status.contains("recording_active") && status.contains("driver_running")) {
        const bool active = status.value("recording_active").toBool();
        recordAction->setText(active ? "停止录制…" : "开始录制…");
        recordAction->setEnabled(active || status.value("driver_running").toBool());
        recordAction->setData(active);
      } else {
        recordAction->setText("录制状态不可用");
        recordAction->setEnabled(false);
      }
      reply->deleteLater();
    });
  });
  connect(recordAction, &QAction::triggered, this, [this, recordAction] {
    OdinDataManagerDialog dialog(QUrl(baseUrl_), window_,
        recordAction->data().toBool()
            ? OdinDataManagerDialog::InitialAction::Stop
            : OdinDataManagerDialog::InitialAction::Start, &client_);
    dialog.exec();
  });
  auto *startup = taskMenu->addMenu("任务启动");
  startup->addAction("启动 Odin1…", this,
                     &MissionController::openOdinStart);
  auto *startEgo = startup->addAction("启动 EGO", this,
                                     [this] { command(mission::Command::START_EGO); });
  startEgo->setToolTip("确认本地或机载航线后启动 EGO 及实物航线主控；保持上锁，主控就绪后手动解锁");
  auto *view = bar->addMenu("视图");
  view->addAction("等轴测", this, [this] { gl_->setView(CC_ISO_VIEW_1); });
  view->addAction("俯视", this, [this] { gl_->setView(CC_TOP_VIEW); });
  view->addAction("定位无人机", this, &MissionController::focusAircraft);
  view->addAction("悬浮视频窗口", this, &MissionController::showVideo);
  auto *display = bar->addMenu("显示");
  display->addSection("界面卡片");
  auto *demo = view->addAction("模拟演示（非真实遥测）");
  demoAction_ = demo;
  demo->setCheckable(true);
  connect(demo, &QAction::toggled, this, &MissionController::setDemo);
  auto *settingsMenu = bar->addMenu("设置");
  settingsMenu->addAction("3DG 配置", this, &MissionController::settings);
  settingsMenuAction_ = settingsMenu->menuAction();
  latest_ = new QPushButton("● 3DG 已就绪 ▾", bar);
  latest_->setFixedHeight(26);
  latest_->setObjectName("latestLog");
  bar->installEventFilter(this);
  QTimer::singleShot(0, this, &MissionController::placeLatestLogButton);
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
  logRecordButton_ = button(logActions, "bag 记录", [this] {
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
  auto *clearLogs = button(logActions, "清除", [this] {
    logList_->clear();
    unread_ = 0;
    logCount_->setText("自动跟随最新日志");
    latest_->setText("● 3DG 日志已清除 ▾");
    latest_->setToolTip("当前显示的日志已清除");
  });
  clearLogs->setToolTip("仅清空当前显示的日志，保留日志文件");
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
  addTool("＋ 航点", [this] { createWaypoint(); });
  picking_ = new QCheckBox("点云拾取");
  tools->addWidget(picking_);
  addTool("适配", [this] { gl_->zoomGlobal(); });
  auto *cloudToolsButton = addTool("点云工具", [this, showTools] {
    showTools->setChecked(true);
    detailsScroll_->verticalScrollBar()->setValue(processingCard_->y());
  });
  cloudToolsButton->setObjectName("openCloudTools");
  cloudToolsButton->setToolTip("定位到右侧点云处理与对比工具");
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
      {"☁", "实时彩色点云", live_},
      {"▧", "普通累积 / 参考地图", map_},
      {"◈", "彩色累积点云", colorMap_},
      {"⌁", "本地任务航线", route_},
      {"◇", "任务机航点", remoteRoute_},
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
    tool->setChecked(std::get<2>(entry)->isEnabled());
    tool->setFixedSize(42, 42);
    if (std::get<2>(entry) == map_)
      ordinaryMapButton_ = tool;
    if (std::get<2>(entry) == colorMap_)
      colorMapButton_ = tool;
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
              if (replayDialog_)
                replayVisibility_[g] = visible;
              else
                g->setEnabled(visible);
              gl_->redraw();
            });
  }
  // Both accumulated layers use the same XYZ. Showing one at a time prevents
  // coincident points in the first layer from hiding the other layer's colors.
  connect(ordinaryMapButton_, &QToolButton::toggled, this, [this](bool on) {
    if (on)
      colorMapButton_->setChecked(false);
  });
  connect(colorMapButton_, &QToolButton::toggled, this, [this](bool on) {
    if (on)
      ordinaryMapButton_->setChecked(false);
  });
  auto *divider = new QFrame(left_);
  divider->setFrameShape(QFrame::HLine);
  l->addWidget(divider);
  objectsButton_ = new QToolButton(left_);
  objectsButton_->setObjectName("layerTool");
  objectsButton_->setText("≡");
  objectsButton_->setToolTip("显示或隐藏点云对象列表");
  objectsButton_->setAccessibleName("点云对象列表");
  objectsButton_->setCheckable(true);
  objectsButton_->setChecked(true);
  objectsButton_->setFixedSize(42, 42);
  l->addWidget(objectsButton_);
  connect(objectsButton_, &QToolButton::toggled, this,
          [this, showObjects](bool visible) {
            objectsCard_->setVisible(visible);
            showObjects->setChecked(visible);
            if (visible)
              objectsCard_->raise();
          });
  heightFilterButton_ = new QToolButton(left_);
  heightFilterButton_->setObjectName("layerTool");
  heightFilterButton_->setText("↕");
  heightFilterButton_->setToolTip("显示或隐藏累积点云高度过滤器");
  heightFilterButton_->setAccessibleName("累积点云高度过滤器");
  heightFilterButton_->setCheckable(true);
  heightFilterButton_->setFixedSize(42, 42);
  l->addWidget(heightFilterButton_);
  heightFilterPanel_ = card(canvas_, "");
  auto *heightBox = box(heightFilterPanel_);
  heightBox->setContentsMargins(7, 8, 7, 8);
  heightBox->setSpacing(5);
  heightFilterTitle_ = new QLabel("高度过滤", heightFilterPanel_);
  heightFilterTitle_->setObjectName("cardTitle");
  heightFilterTitle_->setAlignment(Qt::AlignHCenter);
  heightFilterTitle_->setMinimumHeight(25);
  heightFilterTitle_->setToolTip("拖动标题可移动累积点云高度过滤器");
  heightFilterTitle_->setCursor(Qt::OpenHandCursor);
  heightFilterTitle_->installEventFilter(this);
  heightBox->addWidget(heightFilterTitle_);
  heightFilterEnabled_ = new QCheckBox("启用", heightFilterPanel_);
  heightFilterEnabled_->setToolTip("启用或关闭累积点云显示高度过滤");
  heightFilterEnabled_->setChecked(true);
  heightBox->addWidget(heightFilterEnabled_);
  heightFilterLabel_ = new QLabel("等点云", heightFilterPanel_);
  heightFilterLabel_->setObjectName("mutedText");
  heightFilterLabel_->setWordWrap(true);
  heightFilterLabel_->setAlignment(Qt::AlignHCenter);
  heightBox->addWidget(heightFilterLabel_);
  heightRangeTop_ = new QLabel("—", heightFilterPanel_);
  heightRangeTop_->setObjectName("mutedText");
  heightRangeTop_->setAlignment(Qt::AlignHCenter);
  heightRangeTop_->setToolTip("累积点云最高 Z 高度");
  heightBox->addWidget(heightRangeTop_);
  heightSlider_ = new QSlider(Qt::Vertical, heightFilterPanel_);
  heightSlider_->setObjectName("accumulatedHeightSlider");
  heightSlider_->setRange(0, 1000);
  heightSlider_->setValue(1000);
  heightSlider_->setFixedWidth(80);
  heightSlider_->setMinimumHeight(160);
  heightSlider_->setEnabled(false);
  heightSlider_->setToolTip("向上拖动提高 Z 高度上限，向下拖动降低；到顶显示全部累积点云");
  heightSlider_->setStyleSheet(
      "QSlider::groove:vertical{width:76px;background:#dce9ed;"
      "border:1px solid #b8ced7;border-radius:8px;}"
      "QSlider::sub-page:vertical{background:#65cbbd;border-radius:8px;}"
      "QSlider::handle:vertical{height:13px;margin:0 -2px;background:#fff;"
      "border:1px solid #819eaa;border-radius:6px;}"
      "QSlider:disabled::groove:vertical{background:#e9eff1;}");
  heightBox->addWidget(heightSlider_, 1, Qt::AlignHCenter);
  heightRangeBottom_ = new QLabel("—", heightFilterPanel_);
  heightRangeBottom_->setObjectName("mutedText");
  heightRangeBottom_->setAlignment(Qt::AlignHCenter);
  heightRangeBottom_->setToolTip("累积点云最低 Z 高度");
  heightBox->addWidget(heightRangeBottom_);
  heightFilterPanel_->hide();
  connect(heightFilterButton_, &QToolButton::toggled, this, [this](bool shown) {
    heightFilterPanel_->setVisible(shown);
    if (shown)
      heightFilterPanel_->raise();
  });
  connect(heightFilterEnabled_, &QCheckBox::toggled, this, [this](bool) {
    heightSlider_->setEnabled(heightFilterEnabled_->isChecked() &&
                              heightFilterHasRange_);
    refreshHeightFilterLabel();
    mapDirty_ = colorMapDirty_ = dirty_ = true;
    updateScene();
  });
  connect(heightSlider_, &QSlider::valueChanged, this, [this](int value) {
    if (!heightFilterHasRange_)
      return;
    heightFilterCutoff_ = heightFilterMin_ +
        (heightFilterMax_ - heightFilterMin_) * value / heightSlider_->maximum();
    refreshHeightFilterLabel();
    mapDirty_ = colorMapDirty_ = dirty_ = true;
  });
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
      QString("坐标系  %1\n地图 ID  %2\n普通累积点  0\n彩色累积点  0")
          .arg(frame_, mapId_),
      mapPopover_);
  mapInfo_->setObjectName("mutedText");
  mapInfo_->setWordWrap(true);
  mapBox->addWidget(mapInfo_);
  mode_ = new QLabel("● 离线 · 等待任务机", mapPopover_);
  mode_->setObjectName("mutedText");
  mode_->setWordWrap(true);
  mapBox->addWidget(mode_);
  bitError_ = new QLabel("通讯误码率  —（未连接）", mapPopover_);
  bitError_->setObjectName("mutedText");
  bitError_->setWordWrap(true);
  bitError_->setToolTip("当前 WebSocket/TCP 链路不提供物理比特错误计数，无法计算比特误码率。");
  mapBox->addWidget(bitError_);
  protocolError_ = new QLabel("协议消息异常率  —", mapPopover_);
  protocolError_->setObjectName("mutedText");
  protocolError_->setWordWrap(true);
  protocolError_->setToolTip("本次连接收到的异常消息数 / 收到的消息总数；包括解析失败、协议校验失败、通道或会话不匹配、重复序号。不是比特误码率。");
  mapBox->addWidget(protocolError_);
  networkDown_ = new QLabel("网络下行  —", mapPopover_);
  networkUp_ = new QLabel("网络上行  —", mapPopover_);
  for (QLabel *label : {networkDown_, networkUp_}) {
    label->setObjectName("mutedText");
    label->setToolTip("近 1 秒 3DG WebSocket 协议消息净荷速率；不含 TCP/WebSocket 开销和 RTSP 视频流。");
    mapBox->addWidget(label);
  }
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
  edit->setContentsMargins(8, 4, 8, 4);
  routeStrip_ = new QWidget(editor_);
  routeStrip_->installEventFilter(this);
  routeStrip_->setFocusPolicy(Qt::StrongFocus);
  routeStrip_->setAccessibleName("航点任务，双击或按回车打开");
  routeStrip_->setToolTip("双击打开航点任务");
  auto *strip = new QHBoxLayout(routeStrip_);
  strip->setContentsMargins(0, 0, 0, 0);
  strip->setSpacing(3);
  routeStrip_->setFixedHeight(28);
  edit->addWidget(routeStrip_);
  routeDialog_ = new QDialog(window_);
  routeDialog_->setWindowTitle("航点任务 · 编辑与同步");
  routeDialog_->setModal(false);
  routeDialog_->resize(980, 510);
  routeDialog_->setMinimumSize(680, 360);
  auto *popup = new QVBoxLayout(routeDialog_);
  popup->setContentsMargins(12, 10, 12, 12);
  routeDetails_ = new QWidget(routeDialog_);
  auto *details = new QVBoxLayout(routeDetails_);
  details->setContentsMargins(0, 0, 0, 0);
  details->setSpacing(6);
  popup->addWidget(routeDetails_);
  connect(routeDialog_, &QDialog::finished, this,
          [this] { setEditorCollapsed(true); });
  auto *instruction = new QLabel("双击航点编辑；从任务机载入会替换当前编辑内容，上传后任务机使用这条航线", routeDetails_);
  instruction->setObjectName("mutedText");
  details->addWidget(instruction);
  auto *routeOptions = new QHBoxLayout;
  details->addLayout(routeOptions);
  follow_ = new QCheckBox("跟随飞机");
  routeOptions->addWidget(follow_);
  remoteRefreshButton_ = button(routeOptions, "从任务机载入", [this] {
    loadOnboardRoute();
  });
  remoteRefreshButton_->setToolTip("重新读取任务机当前 EGO 航线，载入同一编辑列表");
  remoteRefreshButton_->setEnabled(false);
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
  routeStats_ = new QLabel("坐标系 — · 地图 — · 全局巡航速度 — · 航线长度 —", routeDetails_);
  routeStats_->setObjectName("mutedText");
  details->addWidget(routeStats_);
  waypoints_ = new QTableWidget(0, 13, routeDetails_);
  waypoints_->setHorizontalHeaderLabels(
      {"状态", "序号", "X m", "Y m", "Z m", "航向 °", "速度 m/s",
       "类型", "停留 s", "云台 °", "变焦 ×", "本段 m", "累计 m"});
  auto *routeHeader = waypoints_->horizontalHeader();
  routeHeader->setSectionResizeMode(QHeaderView::Interactive);
  routeHeader->setMinimumSectionSize(44);
  routeHeader->setSectionsMovable(false);
  routeHeader->setStretchLastSection(true);
  const int widths[] = {90, 50, 58, 58, 58, 70, 82, 72, 68, 70, 68, 70, 72};
  for (int column = 0; column < waypoints_->columnCount(); ++column)
    routeHeader->resizeSection(column, widths[column]);
  waypoints_->horizontalHeaderItem(6)->setToolTip(
      "控制器使用全局巡航速度；修改后应用到全部航点");
  waypoints_->horizontalHeaderItem(11)->setToolTip("从上一个航点到此航点的三维直线距离");
  waypoints_->horizontalHeaderItem(12)->setToolTip("从首个航点累计的三维直线距离");
  waypoints_->verticalHeader()->hide();
  waypoints_->verticalHeader()->setDefaultSectionSize(27);
  waypoints_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  waypoints_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  waypoints_->setAlternatingRowColors(true);
  waypoints_->setSortingEnabled(false);
  waypoints_->setSelectionBehavior(QAbstractItemView::SelectRows);
  waypoints_->setSelectionMode(QAbstractItemView::SingleSelection);
  waypoints_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  details->addWidget(waypoints_, 1);
  remoteRouteInfo_ = new QLabel("任务机航点尚未读取", routeDetails_);
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
            refreshRemoteEditControls();
          });
  connect(waypoints_, &QTableWidget::cellDoubleClicked, this,
          [this](int row, int) {
            waypoints_->selectRow(row);
            editWaypoint();
          });
  auto *row = new QHBoxLayout;
  details->addLayout(row);
  button(row, "添加航点", [this] { createWaypoint(); });
  button(row, "删除航点", [this] {
    int n = waypoints_->currentRow();
    if (n >= 0 && n < mission_.waypoints_size()) {
      mission_.mutable_waypoints()->DeleteSubrange(n, 1);
      rebuildRoute();
    }
  });
  button(row, "航点上移", [this] {
    int n = waypoints_->currentRow();
    if (n > 0 && n < mission_.waypoints_size()) {
      if (mission_.waypoints(n).pointmode() == "Land_point") {
        log("降落点只能是最后一个航点，不能上移");
        return;
      }
      mission_.mutable_waypoints()->SwapElements(n, n - 1);
      rebuildRoute();
      waypoints_->selectRow(n - 1);
    }
  });
  auto *routeActions = new QHBoxLayout;
  details->addLayout(routeActions);
  button(routeActions, "保存本地文件", [this] { saveMission(); });
  auto *uploadRoute = button(routeActions, "上传到任务机", [this] {
    command(mission::Command::UPLOAD_MISSION);
  });
  uploadRoute->setToolTip("上传当前编辑航线到任务机；至少需要两个航点，不会启动任务");
  button(details, "清空飞机轨迹", [this] {
    history_.clear();
    trailDirty_ = dirty_ = true;
  });
  routeDialog_->hide();
  editor_->show();
  refreshRemoteEditControls();
  refreshRouteStrip();
  right_ = new QFrame(canvas_);
  auto *r = new QVBoxLayout(right_);
  r->setContentsMargins(0, 0, 0, 0);
  r->setSpacing(10);
  auto *h = card(right_, "");
  h->setObjectName("hudCard");
  box(h)->setContentsMargins(0, 0, 0, 0);
  box(h)->setSpacing(0);
  r->addWidget(h);
  attitude_ = new AttitudeWidget(h);
  attitude_->reset("等待遥测");
  box(h)->addWidget(attitude_);
  auto *sensorCard = card(right_, "Odin1 视场与图像");
  box(sensorCard)->setContentsMargins(9, 7, 9, 7);
  box(sensorCard)->setSpacing(1);
  auto addSensorToggle = [this, sensorCard](const QString &label,
                                             const QString &hint,
                                             const QString &key,
                                             ccHObject *layer,
                                             const OverlayColor *lineColor) {
    auto *toggle = new QCheckBox(label, sensorCard);
    toggle->setObjectName("sensorToggle");
    if (lineColor) {
      const QString color = QColor(lineColor->red, lineColor->green,
                                   lineColor->blue).name();
      toggle->setStyleSheet(QString("QCheckBox::indicator:checked{background:%1;"
                                    "border-color:%1;}").arg(color));
    }
    toggle->setChecked(QSettings("3DG", "Mission").value(key, true).toBool());
    toggle->setToolTip(hint);
    layer->setEnabled(toggle->isChecked());
    box(sensorCard)->addWidget(toggle);
    connect(toggle, &QCheckBox::toggled, this, [this, key, layer](bool on) {
      layer->setEnabled(on);
      QSettings("3DG", "Mission").setValue(key, on);
      if (gl_)
        gl_->redraw();
    });
    return toggle;
  };
  cameraFovToggle_ = addSensorToggle("相机 129°×104°",
      "Odin1 标称视场；三维位姿来自雷达 TF 和 Tcl_0 标定逆矩阵",
      "odinCameraFov", cameraFov_, &kCameraFovColor);
  lidarFovToggle_ = addSensorToggle("雷达 120°×90°",
      "Odin1 标称视场；三维位姿来自任务机 lidar TF", "odinLidarFov",
      lidarFov_, &kLidarFovColor);
  projectionToggle_ = addSensorToggle("图像投影 · 点云贴图",
      "仅在同步图像与真实点云相交的位置显示像素", "odinProjection",
      imageProjection_, nullptr);
  inflationToggle_ = addSensorToggle("飞机等效膨胀区",
      "按任务机 EGO 膨胀参数与栅格分辨率绘制世界坐标轴对齐的体积",
      "odinInflation", aircraftInflation_, &kAircraftInflationColor);
  sensorOverlayStatus_ = new QLabel(sensorCard);
  sensorOverlayStatus_->setObjectName("mutedText");
  sensorOverlayStatus_->setTextFormat(Qt::PlainText);
  sensorOverlayStatus_->setWordWrap(false);
  sensorOverlayStatus_->setFixedHeight(28);
  box(sensorCard)->addWidget(sensorOverlayStatus_);
  refreshOdinOverlayStatus();
  auto *sensorVideo = new QPushButton("开启视频", sensorCard);
  sensorVideo->setProperty("compactButton", true);
  commands_[mission::Command::VIDEO_START] = sensorVideo;
  sensorVideo->setToolTip("启动或停止任务机视频；图像投影需 Odin1 提供带时间戳的去畸变图像");
  box(sensorCard)->addWidget(sensorVideo);
  connect(sensorVideo, &QPushButton::clicked, this,
          [this] { toggleMedia(mission::Command::VIDEO_START); });
  detailsScroll_ = new QScrollArea(right_);
  detailsScroll_->setFrameShape(QFrame::NoFrame);
  detailsScroll_->setWidgetResizable(true);
  detailsScroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  detailsScroll_->setStyleSheet("QScrollArea{background:transparent;border:0;}"
                                "QScrollArea>QWidget>QWidget{background:transparent;}");
  auto *rightContents = new QWidget(detailsScroll_);
  auto *detailsLayout = new QVBoxLayout(rightContents);
  detailsLayout->setContentsMargins(0, 0, 0, 0);
  detailsLayout->setSpacing(10);
  detailsLayout->addWidget(sensorCard);
  auto *s = card(rightContents, "任务机状态信息");
  box(s)->setContentsMargins(9, 7, 9, 7);
  box(s)->setSpacing(5);
  detailsLayout->addWidget(s);
  auto *statusGrid = new QGridLayout;
  statusGrid->setContentsMargins(0, 0, 0, 0);
  statusGrid->setHorizontalSpacing(6);
  statusGrid->setVerticalSpacing(2);
  for (int i = 0; i < 10; ++i) {
    auto *cell = new QLabel(s);
    cell->setObjectName("statusCell");
    cell->setTextFormat(Qt::PlainText);
    cell->setWordWrap(true);
    cell->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    statusGrid->addWidget(cell, i / 2, i % 2);
    statusCells_.append(cell);
  }
  statusCells_[5]->setToolTip("任务机物理网卡接收与发送的总速率");
  statusGrid->setColumnStretch(0, 1);
  statusGrid->setColumnStretch(1, 1);
  box(s)->addLayout(statusGrid);
  setStatusCells({"任务机 离线", "Odin1 —", "EGO —", "定位 —",
                  "点云 —", "网络 —", "CPU —", "磁盘 —",
                  "视频 —", "bag 记录 —"});
  auto *o = card(right_, "任务机操作");
  box(o)->setContentsMargins(9, 7, 9, 7);
  box(o)->setSpacing(5);
  r->addWidget(o);
  auto *grid = new QGridLayout;
  grid->setHorizontalSpacing(5);
  grid->setVerticalSpacing(4);
  box(o)->addLayout(grid);
  QVector<QPair<QString, mission::Command::Kind>> ops = {
      {"启动 Odin1", mission::Command::START_ODIN},
      {"启动 EGO", mission::Command::START_EGO},
      {"bag 记录", mission::Command::RECORD_START},
      {"结束任务", mission::Command::END_MISSION}};
  int i = 0;
  for (auto op : ops) {
    auto *b = new QPushButton(op.first);
    b->setProperty("compactButton", true);
    if (op.second == mission::Command::END_MISSION)
      b->setObjectName("dangerButton");
    commands_[op.second] = b;
    if (op.second == mission::Command::START_EGO)
      b->setToolTip("确认本地或机载航线后启动 EGO 及实物航线主控；保持上锁，主控就绪后手动解锁");
    if (op.second == mission::Command::RECORD_START) {
      grid->addWidget(b, i / 2, i % 2);
      connect(b, &QPushButton::clicked, this,
              [this, k = op.second] { toggleMedia(k); });
    } else {
      if (op.second == mission::Command::END_MISSION)
        grid->addWidget(b, (i + 1) / 2, 0, 1, 2);
      else
        grid->addWidget(b, i / 2, i % 2);
      connect(b, &QPushButton::clicked, this,
              [this, k = op.second] {
                if (k == mission::Command::START_ODIN)
                  openOdinStart();
                else
                  command(k);
              });
    }
    ++i;
  }
  refreshStartupButtons();
  r->addWidget(detailsScroll_, 1);
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
  connect(player_, &QMediaPlayer::stateChanged, this,
          [this] { refreshMediaButtons(); });
  connect(player_, &QMediaPlayer::videoAvailableChanged, this,
          [this] { refreshMediaButtons(); });
  connect(videoWidget_->surface, &QAbstractVideoSurface::nativeResolutionChanged,
          this, [this](const QSize &size) {
            if (!size.isValid() || size == videoFrameSize_)
              return;
            videoFrameSize_ = size;
            const double aspect = double(size.width()) / size.height();
            const QSize minimum = aspect >= 1.0
                                      ? QSize(160, qRound(160 / aspect))
                                      : QSize(qRound(160 * aspect), 160);
            videoWidget_->setMinimumSize(minimum);
            video_->setMinimumSize(minimum);
            video_->resize(qRound(video_->height() * aspect), video_->height());
            arrange();
          });
  videoRetry_.setInterval(5000);
  connect(&videoRetry_, &QTimer::timeout, this, [this] {
    if (video_->isHidden() || videoUrl_.trimmed().isEmpty()) {
      videoRetry_.stop();
      return;
    }
    if (videoWidget_->surface->framesPresented > videoRetryFrameCount_ &&
        player_->state() == QMediaPlayer::PlayingState) {
      videoRetry_.stop();
      log("视频画面已恢复", "视频");
      return;
    }
    player_->setMedia(QMediaContent());
    showVideo();
  });
  connect(player_, QOverload<QMediaPlayer::Error>::of(&QMediaPlayer::error),
          this, [this](QMediaPlayer::Error error) {
            if (error == QMediaPlayer::NoError)
              return;
            if (!videoRetry_.isActive()) {
              const bool retry =
                  !video_->isHidden() && !videoUrl_.trimmed().isEmpty();
              log(player_->errorString() + (retry ? "；5 秒后重试" : ""), "视频");
              if (retry) {
                videoRetryFrameCount_ = videoWidget_->surface->framesPresented;
                videoRetry_.start();
              }
            }
            videoWidget_->update();
            refreshMediaButtons();
          });
  connect(player_, &QMediaPlayer::mediaStatusChanged, this,
          [this](QMediaPlayer::MediaStatus s) {
            if (s == QMediaPlayer::StalledMedia ||
                s == QMediaPlayer::EndOfMedia) {
              if (!videoRetry_.isActive() && !video_->isHidden()) {
                log("视频流中断，5 秒后重试", "视频");
                videoRetryFrameCount_ = videoWidget_->surface->framesPresented;
                videoRetry_.start();
              }
              videoWidget_->update();
            }
            refreshMediaButtons();
          });
  video_->hide();
  objectsCard_ = card(canvas_, "");
  box(objectsCard_)->setContentsMargins(6, 5, 6, 14);
  box(objectsCard_)->setSpacing(4);
  auto *objectsHeader = new QHBoxLayout;
  objectsHeader->setContentsMargins(0, 0, 0, 0);
  auto *objectsTitle = new QLabel("处理对象", objectsCard_);
  objectsTitle->setObjectName("cardTitle");
  objectsHeader->addWidget(objectsTitle);
  objectsHeader->addStretch();
  objectsFold_ = new QToolButton(objectsCard_);
  objectsFold_->setObjectName("objectsFold");
  objectsFold_->setArrowType(Qt::DownArrow);
  objectsFold_->setToolTip("折叠处理对象卡片");
  objectsFold_->setAccessibleName("折叠处理对象卡片");
  objectsFold_->setCheckable(true);
  objectsFold_->setAutoRaise(true);
  objectsFold_->setFixedSize(24, 22);
  objectsHeader->addWidget(objectsFold_);
  box(objectsCard_)->addLayout(objectsHeader);
  objectsContent_ = new QWidget(objectsCard_);
  auto *contentLayout = new QVBoxLayout(objectsContent_);
  contentLayout->setContentsMargins(0, 0, 0, 0);
  contentLayout->setSpacing(4);
  box(objectsCard_)->addWidget(objectsContent_);
  connect(objectsFold_, &QToolButton::toggled, this, [this](bool collapsed) {
    objectsContent_->setVisible(!collapsed);
    box(objectsCard_)->setContentsMargins(6, 5, 6, collapsed ? 5 : 14);
    objectsResizeGrip_->setVisible(!collapsed);
    objectsFold_->setArrowType(collapsed ? Qt::RightArrow : Qt::DownArrow);
    objectsFold_->setToolTip(collapsed ? "展开处理对象卡片" : "折叠处理对象卡片");
    objectsFold_->setAccessibleName(objectsFold_->toolTip());
    arrange();
  });
  auto *objectsHint = new QLabel("当前地图 / 实时点云可直接取用；下方列出固定副本与导入对象", objectsContent_);
  objectsHint->setObjectName("mutedText");
  objectsHint->setWordWrap(true);
  contentLayout->addWidget(objectsHint);
  processingObjects_ = new QListWidget(objectsContent_);
  processingObjects_->setObjectName("processingObjects");
  processingObjects_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  processingObjects_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  processingObjects_->setTextElideMode(Qt::ElideMiddle);
  processingObjects_->setMinimumHeight(160);
  processingObjects_->setContextMenuPolicy(Qt::CustomContextMenu);
  processingObjects_->setStyleSheet(
      "QListWidget#processingObjects{border:0;border-radius:4px;"
      "background:rgba(255,255,255,215);}");
  contentLayout->addWidget(processingObjects_);
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
  connect(processingObjects_, &QWidget::customContextMenuRequested, this,
          &MissionController::showProcessingObjectMenu);
  auto *objectActions = new QHBoxLayout;
  contentLayout->addLayout(objectActions);
  button(objectActions, "当前点云", [this] { selectCurrentCloud(); });
  button(objectActions, "快照", [this] { snapshotMap(); });
  objectsResizeGrip_ = new ProcessingResizeHandle(objectsCard_);
  objectsResizeGrip_->setObjectName("objectsResizeGrip");
  objectsResizeGrip_->setToolTip("拖动调整处理对象卡片大小");
  objectsResizeGrip_->setAccessibleName("调整处理对象卡片大小");
  objectsResizeGrip_->installEventFilter(this);
  processingCard_ = card(rightContents, "点云工具");
  detailsLayout->addWidget(processingCard_);
  box(processingCard_)->setContentsMargins(8, 7, 8, 8);
  box(processingCard_)->setSpacing(5);
  processingHint_ = new QLabel("选择对象；未选中时处理工具使用当前点云的固定副本。", processingCard_);
  processingHint_->setObjectName("mutedText");
  processingHint_->setWordWrap(true);
  box(processingCard_)->addWidget(processingHint_);
  toolsTabs_ = new QTabWidget(processingCard_);
  toolsTabs_->setObjectName("missionToolsTabs");
  box(processingCard_)->addWidget(toolsTabs_);
  processingOnly_ = new QWidget(toolsTabs_);
  auto *processingActions = new QVBoxLayout(processingOnly_);
  processingActions->setContentsMargins(0, 0, 0, 0);
  processingActions->setSpacing(3);
  toolsTabs_->addTab(processingOnly_, "处理");
  auto addCardTool = [this](QVBoxLayout *layout, const QString &label,
                            const char *name) {
    auto *action = button(layout, label, [this, name] {
      runCloudCompareAction(name);
    });
    action->setObjectName("cloudActionButton");
    action->setCursor(Qt::PointingHandCursor);
  };
  addCardTool(processingActions, "复制对象", "actionClone");
  addCardTool(processingActions, "抽稀", "actionSubsample");
  addCardTool(processingActions, "计算法向量", "actionComputeNormals");
  auto *reconstruct = button(processingActions, "一键生成数模（泊松）", [this] {
    quickReconstruct();
  });
  reconstruct->setObjectName("cloudPrimaryButton");
  reconstruct->setCursor(Qt::PointingHandCursor);
  addCardTool(processingActions, "栅格化", "actionRasterize");
  comparisonOnly_ = new QWidget(toolsTabs_);
  auto *comparisonActions = new QVBoxLayout(comparisonOnly_);
  comparisonActions->setContentsMargins(0, 0, 0, 0);
  comparisonActions->setSpacing(3);
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
  toolsTabs_->addTab(comparisonOnly_, "对比");
  connect(toolsTabs_, &QTabWidget::currentChanged, this, [this](int index) {
    if (index == 1)
      selectComparisonPair();
  });
  auto *exportObject = button(box(processingCard_), "导出选中对象", [this] {
    runCloudCompareAction("actionSave");
  });
  exportObject->setObjectName("cloudActionButton");
  exportObject->setCursor(Qt::PointingHandCursor);
  detailsLayout->addStretch(1);
  detailsScroll_->setWidget(rightContents);
  left_->show();
  right_->show();
  top_->show();
  objectsCard_->show();
  processingCard_->show();
  mapBadge_->show();
  gl_->setView(CC_ISO_VIEW_1);
  window_->statusBar()->showMessage("地图坐标：" + frame_ + "　·　单位：m");
  protocolHint_ = new QLabel("Protobuf v1  ·  未连接", window_->statusBar());
  protocolHint_->setObjectName("mutedText");
  window_->statusBar()->addPermanentWidget(protocolHint_);
  window_->statusBar()->hide();
  for (auto *cardWidget : {left_, mapPopover_, heightFilterPanel_, top_, editor_, h, s, o,
                           objectsCard_, processingCard_}) {
    auto *shadow = new QGraphicsDropShadowEffect(cardWidget);
    shadow->setBlurRadius(22);
    shadow->setOffset(0, 4);
    shadow->setColor(QColor(7, 26, 37, 53));
    cardWidget->setGraphicsEffect(shadow);
  }
  auto registerCard = [this, display](QWidget *widget, const QString &name,
                                       std::function<void(bool)> setShown = {}) {
    auto *action = display->addAction(name);
    action->setCheckable(true);
    action->setChecked(!widget->isHidden());
    cardVisibilityActions_.insert(widget, action);
    widget->installEventFilter(this);
    connect(action, &QAction::toggled, this, [widget, setShown](bool shown) {
      if (setShown)
        setShown(shown);
      else
        widget->setVisible(shown);
      if (shown && widget->isVisible())
        widget->raise();
    });
    widget->setContextMenuPolicy(Qt::CustomContextMenu);
    auto showHideMenu = [widget, action](const QPoint &point) {
      QMenu menu(widget);
      auto *hide = menu.addAction("隐藏此卡片");
      if (menu.exec(widget->mapToGlobal(point)) == hide)
        action->setChecked(false);
    };
    connect(widget, &QWidget::customContextMenuRequested, this, showHideMenu);
    return action;
  };
  registerCard(top_, "顶部工具栏");
  registerCard(left_, "图层工具栏");
  registerCard(mapBadge_, "地图状态");
  registerCard(editor_, "航点任务");
  registerCard(objectsCard_, "处理对象", [this](bool shown) {
    objectsButton_->setChecked(shown);
  });
  registerCard(mapPopover_, "当前地图");
  registerCard(heightFilterPanel_, "高度过滤", [this](bool shown) {
    heightFilterButton_->setChecked(shown);
  });
  registerCard(h, "无人机 HUD");
  registerCard(sensorCard, "Odin1 视场与图像");
  registerCard(s, "任务机状态信息");
  registerCard(o, "任务机操作");
  registerCard(processingCard_, "点云工具", [this, showTools](bool shown) {
    showTools->setChecked(shown);
    if (shown)
      detailsScroll_->verticalScrollBar()->setValue(processingCard_->y());
  });
  registerCard(replayDock, "bag 回放工具栏");
  registerCard(logPopup_, "运行日志", [this](bool shown) {
    if (shown)
      latest_->click();
    else
      logPopup_->hide();
  });
  auto *videoAction = registerCard(video_, "视频窗口", [this](bool shown) {
    if (shown) {
      showVideo();
    } else {
      videoRetry_.stop();
      player_->stop();
      video_->hide();
      refreshMediaButtons();
    }
  });
  videoWidget_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(videoWidget_, &QWidget::customContextMenuRequested, this,
          [videoAction, this](const QPoint &point) {
            QMenu menu(videoWidget_);
            auto *hide = menu.addAction("隐藏此卡片");
            if (menu.exec(videoWidget_->mapToGlobal(point)) == hide)
              videoAction->setChecked(false);
          });
  display->addSeparator();
  display->addAction("清空累积地图", this, [this] { clearAccumulatedMap(); });
  auto *objectRefresh = new QTimer(this);
  objectRefresh->setInterval(900);
  connect(objectRefresh, &QTimer::timeout, this,
          &MissionController::refreshProcessingObjects);
  objectRefresh->start();
}
bool MissionController::selectCurrentCloud() {
  if (dirty_ && (mapDirty_ || liveDirty_))
    updateScene();
  const bool fromMap = map_ && map_->isEnabled() && !mapPoints_.isEmpty();
  const bool fromLive = live_ && live_->isEnabled() && !livePoints_.isEmpty();
  if (!fromMap && !fromLive) {
    processingHint_->setText("当前界面没有可见点云；可等待实时数据或加载地图。");
    return false;
  }
  const quint64 version = fromMap ? mapCloudVersion_ : liveCloudVersion_;
  auto *dbRoot = window_->dbRootObject();
  ccHObject *cloud = dbRoot && automaticCloudId_ &&
                             automaticCloudFromMap_ == fromMap &&
                             automaticCloudVersion_ == version
                         ? dbRoot->find(automaticCloudId_)
                         : nullptr;
  if (!cloud) {
    const QString name = QString("当前%1_%2")
                             .arg(fromMap ? "地图" : "实时点云",
                                  QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    cloud = captureSceneCloud(name, false);
    if (!cloud)
      return false;
    automaticCloudId_ = cloud->getUniqueID();
    automaticCloudFromMap_ = fromMap;
    automaticCloudVersion_ = version;
  }
  window_->db()->selectEntities({cloud});
  refreshProcessingObjects();
  processingHint_->setText("已选中当前点云的固定副本；再次取用可获取更新后的点云。");
  if (objectsButton_)
    objectsButton_->setChecked(true);
  return true;
}
void MissionController::refreshProcessingObjects() {
  if (!processingObjects_)
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
  ccHObject::Container liveSelection;
  window_->db()->getSelectedEntities(liveSelection);
  for (auto *object : liveSelection)
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
    processingHint_->setText("当前界面没有可处理的点云；可等待实时数据或加载地图。");
}
QVector<ccHObject *> MissionController::selectedProcessingObjects() const {
  QVector<ccHObject *> selected;
  auto *root = window_->dbRootObject();
  if (!root || !processingObjects_)
    return selected;
  for (auto *item : processingObjects_->selectedItems()) {
    if (auto *object = root->find(item->data(Qt::UserRole).toUInt()))
      selected.push_back(object);
  }
  return selected;
}
void MissionController::showProcessingObjectMenu(const QPoint &position) {
  auto *item = processingObjects_->itemAt(position);
  if (item && !item->isSelected())
    processingObjects_->setCurrentItem(item, QItemSelectionModel::ClearAndSelect);
  else if (!item)
    processingObjects_->clearSelection();

  const auto selected = selectedProcessingObjects();
  const bool canCopy = !selected.isEmpty() &&
      std::all_of(selected.cbegin(), selected.cend(), canCopyProcessingObject);
  const bool canDelete = !selected.isEmpty() &&
      std::all_of(selected.cbegin(), selected.cend(), [](ccHObject *object) {
        return object->getParent() && !object->isLocked();
      });
  QMenu menu(processingObjects_);
  auto *copy = menu.addAction("复制");
  copy->setEnabled(canCopy);
  auto *paste = menu.addAction("粘贴");
  paste->setEnabled(!objectClipboard_.empty());
  auto *rename = menu.addAction("重命名");
  rename->setEnabled(selected.size() == 1 && !selected.front()->isLocked());
  menu.addSeparator();
  auto *remove = menu.addAction("删除");
  remove->setEnabled(canDelete);
  auto *chosen = menu.exec(processingObjects_->viewport()->mapToGlobal(position));
  if (chosen == copy)
    copyProcessingObjects();
  else if (chosen == paste)
    pasteProcessingObjects();
  else if (chosen == rename)
    renameProcessingObject();
  else if (chosen == remove)
    deleteProcessingObjects();
}
void MissionController::copyProcessingObjects() {
  const auto selected = selectedProcessingObjects();
  if (selected.isEmpty())
    return;
  std::vector<std::unique_ptr<ccHObject>> copies;
  for (auto *object : selected) {
    if (!canCopyProcessingObject(object))
      return;
    auto copy = cloneProcessingObject(object);
    if (!copy) {
      log("复制处理对象失败：无法创建完整副本", "点云处理");
      return;
    }
    copies.push_back(std::move(copy));
  }
  objectClipboard_ = std::move(copies);
  log(QString("已复制 %1 个处理对象").arg(objectClipboard_.size()), "点云处理");
}
void MissionController::pasteProcessingObjects() {
  if (objectClipboard_.empty())
    return;
  std::vector<std::unique_ptr<ccHObject>> copies;
  for (const auto &source : objectClipboard_) {
    auto copy = cloneProcessingObject(source.get());
    if (!copy) {
      log("粘贴处理对象失败：无法创建完整副本", "点云处理");
      return;
    }
    copy->setName(source->getName() + "_副本");
    copy->setDisplay_recursive(gl_);
    copy->setVisible(true);
    copy->setEnabled(true);
    copies.push_back(std::move(copy));
  }
  ccHObject::Container pasted;
  for (auto &copy : copies) {
    auto *object = copy.release();
    window_->addToDB(object, false, true, false, true);
    pasted.push_back(object);
  }
  window_->db()->selectEntities(pasted);
  refreshProcessingObjects();
  objectsButton_->setChecked(true);
  log(QString("已粘贴 %1 个处理对象").arg(pasted.size()), "点云处理");
}
void MissionController::renameProcessingObject() {
  const auto selected = selectedProcessingObjects();
  if (selected.size() != 1 || selected.front()->isLocked())
    return;
  auto *object = selected.front();
  bool accepted = false;
  const QString name = QInputDialog::getText(
      window_, "重命名处理对象", "名称", QLineEdit::Normal,
      object->getName(), &accepted).trimmed();
  if (!accepted || name.isEmpty() || name == object->getName())
    return;
  if (window_->db()->setData(window_->db()->index(object), name, Qt::EditRole)) {
    refreshProcessingObjects();
    log("已重命名处理对象：" + name, "点云处理");
  }
}
void MissionController::deleteProcessingObjects() {
  const auto selected = selectedProcessingObjects();
  if (selected.isEmpty() ||
      !std::all_of(selected.cbegin(), selected.cend(), [](ccHObject *object) {
        return object->getParent() && !object->isLocked();
      }))
    return;
  const QString question = selected.size() == 1
      ? QString("确定删除“%1”？").arg(selected.front()->getName())
      : QString("确定删除选中的 %1 个处理对象？").arg(selected.size());
  if (QMessageBox::question(window_, "删除处理对象", question,
                            QMessageBox::Yes | QMessageBox::No,
                            QMessageBox::No) != QMessageBox::Yes)
    return;
  ccHObject::Container objects(selected.cbegin(), selected.cend());
  window_->db()->selectEntities(objects);
  window_->db()->deleteSelectedEntities();
  if (automaticCloudId_ && !window_->dbRootObject()->find(automaticCloudId_))
    automaticCloudId_ = 0;
  refreshProcessingObjects();
  log(QString("已删除 %1 个处理对象").arg(selected.size()), "点云处理");
}
bool MissionController::selectComparisonPair() {
  if (!processingObjects_ || toolsTabs_->currentWidget() != comparisonOnly_)
    return false;
  const unsigned reference = referenceObject_->currentData().toUInt();
  const unsigned target = targetObject_->currentData().toUInt();
  if (!reference || !target || reference == target) {
    processingHint_->setText("参考和待测必须是两个不同对象。");
    return false;
  }
  auto *root = window_->dbRootObject();
  if (!root)
    return false;
  ccHObject::Container pair;
  if (auto *object = root->find(reference))
    pair.push_back(object);
  if (auto *object = root->find(target))
    pair.push_back(object);
  if (pair.size() != 2) {
    processingHint_->setText("参考或待测对象已不存在，请重新选择。");
    return false;
  }
  window_->db()->selectEntities(pair);
  refreshProcessingObjects();
  processingHint_->setText("已选择参考与待测对象；距离方向可在原工具对话框确认。");
  return true;
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
  refreshProcessingObjects();
  objectsButton_->setChecked(true);
  log(QString("已请求导入 %1 个处理文件").arg(paths.size()), "点云处理");
}
ccPointCloud *MissionController::captureSceneCloud(const QString &name,
                                                   bool visible) {
  const bool fromMap = map_ && map_->isEnabled() &&
                       !mapPoints_.isEmpty();
  const bool fromLive = live_ && live_->isEnabled() &&
                        !livePoints_.isEmpty();
  if (!fromMap && !fromLive) {
    log("当前没有可快照的地图或实时点云", "点云处理");
    return nullptr;
  }
  const auto &source = fromMap ? mapPoints_ : livePoints_;
  auto *cloud = new ccPointCloud(name);
  if (!cloud->reserve(source.size()) || !cloud->reserveTheRGBTable()) {
    delete cloud;
    log("地图快照内存分配失败", "点云处理");
    return nullptr;
  }
  for (const auto &point : source) {
    cloud->addPoint(CCVector3(point.x, point.y, point.z));
    cloud->addColor(point.r, point.g, point.b);
  }
  cloud->showColors(true);
  cloud->setPointSize(2);
  cloud->setMetaData("3dg.source", fromMap ? "map" : "live");
  cloud->setMetaData("3dg.mapId", mapId_);
  cloud->setMetaData("3dg.frame", frame_);
  cloud->setMetaData("3dg.unit", "m");
  cloud->setMetaData("3dg.capturedAt", QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
  cloud->setDisplay(gl_);
  cloud->setVisible(visible);
  window_->addToDB(cloud, false, true, false, true);
  log(QString("已提取当前%1的 %2 个点用于处理")
          .arg(fromMap ? "地图" : "实时点云").arg(source.size()), "点云处理");
  return cloud;
}
void MissionController::snapshotMap() {
  auto *cloud = captureSceneCloud(
      QString("地图快照_%1")
          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
  if (!cloud)
    return;
  window_->db()->selectEntities({cloud});
  refreshProcessingObjects();
  objectsButton_->setChecked(true);
}
void MissionController::quickReconstruct() {
  toolsTabs_->setCurrentWidget(processingOnly_);
  if (window_->getSelectedEntities().empty() && !selectCurrentCloud())
    return;
  const auto &selected = window_->getSelectedEntities();
  if (selected.size() != 1 || !selected.front()->isA(CC_TYPES::POINT_CLOUD)) {
    processingHint_->setText("请在处理对象中只选中一份点云。");
    return;
  }
  for (ccPluginInterface *plugin : ccPluginManager::Get().pluginList()) {
    if (!plugin || plugin->getName() != "PoissonRecon" ||
        plugin->getType() != CC_STD_PLUGIN ||
        !ccPluginManager::Get().isEnabled(plugin))
      continue;
    auto *standard = static_cast<ccStdPluginInterface *>(plugin);
    for (QAction *action : standard->getActions()) {
      if (action->objectName() != "actionQuickPoissonRecon")
        continue;
      if (!action->isEnabled()) {
        processingHint_->setText("当前点云不能执行泊松重建。");
        return;
      }
      action->trigger();
      refreshProcessingObjects();
      const auto &result = window_->getSelectedEntities();
      if (result.size() == 1 && result.front()->isKindOf(CC_TYPES::MESH)) {
        processingHint_->setText("已生成并显示数模；可在对象列表中选择网格导出。");
        log("泊松数模已生成并显示", "点云处理");
      } else {
        processingHint_->setText("数模生成失败；请检查点云数量、法线和日志。");
      }
      return;
    }
  }
  processingHint_->setText("当前构建未加载泊松重建插件。");
  log("当前构建未加载泊松重建插件", "点云处理");
}
void MissionController::runCloudCompareAction(const char *objectName) {
  const QString actionId = QString::fromLatin1(objectName);
  const bool pairTool = actionId == "actionRegister" ||
                        actionId == "actionPointPairsAlign" ||
                        actionId == "actionCloudCloudDist" ||
                        actionId == "actionCloudMeshDist";
  if (pairTool) {
    toolsTabs_->setCurrentWidget(comparisonOnly_);
    if (!selectComparisonPair())
      return;
  } else if (actionId != "actionSave") {
    toolsTabs_->setCurrentWidget(processingOnly_);
    if (window_->getSelectedEntities().empty() && !selectCurrentCloud())
      return;
  }
  auto *action = window_->findChild<QAction *>(actionId);
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
  for (auto *selected : window_->getSelectedEntities()) {
    if (selected->getUniqueID() == automaticCloudId_) {
      automaticCloudId_ = 0;
      break;
    }
  }
  action->trigger();
  QTimer::singleShot(0, this, &MissionController::refreshProcessingObjects);
}
void MissionController::configureNoiseDetection() {
  QDialog dialog(window_);
  dialog.setWindowTitle("噪点检测参数");
  dialog.setMinimumWidth(370);
  auto *layout = new QVBoxLayout(&dialog);
  auto *form = new QFormLayout;
  layout->addLayout(form);
  auto *knn = new QSpinBox(&dialog);
  knn->setObjectName("noiseKnn");
  knn->setRange(3, 256);
  knn->setValue(noiseKnn_);
  knn->setToolTip("每个点用于计算平均邻距的近邻数量；点云点数须大于此值");
  form->addRow("近邻点数 K", knn);
  auto *sigma = new QDoubleSpinBox(&dialog);
  sigma->setObjectName("noiseSigma");
  sigma->setRange(0.1, 10.0);
  sigma->setDecimals(2);
  sigma->setSingleStep(0.1);
  sigma->setValue(noiseSigma_);
  sigma->setToolTip("平均邻距超过全局均值加此倍数标准差的点会标为噪点");
  form->addRow("标准差倍数 σ", sigma);
  auto *pointSize = new QSpinBox(&dialog);
  pointSize->setObjectName("noisePointSize");
  pointSize->setRange(2, 12);
  pointSize->setValue(noisePointSize_);
  form->addRow("噪点显示大小", pointSize);
  auto *hint = new QLabel(
      "保存后再次运行自动筛选生效。σ 越小，筛出的点通常越多；原始点云不修改。",
      &dialog);
  hint->setWordWrap(true);
  layout->addWidget(hint);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok |
                                       QDialogButtonBox::Cancel, &dialog);
  buttons->button(QDialogButtonBox::Ok)->setText("保存");
  buttons->button(QDialogButtonBox::Cancel)->setText("取消");
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  if (dialog.exec() != QDialog::Accepted)
    return;
  noiseKnn_ = knn->value();
  noiseSigma_ = sigma->value();
  noisePointSize_ = pointSize->value();
  QSettings settings("3DG", "Mission");
  settings.setValue("noiseKnn", noiseKnn_);
  settings.setValue("noiseSigma", noiseSigma_);
  settings.setValue("noisePointSize", noisePointSize_);
  log(QString("噪点参数已保存：K=%1，σ=%2，显示大小=%3")
          .arg(noiseKnn_).arg(noiseSigma_, 0, 'f', 2).arg(noisePointSize_),
      "点云处理");
}
void MissionController::detectCloudNoise() {
  toolsTabs_->setCurrentWidget(processingOnly_);
  if (dirty_ && (mapDirty_ || liveDirty_ || colorMapDirty_))
    updateScene();

  ccPointCloud *source = nullptr;
  const auto &selected = window_->getSelectedEntities();
  if (!selected.empty()) {
    if (selected.size() == 1)
      source = dynamic_cast<ccPointCloud *>(selected.front());
    if (!source) {
      log("噪点检测：请只选中一份点云", "点云处理");
      if (processingHint_)
        processingHint_->setText("请只选中一份点云，再运行噪点检测。");
      return;
    }
  } else {
    for (ccHObject *layer : {map_, colorMap_, live_}) {
      if (layer && layer->isEnabled() && layer->isVisible() &&
          layer->getChildrenNumber()) {
        source = dynamic_cast<ccPointCloud *>(layer->getChild(0));
        if (source && source->size())
          break;
      }
    }
    if (!source && selected.size() == 1)
      source = dynamic_cast<ccPointCloud *>(selected.front());
  }
  if (noiseViewActive_ && (!source || source->getUniqueID() == noiseResultId_)) {
    auto *root = window_->dbRootObject();
    source = root && noiseSourceId_
                 ? dynamic_cast<ccPointCloud *>(root->find(noiseSourceId_))
                 : nullptr;
  }
  const int knn = noiseKnn_;
  const double sigma = noiseSigma_;
  const int pointSize = noisePointSize_;
  const unsigned minPoints = static_cast<unsigned>(qMax(8, knn + 1));
  if (!source || source->size() < minPoints) {
    log(QString("噪点检测：当前点云至少需要 %1 个点").arg(minPoints), "点云处理");
    if (processingHint_)
      processingHint_->setText(
          QString("当前没有可检测的点云（至少需要 %1 个点）。").arg(minPoints));
    return;
  }
  const QString sourceName = source->getName();

  // The mission scene replaces its clouds as telemetry arrives. Work on a
  // stable copy so the outlier indices and the displayed result always match.
  std::unique_ptr<ccPointCloud> snapshot(source->cloneThis(nullptr, true));
  if (!snapshot || snapshot->size() != source->size()) {
    log("噪点检测：点云副本创建失败（内存不足）", "点云处理");
    return;
  }
  snapshot->setName(sourceName + "_噪点原图_" +
                    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
  snapshot->setDisplay(gl_);
  snapshot->setVisible(true);

  ccProgressDialog progress(true, window_);
  progress.setAutoClose(false);
  std::unique_ptr<CCCoreLib::ReferenceCloud> retained(
      CCCoreLib::CloudSamplingTools::sorFilter(snapshot.get(), knn, sigma,
                                               nullptr, &progress));
  if (!retained) {
    log(progress.wasCanceled() ? "噪点检测已取消" : "噪点检测失败", "点云处理");
    return;
  }
  const unsigned noiseCount = snapshot->size() - retained->size();
  if (!noiseCount) {
    restoreCloudNoiseView();
    noiseSourceId_ = noiseResultId_ = 0;
    for (auto *action : {noiseOverlayAction_, noiseOnlyAction_, noiseRestoreAction_})
      action->setEnabled(false);
    const QString message = "噪点检测完成：未发现离群点";
    log(message, "点云处理");
    if (processingHint_)
      processingHint_->setText(message);
    return;
  }

  std::vector<unsigned char> retainedMask(snapshot->size(), 0);
  for (unsigned i = 0; i < retained->size(); ++i)
    retainedMask[retained->getPointGlobalIndex(i)] = 1;
  CCCoreLib::ReferenceCloud outliers(snapshot.get());
  if (!outliers.reserve(noiseCount)) {
    log("噪点检测：无法分配噪点索引内存", "点云处理");
    return;
  }
  for (unsigned i = 0; i < snapshot->size(); ++i) {
    if (!retainedMask[i] && !outliers.addPointIndex(i)) {
      log("噪点检测：无法生成噪点索引", "点云处理");
      return;
    }
  }
  std::unique_ptr<ccPointCloud> noise(snapshot->partialClone(&outliers, nullptr, false));
  if (!noise || !noise->setColor(ccColor::Rgb(255, 54, 54))) {
    log("噪点检测：无法生成红色噪点图层（内存不足）", "点云处理");
    return;
  }
  noise->setName(sourceName + "_噪点_" +
                 QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
  noise->setMetaData("3dg.noise.algorithm", "SOR");
  noise->setMetaData("3dg.noise.knn", knn);
  noise->setMetaData("3dg.noise.sigma", sigma);
  noise->showSF(false);
  noise->showColors(true);
  noise->setPointSize(pointSize);
  noise->setDisplay(gl_);
  noise->setVisible(true);

  restoreCloudNoiseView();
  auto *snapshotObject = snapshot.release();
  auto *noiseObject = noise.release();
  window_->addToDB(snapshotObject, false, true, false, true);
  window_->addToDB(noiseObject, false, true, false, true);
  noiseSourceId_ = snapshotObject->getUniqueID();
  noiseResultId_ = noiseObject->getUniqueID();
  for (auto *action : {noiseOverlayAction_, noiseOnlyAction_, noiseRestoreAction_})
    action->setEnabled(true);
  showCloudNoise(false);
  window_->db()->selectEntities({snapshotObject});
  refreshProcessingObjects();
  const QString message =
      QString("噪点检测完成：%1 / %2 个点被标为离群点（SOR，K=%3，σ=%4）")
          .arg(noiseCount).arg(snapshotObject->size()).arg(knn)
          .arg(sigma, 0, 'f', 2);
  log(message, "点云处理");
  if (processingHint_)
    processingHint_->setText(message);
}
void MissionController::showCloudNoise(bool onlyNoise) {
  auto *root = window_->dbRootObject();
  auto *snapshot = root && noiseSourceId_ ? root->find(noiseSourceId_) : nullptr;
  auto *noise = root && noiseResultId_ ? root->find(noiseResultId_) : nullptr;
  if (!snapshot || !noise) {
    restoreCloudNoiseView();
    noiseSourceId_ = noiseResultId_ = 0;
    log("噪点图层已移除，请重新运行检测", "点云处理");
    for (auto *action : {noiseOverlayAction_, noiseOnlyAction_, noiseRestoreAction_})
      action->setEnabled(false);
    return;
  }
  if (!noiseViewActive_) {
    noiseDbVisibility_.clear();
    for (unsigned i = 0; i < root->getChildrenNumber(); ++i) {
      auto *object = root->getChild(i);
      if (object != snapshot && object != noise)
        noiseDbVisibility_.insert(object->getUniqueID(), object->isVisible());
    }
    for (unsigned i = 0; i < root_->getChildrenNumber(); ++i) {
      auto *layer = root_->getChild(i);
      noiseSceneVisibility_.insert(layer, layer->isEnabled());
    }
    noiseViewActive_ = true;
  }
  for (auto it = noiseDbVisibility_.cbegin(); it != noiseDbVisibility_.cend(); ++it) {
    if (auto *object = root->find(it.key()))
      object->setVisible(false);
  }
  for (auto it = noiseSceneVisibility_.cbegin(); it != noiseSceneVisibility_.cend(); ++it) {
    const bool pointLayer = it.key() == map_ || it.key() == colorMap_ ||
                            it.key() == live_;
    it.key()->setEnabled(!onlyNoise && !pointLayer && it.value());
  }
  snapshot->setVisible(!onlyNoise);
  noise->setVisible(true);
  gl_->redraw();
}
void MissionController::restoreCloudNoiseView() {
  auto *root = window_->dbRootObject();
  if (root) {
    if (auto *snapshot = noiseSourceId_ ? root->find(noiseSourceId_) : nullptr)
      snapshot->setVisible(false);
    if (auto *noise = noiseResultId_ ? root->find(noiseResultId_) : nullptr)
      noise->setVisible(false);
    for (auto it = noiseDbVisibility_.cbegin(); it != noiseDbVisibility_.cend(); ++it) {
      if (auto *object = root->find(it.key()))
        object->setVisible(it.value());
    }
  }
  for (auto it = noiseSceneVisibility_.cbegin(); it != noiseSceneVisibility_.cend(); ++it)
    it.key()->setEnabled(it.value());
  noiseDbVisibility_.clear();
  noiseSceneVisibility_.clear();
  noiseViewActive_ = false;
  if (gl_)
    gl_->redraw();
}
void MissionController::refreshMapInfo() {
  if (mapInfo_)
    mapInfo_->setText(QString("坐标系  %1\n地图 ID  %2\n实时点  %3\n%4  %5\n彩色累积点  %6")
                          .arg(frame_, mapId_)
                          .arg(livePoints_.size())
                          .arg(referenceMap_ ? "参考地图点" : "普通累积点")
                          .arg(mapPoints_.size())
                          .arg(colorMapPoints_.size()));
  if (mapBadge_)
    mapBadge_->setText(
        QString("航点 %1 · %2  |  EGO · 轨迹")
            .arg(mission_.waypoints_size())
            .arg(routeSyncedToOnboard_ ? "已同步任务机" : "本地编辑"));
  if (window_)
    window_->statusBar()->showMessage("地图坐标：" + frame_ + "　·　单位：m");
}
void MissionController::refreshRouteStrip() {
  if (!routeStrip_)
    return;
  const int count = mission_.waypoints_size();
  const int selected = waypoints_ ? waypoints_->currentRow() : -1;
  auto *strip = qobject_cast<QHBoxLayout *>(routeStrip_->layout());
  while (auto *item = strip->takeAt(0)) {
    if (auto *widget = item->widget()) {
      widget->hide();
      widget->deleteLater();
    }
    delete item;
  }
  if (count == 0) {
    auto *empty = new QLabel("暂无航点 · 双击打开航点任务", routeStrip_);
    empty->setObjectName("mutedText");
    empty->setAttribute(Qt::WA_TransparentForMouseEvents);
    strip->addWidget(empty);
    strip->addStretch();
    return;
  }
  constexpr int kVisible = 4;
  const int first = qBound(0, selected - 1, qMax(0, count - kVisible));
  const int last = qMin(count, first + kVisible);
  if (first > 0) {
    auto *ellipsis = new QLabel("…", routeStrip_);
    ellipsis->setAttribute(Qt::WA_TransparentForMouseEvents);
    strip->addWidget(ellipsis);
  }
  for (int i = first; i < last; ++i) {
    auto *chip = new QPushButton(
        QString("%1%2").arg(routeSyncedToOnboard_ ? "机" : "本")
            .arg(i + 1), routeStrip_);
    chip->setObjectName("routeChip");
    chip->setCheckable(true);
    chip->setChecked(i == selected);
    chip->setToolTip(QString("航点 %1 · 双击打开航点任务")
                         .arg(i + 1));
    chip->setProperty("routeIndex", i);
    chip->installEventFilter(this);
    strip->addWidget(chip);
    connect(chip, &QPushButton::clicked, this, [this, chip, i] {
      const int generation = chip->property("routeClickGeneration").toInt() + 1;
      chip->setProperty("routeClickGeneration", generation);
      QTimer::singleShot(QApplication::doubleClickInterval(), chip,
                         [this, chip, i, generation] {
        if (chip->property("routeClickGeneration").toInt() != generation)
          return;
        chip->setChecked(true);
        waypoints_->selectRow(i);
      });
    });
  }
  if (last < count) {
    auto *ellipsis = new QLabel(QString("… +%1").arg(count - last), routeStrip_);
    ellipsis->setAttribute(Qt::WA_TransparentForMouseEvents);
    strip->addWidget(ellipsis);
  }
  strip->addStretch();
}
void MissionController::setEditorCollapsed(bool collapsed) {
  if (!editor_ || !routeDialog_ || !routeStrip_)
    return;
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
  const int filterWidth = 100;
  const int filterHeight = qMin(360, h - margin * 2);
  QPoint filterPosition = heightFilterPlaced_
      ? heightFilterPanel_->pos()
      : QPoint(w - rightWidth - margin - filterWidth - 8, margin + 80);
  filterPosition.setX(qBound(0, filterPosition.x(), qMax(0, w - filterWidth)));
  filterPosition.setY(qBound(0, filterPosition.y(), qMax(0, h - filterHeight)));
  heightFilterPanel_->setGeometry(QRect(filterPosition, QSize(filterWidth, filterHeight)));
  right_->setGeometry(w - rightWidth - margin, margin, rightWidth,
                      qMax(0, h - margin * 2));
  top_->setGeometry(centerX, margin,
                    toolbarCollapsed_ ? 44 : qMin(centerWidth, 450), 46);
  const int objectsX = margin + leftWidth + 8;
  const int objectsY = margin + 58;
  const QSize objectsMinimum = objectsCard_->minimumSizeHint();
  const int objectsWidth = qMin(qMax(qMax(180, objectsMinimum.width()),
                                    objectsExpandedSize_.width()),
                                qMax(0, w - objectsX - margin));
  const int objectsHeight = objectsFold_->isChecked()
      ? objectsCard_->sizeHint().height()
      : qMin(qMax(qMax(260, objectsMinimum.height()),
                  objectsExpandedSize_.height()),
             qMax(0, h - objectsY - margin));
  objectsCard_->setGeometry(objectsX, objectsY, objectsWidth, objectsHeight);
  objectsResizeGrip_->move(qMax(0, objectsWidth - 15),
                           qMax(0, objectsHeight - 14));
  objectsResizeGrip_->raise();
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
    const double heightPerWidth =
        double(videoFrameSize_.height()) / videoFrameSize_.width();
    const int maxWidth = qMin(w, qFloor(h / heightPerWidth));
    const int width = qBound(video_->minimumWidth(), video_->width(),
                             qMax(video_->minimumWidth(), maxWidth));
    video_->resize(width, qRound(width * heightPerWidth));
    video_->move(qBound(0, video_->x(), qMax(0, w - video_->width())),
                 qBound(0, video_->y(), qMax(0, h - video_->height())));
  }
  top_->raise();
  mapBadge_->raise();
  left_->raise();
  right_->raise();
  editor_->raise();
  if (objectsCard_->isVisible())
    objectsCard_->raise();
  if (video_->isVisible())
    video_->raise();
  if (mapPopover_->isVisible())
    mapPopover_->raise();
  if (heightFilterPanel_->isVisible())
    heightFilterPanel_->raise();
}
bool MissionController::eventFilter(QObject *o, QEvent *e) {
  if ((e->type() == QEvent::Show || e->type() == QEvent::Hide) &&
      cardVisibilityActions_.contains(o)) {
    auto *widget = static_cast<QWidget *>(o);
    cardVisibilityActions_.value(o)->setChecked(!widget->isHidden());
  }
  if (o == objectsResizeGrip_) {
    if (e->type() == QEvent::MouseButtonPress) {
      auto *mouse = static_cast<QMouseEvent *>(e);
      if (mouse->button() == Qt::LeftButton) {
        objectsResizePress_ = mouse->globalPos();
        objectsResizeStart_ = objectsCard_->size();
        objectsResizing_ = true;
        objectsResizeGrip_->grabMouse();
        return true;
      }
    } else if (e->type() == QEvent::MouseMove && objectsResizing_) {
      auto *mouse = static_cast<QMouseEvent *>(e);
      const QPoint delta = mouse->globalPos() - objectsResizePress_;
      const int maxWidth = qMax(0, canvas_->width() - objectsCard_->x() - 12);
      const int maxHeight = qMax(0, canvas_->height() - objectsCard_->y() - 12);
      const QSize minimum = objectsCard_->minimumSizeHint();
      objectsExpandedSize_ = QSize(
          qMin(qMax(qMax(180, minimum.width()),
                    objectsResizeStart_.width() + delta.x()), maxWidth),
          qMin(qMax(qMax(260, minimum.height()),
                    objectsResizeStart_.height() + delta.y()), maxHeight));
      arrange();
      return true;
    } else if (e->type() == QEvent::MouseButtonRelease && objectsResizing_ &&
               static_cast<QMouseEvent *>(e)->button() == Qt::LeftButton) {
      objectsResizing_ = false;
      objectsResizeGrip_->releaseMouse();
      return true;
    }
  }
  if (o == heightFilterTitle_) {
    if (e->type() == QEvent::MouseButtonPress) {
      auto *mouse = static_cast<QMouseEvent *>(e);
      if (mouse->button() == Qt::LeftButton) {
        heightFilterDrag_ = heightFilterPanel_->mapFromGlobal(mouse->globalPos());
        heightFilterDragging_ = true;
        heightFilterTitle_->setCursor(Qt::ClosedHandCursor);
        heightFilterPanel_->raise();
        return true;
      }
    } else if (e->type() == QEvent::MouseMove && heightFilterDragging_) {
      auto *mouse = static_cast<QMouseEvent *>(e);
      QPoint position = canvas_->mapFromGlobal(mouse->globalPos() - heightFilterDrag_);
      position.setX(qBound(0, position.x(),
                           qMax(0, canvas_->width() - heightFilterPanel_->width())));
      position.setY(qBound(0, position.y(),
                           qMax(0, canvas_->height() - heightFilterPanel_->height())));
      heightFilterPanel_->move(position);
      heightFilterPlaced_ = true;
      return true;
    } else if (e->type() == QEvent::MouseButtonRelease &&
               static_cast<QMouseEvent *>(e)->button() == Qt::LeftButton &&
               heightFilterDragging_) {
      heightFilterDragging_ = false;
      heightFilterTitle_->setCursor(Qt::OpenHandCursor);
      return true;
    }
  }
  auto *chip = qobject_cast<QPushButton *>(o);
  if (chip && chip->objectName() == "routeChip" &&
      e->type() == QEvent::MouseButtonRelease &&
      chip->property("routeDoubleClickPending").toBool()) {
    chip->setProperty("routeDoubleClickPending", false);
    const int row = chip->property("routeIndex").toInt();
    QTimer::singleShot(0, this, [this, row] {
      if (row >= waypoints_->rowCount())
        return;
      waypoints_->selectRow(row);
      setEditorCollapsed(false);
    });
    return true;
  }
  if (e->type() == QEvent::MouseButtonDblClick) {
    if ((o == editor_ || o == routeStrip_) &&
        static_cast<QMouseEvent *>(e)->button() == Qt::LeftButton) {
      setEditorCollapsed(false);
      return true;
    }
    if (chip && chip->objectName() == "routeChip" &&
        static_cast<QMouseEvent *>(e)->button() == Qt::LeftButton) {
      chip->setProperty("routeClickGeneration",
                        chip->property("routeClickGeneration").toInt() + 1);
      chip->setProperty("routeDoubleClickPending", true);
      return true;
    }
  }
  if (o == routeStrip_ && e->type() == QEvent::KeyPress) {
    const int key = static_cast<QKeyEvent *>(e)->key();
    if (key == Qt::Key_Return || key == Qt::Key_Enter || key == Qt::Key_Space) {
      setEditorCollapsed(false);
      return true;
    }
  }
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
  if (o == window_->menuBar() && e->type() == QEvent::Resize && latest_)
    QTimer::singleShot(0, this, &MissionController::placeLatestLogButton);
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
      // Project mouse movement onto the video's upper-right diagonal. The
      // lower-left corner stays fixed as the window follows the frame ratio.
      const double heightPerWidth =
          double(videoFrameSize_.height()) / videoFrameSize_.width();
      const double widthDelta =
          (delta.x() - heightPerWidth * delta.y()) /
          (1.0 + heightPerWidth * heightPerWidth);
      const int maxWidth = qMax(
          video_->minimumWidth(),
          qMin(canvas_->width() - videoResizeBottomLeft_.x(),
               qFloor((videoResizeBottomLeft_.y() + 1) / heightPerWidth)));
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
                                  const QVector<MissionPoint> &pts,
                                  bool usePointColors, bool filterHeight) {
  g->removeAllChildren();
  if (pts.isEmpty())
    return;
  int visibleCount = pts.size();
  if (filterHeight && heightFilterEnabled_ && heightFilterEnabled_->isChecked() &&
      heightFilterHasRange_) {
    visibleCount = 0;
    for (const auto &p : pts)
      visibleCount += p.z <= heightFilterCutoff_;
  } else {
    filterHeight = false;
  }
  if (!visibleCount)
    return;
  auto *c = new ccPointCloud(usePointColors ? "XYZRGB" : "XYZ");
  if (!c->reserve(visibleCount) || !c->reserveTheRGBTable()) {
    delete c;
    log("点云显存数据分配失败");
    return;
  }
  for (const auto &p : pts) {
    if (filterHeight && p.z > heightFilterCutoff_)
      continue;
    c->addPoint(CCVector3(p.x, p.y, p.z));
    c->addColor(usePointColors ? p.r : 190,
                usePointColors ? p.g : 210,
                usePointColors ? p.b : 220);
  }
  c->showColors(true);
  c->setPointSize(2);
  c->setDisplay(gl_);
  g->addChild(c);
}
void MissionController::refreshHeightFilterLabel() {
  if (!heightFilterLabel_)
    return;
  if (!heightFilterHasRange_) {
    heightFilterLabel_->setText("等点云");
  } else if (!heightFilterEnabled_->isChecked()) {
    heightFilterLabel_->setText("全部");
  } else {
    heightFilterLabel_->setText(
        QString("≤ %1 m").arg(heightFilterCutoff_, 0, 'f', 2));
  }
  heightRangeTop_->setText(heightFilterHasRange_
                               ? QString("%1 m").arg(heightFilterMax_, 0, 'f', 2)
                               : QStringLiteral("—"));
  heightRangeBottom_->setText(heightFilterHasRange_
                                  ? QString("%1 m").arg(heightFilterMin_, 0, 'f', 2)
                                  : QStringLiteral("—"));
}
void MissionController::refreshHeightFilterRange() {
  if (!heightSlider_)
    return;
  const bool wasAtMaximum = !heightFilterHasRange_ ||
                            heightSlider_->value() == heightSlider_->maximum();
  bool hasRange = false;
  double minimum = 0, maximum = 0;
  auto scan = [&](const QVector<MissionPoint> &points) {
    for (const auto &p : points) {
      if (!std::isfinite(p.z))
        continue;
      if (!hasRange) {
        minimum = maximum = p.z;
        hasRange = true;
      } else {
        minimum = std::min(minimum, double(p.z));
        maximum = std::max(maximum, double(p.z));
      }
    }
  };
  if (!referenceMap_)
    scan(mapPoints_);
  scan(colorMapPoints_);
  heightFilterHasRange_ = hasRange;
  if (hasRange) {
    heightFilterMin_ = minimum;
    heightFilterMax_ = maximum;
    if (wasAtMaximum)
      heightFilterCutoff_ = maximum;
    else
      heightFilterCutoff_ = qBound(minimum, heightFilterCutoff_, maximum);
    const int sliderValue = maximum > minimum
        ? qRound((heightFilterCutoff_ - minimum) /
                 (maximum - minimum) * heightSlider_->maximum())
        : heightSlider_->maximum();
    const QSignalBlocker block(heightSlider_);
    heightSlider_->setValue(sliderValue);
  } else {
    const QSignalBlocker block(heightSlider_);
    heightSlider_->setValue(heightSlider_->maximum());
  }
  heightSlider_->setEnabled(hasRange && heightFilterEnabled_->isChecked());
  refreshHeightFilterLabel();
}
void MissionController::makeLine(ccHObject *g, const QVector<QVector3D> &pts,
                                 unsigned char r, unsigned char green,
                                 unsigned char b, float width) {
  g->removeAllChildren();
  addOverlayLine(g, pts, r, green, b, width);
}
void MissionController::addOverlayLine(ccHObject *g,
                                       const QVector<QVector3D> &pts,
                                       unsigned char r, unsigned char green,
                                       unsigned char b, float width) {
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
  gridCells_.clear();
  gridVersion_ = 0;
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
  makeVoxelGrid(grid_, gridCenters_, gridResolution_);
}
void MissionController::makeVoxelGrid(ccHObject *target,
                                    const QVector<QVector3D> &centers,
                                    double resolution) {
  target->removeAllChildren();
  // An alpha-zero mesh can still write to the depth buffer and hide other
  // layers, so skip its geometry entirely.
  if (centers.isEmpty() || resolution <= 0 || gridOpacity_ <= 0.0)
    return;
  // EGO publishes centers on one regular lattice. At full size, draw only
  // outer faces; smaller cubes need every face visible through the gaps.
  using GridKey = std::tuple<long long, long long, long long>;
  const auto &origin = centers.first();
  const auto keyFor = [resolution, &origin](const QVector3D &p) -> GridKey {
    return {std::llround((double(p.x()) - origin.x()) / resolution),
            std::llround((double(p.y()) - origin.y()) / resolution),
            std::llround((double(p.z()) - origin.z()) / resolution)};
  };
  std::set<GridKey> occupied;
  std::vector<std::pair<GridKey, int>> cells;
  cells.reserve(size_t(centers.size()));
  for (int i = 0; i < centers.size(); ++i) {
    const auto key = keyFor(centers[i]);
    if (occupied.insert(key).second)
      cells.emplace_back(key, i);
  }
  auto *vertices = new ccPointCloud("EGO voxel vertices");
  auto *mesh = new ccMesh(vertices);
  mesh->addChild(vertices);
  vertices->setEnabled(false);
  const unsigned count = unsigned(cells.size());
  if (!vertices->reserve(count * 8) || !vertices->reserveTheRGBTable() ||
      !mesh->reserve(count * 12)) {
    delete mesh;
    log("EGO 栅格显示内存分配失败");
    return;
  }
  const float half = float(resolution * gridSizePercent_ / 200.0);
  const auto alpha = static_cast<unsigned char>(std::lround(gridOpacity_ * 255.0));
  static const int neighbors[6][3] = {
      {0, 0, -1}, {0, 0, 1}, {0, -1, 0},
      {0, 1, 0},  {-1, 0, 0}, {1, 0, 0}};
  static const unsigned faces[12][3] = {
      {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
      {0, 1, 5}, {0, 5, 4}, {2, 6, 7}, {2, 7, 3},
      {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
  for (unsigned i = 0; i < count; ++i) {
    const auto &key = cells[i].first;
    bool visible[6];
    bool hasVisibleFace = false;
    for (int face = 0; face < 6; ++face) {
      visible[face] = gridSizePercent_ < 100 || occupied.count(
          GridKey{std::get<0>(key) + neighbors[face][0],
                  std::get<1>(key) + neighbors[face][1],
                  std::get<2>(key) + neighbors[face][2]}) == 0;
      hasVisibleFace |= visible[face];
    }
    if (!hasVisibleFace)
      continue;
    const auto &p = centers[cells[i].second];
    const unsigned base = unsigned(vertices->size());
    for (int corner = 0; corner < 8; ++corner) {
      vertices->addPoint(CCVector3(p.x() + ((corner & 1) ? half : -half),
                                  p.y() + ((corner & 2) ? half : -half),
                                  p.z() + ((corner & 4) ? half : -half)));
      vertices->addColor(gridColor_.red(), gridColor_.green(),
                         gridColor_.blue(), alpha);
    }
    for (int face = 0; face < 6; ++face) {
      if (!visible[face])
        continue;
      for (int triangle = 0; triangle < 2; ++triangle) {
        const auto &f = faces[face * 2 + triangle];
        mesh->addTriangle(base + f[0], base + f[1], base + f[2]);
      }
    }
  }
  mesh->setName("EGO occupancy voxels");
  vertices->showColors(true);
  mesh->showColors(true);
  mesh->showNormals(mesh->computePerTriangleNormals());
  mesh->setDisplay(gl_);
  target->addChild(mesh);
}
void MissionController::renderFlightReplay(const FlightReplayFrame &frame) {
  if (!gl_ || !replay_)
    return;
  replay_->setVisible(true);
  makeLine(replayTrail_, frame.trail, 255, 66, 66, 3);
  makeLine(replaySetpoints_, frame.setpoints, 67, 195, 255, 2);
  if (frame.cloudTimeNs != replayCloudTimeNs_) {
    makeCloud(replayCloud_, frame.cloud);
    replayCloudTimeNs_ = frame.cloudTimeNs;
  }
  const bool gridUsable = frame.gridFrame == frame_ && frame.resolution > 0 &&
                          frame.resolution < 10 && frame.voxels.size() <= 50000;
  const double gridTime = gridUsable ? frame.gridTimeNs : 0;
  if (gridTime != replayGridTimeNs_) {
    if (gridTime)
      makeVoxelGrid(replayGrid_, frame.voxels, frame.resolution);
    else
      replayGrid_->removeAllChildren();
    replayGridTimeNs_ = gridTime;
  }
  if (replayAircraft_) {
    replayAircraft_->setVisible(frame.poseValid);
    if (frame.poseValid) {
      QMatrix4x4 transform;
      transform.translate(frame.position);
      transform.rotate(float(frame.yaw * 180.0 / M_PI), 0, 0, 1);
      replayAircraft_->setGLTransformation(ccGLMatrix(transform.constData()));
    }
  }
  const QString angles = frame.hudPoseValid
      ? QString("%1°  /  %2°  /  %3°")
            .arg(frame.rollDeg, 0, 'f', 0)
            .arg(frame.pitchDeg, 0, 'f', 0)
            .arg(frame.yawDeg, 0, 'f', 0) : QStringLiteral("— / — / —");
  const QString position = frame.hudPoseValid
      ? QString("%1 / %2 / %3 m")
            .arg(frame.hudPosition.x(), 0, 'f', 1)
            .arg(frame.hudPosition.y(), 0, 'f', 1)
            .arg(frame.hudPosition.z(), 0, 'f', 1)
      : QStringLiteral("— / — / —");
  attitude_->armed = frame.fcuStateValid
                         ? (frame.armed ? "已解锁" : "未解锁") : "未知";
  attitude_->flightMode = frame.fcuStateValid && !frame.flightMode.isEmpty() &&
                                  frame.flightMode != "UNKNOWN"
                              ? frame.flightMode : "未知";
  attitude_->angles = angles;
  attitude_->speed = frame.hudVelocityValid
                         ? QString::number(frame.speedMps, 'f', 1) + " m/s" : "—";
  attitude_->position = position;
  attitude_->battery = frame.hudBatteryValid
                           ? QString::number(frame.batteryPercent, 'f', 0) + "%"
                           : "—";
  attitude_->flightTime = frame.flightStatsValid
                              ? flightTimeText(frame.flightSeconds) : "—";
  attitude_->flightDistance = frame.flightStatsValid
                                  ? flightDistanceText(frame.flightDistanceM) : "—";
  attitude_->notice.clear();
  attitude_->context = "历史回放";
  attitude_->valid = frame.hudPoseValid;
  attitude_->roll = frame.rollDeg;
  attitude_->pitch = frame.pitchDeg;
  attitude_->update();
  gl_->redraw();
}
void MissionController::clearFlightReplay() {
  if (!gl_ || !replay_)
    return;
  replay_->setVisible(false);
  replayTrail_->removeAllChildren();
  replaySetpoints_->removeAllChildren();
  replayGrid_->removeAllChildren();
  replayCloud_->removeAllChildren();
  replayGridTimeNs_ = 0;
  replayCloudTimeNs_ = 0;
  if (replayAircraft_)
    replayAircraft_->setVisible(false);
  for (auto it = replayVisibility_.cbegin(); it != replayVisibility_.cend(); ++it)
    it.key()->setEnabled(it.value());
  replayVisibility_.clear();
  gl_->redraw();
}
void MissionController::updateScene() {
  if (!gl_ || !dirty_)
    return;
  dirty_ = false;
  if (mapDirty_ || colorMapDirty_)
    refreshHeightFilterRange();
  if (liveDirty_) {
    makeCloud(live_, livePoints_);
    ++liveCloudVersion_;
    liveDirty_ = false;
  }
  if (mapDirty_) {
    makeCloud(map_, mapPoints_, referenceMap_, !referenceMap_);
    ++mapCloudVersion_;
    mapDirty_ = false;
  }
  if (colorMapDirty_) {
    makeCloud(colorMap_, colorMapPoints_, true, true);
    colorMapDirty_ = false;
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
void MissionController::updateOdinSensorPose(const mission::OdinSensorPose &pose) {
  lastSensorPose_ = pose;
  sensorAge_.restart();
  rebuildOdinGeometry();
  refreshOdinOverlayStatus();
}
void MissionController::refreshOdinOverlayStatus() {
  if (!sensorOverlayStatus_)
    return;
  const bool poseFresh = sensorAge_.isValid() && sensorAge_.elapsed() <= 1500;
  const bool projectionFresh = projectionAge_.isValid() &&
                               projectionAge_.elapsed() <= 3000;
  QString poseLine;
  QString projectionLine;
  QString detail;
  if (!ready_) {
    poseLine = "视场等待任务机连接";
    projectionLine = "贴图等待同步图像";
  } else if (!client_.odinProjectionSupported()) {
    poseLine = "视场/贴图暂不可用";
    projectionLine = "请更新任务机代理";
  } else if (!poseFresh) {
    poseLine = "相机/雷达位姿待更新";
    projectionLine = projectionFresh
                         ? QString("贴图 %1 点").arg(projectionPoints_.size())
                         : QStringLiteral("贴图等待同步图像");
  } else {
    const auto &pose = lastSensorPose_;
    poseLine = QString("相机%1 · 雷达%2 · 膨胀%3")
                   .arg(pose.camera_valid() ? "✓" : "×")
                   .arg(pose.lidar_valid() ? "✓" : "×")
                   .arg(pose.inflation_valid()
                            ? QString("%1m").arg(pose.inflation_m(), 0, 'f', 2)
                            : QStringLiteral("待获取"));
    projectionLine = projectionFresh
                         ? QString("贴图 %1 点").arg(projectionPoints_.size())
                         : QStringLiteral("贴图等待同步图像");
    if (pose.inflation_valid()) {
      projectionLine += pose.inflation_runtime() ? " · 运行参数" : " · 配置值";
      detail = QString("EGO 膨胀 %1 m（%2）")
                   .arg(pose.inflation_m(), 0, 'f', 2)
                   .arg(pose.inflation_runtime() ? "运行参数" : "配置值");
      if (pose.grid_resolution_m() > 0)
        detail += QString("；显示半边 %1 m")
                      .arg(std::ceil(pose.inflation_m() /
                                     pose.grid_resolution_m()) *
                                   pose.grid_resolution_m() +
                                   pose.grid_resolution_m() / 2.0,
                           0, 'f', 2);
    }
    detail = QString("相机：%1；雷达：%2\n%3\n图像贴合：%4 个点云落点")
                 .arg(pose.camera_valid() ? "光学位姿已就绪" : "缺少 TF/标定")
                 .arg(pose.lidar_valid() ? "TF 已就绪" : "缺少 TF")
                 .arg(detail.isEmpty() ? "等待 EGO 膨胀参数" : detail)
                 .arg(projectionFresh ? projectionPoints_.size() : 0);
  }
  sensorOverlayStatus_->setText(poseLine + '\n' + projectionLine);
  sensorOverlayStatus_->setToolTip(detail);
}
void MissionController::updateOdinProjection(const mission::OdinProjection &projection) {
  QString error;
  projectionPoints_ = CloudIO::decode(
      QByteArray::fromStdString(projection.point_data()), error);
  if (!error.isEmpty()) {
    log(error, "图像投影");
    return;
  }
  makeCloud(imageProjection_, projectionPoints_, true, false);
  if (imageProjection_->getChildrenNumber())
    if (auto *cloud = dynamic_cast<ccPointCloud *>(imageProjection_->getChild(0)))
      cloud->setPointSize(4);
  // Sparse projector guide rays use the exact camera pose bundled with this
  // image, and terminate at actual cloud points rather than a fixed plane.
  const QVector3D camera = vec(projection.camera_position());
  const int rayStep = qMax(1, projectionPoints_.size() / 6);
  int rayCount = 0;
  for (int i = 0; i < projectionPoints_.size() && rayCount < 6; i += rayStep) {
    const auto &point = projectionPoints_[i];
    const QVector3D end(point.x, point.y, point.z);
    const float distance = (end - camera).length();
    if (distance > .1f && distance < 8.0f) {
      addOverlayLine(imageProjection_, {camera, end}, point.r, point.g,
                     point.b, 1.4f);
      ++rayCount;
    }
  }
  projectionAge_.restart();
  refreshOdinOverlayStatus();
  if (gl_)
    gl_->redraw();
}
void MissionController::clearOdinOverlays() {
  sensorAge_.invalidate();
  projectionAge_.invalidate();
  lastSensorPose_.Clear();
  projectionPoints_.clear();
  for (auto *layer : {cameraFov_, lidarFov_, imageProjection_, aircraftInflation_})
    if (layer)
      layer->removeAllChildren();
  refreshOdinOverlayStatus();
  if (gl_)
    gl_->redraw();
}
void MissionController::rebuildOdinGeometry() {
  if (!gl_ || !sensorAge_.isValid() || sensorAge_.elapsed() > 1500)
    return;
  cameraFov_->removeAllChildren();
  lidarFov_->removeAllChildren();
  aircraftInflation_->removeAllChildren();
  constexpr double deg = 3.14159265358979323846 / 180.0;
  const auto drawFov = [this](ccHObject *layer, const mission::Vec3 &position,
                              const mission::Quaternion &orientation,
                              double horizontal, double vertical,
                              bool optical, unsigned char r, unsigned char g,
                              unsigned char b) {
    const QVector3D origin = vec(position);
    const QQuaternion rotation(orientation.w(), orientation.x(),
                               orientation.y(), orientation.z());
    const float dx = float(std::tan(horizontal * deg / 2.0));
    const float dy = float(std::tan(vertical * deg / 2.0));
    const float range = 1.8f;
    QVector<QVector3D> corners;
    for (int sy : {-1, 1})
      for (int sx : {-1, 1}) {
        const QVector3D local = optical
            ? QVector3D(sx * dx, sy * dy, 1.0f)
            : QVector3D(1.0f, sx * dx, sy * dy);
        corners << origin + rotation.rotatedVector(local.normalized() * range);
      }
    for (const auto &corner : corners)
      addOverlayLine(layer, {origin, corner}, r, g, b, 1.7f);
    for (const auto &edge : {QPair<int,int>(0,1), {1,3}, {3,2}, {2,0}})
      addOverlayLine(layer, {corners[edge.first], corners[edge.second]},
                     r, g, b, 2.2f);
    const QVector3D forward = optical ? QVector3D(0,0,1) : QVector3D(1,0,0);
    addOverlayLine(layer,
                   {origin, origin + rotation.rotatedVector(forward * range)},
                   r, g, b, 1.2f);
  };
  if (lastSensorPose_.camera_valid())
    drawFov(cameraFov_, lastSensorPose_.camera_position(),
            lastSensorPose_.camera_orientation(), 129, 104, true,
            kCameraFovColor.red, kCameraFovColor.green, kCameraFovColor.blue);
  if (lastSensorPose_.lidar_valid())
    drawFov(lidarFov_, lastSensorPose_.lidar_position(),
            lastSensorPose_.lidar_orientation(), 120, 90, false,
            kLidarFovColor.red, kLidarFovColor.green, kLidarFovColor.blue);
  if (lastSensorPose_.inflation_valid() && lastVehicle_.pose_valid() &&
      vehicleAge_.isValid() && vehicleAge_.elapsed() < 1500) {
    // EGO inflates each occupied voxel by ceil(distance/resolution) steps on
    // each world axis. This box is the equivalent point-robot collision volume.
    const double step = lastSensorPose_.grid_resolution_m();
    const float radius = float(std::ceil(lastSensorPose_.inflation_m() / step) *
                               step + step / 2.0);
    const QVector3D p = vec(lastVehicle_.position());
    QVector<QVector3D> corners;
    for (int z : {-1, 1})
      for (int y : {-1, 1})
        for (int x : {-1, 1})
          corners << p + QVector3D(x * radius, y * radius, z * radius);
    for (int i = 0; i < 8; ++i)
      for (int bit : {1, 2, 4})
        if (!(i & bit))
          addOverlayLine(aircraftInflation_, {corners[i], corners[i | bit]},
                         kAircraftInflationColor.red,
                         kAircraftInflationColor.green,
                         kAircraftInflationColor.blue, 2.0f);
  }
  gl_->redraw();
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
