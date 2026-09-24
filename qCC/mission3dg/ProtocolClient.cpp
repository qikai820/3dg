#include "ProtocolClient.h"
#include <QDateTime>
#include <QUuid>
#include <cmath>

static bool finiteVec(const mission::Vec3 &p) {
  return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
         std::abs(p.x()) < 1e7 && std::abs(p.y()) < 1e7 &&
         std::abs(p.z()) < 1e7;
}
ProtocolClient::ProtocolClient(QObject *parent)
    : QObject(parent),
      localSession_(QUuid::createUuid().toString(QUuid::WithoutBraces)) {
  auto wire = [this](QWebSocket &socket, int channel) {
    connect(&socket, &QWebSocket::binaryMessageReceived, this,
            [this, channel](const QByteArray &b) {
              if (b.size() > 8 * 1024 * 1024) {
                Q_EMIT diagnostic("消息超过 8 MiB 限制");
                return;
              }
              ingest(b, channel);
            });
    connect(&socket,
            QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error),
            this, [this, &socket](QAbstractSocket::SocketError) {
              Q_EMIT diagnostic(socket.errorString());
              if (&socket == &control_ && desired_ && !reconnect_.isActive())
                reconnect_.start(3000);
            });
    socket.setMaxAllowedIncomingMessageSize(8 * 1024 * 1024);
    socket.setMaxAllowedIncomingFrameSize(8 * 1024 * 1024);
  };
  wire(control_, 0);
  wire(cloud_, 1);
  wire(files_, 2);
  connect(&control_, &QWebSocket::connected, this, [this] {
    remoteSession_.clear();
    sequences_.clear();
    lastRx_.start();
    mission::Envelope e;
    e.mutable_hello()->set_name("3DG");
    e.mutable_hello()->add_capabilities("xyzrgb-le16");
    e.mutable_hello()->add_capabilities("sampled-trajectory");
    e.mutable_hello()->add_capabilities("ego-voxel-grid-v1");
    e.mutable_hello()->add_capabilities("yaml-config-v1");
    transmit(e);
    Q_EMIT stateChanged("已连接，等待协议握手", false);
  });
  connect(&control_, &QWebSocket::disconnected, this, [this] {
    ready_ = false;
    latestCloud_.Clear();
    latestGrid_.Clear();
    cloud_.abort();
    files_.abort();
    pending_.clear();
    Q_EMIT stateChanged(desired_ ? "连接中断，等待重连" : "未连接", false);
    if (desired_)
      reconnect_.start(3000);
  });
  reconnect_.setSingleShot(true);
  connect(&reconnect_, &QTimer::timeout, this, &ProtocolClient::connectSockets);
  heartbeat_.setInterval(1000);
  connect(&heartbeat_, &QTimer::timeout, this, [this] {
    if (control_.state() != QAbstractSocket::ConnectedState)
      return;
    if (lastRx_.isValid() && lastRx_.elapsed() > 6000) {
      Q_EMIT diagnostic("心跳超时，当前状态已失效");
      control_.abort();
      return;
    }
    mission::Envelope e;
    e.mutable_heartbeat();
    transmit(e);
    if (ready_) {
      QUrl u = base_;
      if (cloud_.state() == QAbstractSocket::UnconnectedState) {
        u.setPath("/cloud");
        cloud_.open(u);
      }
      if (files_.state() == QAbstractSocket::UnconnectedState) {
        u.setPath("/files");
        files_.open(u);
      }
    }
    const auto now = QDateTime::currentMSecsSinceEpoch();
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (now - it.value() > 30000) {
        Q_EMIT diagnostic("命令超时（结果未知，不自动重发）: " + it.key());
        it = pending_.erase(it);
      } else
        ++it;
    }
  });
  heartbeat_.start();
  cloudDispatch_.setInterval(100);
  connect(&cloudDispatch_, &QTimer::timeout, this, [this] {
    // Sequence/session validation occurs on arrival, before coalescing. Keep
    // independent slots so a sensor frame cannot overwrite the EGO snapshot.
    if (latestCloud_.has_cloud()) {
      mission::Envelope e;
      e.Swap(&latestCloud_);
      Q_EMIT message(e);
    }
    if (latestGrid_.has_grid()) {
      mission::Envelope e;
      e.Swap(&latestGrid_);
      Q_EMIT message(e);
    }
  });
  cloudDispatch_.start();
}
void ProtocolClient::open(const QUrl &url) {
  stop();
  base_ = url;
  desired_ = true;
  connectSockets();
}
void ProtocolClient::connectSockets() {
  if (!desired_)
    return;
  QUrl u = base_;
  u.setPath("/control");
  control_.open(u);
}
void ProtocolClient::stop() {
  desired_ = false;
  ready_ = false;
  reconnect_.stop();
  latestCloud_.Clear();
  latestGrid_.Clear();
  pending_.clear();
  control_.abort();
  cloud_.abort();
  files_.abort();
  remoteSession_.clear();
  sequences_.clear();
  Q_EMIT stateChanged("未连接", false);
}
QByteArray ProtocolClient::encode(const mission::Envelope &e) {
  const auto s = e.SerializeAsString();
  return QByteArray(s.data(), int(s.size()));
}
void ProtocolClient::transmit(mission::Envelope e) {
  e.set_protocol_version(1);
  e.set_session_id(localSession_.toStdString());
  e.set_sequence(++sequence_);
  e.set_timestamp_ns(quint64(QDateTime::currentMSecsSinceEpoch()) * 1000000);
  e.set_time_domain("unix");
  control_.sendBinaryMessage(encode(e));
}
QString ProtocolClient::sendCommand(const mission::Command &command) {
  if (!connected() || control_.bytesToWrite() > 1024 * 1024) {
    Q_EMIT diagnostic("控制连接未就绪，命令未发送");
    return {};
  }
  mission::Envelope e;
  const auto id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  e.set_request_id(id.toStdString());
  *e.mutable_command() = command;
  pending_[id] = QDateTime::currentMSecsSinceEpoch();
  transmit(e);
  return id;
}
QString ProtocolClient::validate(const mission::Envelope &e) {
  if (e.protocol_version() != 1 || e.session_id().empty() ||
      e.session_id().size() > 128 || !e.sequence())
    return "协议版本、会话或序号无效";
  if (e.payload_case() == mission::Envelope::PAYLOAD_NOT_SET)
    return "缺少消息内容";
  if (e.has_cloud()) {
    const auto &c = e.cloud();
    if (c.frame_id().empty() || c.map_id().empty() ||
        c.layout() != mission::CloudFrame::XYZ_RGB_LE16 ||
        c.point_count() > 300000 ||
        c.point_data().size() != quint64(c.point_count()) * 16)
      return "点云布局或长度无效";
  }
  if (e.has_vehicle()) {
    const auto &v = e.vehicle();
    if (v.frame_id().empty() || v.map_id().empty())
      return "遥测缺少坐标系或地图";
    if (v.pose_valid()) {
      const auto &q = v.orientation();
      double norm =
          q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w();
      if (!finiteVec(v.position()) || !std::isfinite(norm) ||
          std::abs(norm - 1) > 0.02)
        return "位姿或四元数无效";
    }
    if (v.velocity_valid() && !finiteVec(v.velocity()))
      return "速度无效";
    if (v.battery_valid() &&
        (!std::isfinite(v.battery_percent()) || v.battery_percent() < 0 ||
         v.battery_percent() > 100))
      return "电量无效";
  }
  if (e.has_grid()) {
    const auto &g = e.grid();
    if (g.frame_id().empty() || g.map_id().empty() ||
        !std::isfinite(g.resolution_m()) || g.resolution_m() < .02 ||
        g.resolution_m() > 5 || g.centers_size() > 50000)
      return "EGO 栅格坐标系、分辨率或数量无效";
    for (const auto &p : g.centers())
      if (!finiteVec(p))
        return "EGO 栅格包含无效坐标";
  }
  if (e.has_trajectory()) {
    const auto &t = e.trajectory();
    if (!mission::PlannerTrajectory_State_IsValid(t.state()))
      return "未知轨迹状态";
    if (t.frame_id().empty() || t.map_id().empty() || t.points_size() > 10000)
      return "轨迹坐标系或点数无效";
    if (t.state() == mission::PlannerTrajectory::VALID && t.points_size() < 2)
      return "有效轨迹至少需要两个点";
    double prev = -1;
    for (const auto &p : t.points()) {
      if (!finiteVec(p.position()) || !std::isfinite(p.time_from_start_s()) ||
          p.time_from_start_s() < 0 || p.time_from_start_s() <= prev)
        return "轨迹坐标或时间顺序无效";
      prev = p.time_from_start_s();
    }
  }
  if (e.has_file() &&
      (e.file().total_size() > 512ull * 1024 * 1024 ||
       e.file().data().size() > 1024 * 1024 ||
       e.file().offset() > e.file().total_size() ||
       e.file().data().size() > e.file().total_size() - e.file().offset()))
    return "文件大小超过限制";
  if (e.has_result() &&
      !mission::CommandResult_State_IsValid(e.result().state()))
    return "未知命令结果";
  if (e.has_log() &&
      (e.log().text().size() > 8192 || e.log().source().size() > 128))
    return "日志过长";
  if (e.has_yaml_catalog() && e.yaml_catalog().files_size() > 16)
    return "配置文件列表过长";
  if (e.has_yaml_document() &&
      (e.yaml_document().id().empty() ||
       e.yaml_document().content_json().size() > 1024 * 1024 ||
       e.yaml_document().metadata_json().size() > 256 * 1024 ||
       e.yaml_document().revision().size() != 64))
    return "YAML 配置内容或版本无效";
  return {};
}
void ProtocolClient::ingest(const QByteArray &bytes, int channel) {
  if (bytes.size() > 8 * 1024 * 1024)
    return;
  mission::Envelope e;
  if (!e.ParseFromArray(bytes.constData(), bytes.size())) {
    Q_EMIT diagnostic("无法解析 Protobuf 消息");
    return;
  }
  const auto error = validate(e);
  if (!error.isEmpty()) {
    Q_EMIT diagnostic(error);
    return;
  }
  if ((channel == 1 && !e.has_cloud() && !e.has_grid()) ||
      (channel == 2 && !e.has_file()) ||
      (channel == 0 && (e.has_cloud() || e.has_grid() || e.has_file()))) {
    Q_EMIT diagnostic("消息与通道不匹配");
    return;
  }
  const QString session = QString::fromStdString(e.session_id());
  if (e.has_hello() && channel == 0) {
    if (session != remoteSession_) {
      remoteSession_ = session;
      sequences_.clear();
      latestCloud_.Clear();
      latestGrid_.Clear();
      Q_EMIT sessionChanged();
    }
    ready_ = true;
    Q_EMIT stateChanged("任务机在线", true);
    QUrl u = base_;
    if (desired_) {
      u.setPath("/cloud");
      if (cloud_.state() == QAbstractSocket::UnconnectedState)
        cloud_.open(u);
      u.setPath("/files");
      if (files_.state() == QAbstractSocket::UnconnectedState)
        files_.open(u);
    }
  } else if (session != remoteSession_ || !ready_) {
    Q_EMIT diagnostic("未握手或会话不匹配，消息已丢弃");
    return;
  }
  if (e.sequence() <= sequences_.value(channel))
    return;
  sequences_[channel] = e.sequence();
  if (channel == 0)
    lastRx_.restart();
  if (e.has_result()) {
    const QString id = QString::fromStdString(e.request_id());
    if (!pending_.contains(id))
      return;
    if (e.result().state() != mission::CommandResult::ACCEPTED)
      pending_.remove(id);
  }
  if (channel == 1 && desired_) {
    if (e.has_grid())
      latestGrid_.Swap(&e);
    else
      latestCloud_.Swap(&e);
    return;
  }
  Q_EMIT message(e);
}
