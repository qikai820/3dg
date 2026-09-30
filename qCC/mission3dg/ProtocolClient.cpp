#include "ProtocolClient.h"
#include <QDateTime>
#include <QNetworkProxy>
#include <QUuid>
#include <algorithm>
#include <cmath>

static bool finiteVec(const mission::Vec3 &p) {
  return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) &&
         std::abs(p.x()) < 1e7 && std::abs(p.y()) < 1e7 &&
         std::abs(p.z()) < 1e7;
}
ProtocolClient::ProtocolClient(QObject *parent)
    : QObject(parent),
      localSession_(QUuid::createUuid().toString(QUuid::WithoutBraces)) {
  // The task-machine agent is a direct LAN peer; desktop HTTP proxies cannot
  // carry these WebSocket channels and may change the source address.
  const QNetworkProxy direct(QNetworkProxy::NoProxy);
  control_.setProxy(direct);
  cloud_.setProxy(direct);
  files_.setProxy(direct);
  auto wire = [this](QWebSocket &socket, int channel) {
    connect(&socket, &QWebSocket::binaryMessageReceived, this,
            [this, channel](const QByteArray &b) {
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
    resetLinkStats();
    resetTimeSync();
    remoteSession_.clear();
    sequences_.clear();
    lastRx_.start();
    mission::Envelope e;
    e.mutable_hello()->set_name("3DG");
    e.mutable_hello()->add_capabilities("xyzrgb-le16");
    e.mutable_hello()->add_capabilities("sampled-trajectory");
    e.mutable_hello()->add_capabilities("ego-voxel-grid-v1");
    e.mutable_hello()->add_capabilities("ego-voxel-grid-delta-v1");
    e.mutable_hello()->add_capabilities("yaml-config-v1");
    e.mutable_hello()->add_capabilities("odin-start-modes-v1");
    transmit(e);
    Q_EMIT stateChanged("已连接，等待协议握手", false);
  });
  connect(&control_, &QWebSocket::disconnected, this, [this] {
    ready_ = false;
    resetTimeSync();
    latestCloud_.Clear();
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
    if (ready_ && timeSyncSupported_ && timeProbeRequest_.isEmpty() &&
        setTimeRequest_.isEmpty() &&
        (!lastTimeProbe_.isValid() || lastTimeProbe_.elapsed() >= 60000))
      probeTime();
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
      if (now > it.value()) {
        Q_EMIT diagnostic("命令超时（结果未知，不自动重发）: " + it.key());
        if (it.key() == timeProbeRequest_)
          timeProbeRequest_.clear();
        if (it.key() == setTimeRequest_)
          setTimeRequest_.clear();
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
  });
  cloudDispatch_.start();
  linkStatsTimer_.setInterval(1000);
  connect(&linkStatsTimer_, &QTimer::timeout, this, [this] {
    const qint64 elapsedMs = linkStatsAge_.restart();
    if (elapsedMs <= 0)
      return;
    receiveMbps_ = double(receivedBytes_ - previousReceivedBytes_) * 8.0 /
                   (double(elapsedMs) * 1000.0);
    transmitMbps_ = double(transmittedBytes_ - previousTransmittedBytes_) * 8.0 /
                    (double(elapsedMs) * 1000.0);
    previousReceivedBytes_ = receivedBytes_;
    previousTransmittedBytes_ = transmittedBytes_;
    Q_EMIT linkStatsChanged();
  });
  linkStatsAge_.start();
  linkStatsTimer_.start();
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
  routeEditSupported_ = false;
  routeDeleteSupported_ = false;
  resetTimeSync();
  reconnect_.stop();
  latestCloud_.Clear();
  pending_.clear();
  control_.abort();
  cloud_.abort();
  files_.abort();
  remoteSession_.clear();
  sequences_.clear();
  resetLinkStats();
  Q_EMIT stateChanged("未连接", false);
}
void ProtocolClient::resetLinkStats() {
  receivedMessages_ = invalidMessages_ = 0;
  receivedBytes_ = transmittedBytes_ = 0;
  previousReceivedBytes_ = previousTransmittedBytes_ = 0;
  receiveMbps_ = transmitMbps_ = 0;
  linkStatsAge_.restart();
  Q_EMIT linkStatsChanged();
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
  const QByteArray bytes = encode(e);
  const qint64 queued = control_.sendBinaryMessage(bytes);
  if (queued > 0)
    transmittedBytes_ += quint64(queued);
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
  // START_EGO now waits for the real controller and setpoint stream. Its
  // bounded device-side startup can take longer than the usual 30 seconds.
  const qint64 timeoutMs = command.kind() == mission::Command::START_EGO
                               ? 120000 : 30000;
  pending_[id] = QDateTime::currentMSecsSinceEpoch() + timeoutMs;
  transmit(e);
  return id;
}
void ProtocolClient::requestGridResync() {
  // Reopening /cloud starts a new server-side delta stream with a full frame.
  cloud_.abort();
}
void ProtocolClient::resetTimeSync() {
  timeSyncSupported_ = false;
  timeChecked_ = false;
  timeProbeRequest_.clear();
  setTimeRequest_.clear();
  timeProbeAge_.invalidate();
  lastTimeProbe_.invalidate();
  lastTimeSync_.invalidate();
}
void ProtocolClient::probeTime() {
  mission::Command command;
  command.set_kind(mission::Command::GET_TIME);
  timeProbeSentMs_ = QDateTime::currentMSecsSinceEpoch();
  timeProbeAge_.start();
  timeProbeRequest_ = sendCommand(command);
  if (!timeProbeRequest_.isEmpty())
    lastTimeProbe_.start();
}
void ProtocolClient::handleTimeResult(const mission::Envelope &e) {
  if (e.result().state() == mission::CommandResult::ACCEPTED)
    return;
  const QString id = QString::fromStdString(e.request_id());
  if (id == setTimeRequest_) {
    setTimeRequest_.clear();
    if (e.result().state() == mission::CommandResult::FAILED) {
      Q_EMIT diagnostic("任务机校时失败：" + QString::fromStdString(e.result().detail()));
    } else {
      Q_EMIT diagnostic("任务机已执行校时，正在复查时间差");
      lastTimeProbe_.invalidate();
      probeTime();
    }
    return;
  }
  if (id != timeProbeRequest_)
    return;
  timeProbeRequest_.clear();
  if (e.result().state() == mission::CommandResult::FAILED) {
    Q_EMIT diagnostic("读取任务机时间失败：" + QString::fromStdString(e.result().detail()));
    return;
  }
  const qint64 rttMs = timeProbeAge_.elapsed();
  const qint64 remoteMs = qint64(e.timestamp_ns() / 1000000);
  if (rttMs > 2000 || e.time_domain() != "unix" || remoteMs <= 0 ||
      remoteMs >= 4102444800000LL) {
    Q_EMIT diagnostic("时间测量延迟过大或任务机时间戳无效，本次不校时");
    return;
  }
  const qint64 skewMs = remoteMs - (timeProbeSentMs_ + rttMs / 2);
  if (qAbs(skewMs) < 10000) {
    if (!timeChecked_ || lastTimeSync_.isValid())
      Q_EMIT diagnostic(QString("任务机与 3DG 时间差约 %1 秒，无需校时")
                            .arg(double(skewMs) / 1000, 0, 'f', 2));
    timeChecked_ = true;
    lastTimeSync_.invalidate();
    return;
  }
  timeChecked_ = true;
  if (lastTimeSync_.isValid() && lastTimeSync_.elapsed() < 60000) {
    Q_EMIT diagnostic(QString("任务机与 3DG 仍相差约 %1 秒，稍后复查")
                          .arg(double(skewMs) / 1000, 0, 'f', 2));
    return;
  }
  mission::Command command;
  command.set_kind(mission::Command::SET_TIME);
  command.set_unix_time_ns(quint64(QDateTime::currentMSecsSinceEpoch()) * 1000000);
  setTimeRequest_ = sendCommand(command);
  if (!setTimeRequest_.isEmpty()) {
    lastTimeSync_.start();
    Q_EMIT diagnostic(QString("任务机与 3DG 相差约 %1 秒，已请求校时")
                          .arg(double(skewMs) / 1000, 0, 'f', 2));
  }
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
    if (v.flight_mode().size() > 32 ||
        std::any_of(v.flight_mode().begin(), v.flight_mode().end(),
                    [](char ch) { return ch < 32 || ch > 126; }))
      return "飞行模式无效";
  }
  if (e.has_grid()) {
    const auto &g = e.grid();
    if (g.frame_id().empty() || g.map_id().empty() ||
        !std::isfinite(g.resolution_m()) || g.resolution_m() < .02 ||
        g.resolution_m() > 5 || g.centers_size() > 50000 ||
        g.added_size() > 50000 || g.removed_size() > 50000 ||
        (g.delta() ? (g.version() == 0 || g.base_version() == 0 ||
                      g.version() <= g.base_version() || g.centers_size() != 0)
                   : (g.base_version() != 0 || g.added_size() != 0 ||
                      g.removed_size() != 0)))
      return "EGO 栅格坐标系、分辨率或数量无效";
    for (const auto &p : g.centers())
      if (!finiteVec(p))
        return "EGO 栅格包含无效坐标";
    for (const auto &p : g.added())
      if (!finiteVec(p))
        return "EGO 栅格增量包含无效新增坐标";
    for (const auto &p : g.removed())
      if (!finiteVec(p))
        return "EGO 栅格增量包含无效删除坐标";
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
  if (e.has_odin_maps()) {
    if (e.odin_maps().maps_size() > 200)
      return "Odin 地图列表过长";
    for (const auto &map : e.odin_maps().maps())
      if (map.id().size() != 64 || map.name().empty() ||
          map.name().size() > 256 || map.size_bytes() == 0)
        return "Odin 地图条目无效";
  }
  return {};
}
void ProtocolClient::ingest(const QByteArray &bytes, int channel) {
  ++receivedMessages_;
  receivedBytes_ += quint64(bytes.size());
  if (bytes.size() > 8 * 1024 * 1024) {
    ++invalidMessages_;
    Q_EMIT diagnostic("消息超过 8 MiB 限制");
    return;
  }
  mission::Envelope e;
  if (!e.ParseFromArray(bytes.constData(), bytes.size())) {
    ++invalidMessages_;
    Q_EMIT diagnostic("无法解析 Protobuf 消息");
    return;
  }
  const auto error = validate(e);
  if (!error.isEmpty()) {
    ++invalidMessages_;
    Q_EMIT diagnostic(error);
    return;
  }
  if ((channel == 1 && !e.has_cloud() && !e.has_grid()) ||
      (channel == 2 && !e.has_file()) ||
      (channel == 0 && (e.has_cloud() || e.has_grid() || e.has_file()))) {
    ++invalidMessages_;
    Q_EMIT diagnostic("消息与通道不匹配");
    return;
  }
  const QString session = QString::fromStdString(e.session_id());
  if (e.has_hello() && channel == 0) {
    routeEditSupported_ = false;
    routeDeleteSupported_ = false;
    if (session != remoteSession_) {
      remoteSession_ = session;
      sequences_.clear();
      latestCloud_.Clear();
      resetTimeSync();
      Q_EMIT sessionChanged();
    }
    for (const auto &capability : e.hello().capabilities())
      if (capability == "time-sync-v1")
        timeSyncSupported_ = true;
      else if (capability == "route-edit-v1")
        routeEditSupported_ = true;
      else if (capability == "route-delete-v1")
        routeDeleteSupported_ = true;
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
    ++invalidMessages_;
    Q_EMIT diagnostic("未握手或会话不匹配，消息已丢弃");
    return;
  }
  if (e.sequence() <= sequences_.value(channel)) {
    ++invalidMessages_;
    return;
  }
  sequences_[channel] = e.sequence();
  if (channel == 0)
    lastRx_.restart();
  if (e.has_result()) {
    const QString id = QString::fromStdString(e.request_id());
    if (!pending_.contains(id))
      return;
    if (e.result().state() != mission::CommandResult::ACCEPTED)
      pending_.remove(id);
    if (id == timeProbeRequest_ || id == setTimeRequest_) {
      handleTimeResult(e);
      return;
    }
  }
  if (channel == 1 && desired_) {
    // Deltas are ordered state transitions; coalescing here would lose removals.
    if (e.has_grid())
      Q_EMIT message(e);
    else
      latestCloud_.Swap(&e);
    return;
  }
  Q_EMIT message(e);
  if (e.has_hello() && timeSyncSupported_ && desired_)
    probeTime();
}
