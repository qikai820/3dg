#include "MissionController.h"
#include "YamlSettingsPanel.h"
#include "mainwindow.h"
#include <FileIOFilter.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>
#include <QMediaPlayer>
#include <QQuaternion>
#include <QStandardPaths>
#include <QtWidgets>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <cmath>
#include <google/protobuf/util/json_util.h>
namespace {
QVector3D v(const mission::Vec3 &p) {
  return {float(p.x()), float(p.y()), float(p.z())};
}
void put(mission::Vec3 *p, QVector3D a) {
  p->set_x(a.x());
  p->set_y(a.y());
  p->set_z(a.z());
}
int mediaStart(mission::Command::Kind kind) {
  switch (kind) {
  case mission::Command::VIDEO_START:
  case mission::Command::VIDEO_STOP:
    return mission::Command::VIDEO_START;
  case mission::Command::RECORD_START:
  case mission::Command::RECORD_STOP:
    return mission::Command::RECORD_START;
  default:
    return -1;
  }
}
} // namespace
void MissionController::setDemo(bool enabled) {
  const bool modeChanged = demo_ != enabled;
  if (modeChanged && accumulationDirty_ && !saveAccumulatedMap()) {
    log("当前累积点云保存失败，未切换模拟模式");
    return;
  }
  client_.stop();
  demo_ = enabled;
  demoTimer_.stop();
  resetSession();
  if (modeChanged) {
    mapPoints_.clear();
    voxels_.clear();
    referenceMap_ = false;
    accumulationDirty_ = false;
    autosavePathLogged_ = false;
    restoreAccumulatedMap();
    mapDirty_ = dirty_ = true;
    refreshMapInfo();
  }
  if (!enabled) {
    mode_->setText("离线");
    return;
  }
  demoStep_ = 0;
  demoSequence_ = 0;
  mode_->setText("模拟演示 · 未连接真实飞机");
  mission::Envelope e;
  e.set_protocol_version(1);
  e.set_session_id("demo");
  e.set_sequence(++demoSequence_);
  e.mutable_hello()->set_name("3DG simulator");
  client_.ingest(ProtocolClient::encode(e));
  ready_ = false;
  mode_->setText("模拟演示 · 非真实遥测");
  connectButton_->setText("连接设备");
  protocolHint_->setText("Protobuf v1  ·  模拟数据");
  lastStatus_.set_odin("模拟运行");
  lastStatus_.set_ego("模拟规划");
  lastStatus_.set_localization("正常（模拟）");
  lastStatus_.set_cloud_hz(10);
  lastStatus_.set_cpu_percent(26);
  lastStatus_.set_disk_free_gb(64);
  if (mission_.waypoints_size() == 0) {
    addWaypoint({-6, -6, 2});
    addWaypoint({6, -6, 3});
    addWaypoint({6, 6, 4});
    addWaypoint({-6, 6, 2});
  }
  demoTick();
  demoTimer_.start();
  QTimer::singleShot(600, this, [this] {
    if (gl_) {
      updateScene();
      gl_->zoomGlobal();
    }
  });
}
void MissionController::demoTick() {
  double t = ++demoStep_ * .005; // Same simulated speed, now 50 Hz odometry.
  auto base = [this] {
    mission::Envelope e;
    e.set_protocol_version(1);
    e.set_session_id("demo");
    e.set_sequence(++demoSequence_);
    e.set_timestamp_ns(QDateTime::currentMSecsSinceEpoch() * 1000000ULL);
    e.set_time_domain("unix");
    return e;
  };
  auto e = base();
  auto *s = e.mutable_vehicle();
  s->set_frame_id(frame_.toStdString());
  s->set_map_id(mapId_.toStdString());
  s->set_pose_valid(true);
  s->set_velocity_valid(true);
  s->set_battery_valid(true);
  s->set_battery_percent(86);
  put(s->mutable_position(),
      {float(6 * cos(t)), float(6 * sin(t)), float(2 + sin(t * .6))});
  put(s->mutable_velocity(),
      {float(-1.5 * sin(t)), float(1.5 * cos(t)), float(.15 * cos(t * .6))});
  s->mutable_orientation()->set_z(sin((t + M_PI / 2) / 2));
  s->mutable_orientation()->set_w(cos((t + M_PI / 2) / 2));
  client_.ingest(ProtocolClient::encode(e));
  if (demoStep_ % 5 != 1)
    return; // Keep trajectory/status at 10 Hz; only pose uses the fast tick.
  e = base();
  auto *trajectory = e.mutable_trajectory();
  trajectory->set_frame_id(frame_.toStdString());
  trajectory->set_map_id(mapId_.toStdString());
  trajectory->set_trajectory_id(std::to_string(demoStep_));
  for (int i = 0; i < 60; ++i) {
    double a = t + i * .025;
    auto *p = trajectory->add_points();
    put(p->mutable_position(),
        {float(6 * cos(a)), float(6 * sin(a)), float(2 + sin(a * .6))});
    p->set_time_from_start_s(i * .1);
  }
  client_.ingest(ProtocolClient::encode(e));
  e = base();
  *e.mutable_status() = lastStatus_;
  client_.ingest(ProtocolClient::encode(e));
  if (demoStep_ % 15 == 1) {
    QVector<MissionPoint> pts;
    for (int x = -50; x <= 50; ++x)
      for (int y = -50; y <= 50; ++y) {
        MissionPoint p;
        p.x = x * .2;
        p.y = y * .2;
        p.z = 0;
        p.r = 65 + ((x + 50) * 2);
        p.g = 120 + (y + 50);
        p.b = 125;
        pts << p;
        if ((x > -25 && x < -12 && y > 10 && y < 22) ||
            (x > 20 && x < 32 && y > -20 && y < -5)) {
          for (int z = 1; z < 18; ++z) {
            p.z = z * .2;
            p.r = 170;
            p.g = 130;
            p.b = 90;
            pts << p;
          }
        }
      }
    e = base();
    auto *c = e.mutable_cloud();
    c->set_frame_id(frame_.toStdString());
    c->set_map_id(mapId_.toStdString());
    c->set_point_count(pts.size());
    c->set_snapshot(true);
    c->set_point_data(CloudIO::encode(pts).toStdString());
    client_.ingest(ProtocolClient::encode(e), 1);
  }
  if (demoStep_ % 150 == 1)
    log(QString("模拟遥测正常 · 实际轨迹 %1 点 · EGO 规划 60 点")
            .arg(history_.size()),
        "模拟器");
  if (demoStep_ % 50 == 1) {
    e = base();
    auto *grid = e.mutable_grid();
    grid->set_frame_id(frame_.toStdString());
    grid->set_map_id(mapId_.toStdString());
    grid->set_resolution_m(.4);
    grid->set_inflated(true);
    // Synthetic obstacle cells; only used in explicitly enabled demo mode.
    for (int x = -13; x <= -6; ++x)
      for (int y = 4; y <= 11; ++y)
        for (int z = 0; z < 10; ++z)
          if (x == -13 || x == -6 || y == 4 || y == 11 || z == 9)
            put(grid->add_centers(), {x * .4f, y * .4f, z * .4f + .2f});
    client_.ingest(ProtocolClient::encode(e), 1);
  }
}
void MissionController::settings() {
  QDialog d(window_);
  d.setWindowTitle("3DG 配置");
  d.resize(620, 440);
  QVBoxLayout outer(&d);
  QFormLayout f;
  QLineEdit address(baseUrl_), frame(frame_), map(mapId_), video(videoUrl_);
  QDoubleSpinBox voxel, hz, clearance;
  QSpinBox gridOpacity;
  QColor selectedGridColor = gridColor_;
  QPushButton gridColor;
  auto updateGridColorButton = [&] {
    QPixmap swatch(24, 16);
    swatch.fill(selectedGridColor);
    gridColor.setIcon(QIcon(swatch));
    gridColor.setText(selectedGridColor.name().toUpper());
  };
  updateGridColorButton();
  connect(&gridColor, &QPushButton::clicked, &d, [&] {
    const QColor chosen = QColorDialog::getColor(selectedGridColor, &d,
                                                "选择 EGO 栅格颜色");
    if (chosen.isValid()) {
      selectedGridColor = chosen;
      updateGridColorButton();
    }
  });
  gridOpacity.setRange(0, 100);
  gridOpacity.setSuffix(" %");
  gridOpacity.setValue(qRound(gridOpacity_ * 100.0));
  gridOpacity.setToolTip("0% 完全透明，100% 完全不透明；仅影响 3DG 本地显示");
  voxel.setRange(.02, 5);
  voxel.setValue(voxel_);
  hz.setRange(1, 30);
  hz.setValue(cloudHz_);
  clearance.setRange(.1, 20);
  clearance.setValue(clearance_);
  f.addRow("任务机 WebSocket", &address);
  f.addRow("坐标系（米，右手系）", &frame);
  f.addRow("地图 ID", &map);
  f.addRow("视频地址", &video);
  f.addRow("累积体素 / m", &voxel);
  f.addRow("点云频率 / Hz", &hz);
  f.addRow("安全间距 / m（接口参数）", &clearance);
  f.addRow("EGO 栅格颜色", &gridColor);
  f.addRow("EGO 栅格不透明度", &gridOpacity);
  QDialogButtonBox buttons(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
  buttons.button(QDialogButtonBox::Save)->setText("保存 3DG 配置");
  buttons.button(QDialogButtonBox::Cancel)->setText("关闭");
  auto *applyRemote = new QPushButton("发送当前本地配置到任务机");
  applyRemote->setEnabled(client_.ready() && !demo_);
  f.addRow(applyRemote);
  connect(applyRemote, &QPushButton::clicked, this,
          [this] { command(mission::Command::SET_CONFIG); });
  outer.addLayout(&f);
  outer.addWidget(&buttons);
  connect(&buttons, &QDialogButtonBox::accepted, &d, &QDialog::accept);
  connect(&buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
  if (d.exec() != QDialog::Accepted)
    return;
  QUrl u(address.text());
  if ((u.scheme() != "ws" && u.scheme() != "wss") || u.host().isEmpty() ||
      frame.text().isEmpty() || map.text().isEmpty()) {
    log("参数无效：需要 ws/wss 地址和非空坐标系、地图 ID");
    return;
  }
  bool changed =
      frame_ != frame.text() || mapId_ != map.text() || voxel_ != voxel.value();
  const bool videoChanged = videoUrl_ != video.text();
  if (changed && accumulationDirty_ && !saveAccumulatedMap()) {
    log("旧累积地图保存失败，参数未切换");
    return;
  }
  const bool localDisplayOnly =
      !changed && baseUrl_ == address.text() &&
      cloudHz_ == hz.value() && clearance_ == clearance.value();
  if (!localDisplayOnly) {
    setDemo(false);
    if (demo_)
      return;
  }
  baseUrl_ = address.text();
  frame_ = frame.text();
  mapId_ = map.text();
  videoUrl_ = video.text();
  voxel_ = voxel.value();
  cloudHz_ = hz.value();
  clearance_ = clearance.value();
  const bool gridAppearanceChanged =
      gridColor_ != selectedGridColor ||
      gridOpacity_ != gridOpacity.value() / 100.0;
  gridColor_ = selectedGridColor;
  gridOpacity_ = gridOpacity.value() / 100.0;
  if (changed) {
    resetSession();
    mapPoints_.clear();
    voxels_.clear();
    referenceMap_ = false;
    accumulationDirty_ = false;
    autosavePathLogged_ = false;
    restoreAccumulatedMap();
    mission_.clear_waypoints();
    rebuildRoute();
    mapDirty_ = dirty_ = true;
  }
  if (gridAppearanceChanged) {
    gridDirty_ = dirty_ = true;
    updateScene();
  }
  if (videoChanged) {
    if (videoUrl_.trimmed().isEmpty()) {
      player_->stop();
      video_->hide();
    } else
      showVideo();
  }
  refreshMapInfo();
  QSettings s("3DG", "Mission");
  s.setValue("url", baseUrl_);
  s.setValue("frame", frame_);
  s.setValue("map", mapId_);
  s.setValue("video", videoUrl_);
  s.setValue("voxel", voxel_);
  s.setValue("hz", cloudHz_);
  s.setValue("clearance", clearance_);
  s.setValue("gridColor", gridColor_.name());
  s.setValue("gridOpacity", gridOpacity_);
  log(localDisplayOnly ? "本地显示参数已保存并生效"
                     : "本地参数已保存；重新连接后才能发送任务。坐标系变化会清除旧地图和航线。");
}
void MissionController::yamlSettings() {
  QDialog d(window_);
  d.setWindowTitle("任务机 YAML");
  d.resize(900, 650);
  QVBoxLayout layout(&d);
  YamlSettingsPanel yaml(&client_, &d);
  layout.addWidget(&yaml, 1);
  QDialogButtonBox buttons(QDialogButtonBox::Close, &d);
  buttons.button(QDialogButtonBox::Close)->setText("关闭");
  layout.addWidget(&buttons);
  connect(&buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
  yaml.refresh();
  d.exec();
}
bool MissionController::frameMatches(const std::string &f,
                                     const std::string &m) {
  bool matches = QString::fromStdString(f) == frame_ &&
                 QString::fromStdString(m) == mapId_;
  if (!matches && (!mismatchAge_.isValid() || mismatchAge_.elapsed() > 5000)) {
    log("坐标系或地图 ID 不匹配，数据未叠加显示；请核对任务机配置");
    mismatchAge_.restart();
  }
  return matches;
}
void MissionController::resetSession() {
  livePoints_.clear();
  history_.clear();
  egoPoints_.clear();
  liveDirty_ = trailDirty_ = egoDirty_ = true;
  clearRemoteRoute();
  expireGrid();
  lastVehicle_.Clear();
  vehicleAge_.invalidate();
  plannerAge_.invalidate();
  lastStatus_.Clear();
  statusAge_.invalidate();
  pendingKinds_.clear();
  mediaOperations_.clear();
  mediaErrors_.clear();
  refreshStartupButtons();
  drawStep_ = 0;
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  if (aircraftModel_)
    aircraftModel_->setVisible(false);
  log("新会话：清除旧遥测和轨迹；当前地图保留，请核对地图 ID。");
}
void MissionController::receive(const mission::Envelope &e) {
  if (e.has_vehicle()) {
    if (frameMatches(e.vehicle().frame_id(), e.vehicle().map_id()))
      updateVehicle(e.vehicle());
  } else if (e.has_cloud()) {
    auto &c = e.cloud();
    if (!frameMatches(c.frame_id(), c.map_id()))
      return;
    QString err;
    livePoints_ =
        CloudIO::decode(QByteArray::fromStdString(c.point_data()), err);
    if (!err.isEmpty()) {
      log(err);
      return;
    }
    if (accumulate_->isChecked()) {
      if (referenceMap_)
        mapPoints_.clear();
      referenceMap_ = false;
      if (c.snapshot())
        voxels_.clear();
      for (auto &p : livePoints_) {
        QString k = QString("%1/%2/%3")
                        .arg(qint64(std::floor(p.x / voxel_)))
                        .arg(qint64(std::floor(p.y / voxel_)))
                        .arg(qint64(std::floor(p.z / voxel_)));
        if (voxels_.size() < 400000 || voxels_.contains(k))
          voxels_[k] = p;
      }
      ++drawStep_;
      accumulationDirty_ = true;
      // Show the first frame immediately; full snapshots must also replace
      // the displayed map immediately, including an empty snapshot.
      if (mapPoints_.isEmpty() || c.snapshot() || drawStep_ % 5 == 0) {
        mapPoints_ = voxels_.values().toVector();
        mapDirty_ = true;
      }
    }
    liveDirty_ = dirty_ = true;
    refreshMapInfo();
  } else if (e.has_grid()) {
    const auto &g = e.grid();
    if (!frameMatches(g.frame_id(), g.map_id()))
      return;
    gridCenters_.clear();
    gridCenters_.reserve(g.centers_size());
    for (const auto &p : g.centers())
      gridCenters_ << v(p);
    gridResolution_ = g.resolution_m();
    gridAge_.restart();
    grid_->setVisible(true);
    gridDirty_ = dirty_ = true;
    gridButton_->setToolTip(
        QString("EGO 栅格地图 · %1\n%2 个体素 · 分辨率 %3 m%4")
            .arg(g.inflated() ? "膨胀占据" : "原始占据")
            .arg(gridCenters_.size())
            .arg(gridResolution_, 0, 'g', 3)
            .arg(demo_ ? " · 模拟数据" : ""));
  } else if (e.has_trajectory()) {
    auto &t = e.trajectory();
    if (!frameMatches(t.frame_id(), t.map_id()))
      return;
    egoPoints_.clear();
    if (t.state() == mission::PlannerTrajectory::VALID)
      for (auto &p : t.points())
        egoPoints_ << v(p.position());
    ego_->setVisible(true);
    plannerAge_.restart();
    egoDirty_ = dirty_ = true;
  } else if (e.has_status()) {
    statusAge_.restart();
    lastStatus_ = e.status();
    // A successful command still needs a subsequent matching status report.
    for (auto it = mediaOperations_.begin(); it != mediaOperations_.end();) {
      const bool running = it.key() == mission::Command::VIDEO_START
                               ? lastStatus_.video_running()
                               : lastStatus_.recording();
      if (it->succeeded && running == it->target)
        it = mediaOperations_.erase(it);
      else
        ++it;
    }
    refreshStartupButtons();
    auto &s = lastStatus_;
    status_->setText(
        QString("Odin1　　　%1\nEGO　　　　%2\n定位　　　　%3\n点云 %4Hz  网络 "
                "%5Mbps\nCPU %6%  磁盘 %7GB\n视频 %8  记录 %9")
            .arg(QString::fromStdString(s.odin()).toHtmlEscaped(),
                 QString::fromStdString(s.ego()).toHtmlEscaped(),
                 QString::fromStdString(s.localization()).toHtmlEscaped())
            .arg(s.cloud_hz(), 0, 'f', 1)
            .arg(s.network_mbps(), 0, 'f', 1)
            .arg(s.cpu_percent(), 0, 'f', 0)
            .arg(s.disk_free_gb(), 0, 'f', 1)
            .arg(s.video_running() ? "运行" : "停止",
                 s.recording() ? "记录中" : "停止"));
  } else if (e.has_yaml_document()) {
    const QString request = QString::fromStdString(e.request_id());
    if (request == remoteRouteRequest_ && e.yaml_document().id() == "route")
      receiveRemoteRoute(e.yaml_document());
  } else if (e.has_log())
    log(QString::fromStdString(e.log().text()),
        QString::fromStdString(e.log().source()));
  else if (e.has_result()) {
    const QString request = QString::fromStdString(e.request_id());
    if (request == remoteRouteRequest_ &&
        e.result().state() != mission::CommandResult::ACCEPTED) {
      if (e.result().state() == mission::CommandResult::FAILED)
        clearRemoteRoute();
      remoteRouteRequest_.clear();
    }
    log(QString("命令 %1：%2")
            .arg(QString::fromStdString(e.request_id()),
                 QString::fromStdString(e.result().detail())),
        "任务机");
    if (e.result().state() != mission::CommandResult::ACCEPTED) {
      const int start = mediaStart(static_cast<mission::Command::Kind>(
          pendingKinds_.value(request, -1)));
      auto op = mediaOperations_.find(start);
      if (op != mediaOperations_.end() && op->request == request) {
        if (e.result().state() == mission::CommandResult::SUCCEEDED)
          op->succeeded = true;
        else {
          mediaErrors_[start] = "操作失败：" + QString::fromStdString(e.result().detail());
          mediaOperations_.erase(op);
        }
      }
      if (e.result().state() == mission::CommandResult::SUCCEEDED &&
          pendingKinds_.value(request) == mission::Command::UPLOAD_MISSION)
        QTimer::singleShot(0, this, &MissionController::requestRemoteRoute);
      pendingKinds_.remove(request);
      refreshMediaButtons();
    }
  } else if (e.has_video()) {
    const QUrl u(QString::fromStdString(e.video().url()));
    if (!e.video().running()) {
      player_->stop();
      video_->hide();
    } else if (u.scheme() == "rtsp" || u.scheme() == "http" ||
               u.scheme() == "https") {
      const bool addressChanged = videoUrl_ != u.toString();
      videoUrl_ = u.toString();
      if (addressChanged)
        log(u.path() == "/mission3dg" ? "已收到 Odin1 去畸变视频地址"
                                        : "已收到视频地址", "视频");
      showVideo();
    } else
      log("收到无效的视频地址", "视频");
  } else if (e.has_file())
    fileChunk(e);
}
void MissionController::refreshStartupButtons() {
  const bool fresh = (ready_ || demo_) && statusAge_.isValid() &&
                     statusAge_.elapsed() <= 3000;
  auto update = [this, fresh](mission::Command::Kind kind, const QString &name,
                              const std::string &reported) {
    auto *b = commands_.value(kind, nullptr);
    if (!b)
      return;
    const QString state = QString::fromStdString(reported).trimmed();
    // Match positive agent states exactly: "未运行" must never count as running.
    const bool running = fresh &&
        (state == "定位就绪" || state == "规划器运行" || state == "运行" ||
         state.compare("running", Qt::CaseInsensitive) == 0 ||
         (demo_ && (state == "模拟运行" || state == "模拟规划")));
    b->setText(running ? QString("%1 %2").arg(name, demo_ ? "模拟中" : "已启动")
                       : QString("启动 %1").arg(name));
    b->setToolTip(QString("%1：%2").arg(
        name, fresh ? state : QString("尚未收到有效运行状态或状态已过期")));
    if (b->property("serviceRunning").isValid() &&
        b->property("serviceRunning").toBool() == running)
      return;
    b->setProperty("serviceRunning", running);
    b->style()->unpolish(b);
    b->style()->polish(b);
    b->update();
  };
  update(mission::Command::START_ODIN, "Odin1", lastStatus_.odin());
  update(mission::Command::START_EGO, "EGO", lastStatus_.ego());
  refreshMediaButtons();
}
void MissionController::refreshMediaButtons() {
  const bool online = ready_ || demo_;
  const bool fresh = online && statusAge_.isValid() && statusAge_.elapsed() <= 3000;
  for (auto start : {mission::Command::VIDEO_START, mission::Command::RECORD_START}) {
    auto op = mediaOperations_.find(start);
    if (op != mediaOperations_.end() && op->age.elapsed() > 30000) {
      mediaErrors_[start] = "操作超时，结果未知；请核对任务机状态";
      pendingKinds_.remove(op->request);
      mediaOperations_.erase(op);
      log(mediaErrors_[start], "任务机");
    }
    const bool video = start == mission::Command::VIDEO_START;
    const bool running = fresh && (video ? lastStatus_.video_running() : lastStatus_.recording());
    const bool pending = mediaOperations_.contains(start);
    QString text = video ? (running ? "停止视频" : "开启视频")
                         : (running ? "停止记录" : "开始记录");
    QString state = !online ? "未连接" : !fresh ? "等待有效状态"
                    : running ? (video ? "● 视频运行中" : "● 正在记录")
                              : (video ? "视频已停止" : "记录已停止");
    QString tip = state;
    if (pending) {
      const auto &operation = mediaOperations_[start];
      text = operation.target ? (video ? "视频开启中…" : "记录启动中…")
                              : (video ? "视频停止中…" : "记录停止中…");
      state = operation.succeeded ? "等待状态确认" : "等待任务机执行";
      tip = state;
    } else if (mediaErrors_.contains(start)) {
      state += mediaErrors_[start].startsWith("操作超时") ? " · 操作超时" : " · 操作失败";
      tip = mediaErrors_[start];
    }
    if (demo_)
      state.prepend("模拟 · ");
    auto update = [&](QPushButton *b, QLabel *label) {
      if (!b || !label)
        return;
      b->setText(text);
      b->setEnabled(fresh && !pending);
      b->setToolTip(tip);
      label->setText(state);
      label->setToolTip(tip);
      const bool active = running && !pending;
      if (!b->property("serviceRunning").isValid() ||
          b->property("serviceRunning").toBool() != active) {
        b->setProperty("serviceRunning", active);
        b->style()->unpolish(b);
        b->style()->polish(b);
        b->update();
      }
    };
    update(commands_.value(start, nullptr), mediaStates_.value(start, nullptr));
    if (!video)
      update(logRecordButton_, logRecordState_);
  }
}
void MissionController::toggleMedia(mission::Command::Kind start) {
  const bool running = start == mission::Command::VIDEO_START
                           ? lastStatus_.video_running() : lastStatus_.recording();
  command(running ? (start == mission::Command::VIDEO_START
                         ? mission::Command::VIDEO_STOP : mission::Command::RECORD_STOP)
                  : start);
}
void MissionController::command(mission::Command::Kind k) {
  const int start = mediaStart(k);
  if (start >= 0) {
    refreshMediaButtons();
    if (!(ready_ || demo_) || !statusAge_.isValid() || statusAge_.elapsed() > 3000 ||
        mediaOperations_.contains(start))
      return;
    mediaErrors_.remove(start);
  }
  if (k == mission::Command::END_MISSION &&
      QMessageBox::question(
          window_, "结束任务",
          "请求任务机结束任务？此操作不是急停，飞控安全策略由任务机负责。") !=
          QMessageBox::Yes)
    return;
  if (demo_) {
    log("模拟命令：" + QString::fromStdString(mission::Command_Kind_Name(k)) +
        "（未发送到设备）");
    if (k == mission::Command::RECORD_START)
      lastStatus_.set_recording(true);
    if (k == mission::Command::RECORD_STOP)
      lastStatus_.set_recording(false);
    if (k == mission::Command::VIDEO_START)
      lastStatus_.set_video_running(true);
    if (k == mission::Command::VIDEO_STOP)
      lastStatus_.set_video_running(false);
    mission::Envelope status;
    *status.mutable_status() = lastStatus_;
    receive(status);
    return;
  }
  if (!client_.ready()) {
    log("任务机未连接，命令未发送");
    return;
  }
  mission::Command c;
  c.set_kind(k);
  if (k == mission::Command::SET_CONFIG) {
    auto *p = c.mutable_config();
    p->set_cloud_hz(cloudHz_);
    p->set_voxel_m(voxel_);
    p->set_frame_id(frame_.toStdString());
    p->set_map_id(mapId_.toStdString());
    p->set_clearance_m(clearance_);
  }
  if (k == mission::Command::UPLOAD_MISSION) {
    if (mission_.waypoints_size() < 2) {
      log("至少需要两个航点");
      return;
    }
    *c.mutable_mission() = mission_;
  }
  auto id = client_.sendCommand(c);
  if (!id.isEmpty()) {
    pendingKinds_[id] = k;
    if (start >= 0) {
      MediaOperation operation;
      operation.request = id;
      operation.target = k == start;
      operation.age.start();
      mediaOperations_[start] = operation;
    }
  } else if (start >= 0)
    mediaErrors_[start] = "操作失败：命令未发送，请检查控制连接";
  refreshMediaButtons();
}
void MissionController::requestRemoteRoute() {
  if (demo_ || !client_.connected()) {
    log("请先连接任务机，再读取机载航点");
    return;
  }
  mission::Command request;
  request.set_kind(mission::Command::GET_YAML_DOCUMENT);
  request.set_yaml_document_id("route");
  remoteRouteRequest_ = client_.sendCommand(request);
  if (!remoteRouteRequest_.isEmpty())
    log("正在读取任务机当前 EGO 航点…");
}
void MissionController::clearRemoteRoute() {
  remoteRouteRequest_.clear();
  remoteRouteSource_.clear();
  remoteEntries_ = QJsonArray();
  remotePoints_.clear();
  if (remoteRoute_)
    remoteRoute_->removeAllChildren();
  refreshWaypointTable();
  refreshMapInfo();
  refreshRouteStrip();
  if (gl_)
    gl_->redraw();
}
void MissionController::receiveRemoteRoute(const mission::YamlDocument &document) {
  QJsonParseError error;
  const auto parsed = QJsonDocument::fromJson(
      QByteArray::fromStdString(document.content_json()), &error);
  if (error.error != QJsonParseError::NoError || !parsed.isObject() ||
      !parsed.object().value("waypoints").isArray()) {
    clearRemoteRoute();
    log("任务机航点数据无效：缺少 waypoints 数组");
    return;
  }
  const auto entries = parsed.object().value("waypoints").toArray();
  if (entries.size() > 1000) {
    clearRemoteRoute();
    log("任务机航点超过 1000 个，拒绝显示");
    return;
  }
  QVector<QVector3D> points;
  points.reserve(entries.size());
  for (const auto &value : entries) {
    const auto waypoint = value.toObject();
    const auto x = waypoint.value("x"), y = waypoint.value("y"), z = waypoint.value("z");
    if (!value.isObject() || !x.isDouble() || !y.isDouble() || !z.isDouble() ||
        !std::isfinite(x.toDouble()) || !std::isfinite(y.toDouble()) ||
        !std::isfinite(z.toDouble()) || std::abs(x.toDouble()) > 10000 ||
        std::abs(y.toDouble()) > 10000 || std::abs(z.toDouble()) > 10000) {
      clearRemoteRoute();
      log("任务机航点包含无效坐标，拒绝显示");
      return;
    }
    points << QVector3D(float(x.toDouble()), float(y.toDouble()), float(z.toDouble()));
  }
  remoteEntries_ = entries;
  remotePoints_ = points;
  remoteRouteSource_ = QString::fromStdString(document.usage());
  makeLine(remoteRoute_, remotePoints_, 118, 218, 247, 3);
  refreshWaypointTable();
  refreshMapInfo();
  refreshRouteStrip();
  gl_->redraw();
  log(QString("已读取任务机航点 %1 个：%2")
          .arg(remotePoints_.size()).arg(remoteRouteSource_));
}
void MissionController::refreshWaypointTable() {
  if (remoteRouteInfo_)
    remoteRouteInfo_->setText(remoteRouteSource_.isEmpty()
                                  ? "任务机航点尚未读取 · 只读"
                                  : QString("任务机航点 %1 个 · %2")
                                        .arg(remotePoints_.size()).arg(remoteRouteSource_));
  if (!waypoints_)
    return;
  const int localCount = mission_.waypoints_size();
  const int selected = waypoints_->currentRow();
  QSignalBlocker blocker(waypoints_);
  waypoints_->clearContents();
  waypoints_->setRowCount(localCount + remoteEntries_.size());
  for (int i = 0; i < localCount; ++i) {
    const auto &point = mission_.waypoints(i);
    const QStringList values{
        "本地", QString::number(i + 1),
        QString::number(point.position().x(), 'f', 2),
        QString::number(point.position().y(), 'f', 2),
        QString::number(point.position().z(), 'f', 2),
        "航点", QString::number(point.hold_s(), 'f', 1)};
    for (int column = 0; column < values.size(); ++column) {
      auto *item = new QTableWidgetItem(values[column]);
      item->setToolTip(QString("本地可编辑 · 航向 %1° · 速度 %2 m/s")
                           .arg(point.yaw() * 180 / M_PI, 0, 'f', 1)
                           .arg(point.speed(), 0, 'f', 1));
      waypoints_->setItem(i, column, item);
    }
  }
  for (int i = 0; i < remoteEntries_.size(); ++i) {
    const auto entry = remoteEntries_.at(i).toObject();
    const QStringList values{
        "机载", QString::number(i + 1),
        QString::number(entry.value("x").toDouble(), 'f', 2),
        QString::number(entry.value("y").toDouble(), 'f', 2),
        QString::number(entry.value("z").toDouble(), 'f', 2),
        entry.value("pointmode").toString("—"),
        QString::number(entry.value("dwell_time").toDouble(), 'f', 1)};
    for (int column = 0; column < values.size(); ++column) {
      auto *item = new QTableWidgetItem(values[column]);
      item->setForeground(QColor("#688297"));
      item->setToolTip(QString("机载只读 · 航向 %1° · 俯仰 %2° · 变焦 %3")
                           .arg(entry.value("yaw").toDouble())
                           .arg(entry.value("inspection_pitch_deg").toDouble())
                           .arg(entry.value("inspection_zoom_ratio").toDouble()));
      waypoints_->setItem(localCount + i, column, item);
    }
  }
  if (selected >= 0 && selected < waypoints_->rowCount())
    waypoints_->selectRow(selected);
  updateWaypointGizmo();
}
void MissionController::addWaypoint(const QVector3D &p) {
  if (mission_.waypoints_size() >= 1000)
    return;
  auto *w = mission_.add_waypoints();
  put(w->mutable_position(), p);
  w->set_speed(1);
  rebuildRoute();
}
int MissionController::addWaypointAtAircraft() {
  if (!lastVehicle_.pose_valid() || !vehicleAge_.isValid() ||
      vehicleAge_.elapsed() > 3000) {
    log("当前没有有效且未过期的飞机位置，无法添加航点");
    return -1;
  }
  const QVector3D position = v(lastVehicle_.position());
  if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
      !std::isfinite(position.z())) {
    log("飞机位置坐标无效，无法添加航点");
    return -1;
  }
  if (mission_.waypoints_size() >= 1000) {
    log("航点已达到 1000 个，无法继续添加");
    return -1;
  }
  const int row = mission_.waypoints_size();
  addWaypoint(position);
  return row;
}
void MissionController::onItemPicked(const PickedItem &i) {
  if (picking_->isChecked())
    addWaypoint({i.P3D.x, i.P3D.y, float(altitude_->value())});
}
void MissionController::rebuildRoute() {
  mission_.set_frame_id(frame_.toStdString());
  mission_.set_map_id(mapId_.toStdString());
  mission_.set_revision(mission_.revision() + 1);
  refreshMapInfo();
  QVector<QVector3D> pts;
  for (const auto &point : mission_.waypoints())
    pts << v(point.position());
  refreshWaypointTable();
  makeLine(route_, pts, 255, 190, 67, 3);
  refreshRouteStrip();
  gl_->redraw();
  updateWaypointGizmo();
}
void MissionController::moveWaypointAlongAxis(int axis, double distance) {
  const int row = waypoints_->currentRow();
  if (row < 0 || row >= mission_.waypoints_size() || axis < 0 || axis > 2)
    return;
  auto *position = mission_.mutable_waypoints(row)->mutable_position();
  if (axis == 0)
    position->set_x(position->x() + distance);
  else if (axis == 1)
    position->set_y(position->y() + distance);
  else
    position->set_z(position->z() + distance);
  rebuildRoute();
}
void MissionController::editWaypoint() {
  int row = waypoints_->currentRow();
  if (row < 0 || row >= mission_.waypoints_size())
    return; // The unified table also contains read-only task-machine rows.
  auto *w = mission_.mutable_waypoints(row);
  QDialog d(window_);
  d.setWindowTitle("编辑三维航点");
  QFormLayout f(&d);
  QDoubleSpinBox x, y, z, yaw, speed, hold;
  for (auto *s : {&x, &y, &z}) {
    s->setRange(-100000, 100000);
    s->setDecimals(3);
  }
  x.setValue(w->position().x());
  y.setValue(w->position().y());
  z.setValue(w->position().z());
  yaw.setRange(-180, 180);
  yaw.setValue(w->yaw() * 180 / M_PI);
  speed.setRange(.1, 20);
  speed.setValue(w->speed());
  hold.setRange(0, 3600);
  hold.setValue(w->hold_s());
  f.addRow("X / m", &x);
  f.addRow("Y / m", &y);
  f.addRow("Z / m", &z);
  f.addRow("航向 / °", &yaw);
  f.addRow("速度 / m/s", &speed);
  f.addRow("停留 / s", &hold);
  QDialogButtonBox b(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  f.addRow(&b);
  connect(&b, &QDialogButtonBox::accepted, &d, &QDialog::accept);
  connect(&b, &QDialogButtonBox::rejected, &d, &QDialog::reject);
  if (d.exec() == QDialog::Accepted) {
    put(w->mutable_position(),
        {float(x.value()), float(y.value()), float(z.value())});
    w->set_yaw(yaw.value() * M_PI / 180);
    w->set_speed(speed.value());
    w->set_hold_s(hold.value());
    rebuildRoute();
  }
}
bool MissionController::exportMission(const QString &p) {
  std::string json;
  google::protobuf::util::JsonPrintOptions o;
  o.add_whitespace = true;
  o.preserve_proto_field_names = true;
  if (!google::protobuf::util::MessageToJsonString(mission_, &json, o).ok())
    return false;
  QSaveFile f(p);
  return f.open(QIODevice::WriteOnly) &&
         f.write(QByteArray::fromStdString(json)) == qint64(json.size()) &&
         f.commit();
}
bool MissionController::importMission(const QString &p) {
  QFile f(p);
  if (!f.open(QIODevice::ReadOnly) || f.size() > 2 * 1024 * 1024)
    return false;
  mission::MissionPlan m;
  if (!google::protobuf::util::JsonStringToMessage(f.readAll().toStdString(),
                                                   &m)
           .ok() ||
      !frameMatches(m.frame_id(), m.map_id()) || m.waypoints_size() > 1000)
    return false;
  for (auto &w : m.waypoints()) {
    auto &a = w.position();
    if (!std::isfinite(a.x()) || !std::isfinite(a.y()) ||
        !std::isfinite(a.z()) || std::abs(a.x()) > 100000 ||
        std::abs(a.y()) > 100000 || std::abs(a.z()) > 100000 ||
        !std::isfinite(w.speed()) || w.speed() <= 0 || w.speed() > 20 ||
        !std::isfinite(w.yaw()) || !std::isfinite(w.hold_s()) || w.hold_s() < 0)
      return false;
  }
  mission_ = m;
  rebuildRoute();
  return true;
}
void MissionController::saveMission() {
  auto p = QFileDialog::getSaveFileName(window_, "保存航线", {},
                                        "3DG 航线 (*.3dg.json)");
  if (!p.isEmpty())
    log(exportMission(p) ? "航线已保存" : "航线保存失败");
}
void MissionController::openMission() {
  auto p = QFileDialog::getOpenFileName(window_, "打开航线", {},
                                        "3DG 航线 (*.json)");
  if (!p.isEmpty())
    log(importMission(p) ? "航线已加载" : "航线无效或地图/坐标系不一致");
}
void MissionController::loadMap() {
  auto p = QFileDialog::getOpenFileName(window_, "加载地图", {},
                                        "点云 (*.pcd *.ply)");
  if (!p.isEmpty())
    loadMapPath(p);
}
bool MissionController::loadMapPath(const QString &p) {
  QFile metadata(p + ".meta.json");
  if (metadata.exists()) {
    if (!metadata.open(QIODevice::ReadOnly) || metadata.size() > 65536) {
      log("地图元数据不可读");
      return false;
    }
    QJsonParseError parse;
    auto object = QJsonDocument::fromJson(metadata.readAll(), &parse).object();
    if (parse.error != QJsonParseError::NoError ||
        object.value("frame_id").toString() != frame_ ||
        object.value("map_id").toString() != mapId_) {
      log("地图元数据与当前坐标系/地图 ID 不一致，请先修改任务机配置");
      return false;
    }
  }
  QVector<MissionPoint> points;
  quint64 sourceCount = 0;
  QString err;
  if (p.endsWith(".pcd", Qt::CaseInsensitive)) {
    if (!CloudIO::readPCD(p, points, err, &sourceCount)) {
      log(err);
      return false;
    }
  } else {
    FileIOFilter::LoadParameters params;
    params.alwaysDisplayLoadDialog = false;
    CC_FILE_ERROR error;
    auto *object = FileIOFilter::LoadFromFile(p, params, error);
    if (!object) {
      log("PLY 加载失败");
      return false;
    }
    std::function<void(ccHObject *)> visit = [&](ccHObject *o) {
      if (auto *c = dynamic_cast<ccPointCloud *>(o)) {
        for (unsigned i = 0; i < c->size() && points.size() < 2000000; ++i) {
          auto a = c->toGlobal3d(*c->getPoint(i));
          MissionPoint point;
          point.x = a.x;
          point.y = a.y;
          point.z = a.z;
          if (c->hasColors()) {
            auto rgb = c->getPointColor(i);
            point.r = rgb.r;
            point.g = rgb.g;
            point.b = rgb.b;
          }
          if (std::isfinite(point.x) && std::isfinite(point.y) &&
              std::isfinite(point.z))
            points << point;
        }
      }
      for (unsigned i = 0; i < o->getChildrenNumber(); ++i)
        visit(o->getChild(i));
    };
    visit(object);
    delete object;
    sourceCount = points.size();
  }
  if (points.isEmpty()) {
    log("文件中没有可显示的有效三维点");
    return false;
  }
  if (accumulationDirty_ && !saveAccumulatedMap()) {
    log("旧累积地图保存失败，未切换到参考地图");
    return false;
  }
  mapPoints_ = points;
  referenceMap_ = true;
  voxels_.clear();
  accumulate_->setChecked(false);
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  updateScene();
  gl_->zoomGlobal();
  const QString count = sourceCount > quint64(points.size())
                            ? QString("原始 %1 点，显示 %2 点")
                                  .arg(sourceCount)
                                  .arg(points.size())
                            : QString("加载 %1 点").arg(points.size());
  log(QString("%1，按当前坐标系 %2 / %3 "
              "解释；已关闭实时累积，避免覆盖参考地图")
          .arg(count)
          .arg(frame_, mapId_));
  return true;
}
QString MissionController::accumulatedMapPath() const {
  const QByteArray identity = frame_.toUtf8() + '\0' + mapId_.toUtf8() + '\0' +
      QByteArray::number(voxel_, 'g', 17) + '\0' +
      (demo_ ? "demo" : "live");
  const QString id = QString::fromLatin1(
      QCryptographicHash::hash(identity, QCryptographicHash::Sha256).toHex().left(16));
  const QString smokeDir = qEnvironmentVariable("THREEDG_SMOKE_DIR");
  const QString directory = smokeDir.isEmpty()
      ? QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) +
            "/3dg/maps"
      : smokeDir + "/autosave";
  return directory + "/accumulated-" + id + ".pcd";
}
bool MissionController::saveAccumulatedMap() {
  if (!accumulationDirty_ || referenceMap_)
    return true;
  const QString path = accumulatedMapPath();
  if (voxels_.isEmpty()) {
    const bool pcdRemoved = !QFile::exists(path) || QFile::remove(path);
    const bool metaRemoved = !QFile::exists(path + ".meta.json") ||
                             QFile::remove(path + ".meta.json");
    if (!pcdRemoved || !metaRemoved) {
      log("自动保存文件清理失败：" + path);
      return false;
    }
    accumulationDirty_ = false;
    return true;
  }
  if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
    log("无法创建累积点云目录：" + QFileInfo(path).absolutePath());
    return false;
  }
  const auto points = voxels_.values().toVector();
  QString error;
  if (!CloudIO::writePCD(path, points, error)) {
    log("累积点云自动保存失败：" + error);
    return false;
  }
  const QByteArray data = QJsonDocument(QJsonObject{{"frame_id", frame_},
                                                    {"map_id", mapId_},
                                                    {"voxel_m", voxel_},
                                                    {"mode", demo_ ? "demo" : "live"},
                                                    {"point_count", points.size()},
                                                    {"unit", "m"},
                                                    {"format", "XYZRGB"}}).toJson();
  QSaveFile metadata(path + ".meta.json");
  if (!metadata.open(QIODevice::WriteOnly) ||
      metadata.write(data) != data.size() || !metadata.commit()) {
    log("累积点云已写入，但元数据自动保存失败：" + metadata.errorString());
    return false;
  }
  accumulationDirty_ = false;
  if (!autosavePathLogged_) {
    log("累积点云自动保存到 " + path);
    autosavePathLogged_ = true;
  }
  return true;
}
bool MissionController::restoreAccumulatedMap() {
  const QString path = accumulatedMapPath();
  if (!QFile::exists(path))
    return false;
  QFile metadata(path + ".meta.json");
  if (!metadata.open(QIODevice::ReadOnly) || metadata.size() > 65536) {
    log("累积点云元数据不可读，未自动加载：" + path);
    return false;
  }
  QJsonParseError parse;
  const auto object = QJsonDocument::fromJson(metadata.readAll(), &parse).object();
  if (parse.error != QJsonParseError::NoError ||
      object.value("frame_id").toString() != frame_ ||
      object.value("map_id").toString() != mapId_ ||
      object.value("voxel_m").toDouble() != voxel_ ||
      object.value("mode").toString() != (demo_ ? "demo" : "live")) {
    log("累积点云坐标系、地图 ID、体素或模式不一致，未自动加载：" + path);
    return false;
  }
  const int expectedCount = object.value("point_count").toInt();
  if (expectedCount < 1 || expectedCount > 400000) {
    log("累积点云点数无效，未自动加载：" + path);
    return false;
  }
  QVector<MissionPoint> points;
  QString error;
  if (!CloudIO::readPCD(path, points, error) || points.isEmpty() ||
      points.size() != expectedCount) {
    log("累积点云文件无效，未自动加载：" + path + " " + error);
    return false;
  }
  voxels_.clear();
  for (const auto &point : points) {
    const QString key = QString("%1/%2/%3")
                            .arg(qint64(std::floor(point.x / voxel_)))
                            .arg(qint64(std::floor(point.y / voxel_)))
                            .arg(qint64(std::floor(point.z / voxel_)));
    voxels_.insert(key, point);
  }
  mapPoints_ = voxels_.values().toVector();
  referenceMap_ = false;
  accumulationDirty_ = false;
  autosavePathLogged_ = true;
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  log(QString("已自动加载累积点云 %1 点：%2").arg(mapPoints_.size()).arg(path));
  return true;
}
void MissionController::clearAccumulatedMap() {
  voxels_.clear();
  mapPoints_.clear();
  referenceMap_ = false;
  accumulationDirty_ = true;
  const bool removed = saveAccumulatedMap();
  autosavePathLogged_ = false;
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  updateScene();
  log(removed ? "累积地图已清空，本地自动保存文件已删除"
              : "累积地图已清空，但本地自动保存文件删除失败");
}
bool MissionController::exportMap(const QString &p) {
  if (mapPoints_.isEmpty() && livePoints_.isEmpty()) {
    log("没有可保存的点云");
    return false;
  }
  QString err;
  bool ok = CloudIO::writePCD(
      p, mapPoints_.isEmpty() ? livePoints_ : mapPoints_, err);
  if (!ok)
    log(err);
  if (ok) {
    QSaveFile meta(p + ".meta.json");
    QByteArray data = QJsonDocument(QJsonObject{{"frame_id", frame_},
                                                {"map_id", mapId_},
                                                {"unit", "m"},
                                                {"format", "XYZRGB"}})
                          .toJson();
    ok = meta.open(QIODevice::WriteOnly) && meta.write(data) == data.size() &&
         meta.commit();
    if (!ok)
      log("PCD 已写入，但坐标元数据保存失败");
  }
  return ok;
}
void MissionController::saveMap() {
  auto p = QFileDialog::getSaveFileName(window_, "保存点云", {}, "PCD (*.pcd)");
  if (!p.isEmpty())
    log(exportMap(p) ? "PCD 已保存（XYZRGB，单位米）" : "PCD 保存失败");
}
void MissionController::showVideo() {
  video_->show();
  video_->raise();
  if (videoUrl_.trimmed().isEmpty()) {
    log("请先在 3DG 配置中设置视频地址，或启动机载视频", "视频");
    return;
  }
  const QUrl url = QUrl::fromUserInput(videoUrl_.trimmed());
  if (player_->currentMedia().canonicalUrl() != url)
    player_->setMedia(url);
  if (player_->state() != QMediaPlayer::PlayingState)
    player_->play();
}
void MissionController::requestFile() {
  if (!client_.ready()) {
    log("请先连接任务机");
    return;
  }
  bool ok;
  QString id =
      QInputDialog::getText(window_, "下载文件", "任务机文件 ID（非本地路径）",
                            QLineEdit::Normal, {}, &ok);
  if (!ok || id.isEmpty())
    return;
  QString p = QFileDialog::getSaveFileName(window_, "保存任务机文件", {},
                                           "PCD (*.pcd);;全部 (*)");
  if (p.isEmpty())
    return;
  download_.reset(new QSaveFile(p));
  if (!download_->open(QIODevice::WriteOnly)) {
    download_.reset();
    log("无法创建下载文件");
    return;
  }
  downloadHash_.reset(new QCryptographicHash(QCryptographicHash::Sha256));
  downloadId_ = id;
  downloadOffset_ = downloadSize_ = 0;
  mission::Command c;
  c.set_kind(mission::Command::GET_FILE);
  c.set_file_id(id.toStdString());
  downloadRequest_ = client_.sendCommand(c);
  downloadAge_.restart();
  log("开始下载 " + id);
}
void MissionController::fileChunk(const mission::Envelope &e) {
  if (!download_ || QString::fromStdString(e.request_id()) != downloadRequest_)
    return;
  auto &f = e.file();
  downloadAge_.restart();
  auto fail = [this] {
    download_.reset();
    log("文件分块校验失败，下载取消");
  };
  if (QString::fromStdString(f.file_id()) != downloadId_ ||
      f.offset() != downloadOffset_ ||
      (downloadOffset_ && f.total_size() != downloadSize_)) {
    fail();
    return;
  }
  downloadSize_ = f.total_size();
  QByteArray data = QByteArray::fromStdString(f.data());
  if (download_->write(data) != data.size()) {
    fail();
    return;
  }
  downloadHash_->addData(data);
  downloadOffset_ += data.size();
  if (f.last()) {
    if (downloadOffset_ != downloadSize_ ||
        downloadHash_->result() != QByteArray::fromStdString(f.sha256())) {
      fail();
      return;
    }
    log(download_->commit() ? "文件下载并校验完成" : "文件保存失败");
    download_.reset();
  }
}
void MissionController::smoke() {
  QString p = qEnvironmentVariable("THREEDG_SMOKE_DIR");
  QDir().mkpath(p);
  demoTimer_.stop();
  updateScene();
  window_->screen()->grabWindow(window_->winId()).save(p + "/3dg.png");
  bool emptyVideoVisible = true;
  if (videoUrl_.trimmed().isEmpty()) {
    QAction *videoAction = nullptr;
    for (auto *action : window_->menuBar()->findChildren<QAction *>()) {
      if (action->text() == QStringLiteral("悬浮视频窗口")) {
        videoAction = action;
        break;
      }
    }
    if (videoAction)
      videoAction->trigger();
    QApplication::processEvents();
    emptyVideoVisible = videoAction && video_->isVisible();
    video_->grab().save(p + "/video-empty.png");
    video_->hide();
  }
  // Exercise both UI entry points and asynchronous result/status ordering.
  bool mediaControls = true;
  const auto savedStatus = lastStatus_;
  for (auto start : {mission::Command::VIDEO_START, mission::Command::RECORD_START}) {
    auto *b = commands_.value(start);
    const bool video = start == mission::Command::VIDEO_START;
    b->click();
    mediaControls &= b->isEnabled() && b->property("serviceRunning").toBool();
    if (!video)
      mediaControls &= logRecordButton_->text() == b->text();
  }
  QApplication::processEvents();
  right_->grab().save(p + "/media-running.png");
  commands_.value(mission::Command::VIDEO_START)->click();
  logRecordButton_->click();
  mediaControls &= !lastStatus_.video_running() && !lastStatus_.recording();
  for (auto start : {mission::Command::VIDEO_START, mission::Command::RECORD_START}) {
    auto *b = commands_.value(start);
    MediaOperation operation;
    operation.request = QString("smoke-media-%1").arg(start);
    operation.target = true;
    operation.age.start();
    mediaOperations_[start] = operation;
    pendingKinds_[operation.request] = start;
    refreshMediaButtons();
    b->click();
    mediaControls &= !b->isEnabled() && !lastStatus_.video_running() && !lastStatus_.recording();
    mission::Envelope result;
    result.set_request_id(operation.request.toStdString());
    result.mutable_result()->set_state(mission::CommandResult::ACCEPTED);
    receive(result);
    mediaControls &= !b->isEnabled();
    result.mutable_result()->set_state(mission::CommandResult::SUCCEEDED);
    receive(result);
    mediaControls &= !b->isEnabled();
    mission::Envelope status;
    *status.mutable_status() = lastStatus_;
    receive(status); // Old state cannot complete the requested transition.
    mediaControls &= !b->isEnabled();
    if (start == mission::Command::VIDEO_START)
      status.mutable_status()->set_video_running(true);
    else
      status.mutable_status()->set_recording(true);
    receive(status);
    mediaControls &= b->isEnabled() && b->property("serviceRunning").toBool();
    b->click(); // Restore stopped state in demo.
    mediaOperations_[start] = operation;
    pendingKinds_[operation.request] = start;
    result.mutable_result()->set_state(mission::CommandResult::FAILED);
    result.mutable_result()->set_detail("smoke failure");
    receive(result);
    mediaControls &= b->isEnabled() && !b->property("serviceRunning").toBool() &&
                     mediaStates_[start]->text().contains("操作失败");
  }
  statusAge_.invalidate();
  refreshMediaButtons();
  mediaControls &= !commands_[mission::Command::VIDEO_START]->isEnabled() &&
                   !logRecordButton_->isEnabled();
  mediaErrors_.clear();
  lastStatus_ = savedStatus;
  statusAge_.restart();
  refreshMediaButtons();
  const bool aircraftGeometry = aircraftModel_ && aircraftModel_->size() > 1000 &&
                                aircraftModel_->isVisible();
  const auto savedVehicle = lastVehicle_;
  const auto savedHistory = history_;
  const auto savedViewport = gl_->getViewportParameters();
  const auto childId = [](ccHObject *group) {
    return group->getChildrenNumber() ? group->getChild(0)->getUniqueID() : 0u;
  };
  const unsigned liveId = childId(live_), egoId = childId(ego_);
  bool aircraftPose = aircraftGeometry;
  if (aircraftGeometry) {
    // Known rotations exercise column-major placement and all three body axes.
    const QVector<QVector3D> axes{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const QVector<QVector3D> probes{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
    const QVector<QVector3D> expected{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
    for (int axis = 0; axis < 3; ++axis) {
      auto state = savedVehicle;
      put(state.mutable_position(), {3, -2, 5});
      const auto rotation = QQuaternion::fromAxisAndAngle(axes[axis], 90);
      auto *q = state.mutable_orientation();
      q->set_w(rotation.scalar());
      q->set_x(rotation.x());
      q->set_y(rotation.y());
      q->set_z(rotation.z());
      updateVehicle(state);
      // Assert before the bulk scene pass: the new pose must be visible now.
      CCVector3 point(probes[axis].x(), probes[axis].y(), probes[axis].z());
      aircraftModel_->getGLTransformation().apply(point);
      aircraftPose &= (QVector3D(point.x, point.y, point.z) -
                       (expected[axis] + QVector3D(3, -2, 5))).length() < 1e-5f;
      updateScene();
    }
  }
  const bool aircraftNoGeometryRebuild = childId(live_) == liveId &&
                                         childId(ego_) == egoId;
  auto invalidVehicle = savedVehicle;
  invalidVehicle.set_pose_valid(false);
  updateVehicle(invalidVehicle);
  const bool aircraftInvalidHidden = aircraftModel_ && !aircraftModel_->isVisible();
  updateVehicle(savedVehicle);
  history_ = savedHistory;
  trailDirty_ = dirty_ = true;
  updateScene();
  const bool aircraftRecovered = aircraftModel_ && aircraftModel_->isVisible() &&
                                 aircraft_->getChildrenNumber() == 1;
  focusAircraft();
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/quadrotor-closeup.png");
  gl_->setViewportParameters(savedViewport);
  gl_->redraw();
  const int gridCells = gridCenters_.size();
  auto *gridMesh = grid_->getChildrenNumber()
                       ? dynamic_cast<ccMesh *>(grid_->getChild(0)) : nullptr;
  const bool gridGeometry = gridCells > 0 && gridMesh &&
                            gridMesh->size() == unsigned(gridCells * 12);
  const bool liveEnabled = live_->isEnabled(), mapEnabled = map_->isEnabled(),
             egoEnabled = ego_->isEnabled();
  mission::Envelope gridFrame;
  gridFrame.set_protocol_version(1);
  gridFrame.set_session_id("demo");
  auto *gridSnapshot = gridFrame.mutable_grid();
  gridSnapshot->set_frame_id(frame_.toStdString());
  gridSnapshot->set_map_id(mapId_.toStdString());
  gridSnapshot->set_resolution_m(gridResolution_);
  gridSnapshot->set_inflated(true);
  for (const auto &cell : gridCenters_)
    put(gridSnapshot->add_centers(), cell);
  auto sendGrid = [this](mission::Envelope message) {
    message.set_sequence(++demoSequence_);
    client_.ingest(ProtocolClient::encode(message), 1);
    updateScene();
  };
  gridButton_->click();
  sendGrid(gridFrame); // A new snapshot must respect the user's hidden layer.
  const bool gridToggle = !grid_->isEnabled() &&
                          live_->isEnabled() == liveEnabled &&
                          map_->isEnabled() == mapEnabled &&
                          ego_->isEnabled() == egoEnabled;
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/ego-grid-hidden.png");
  gridButton_->click();
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/ego-grid-visible.png");
  auto emptyGrid = gridFrame;
  emptyGrid.mutable_grid()->clear_centers();
  sendGrid(emptyGrid);
  const bool gridCleared = gridCenters_.isEmpty() && !grid_->getChildrenNumber();
  sendGrid(gridFrame);
  auto wrongFrame = emptyGrid;
  wrongFrame.mutable_grid()->set_map_id("different-map");
  sendGrid(wrongFrame);
  const bool gridFrameGuard = gridCenters_.size() == gridCells;
  expireGrid();
  const bool gridExpired = !grid_->isVisible() && !grid_->getChildrenNumber();
  sendGrid(gridFrame);
  // Exercise the normal startup setting and incremental cloud path: the demo
  // itself uses full snapshots, which cannot prove that history is retained.
  const bool accumulationDefault = accumulate_->isChecked();
  const auto savedMap = mapPoints_, savedLive = livePoints_;
  const auto savedVoxels = voxels_;
  const bool savedReference = referenceMap_;
  const int savedDrawStep = drawStep_;
  mapPoints_.clear();
  voxels_.clear();
  referenceMap_ = false;
  drawStep_ = 0;
  auto sendCloud = [this](const QVector<MissionPoint> &points, bool snapshot) {
    mission::Envelope message;
    message.set_protocol_version(1);
    message.set_session_id("demo");
    message.set_sequence(++demoSequence_);
    auto *cloud = message.mutable_cloud();
    cloud->set_frame_id(frame_.toStdString());
    cloud->set_map_id(mapId_.toStdString());
    cloud->set_point_count(points.size());
    cloud->set_point_data(CloudIO::encode(points).toStdString());
    cloud->set_snapshot(snapshot);
    client_.ingest(ProtocolClient::encode(message), 1);
    updateScene();
  };
  auto displayedMapSize = [this]() -> unsigned {
    const auto *cloud = map_->getChildrenNumber()
                            ? dynamic_cast<ccPointCloud *>(map_->getChild(0))
                            : nullptr;
    return cloud && map_->isEnabled() ? cloud->size() : 0;
  };
  sendCloud({MissionPoint{0, 0, 0}}, false);
  const bool accumulationFirstFrame = displayedMapSize() == 1;
  for (int i = 1; i < 5; ++i)
    sendCloud({MissionPoint{float(i * voxel_ * 2), 0, 0}}, false);
  const bool accumulationHistory = livePoints_.size() == 1 &&
                                   mapPoints_.size() == 5 && displayedMapSize() == 5;
  QVector<MissionPoint> accumulatedPcd;
  QString accumulatedError;
  const bool accumulationExport = exportMap(p + "/accumulated.pcd") &&
      CloudIO::readPCD(p + "/accumulated.pcd", accumulatedPcd, accumulatedError) &&
      accumulatedPcd.size() == 5;
  const bool accumulationSaved = saveAccumulatedMap();
  mapPoints_.clear();
  voxels_.clear();
  mapDirty_ = dirty_ = true;
  updateScene();
  const bool accumulationRestored = accumulationSaved &&
      restoreAccumulatedMap() && mapPoints_.size() == 5 &&
      voxels_.size() == 5;
  updateScene();
  const bool accumulationRestoreVisible = displayedMapSize() == 5;
  sendCloud({MissionPoint{0, 0, 0}}, true);
  const bool accumulationSnapshot = displayedMapSize() == 1 && voxels_.size() == 1;
  sendCloud({}, true);
  const bool accumulationEmptySnapshot = mapPoints_.isEmpty() &&
                                         !map_->getChildrenNumber();
  const bool accumulationDeleted = saveAccumulatedMap() &&
      !QFile::exists(accumulatedMapPath());
  for (int i = 0; i < 5; ++i)
    sendCloud({MissionPoint{float(i * voxel_ * 2), 0, 0}}, false);
  const bool accumulationResaved = saveAccumulatedMap();
  mapPoints_ = savedMap;
  livePoints_ = savedLive;
  liveDirty_ = true;
  voxels_ = savedVoxels;
  referenceMap_ = savedReference;
  drawStep_ = savedDrawStep;
  accumulationDirty_ = false; // Smoke data has its own isolated save directory.
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  updateScene();
  setEditorCollapsed(false);
  QPushButton *addLocal = nullptr;
  for (auto *button : routeDialog_->findChildren<QPushButton *>()) {
    if (button->text() == "添加本地") {
      addLocal = button;
      break;
    }
  }
  const int countBeforeAdd = mission_.waypoints_size();
  const QVector3D aircraftPosition = v(lastVehicle_.position());
  bool waypointDefaultsToAircraft = false, waypointRejectsInvalidPose = false;
  if (addLocal) {
    QTimer::singleShot(0, this, [] {
      if (auto *dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget()))
        dialog->reject();
    });
    addLocal->click();
    waypointDefaultsToAircraft = mission_.waypoints_size() == countBeforeAdd + 1 &&
        (v(mission_.waypoints(countBeforeAdd).position()) - aircraftPosition).length() < 1e-5f;
    auto invalidPose = lastVehicle_;
    invalidPose.set_pose_valid(false);
    updateVehicle(invalidPose);
    addLocal->click();
    waypointRejectsInvalidPose = mission_.waypoints_size() == countBeforeAdd + 1;
    updateVehicle(savedVehicle);
    if (waypointDefaultsToAircraft) {
      mission_.mutable_waypoints()->DeleteSubrange(countBeforeAdd, 1);
      rebuildRoute();
    }
  }
  mission::YamlDocument routeSample;
  routeSample.set_usage("GUI 自检样例（只读）");
  routeSample.set_content_json(
      R"({"waypoints":[{"x":4.75,"y":0.22,"z":1.39,"pointmode":"Pass_through","dwell_time":0},{"x":6.75,"y":-0.53,"z":1.33,"pointmode":"Detect_point","dwell_time":3}]})");
  receiveRemoteRoute(routeSample);
  const int localWaypoints = mission_.waypoints_size();
  const bool waypointPopupCombined =
      waypoints_->rowCount() == localWaypoints + 2 &&
      waypoints_->item(0, 0)->text() == "本地" &&
      waypoints_->item(localWaypoints, 0)->text() == "机载";
  waypoints_->selectRow(localWaypoints);
  editWaypoint(); // A machine row must never open the local waypoint editor.
  const bool waypointRemoteReadOnly = mission_.waypoints_size() == localWaypoints;
  QApplication::processEvents();
  routeDialog_->grab().save(p + "/waypoints-popup.png");
  window_->screen()->grabWindow(window_->winId()).save(p + "/editor.png");
  clearRemoteRoute();
  setEditorCollapsed(true);
  QApplication::processEvents();
  for (int i = 0; i < 8; ++i)
    log(QString("日志展开测试 %1").arg(i + 1), "自检");
  latest_->click();
  QApplication::processEvents();
  logPopup_->grab().save(p + "/logs.png");
  logPopup_->hide();
  QTimer::singleShot(150, this, [p] {
    if (auto *d = qobject_cast<QDialog *>(QApplication::activeModalWidget())) {
      d->grab().save(p + "/settings.png");
      d->reject();
    }
  });
  settings();
  QTimer::singleShot(150, this, [p] {
    if (auto *d = qobject_cast<QDialog *>(QApplication::activeModalWidget())) {
      d->grab().save(p + "/yaml-settings.png");
      d->reject();
    }
  });
  yamlSettings();
  bool a = exportMap(p + "/map.pcd"),
       b = exportMission(p + "/mission.3dg.json");
  QVector<MissionPoint> pts;
  QString err;
  bool c = CloudIO::readPCD(p + "/map.pcd", pts, err);
  const QString externalLoad = qEnvironmentVariable("THREEDG_SMOKE_LOAD_PCD");
  if (!externalLoad.isEmpty())
    setDemo(false);
  const QString loadPath = externalLoad.isEmpty() ? p + "/map.pcd"
                                                  : externalLoad;
  const bool pcdLoaded = loadMapPath(loadPath);
  const auto *loadedCloud = map_->getChildrenNumber()
                                ? dynamic_cast<ccPointCloud *>(map_->getChild(0))
                                : nullptr;
  const bool pcdVisible = pcdLoaded && loadedCloud && map_->isEnabled() &&
                          loadedCloud->size() == unsigned(mapPoints_.size());
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/pcd-loaded.png");
  bool missionRead = importMission(p + "/mission.3dg.json");
  bool videoOk = player_->state() == QMediaPlayer::PlayingState &&
                 player_->isVideoAvailable() && player_->position() > 0;
  QSaveFile f(p + "/result.json");
  f.open(QIODevice::WriteOnly);
  f.write(QJsonDocument(QJsonObject{{"pcd_write", a},
                                    {"waypoint_defaults_to_aircraft", waypointDefaultsToAircraft},
                                    {"waypoint_rejects_invalid_pose", waypointRejectsInvalidPose},
                                    {"waypoint_popup_combined", waypointPopupCombined},
                                    {"waypoint_remote_readonly", waypointRemoteReadOnly},
                                    {"media_controls", mediaControls},
                                    {"aircraft_geometry", aircraftGeometry},
                                    {"aircraft_pose_xyz", aircraftPose},
                                    {"aircraft_pose_immediate", aircraftPose},
                                    {"aircraft_no_geometry_rebuild", aircraftNoGeometryRebuild},
                                    {"aircraft_invalid_hidden", aircraftInvalidHidden},
                                    {"aircraft_recovered", aircraftRecovered},
                                    {"accumulation_default", accumulationDefault},
                                    {"accumulation_first_frame", accumulationFirstFrame},
                                    {"accumulation_history", accumulationHistory},
                                    {"accumulation_export", accumulationExport},
                                    {"accumulation_saved", accumulationSaved},
                                    {"accumulation_restored", accumulationRestored},
                                    {"accumulation_restore_visible", accumulationRestoreVisible},
                                    {"accumulation_deleted", accumulationDeleted},
                                    {"accumulation_resaved", accumulationResaved},
                                    {"accumulation_snapshot", accumulationSnapshot},
                                    {"accumulation_empty_snapshot", accumulationEmptySnapshot},
                                    {"ego_grid_cells", gridCells},
                                    {"ego_grid_geometry", gridGeometry},
                                    {"ego_grid_toggle", gridToggle},
                                    {"ego_grid_empty_clears", gridCleared},
                                    {"ego_grid_frame_guard", gridFrameGuard},
                                    {"ego_grid_expiry", gridExpired},
                                    {"mission_write", b},
                                    {"pcd_read", c},
                                    {"pcd_loaded", pcdLoaded},
                                    {"pcd_loaded_points", mapPoints_.size()},
                                    {"pcd_scene_visible", pcdVisible},
                                    {"pcd_load_file", loadPath},
                                    {"mission_read", missionRead},
                                    {"video_playing", videoOk},
                                    {"video_empty_visible", emptyVideoVisible},
                                    {"points", pts.size()},
                                    {"trajectory_points", history_.size()},
                                    {"ego_points", egoPoints_.size()}})
              .toJson());
  f.commit();
  QApplication::exit(
      a && b && c && pcdLoaded && pcdVisible && missionRead && mediaControls &&
              waypointDefaultsToAircraft && waypointRejectsInvalidPose &&
              waypointPopupCombined && waypointRemoteReadOnly &&
              aircraftGeometry && aircraftPose && aircraftInvalidHidden && aircraftRecovered &&
              aircraftNoGeometryRebuild &&
              accumulationDefault && accumulationFirstFrame && accumulationHistory &&
              accumulationExport && accumulationSaved && accumulationRestored &&
              accumulationRestoreVisible && accumulationDeleted && accumulationResaved &&
              accumulationSnapshot && accumulationEmptySnapshot &&
              gridGeometry && gridToggle && gridCleared && gridFrameGuard && gridExpired &&
              emptyVideoVisible &&
              (!qEnvironmentVariableIsSet("THREEDG_SMOKE_VIDEO") || videoOk)
          ? 0
          : 1);
}
