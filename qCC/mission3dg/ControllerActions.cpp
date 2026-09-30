#include "MissionController.h"
#include "FlightReplayDialog.h"
#include "YamlSettingsPanel.h"
#include "mainwindow.h"
#include <FileIOFilter.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>
#include <QMediaPlayer>
#include <QQuaternion>
#include <QSet>
#include <QStandardPaths>
#include <QtWidgets>
#include <ccGLWindowInterface.h>
#include <ccDBRoot.h>
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
  if (replayDialog_)
    return;
  const bool modeChanged = demo_ != enabled;
  if (modeChanged && (accumulationDirty_ || colorAccumulationDirty_) &&
      !saveAccumulatedMap()) {
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
    colorMapPoints_.clear();
    colorVoxels_.clear();
    referenceMap_ = false;
    accumulationDirty_ = colorAccumulationDirty_ = false;
    autosavePathLogged_ = colorAutosavePathLogged_ = false;
    restoreAccumulatedMap();
    mapDirty_ = colorMapDirty_ = dirty_ = true;
    refreshMapInfo();
  }
  if (!enabled) {
    mode_->setText("离线");
    refreshLinkStats();
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
  refreshLinkStats();
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
  s->set_fcu_state_valid(true);
  s->set_armed(true);
  s->set_flight_mode("OFFBOARD");
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
  if (replayDialog_) {
    log("请先关闭飞行记录回放，再修改 3DG 配置");
    return;
  }
  QDialog d(window_);
  d.setWindowTitle("3DG 配置");
  d.resize(620, 440);
  QVBoxLayout outer(&d);
  QFormLayout f;
  QLineEdit address(baseUrl_), frame(frame_), map(mapId_), video(videoUrl_);
  QDoubleSpinBox voxel, hz, clearance;
  QSpinBox gridOpacity, gridSize;
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
  gridSize.setRange(10, 100);
  gridSize.setSuffix(" %");
  gridSize.setValue(gridSizePercent_);
  gridSize.setToolTip("可视立方体边长占 EGO 栅格分辨率的比例；仅影响 3DG 本地显示");
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
  f.addRow("EGO 栅格显示大小", &gridSize);
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
  if (changed && (accumulationDirty_ || colorAccumulationDirty_) &&
      !saveAccumulatedMap()) {
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
  if (videoChanged) {
    videoUrlFromAgent_ = false;
    videoUrlBeforeAgent_.clear();
  }
  voxel_ = voxel.value();
  cloudHz_ = hz.value();
  clearance_ = clearance.value();
  const bool gridAppearanceChanged =
      gridColor_ != selectedGridColor ||
      gridOpacity_ != gridOpacity.value() / 100.0 ||
      gridSizePercent_ != gridSize.value();
  gridColor_ = selectedGridColor;
  gridOpacity_ = gridOpacity.value() / 100.0;
  gridSizePercent_ = gridSize.value();
  if (changed) {
    resetSession();
    mapPoints_.clear();
    voxels_.clear();
    colorMapPoints_.clear();
    colorVoxels_.clear();
    referenceMap_ = false;
    accumulationDirty_ = colorAccumulationDirty_ = false;
    autosavePathLogged_ = colorAutosavePathLogged_ = false;
    restoreAccumulatedMap();
    mission_.clear_waypoints();
    rebuildRoute();
    mapDirty_ = colorMapDirty_ = dirty_ = true;
  }
  if (gridAppearanceChanged) {
    gridDirty_ = dirty_ = true;
    updateScene();
  }
  if (videoChanged) {
    if (videoUrl_.trimmed().isEmpty()) {
      videoRetry_.stop();
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
  s.setValue("gridSizePercent", gridSizePercent_);
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
void MissionController::openOdinStart() {
  if (!demo_ && taskModeSupported_ &&
      (taskMode_ != mission::REAL || !statusAge_.isValid() ||
       statusAge_.elapsed() > 3000)) {
    log("任务机未处于确认的实物模式，不能启动 Odin1", "任务机");
    return;
  }
  if (odinStartDialog_) {
    odinStartDialog_->raise();
    odinStartDialog_->activateWindow();
    return;
  }
  auto *dialog = new QDialog(window_);
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->setWindowTitle("任务启动 · Odin1");
  dialog->resize(560, 440);
  odinStartDialog_ = dialog;
  auto *layout = new QVBoxLayout(dialog);
  auto *modes = new QGroupBox("启动模式", dialog);
  auto *modeLayout = new QGridLayout(modes);
  auto *group = new QButtonGroup(dialog);
  odinModeGroup_ = group;
  const QStringList titles{"视觉里程计", "普通 SLAM", "地图重定位"};
  const QStringList descriptions{"仅运行 Odin 里程计，不保存地图",
                                 "建图和回环；可使用“保存机载地图”",
                                 "载入任务机已有的 Odin 原生 .bin 地图"};
  for (int mode = 0; mode < 3; ++mode) {
    auto *radio = new QRadioButton(titles[mode], modes);
    group->addButton(radio, mode);
    modeLayout->addWidget(radio, mode, 0);
    auto *description = new QLabel(descriptions[mode], modes);
    description->setObjectName("mutedText");
    modeLayout->addWidget(description, mode, 1);
    connect(radio, &QRadioButton::toggled, this,
            [this](bool) { refreshOdinStartDialog(); });
  }
  group->button(0)->setChecked(true);
  layout->addWidget(modes);
  auto *mapBox = new QGroupBox("任务机已保存地图", dialog);
  mapBox->setObjectName("odinMapBox");
  auto *mapLayout = new QVBoxLayout(mapBox);
  auto *refresh = new QPushButton("刷新列表", mapBox);
  mapLayout->addWidget(refresh, 0, Qt::AlignRight);
  odinMapList_ = new QListWidget(mapBox);
  odinMapList_->setMinimumHeight(125);
  mapLayout->addWidget(odinMapList_);
  odinMapHint_ = new QLabel("只列出任务机保存的 Odin 原生 .bin 地图；PCD 不能用于重定位。", mapBox);
  odinMapHint_->setWordWrap(true);
  odinMapHint_->setObjectName("mutedText");
  mapLayout->addWidget(odinMapHint_);
  layout->addWidget(mapBox, 1);
  connect(refresh, &QPushButton::clicked, this,
          &MissionController::requestOdinMaps);
  connect(odinMapList_, &QListWidget::itemSelectionChanged, this,
          &MissionController::refreshOdinStartDialog);
  auto *buttons = new QDialogButtonBox(dialog);
  odinStartButton_ = buttons->addButton("启动 Odin1", QDialogButtonBox::AcceptRole);
  buttons->addButton("取消", QDialogButtonBox::RejectRole);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
  connect(odinStartButton_, &QPushButton::clicked, this, [this] {
    if (!odinModeGroup_ || !odinStartDialog_ ||
        !odinStartButton_->isEnabled())
      return;
    const int mode = odinModeGroup_->checkedId();
    mission::Command command;
    command.set_kind(mission::Command::START_ODIN);
    auto *options = command.mutable_odin_start();
    options->set_mode(static_cast<mission::OdinStart::Mode>(mode));
    if (mode == mission::OdinStart::RELOCALIZATION)
      options->set_map_id(odinMapList_->currentItem()
                              ->data(Qt::UserRole).toString().toStdString());
    const QString request = client_.sendCommand(command);
    if (!request.isEmpty()) {
      pendingKinds_[request] = mission::Command::START_ODIN;
      log("已请求启动 Odin1：" + QStringList{"视觉里程计", "普通 SLAM", "地图重定位"}[mode],
          "任务机");
      odinStartDialog_->accept();
    }
  });
  connect(dialog, &QDialog::finished, this, [this] {
    odinMapsRequest_.clear();
    odinStartDialog_.clear();
    odinModeGroup_.clear();
    odinMapList_.clear();
    odinMapHint_.clear();
    odinStartButton_.clear();
  });
  refreshOdinStartDialog();
  dialog->show();
  requestOdinMaps();
}
void MissionController::requestOdinMaps() {
  if (!odinMapList_ || !odinMapHint_)
    return;
  odinMapList_->clear();
  if (demo_ || !client_.connected() || !odinModesSupported_) {
    odinMapHint_->setText("任务机未连接，或代理版本不支持模式选择；请更新任务机代理。");
    refreshOdinStartDialog();
    return;
  }
  mission::Command command;
  command.set_kind(mission::Command::GET_ODIN_MAPS);
  odinMapsRequest_ = client_.sendCommand(command);
  odinMapHint_->setText(odinMapsRequest_.isEmpty()
                            ? "地图列表请求未发送，请检查连接。"
                            : "正在读取任务机原生地图…");
  refreshOdinStartDialog();
}
void MissionController::refreshOdinStartDialog() {
  if (!odinStartDialog_ || !odinModeGroup_ || !odinStartButton_)
    return;
  const int mode = odinModeGroup_->checkedId();
  if (auto *mapBox = odinStartDialog_->findChild<QGroupBox *>("odinMapBox"))
    mapBox->setVisible(mode == mission::OdinStart::RELOCALIZATION);
  odinStartButton_->setEnabled(!demo_ && client_.connected() &&
      odinModesSupported_ && mode >= 0 &&
      (mode != mission::OdinStart::RELOCALIZATION ||
       (odinMapList_ && odinMapList_->currentItem())));
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
  if (!replayDialog_ && hud_) {
    hud_->setText("解锁状态 未知\n飞行模式 未知\n姿态 / 速度 / 位置 / 电量：—");
    batteryBar_->setValue(0);
  }
  plannerAge_.invalidate();
  lastStatus_.Clear();
  statusAge_.invalidate();
  taskMode_ = mission::MODE_UNKNOWN;
  modeSwitchRequest_.clear();
  refreshTaskModeControls();
  pendingKinds_.clear();
  mediaOperations_.clear();
  mediaErrors_.clear();
  refreshStartupButtons();
  drawStep_ = 0;
  colorDrawStep_ = 0;
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  if (aircraftModel_)
    aircraftModel_->setVisible(false);
  log("新会话：清除旧遥测和轨迹；当前地图保留，请核对地图 ID。");
}
void MissionController::receive(const mission::Envelope &e) {
  if (e.has_hello()) {
    odinModesSupported_ = false;
    taskModeSupported_ = false;
    for (const auto &capability : e.hello().capabilities()) {
      if (capability == "odin-start-modes-v1")
        odinModesSupported_ = true;
      if (capability == "task-mode-v1")
        taskModeSupported_ = true;
    }
    refreshTaskModeControls();
    if (odinStartDialog_ && odinModesSupported_ && odinMapList_ &&
        odinMapList_->count() == 0)
      requestOdinMaps();
    refreshOdinStartDialog();
  } else if (e.has_odin_maps()) {
    const QString request = QString::fromStdString(e.request_id());
    if (request == odinMapsRequest_ && odinMapList_ && odinMapHint_) {
      odinMapList_->clear();
      for (const auto &map : e.odin_maps().maps()) {
        const QDateTime modified = QDateTime::fromMSecsSinceEpoch(
            qint64(map.modified_unix_ms()));
        const QString name = QString::fromStdString(map.name());
        auto *item = new QListWidgetItem(
            QString("%1   ·   %2   ·   %3 MB")
                .arg(name, modified.toString("yyyy-MM-dd HH:mm"))
                .arg(double(map.size_bytes()) / (1024 * 1024), 0, 'f', 1),
            odinMapList_);
        item->setData(Qt::UserRole, QString::fromStdString(map.id()));
        item->setToolTip(name);
      }
      odinMapHint_->setText(odinMapList_->count()
                                ? "请选择与当前现场匹配的地图；仅原生 .bin 可用于重定位。"
                                : "任务机未找到可用的 Odin 原生地图；可先用普通 SLAM 建图并保存。");
      refreshOdinStartDialog();
    }
  } else if (e.has_vehicle()) {
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
        if (voxels_.size() < 400000 || voxels_.contains(k)) {
          MissionPoint ordinary = p;
          ordinary.r = 190;
          ordinary.g = 210;
          ordinary.b = 220;
          voxels_[k] = ordinary;
        }
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
    receiveColorCloud(livePoints_, c.snapshot());
    liveDirty_ = dirty_ = true;
    refreshMapInfo();
  } else if (e.has_grid()) {
    const auto &g = e.grid();
    if (!frameMatches(g.frame_id(), g.map_id()))
      return;
    if (g.delta()) {
      if (!gridAge_.isValid() || gridVersion_ != g.base_version() ||
          gridResolution_ != g.resolution_m() || gridInflated_ != g.inflated()) {
        log("EGO 栅格增量版本不匹配，重新请求完整快照", "协议");
        expireGrid();
        client_.requestGridResync();
        return;
      }
      auto next = gridCells_;
      bool valid = true;
      for (const auto &p : g.removed())
        valid = next.erase({p.x(), p.y(), p.z()}) == 1 && valid;
      for (const auto &p : g.added())
        valid = next.insert({p.x(), p.y(), p.z()}).second && valid;
      if (!valid || next.size() > 50000) {
        log("EGO 栅格增量内容不一致，重新请求完整快照", "协议");
        expireGrid();
        client_.requestGridResync();
        return;
      }
      gridCells_.swap(next);
    } else {
      gridCells_.clear();
      for (const auto &p : g.centers())
        gridCells_.insert({p.x(), p.y(), p.z()});
    }
    gridCenters_.clear();
    gridCenters_.reserve(int(gridCells_.size()));
    for (const auto &cell : gridCells_)
      gridCenters_ << QVector3D(float(std::get<0>(cell)),
                                float(std::get<1>(cell)),
                                float(std::get<2>(cell)));
    gridVersion_ = g.version();
    gridResolution_ = g.resolution_m();
    gridInflated_ = g.inflated();
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
    taskMode_ = e.status().task_mode();
    refreshTaskModeControls();
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
    const double networkMbps = s.network_mbps();
    const QString networkRate = networkMbps >= 0.1
                                    ? QString("%1 Mbps").arg(networkMbps, 0, 'f', 1)
                                    : QString("%1 kbps").arg(networkMbps * 1000.0, 0, 'f', 1);
    setStatusCells({
        demo_ ? "任务机 演示" : !taskModeSupported_ ? "任务机 在线"
                             : taskMode_ == mission::REAL ? "任务机 实物"
                             : taskMode_ == mission::SIMULATION ? "任务机 仿真"
                             : taskMode_ == mission::MODE_CONFLICT ? "任务机 模式冲突"
                                                                  : "任务机 模式未知",
        "Odin1 " + QString::fromStdString(s.odin()),
        "EGO " + QString::fromStdString(s.ego()),
        "定位 " + QString::fromStdString(s.localization()),
        QString("点云 %1 Hz").arg(s.cloud_hz(), 0, 'f', 1),
        "网络 " + networkRate,
        QString("CPU %1%").arg(s.cpu_percent(), 0, 'f', 0),
        QString("磁盘 %1 GB").arg(s.disk_free_gb(), 0, 'f', 1),
        s.video_running() ? "视频 机载推流中"
                          : localVideoPlaying() ? "视频 本机播放中"
                          : localVideoRequested() ? "视频 本机连接中"
                                                  : "视频 停止",
        s.recording() ? "记录 记录中" : "记录 停止"});
  } else if (e.has_yaml_document()) {
    const QString request = QString::fromStdString(e.request_id());
    if ((request == remoteRouteRequest_ ||
         request == remoteRoutePatchRequest_) &&
        e.yaml_document().id() == "route")
      receiveRemoteRoute(e.yaml_document());
  } else if (e.has_log())
    log(QString::fromStdString(e.log().text()),
        QString::fromStdString(e.log().source()));
  else if (e.has_result()) {
    const QString request = QString::fromStdString(e.request_id());
    if (request == modeSwitchRequest_ &&
        e.result().state() != mission::CommandResult::ACCEPTED) {
      modeSwitchRequest_.clear();
      refreshTaskModeControls();
    }
    if (request == odinMapsRequest_ &&
        e.result().state() != mission::CommandResult::ACCEPTED) {
      if (e.result().state() == mission::CommandResult::FAILED && odinMapHint_)
        odinMapHint_->setText("读取地图失败：" +
                              QString::fromStdString(e.result().detail()));
      odinMapsRequest_.clear();
    }
    if (request == remoteRouteRequest_ &&
        e.result().state() != mission::CommandResult::ACCEPTED) {
      if (e.result().state() == mission::CommandResult::FAILED)
        clearRemoteRoute();
      remoteRouteRequest_.clear();
      refreshRemoteEditControls();
    }
    if (request == remoteRoutePatchRequest_ &&
        e.result().state() != mission::CommandResult::ACCEPTED) {
      const bool saved = e.result().state() == mission::CommandResult::SUCCEEDED;
      remoteRoutePatchRequest_.clear();
      refreshRemoteEditControls();
      if (!saved && remoteRouteInfo_)
        remoteRouteInfo_->setText("机载航点保存失败：" +
            QString::fromStdString(e.result().detail()) + "；修改仍保留");
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
          (pendingKinds_.value(request) == mission::Command::UPLOAD_MISSION ||
           pendingKinds_.value(request) == mission::Command::START_EGO))
        QTimer::singleShot(0, this, [this] {
          importRemoteOnReceive_ = true;
          importRemoteAsUploaded_ = true;
          routeImportMissionRevision_ = mission_.revision();
          requestRemoteRoute();
        });
      if (e.result().state() == mission::CommandResult::SUCCEEDED &&
          pendingKinds_.value(request) == mission::Command::END_MISSION)
        expireGrid();
      pendingKinds_.remove(request);
      refreshMediaButtons();
    }
  } else if (e.has_video()) {
    const QUrl u(QString::fromStdString(e.video().url()));
    if (!e.video().running()) {
      if (videoUrlFromAgent_) {
        videoRetry_.stop();
        player_->stop();
        video_->hide();
        videoUrl_ = videoUrlBeforeAgent_;
        videoUrlBeforeAgent_.clear();
        videoUrlFromAgent_ = false;
      }
    } else if (u.scheme() == "rtsp" || u.scheme() == "http" ||
               u.scheme() == "https") {
      const bool addressChanged = videoUrl_ != u.toString();
      if (!videoUrlFromAgent_)
        videoUrlBeforeAgent_ = videoUrl_;
      videoUrl_ = u.toString();
      videoUrlFromAgent_ = true;
      if (addressChanged)
        log(u.path() == "/mission3dg" ? "已收到 Odin1 去畸变视频地址"
                                        : "已收到视频地址", "视频");
      showVideo();
    } else
      log("收到无效的视频地址", "视频");
    refreshMediaButtons();
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
        (state == "定位就绪" || state == "里程计就绪" ||
         state == "SLAM 建图中" || state == "重定位搜索中" ||
         state == "重定位就绪" || state == "规划器运行" ||
         state == "航线主控运行" || state == "运行" ||
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
  if (!demo_ && taskModeSupported_) {
    const bool real = taskMode_ == mission::REAL &&
                      statusAge_.isValid() && statusAge_.elapsed() <= 3000;
    for (auto kind : {mission::Command::START_ODIN, mission::Command::START_EGO})
      if (auto *control = commands_.value(kind, nullptr))
        control->setEnabled(real);
  }
  refreshMediaButtons();
}
void MissionController::refreshTaskModeControls() {
  if (!taskModeStatus_ || !realModeAction_ || !simModeAction_)
    return;
  const bool fresh = ready_ && taskModeSupported_ && statusAge_.isValid() &&
                     statusAge_.elapsed() <= 3000;
  const QString label = !ready_ ? "模式状态：任务机未连接"
      : !taskModeSupported_ ? "模式状态：任务机代理不支持切换"
      : !fresh ? "模式状态：等待任务机状态"
      : taskMode_ == mission::REAL ? "模式状态：实物模式"
      : taskMode_ == mission::SIMULATION ? "模式状态：仿真模式"
      : taskMode_ == mission::MODE_CONFLICT ? "模式状态：模式选择与运行进程冲突"
                                           : "模式状态：未知";
  taskModeStatus_->setText(label);
  auto *group = realModeAction_->actionGroup();
  if (group)
    group->setExclusive(false);
  realModeAction_->setChecked(fresh && taskMode_ == mission::REAL);
  simModeAction_->setChecked(fresh && taskMode_ == mission::SIMULATION);
  if (group)
    group->setExclusive(true);
  const bool canSwitch = fresh && modeSwitchRequest_.isEmpty() &&
                         taskMode_ != mission::MODE_CONFLICT;
  realModeAction_->setEnabled(canSwitch);
  simModeAction_->setEnabled(canSwitch);
}
void MissionController::switchTaskMode(mission::TaskMode mode) {
  if (!ready_ || !taskModeSupported_ || !modeSwitchRequest_.isEmpty() ||
      !statusAge_.isValid() || statusAge_.elapsed() > 3000 ||
      (mode != mission::REAL && mode != mission::SIMULATION)) {
    refreshTaskModeControls();
    return;
  }
  mission::Command request;
  request.set_kind(mission::Command::SET_TASK_MODE);
  request.set_task_mode(mode);
  modeSwitchRequest_ = client_.sendCommand(request);
  if (modeSwitchRequest_.isEmpty())
    log("模式切换命令未发送，请检查任务机连接", "任务机");
  else {
    const QString id = modeSwitchRequest_;
    QTimer::singleShot(31000, this, [this, id] {
      if (modeSwitchRequest_ == id) {
        modeSwitchRequest_.clear();
        log("模式切换回执超时；请核对任务机当前模式", "任务机");
        refreshTaskModeControls();
      }
    });
  }
  refreshTaskModeControls();
}
bool MissionController::localVideoRequested() const {
  return player_ && video_ && !video_->isHidden() &&
         !videoUrl_.trimmed().isEmpty() &&
         (player_->state() != QMediaPlayer::StoppedState || videoRetry_.isActive());
}
bool MissionController::localVideoPlaying() const {
  return localVideoRequested() &&
         player_->state() == QMediaPlayer::PlayingState &&
         player_->isVideoAvailable() &&
         player_->mediaStatus() != QMediaPlayer::StalledMedia &&
         player_->mediaStatus() != QMediaPlayer::InvalidMedia;
}
bool MissionController::controlsLocalVideo() const {
  const bool fresh = (ready_ || demo_) && statusAge_.isValid() &&
                     statusAge_.elapsed() <= 3000;
  return !(fresh && lastStatus_.video_running()) &&
         (localVideoRequested() ||
          (!videoUrlFromAgent_ && !videoUrl_.trimmed().isEmpty()));
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
    const bool local = video && controlsLocalVideo();
    const bool playing = local && localVideoPlaying();
    const bool requested = local && localVideoRequested();
    const bool pending = mediaOperations_.contains(start);
    QString text = local ? (requested ? "停止播放" : "播放视频")
                   : video ? (running ? "停止视频" : "开启视频")
                         : (running ? "停止记录" : "开始记录");
    QString state = local ? (playing ? "● 本机播放中"
                                     : requested ? "本机视频连接中…" : "本机播放已停止")
                    : !online ? "未连接" : !fresh ? "等待有效状态"
                    : running ? (video ? "● 视频运行中" : "● 正在记录")
                              : (video ? "视频已停止" : "记录已停止");
    QString tip = state;
    if (pending) {
      const auto &operation = mediaOperations_[start];
      text = operation.target ? (video ? "视频开启中…" : "记录启动中…")
                              : (video ? "视频停止中…" : "记录停止中…");
      state = operation.succeeded ? "等待状态确认" : "等待任务机执行";
      tip = state;
    } else if (!local && mediaErrors_.contains(start)) {
      state += mediaErrors_[start].startsWith("操作超时") ? " · 操作超时" : " · 操作失败";
      tip = mediaErrors_[start];
    }
    if (demo_)
      state.prepend("模拟 · ");
    auto update = [&](QPushButton *b, QLabel *label) {
      if (!b || !label)
        return;
      b->setText(text);
      b->setEnabled((local || fresh) && !pending);
      b->setToolTip(tip);
      label->setText(state);
      label->setToolTip(tip);
      const bool active = (running || playing) && !pending;
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
    else if (statusCells_.size() > 8 && local)
      statusCells_[8]->setText(playing ? "视频 本机播放中"
                                   : requested ? "视频 本机连接中"
                                               : "视频 本机播放已停止");
  }
}
void MissionController::toggleMedia(mission::Command::Kind start) {
  if (start == mission::Command::VIDEO_START && controlsLocalVideo()) {
    if (localVideoRequested()) {
      videoRetry_.stop();
      player_->stop();
      video_->hide();
    } else {
      showVideo();
    }
    refreshMediaButtons();
    return;
  }
  const bool running = start == mission::Command::VIDEO_START
                           ? lastStatus_.video_running() : lastStatus_.recording();
  command(running ? (start == mission::Command::VIDEO_START
                         ? mission::Command::VIDEO_STOP : mission::Command::RECORD_STOP)
                  : start);
}
void MissionController::command(mission::Command::Kind k) {
  if (!demo_ && taskModeSupported_ &&
      (k == mission::Command::START_ODIN || k == mission::Command::START_EGO) &&
      (taskMode_ != mission::REAL || !statusAge_.isValid() ||
       statusAge_.elapsed() > 3000)) {
    log("任务机未处于确认的实物模式，启动命令未发送", "任务机");
    return;
  }
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
  const bool onboardEgoRoute = routeSyncedToOnboard_ &&
      mission_.waypoints_size() >= 2 &&
      remotePoints_.size() >= 2 && !remoteRouteRevision_.isEmpty() &&
      remoteRouteRequest_.isEmpty() && remoteRoutePatchRequest_.isEmpty() &&
      remoteDrafts_.isEmpty() && remoteDeleted_.isEmpty();
  const bool localEgoRoute = mission_.waypoints_size() >= 2 && !onboardEgoRoute;
  if (k == mission::Command::START_EGO && !demo_) {
    if (!localEgoRoute && !onboardEgoRoute) {
      const QString reason = "当前编辑航线不足两个航点；请从任务机载入或添加航点。";
      log(reason + " 启动 EGO 未发送");
      QMessageBox::warning(window_, "无法启动 EGO", reason);
      return;
    }
    QString detail;
    if (localEgoRoute) {
      detail = QString("将上传当前 %1 个编辑航点，并启动实物 EGO 及航线主控。")
                   .arg(mission_.waypoints_size());
    } else {
      const auto first = remoteEntries_.first().toObject();
      const auto last = remoteEntries_.last().toObject();
      detail = QString("将按任务机当前 %1 个机载航点启动实物 EGO 及航线主控。\n"
                       "首点 (%2, %3, %4)；末点 (%5, %6, %7) 米。\n%8")
                   .arg(remotePoints_.size())
                   .arg(first.value("x").toDouble(), 0, 'f', 2)
                   .arg(first.value("y").toDouble(), 0, 'f', 2)
                   .arg(first.value("z").toDouble(), 0, 'f', 2)
                   .arg(last.value("x").toDouble(), 0, 'f', 2)
                   .arg(last.value("y").toDouble(), 0, 'f', 2)
                   .arg(last.value("z").toDouble(), 0, 'f', 2)
                   .arg(remoteRouteSource_);
    }
    detail += "\n飞机须保持落地上锁；主控就绪后由飞手手动解锁，随后请求 OFFBOARD。\n确认使用这条航线？";
    if (QMessageBox::question(window_, "启动 EGO 航线", detail) != QMessageBox::Yes)
      return;
  }
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
  if ((k == mission::Command::UPLOAD_MISSION || k == mission::Command::START_EGO) &&
      !remoteRouteRequest_.isEmpty()) {
    log("机载航线正在刷新，请等待完成后再上传或启动");
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
  if (k == mission::Command::UPLOAD_MISSION ||
      (k == mission::Command::START_EGO && localEgoRoute)) {
    if (mission_.waypoints_size() < 2) {
      log("至少需要两个航点");
      return;
    }
    *c.mutable_mission() = mission_;
    if (!remoteRouteRevision_.isEmpty())
      c.set_yaml_revision(remoteRouteRevision_.toStdString());
  }
  if (k == mission::Command::START_EGO && onboardEgoRoute)
    c.set_yaml_revision(remoteRouteRevision_.toStdString());
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
  if (!remoteDrafts_.isEmpty() || !remoteDeleted_.isEmpty() ||
      !remoteRoutePatchRequest_.isEmpty()) {
    log("机载航点有未保存修改或正在保存；请先处理编辑模式");
    return;
  }
  if (!remoteRouteRequest_.isEmpty())
    return;
  mission::Command request;
  request.set_kind(mission::Command::GET_YAML_DOCUMENT);
  request.set_yaml_document_id("route");
  remoteRouteRequest_ = client_.sendCommand(request);
  if (!remoteRouteRequest_.isEmpty()) {
    log("正在读取任务机当前 EGO 航点…");
    if (remoteRouteInfo_)
      remoteRouteInfo_->setText("正在刷新机载航线…");
    refreshRemoteEditControls();
  } else {
    importRemoteOnReceive_ = false;
    importRemoteAsUploaded_ = false;
    log("机载航线刷新请求未发送，请检查任务机连接");
  }
}
void MissionController::loadOnboardRoute() {
  if (demo_ || !client_.connected()) {
    log("任务机未连接，无法载入机载航线");
    return;
  }
  if (pendingKinds_.values().contains(mission::Command::UPLOAD_MISSION) ||
      pendingKinds_.values().contains(mission::Command::START_EGO)) {
    log("航线正在上传或启动，请等待任务机回执后再载入");
    return;
  }
  if (mission_.waypoints_size() > 0 && !routeSyncedToOnboard_ &&
      QMessageBox::question(
          window_, "载入机载航线",
          "从任务机载入会替换当前未上传的编辑航线。确认继续？") != QMessageBox::Yes)
    return;
  importRemoteOnReceive_ = true;
  importRemoteAsUploaded_ = false;
  routeImportMissionRevision_ = mission_.revision();
  requestRemoteRoute();
}
void MissionController::clearRemoteRoute() {
  remoteRouteRequest_.clear();
  remoteRoutePatchRequest_.clear();
  remoteRouteSource_.clear();
  remoteRouteRevision_.clear();
  remoteEntries_ = QJsonArray();
  remotePoints_.clear();
  remoteDrafts_.clear();
  remoteDeleted_.clear();
  remoteEditMode_ = false;
  importRemoteOnReceive_ = false;
  importRemoteAsUploaded_ = false;
  routeSyncedToOnboard_ = false;
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
  remoteRouteRevision_ = QString::fromStdString(document.revision());
  remoteDrafts_.clear();
  remoteDeleted_.clear();
  if (remoteRoute_)
    remoteRoute_->removeAllChildren();
  if (importRemoteOnReceive_) {
    const bool importedAfterUpload = importRemoteAsUploaded_;
    importRemoteOnReceive_ = false;
    importRemoteAsUploaded_ = false;
    routeSyncedToOnboard_ = false;
    if (mission_.revision() != routeImportMissionRevision_) {
      log("读取期间编辑航线已变化，保留当前本地修改；可再次从任务机载入");
      refreshWaypointTable();
      refreshMapInfo();
      refreshRouteStrip();
      return;
    }
    const auto speed = parsed.object().value("ego").toObject().value("max_vel");
    if (entries.size() < 2 || !speed.isDouble() ||
        !std::isfinite(speed.toDouble()) || speed.toDouble() < .1 ||
        speed.toDouble() > 20) {
      log("机载航线或全局巡航速度无效，原编辑航线未替换");
      refreshWaypointTable();
      refreshMapInfo();
      refreshRouteStrip();
      return;
    }
    mission::MissionPlan loaded;
    loaded.set_mission_id(QUuid::createUuid().toString(
        QUuid::WithoutBraces).toStdString());
    loaded.set_frame_id(frame_.toStdString());
    loaded.set_map_id(mapId_.toStdString());
    for (const auto &value : entries) {
      const auto point = value.toObject();
      const auto yaw = point.value("yaw");
      const auto dwell = point.value("dwell_time").isUndefined()
                             ? QJsonValue(0.0) : point.value("dwell_time");
      const auto pitch = point.value("inspection_pitch_deg").isUndefined()
                             ? (point.value("gimbal_pitch").isUndefined()
                                    ? QJsonValue(0.0) : point.value("gimbal_pitch"))
                             : point.value("inspection_pitch_deg");
      const auto zoom = point.value("inspection_zoom_ratio").isUndefined()
                            ? (point.value("cam_zoom").isUndefined()
                                   ? QJsonValue(1.0) : point.value("cam_zoom"))
                            : point.value("inspection_zoom_ratio");
      const QString mode = point.value("pointmode").toString("Pass_through");
      if (!yaw.isDouble() || !dwell.isDouble() || !pitch.isDouble() ||
          !zoom.isDouble() || !std::isfinite(yaw.toDouble()) ||
          !std::isfinite(dwell.toDouble()) || !std::isfinite(pitch.toDouble()) ||
          !std::isfinite(zoom.toDouble()) || std::abs(yaw.toDouble()) > 180 ||
          dwell.toDouble() < 0 || dwell.toDouble() > 3600 ||
          std::abs(pitch.toDouble()) > 180 || zoom.toDouble() < 0 ||
          zoom.toDouble() > 100 ||
          (mode != "Pass_through" && mode != "Detect_point" &&
           mode != "Land_point")) {
        log("机载航点属性无效，原编辑航线未替换");
        refreshWaypointTable();
        refreshMapInfo();
        refreshRouteStrip();
        return;
      }
      auto *w = loaded.add_waypoints();
      w->mutable_position()->set_x(point.value("x").toDouble());
      w->mutable_position()->set_y(point.value("y").toDouble());
      w->mutable_position()->set_z(point.value("z").toDouble());
      w->set_yaw(yaw.toDouble() * M_PI / 180);
      w->set_speed(speed.toDouble());
      w->set_pointmode(mode.toStdString());
      w->set_hold_s(dwell.toDouble());
      w->set_inspection_pitch_deg(pitch.toDouble());
      w->set_inspection_zoom_ratio(zoom.toDouble());
    }
    mission_ = std::move(loaded);
    rebuildRoute();
    routeSyncedToOnboard_ = importedAfterUpload;
    refreshMapInfo();
    refreshWaypointTable();
    refreshRouteStrip();
    log(QString("已将任务机 %1 个航点载入%2：%3")
            .arg(mission_.waypoints_size())
            .arg(importedAfterUpload ? "机载同步列表" : "本地编辑列表")
            .arg(remoteRouteSource_));
  }
  refreshWaypointTable();
  refreshMapInfo();
  refreshRouteStrip();
  gl_->redraw();
  log(QString("已读取任务机航点 %1 个：%2")
          .arg(remotePoints_.size()).arg(remoteRouteSource_));
}
void MissionController::refreshRemoteEditControls() {
  const bool available = !demo_ && client_.connected() &&
                         client_.routeEditSupported() &&
                         !remoteRouteRevision_.isEmpty() && !remoteEntries_.isEmpty();
  const bool idle = remoteRoutePatchRequest_.isEmpty();
  if (remoteRefreshButton_) {
    const bool reading = !remoteRouteRequest_.isEmpty();
    remoteRefreshButton_->setText(reading ? "载入中…" : "从任务机载入");
    remoteRefreshButton_->setEnabled(!demo_ && client_.connected() && !reading && idle &&
        remoteDrafts_.isEmpty() && remoteDeleted_.isEmpty());
  }
  if (remoteEditButton_) {
    remoteEditButton_->setText(remoteEditMode_ ? "退出机载编辑" : "编辑机载");
    remoteEditButton_->setEnabled(available && idle);
    remoteEditButton_->setToolTip(
        client_.connected() && !client_.routeEditSupported()
            ? "任务机代理版本不支持机载航点编辑，请更新代理"
            : "进入机载航点编辑模式；保存会写回当前任务机航线");
  }
  if (remoteSaveButton_)
    remoteSaveButton_->setEnabled(available && idle && remoteEditMode_ &&
                                  (!remoteDrafts_.isEmpty() || !remoteDeleted_.isEmpty()));
  if (remoteDeleteButton_) {
    const int row = waypoints_ ? waypoints_->currentRow() - mission_.waypoints_size() : -1;
    const bool selected = row >= 0 && row < remoteEntries_.size();
    const bool deleted = selected && remoteDeleted_.contains(row);
    remoteDeleteButton_->setText(deleted ? "撤销机载删除" : "删除机载");
    remoteDeleteButton_->setEnabled(available && client_.routeDeleteSupported() &&
        idle && remoteEditMode_ && selected &&
        (deleted || remoteEntries_.size() - remoteDeleted_.size() > 2));
    remoteDeleteButton_->setToolTip(client_.routeDeleteSupported()
        ? "标记所选机载航点待删除；保存机载修改后生效"
        : "任务机代理版本不支持删除机载航点，请更新代理");
  }
}
void MissionController::refreshRemoteRoutePreview() {
  if (!remoteRoute_ || !gl_)
    return;
  QVector<QVector3D> points;
  points.reserve(remoteEntries_.size());
  for (int i = 0; i < remoteEntries_.size(); ++i) {
    if (remoteDeleted_.contains(i))
      continue;
    const auto entry = remoteDrafts_.value(i, remoteEntries_.at(i).toObject());
    points << QVector3D(entry.value("x").toDouble(),
                        entry.value("y").toDouble(),
                        entry.value("z").toDouble());
  }
  if (remoteDrafts_.isEmpty() && remoteDeleted_.isEmpty())
    makeLine(remoteRoute_, points, 118, 218, 247, 3);
  else
    makeLine(remoteRoute_, points, 255, 167, 69, 3);
  gl_->redraw();
}
void MissionController::toggleRemoteEdit() {
  if (!remoteEditMode_) {
    if (!client_.connected() || remoteRouteRevision_.isEmpty() ||
        remoteEntries_.isEmpty())
      return;
    remoteEditMode_ = true;
  } else {
    if ((!remoteDrafts_.isEmpty() || !remoteDeleted_.isEmpty()) &&
        QMessageBox::question(routeDialog_, "放弃机载修改",
                              "放弃尚未保存的机载航点修改？") != QMessageBox::Yes)
      return;
    remoteEditMode_ = false;
    remoteDrafts_.clear();
    remoteDeleted_.clear();
    refreshRemoteRoutePreview();
  }
  refreshWaypointTable();
  refreshRouteStrip();
}
void MissionController::editRemoteWaypoint(int row) {
  if (!remoteEditMode_ || !client_.connected() ||
      !remoteRoutePatchRequest_.isEmpty() ||
      row < 0 || row >= remoteEntries_.size() || remoteDeleted_.contains(row))
    return;
  const auto original = remoteEntries_.at(row).toObject();
  const auto current = remoteDrafts_.value(row, original);
  QDialog dialog(window_);
  dialog.setWindowTitle(QString("编辑机载航点 %1").arg(row + 1));
  QFormLayout form(&dialog);
  QDoubleSpinBox x, y, z, yaw, dwell, pitch, zoom;
  for (auto *spin : {&x, &y, &z}) {
    spin->setRange(-10000, 10000);
    spin->setDecimals(3);
  }
  x.setValue(current.value("x").toDouble());
  y.setValue(current.value("y").toDouble());
  z.setValue(current.value("z").toDouble());
  yaw.setRange(-180, 180);
  yaw.setDecimals(2);
  yaw.setValue(current.value("yaw").toDouble());
  dwell.setRange(0, 3600);
  dwell.setDecimals(2);
  dwell.setValue(current.value("dwell_time").toDouble());
  pitch.setRange(-180, 180);
  pitch.setDecimals(2);
  pitch.setValue(current.value("inspection_pitch_deg").toDouble());
  zoom.setRange(0, 100);
  zoom.setDecimals(2);
  zoom.setValue(current.value("inspection_zoom_ratio").toDouble(1));
  QComboBox mode;
  mode.addItems({"Pass_through", "Detect_point", "Land_point"});
  mode.setCurrentText(current.value("pointmode").toString());
  QSet<QString> changed;
  auto watch = [&dialog, &changed](QDoubleSpinBox &spin, const QString &field) {
    connect(&spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dialog, [&changed, field] { changed.insert(field); });
  };
  watch(x, "x");
  watch(y, "y");
  watch(z, "z");
  watch(yaw, "yaw");
  watch(dwell, "dwell_time");
  watch(pitch, "inspection_pitch_deg");
  watch(zoom, "inspection_zoom_ratio");
  connect(&mode, &QComboBox::currentTextChanged, &dialog,
          [&changed] { changed.insert("pointmode"); });
  form.addRow("X / m", &x);
  form.addRow("Y / m", &y);
  form.addRow("Z / m", &z);
  form.addRow("航向 / °", &yaw);
  form.addRow("类型", &mode);
  form.addRow("停留 / s", &dwell);
  form.addRow("云台俯仰 / °", &pitch);
  form.addRow("相机变焦", &zoom);
  QDialogButtonBox buttons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  form.addRow(&buttons);
  connect(&buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(&buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted)
    return;
  if (changed.isEmpty())
    return;
  auto edited = current;
  if (changed.contains("x"))
    edited.insert("x", x.value());
  if (changed.contains("y"))
    edited.insert("y", y.value());
  if (changed.contains("z"))
    edited.insert("z", z.value());
  if (changed.contains("yaw"))
    edited.insert("yaw", yaw.value());
  if (changed.contains("pointmode"))
    edited.insert("pointmode", mode.currentText());
  if (changed.contains("dwell_time"))
    edited.insert("dwell_time", dwell.value());
  if (changed.contains("inspection_pitch_deg"))
    edited.insert("inspection_pitch_deg", pitch.value());
  if (changed.contains("inspection_zoom_ratio"))
    edited.insert("inspection_zoom_ratio", zoom.value());
  if (edited == original)
    remoteDrafts_.remove(row);
  else
    remoteDrafts_.insert(row, edited);
  refreshWaypointTable();
  waypoints_->selectRow(mission_.waypoints_size() + row);
  refreshRemoteRoutePreview();
}
void MissionController::deleteRemoteWaypoint() {
  const int row = waypoints_ ? waypoints_->currentRow() - mission_.waypoints_size() : -1;
  if (!remoteEditMode_ || !client_.connected() ||
      !client_.routeDeleteSupported() ||
      !remoteRoutePatchRequest_.isEmpty() || row < 0 || row >= remoteEntries_.size())
    return;
  if (remoteDeleted_.contains(row)) {
    remoteDeleted_.remove(row);
  } else {
    if (remoteEntries_.size() - remoteDeleted_.size() <= 2) {
      log("机载航线至少保留两个航点");
      return;
    }
    remoteDrafts_.remove(row);
    remoteDeleted_.insert(row);
  }
  refreshWaypointTable();
  waypoints_->selectRow(mission_.waypoints_size() + row);
  refreshRemoteRoutePreview();
  refreshRouteStrip();
}
void MissionController::saveRemoteRoute() {
  if (!remoteEditMode_ || (remoteDrafts_.isEmpty() && remoteDeleted_.isEmpty()) ||
      remoteRouteRevision_.isEmpty() || !client_.connected() ||
      (!remoteDeleted_.isEmpty() && !client_.routeDeleteSupported()) ||
      !remoteRoutePatchRequest_.isEmpty())
    return;
  QJsonObject changes;
  const QStringList fields{"x", "y", "z", "yaw", "pointmode",
                           "dwell_time", "inspection_pitch_deg",
                           "inspection_zoom_ratio"};
  for (auto it = remoteDrafts_.cbegin(); it != remoteDrafts_.cend(); ++it) {
    const int row = it.key();
    if (row < 0 || row >= remoteEntries_.size())
      return;
    const auto original = remoteEntries_.at(row).toObject();
    for (const auto &field : fields) {
      if (it.value().value(field) != original.value(field))
        changes.insert(QString("/waypoints/%1/%2").arg(row).arg(field),
                       it.value().value(field));
    }
  }
  for (int row : remoteDeleted_)
    changes.insert(QString("/waypoints/%1").arg(row), QJsonValue::Null);
  if (changes.isEmpty()) {
    remoteDrafts_.clear();
    remoteDeleted_.clear();
    refreshWaypointTable();
    refreshRemoteRoutePreview();
    return;
  }
  mission::Command command;
  command.set_kind(mission::Command::PATCH_YAML_DOCUMENT);
  command.set_yaml_document_id("route");
  command.set_yaml_revision(remoteRouteRevision_.toStdString());
  command.set_yaml_patch_json(
      QJsonDocument(changes).toJson(QJsonDocument::Compact).toStdString());
  remoteRoutePatchRequest_ = client_.sendCommand(command);
  if (!remoteRoutePatchRequest_.isEmpty()) {
    remoteRouteInfo_->setText(QString("正在保存 %1 项机载修改到任务机…")
                                  .arg(remoteDrafts_.size() + remoteDeleted_.size()));
    refreshRemoteEditControls();
  } else
    log("机载航点保存命令未发送，请检查任务机连接");
}
void MissionController::refreshWaypointTable() {
  if (remoteRouteInfo_) {
    QString description = remoteRouteSource_.isEmpty()
                              ? QStringLiteral("任务机航线尚未读取")
                              : QString("任务机当前航线 %1 个航点 · %2")
                                    .arg(remotePoints_.size()).arg(remoteRouteSource_);
    if (routeSyncedToOnboard_)
      description += QStringLiteral(" · 已载入编辑区并同步");
    else if (mission_.waypoints_size() > 0)
      description += QStringLiteral(" · 编辑区有未上传航线");
    remoteRouteInfo_->setText(description);
  }
  refreshRemoteEditControls();
  if (!waypoints_)
    return;
  const int localCount = mission_.waypoints_size();
  const int selected = waypoints_->currentRow();
  QSignalBlocker blocker(waypoints_);
  waypoints_->clearContents();
  waypoints_->setRowCount(localCount);
  double cumulativeDistance = 0;
  for (int i = 0; i < localCount; ++i) {
    const auto &point = mission_.waypoints(i);
    const double zoom = point.has_inspection_zoom_ratio()
                            ? point.inspection_zoom_ratio() : 1;
    double segmentDistance = 0;
    if (i > 0) {
      const auto &previous = mission_.waypoints(i - 1).position();
      const auto &current = point.position();
      const double dx = current.x() - previous.x();
      const double dy = current.y() - previous.y();
      const double dz = current.z() - previous.z();
      segmentDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
      cumulativeDistance += segmentDistance;
    }
    const QString mode = point.pointmode().empty()
                             ? QStringLiteral("Pass_through")
                             : QString::fromStdString(point.pointmode());
    const QString modeLabel = mode == "Detect_point" ? "检测点"
                              : mode == "Land_point" ? "降落点" : "直通点";
    const QStringList values{
        routeSyncedToOnboard_ ? "机载已同步" : "本地编辑", QString::number(i + 1),
        QString::number(point.position().x(), 'f', 2),
        QString::number(point.position().y(), 'f', 2),
        QString::number(point.position().z(), 'f', 2),
        QString::number(point.yaw() * 180 / M_PI, 'f', 1),
        QString::number(point.speed(), 'f', 2),
        modeLabel,
        QString::number(point.hold_s(), 'f', 1),
        QString::number(point.inspection_pitch_deg(), 'f', 1),
        QString::number(zoom, 'f', 1),
        i == 0 ? QStringLiteral("—") : QString::number(segmentDistance, 'f', 2),
        QString::number(cumulativeDistance, 'f', 2)};
    for (int column = 0; column < values.size(); ++column) {
      auto *item = new QTableWidgetItem(values[column]);
      item->setTextAlignment((column == 0 || column == 7)
                                 ? Qt::AlignCenter
                                 : Qt::AlignRight | Qt::AlignVCenter);
      item->setToolTip(QString("航点 %1 · %2 · 全局巡航速度 %3 m/s\n"
                               "本段 %4 · 累计 %5 m · 双击编辑")
                           .arg(i + 1).arg(mode)
                           .arg(point.speed(), 0, 'f', 2)
                           .arg(i == 0 ? QStringLiteral("无上一航点")
                                       : QString("%1 m").arg(segmentDistance, 0, 'f', 2))
                           .arg(cumulativeDistance, 0, 'f', 2));
      waypoints_->setItem(i, column, item);
    }
  }
  if (routeStats_) {
    routeStats_->setText(localCount == 0
        ? QString("坐标系 %1 · 地图 %2 · 暂无航点").arg(frame_, mapId_)
        : QString("坐标系 %1 · 地图 %2 · 全局巡航速度 %3 m/s · 航线长度 %4 m")
              .arg(frame_, mapId_)
              .arg(mission_.waypoints(0).speed(), 0, 'f', 2)
              .arg(cumulativeDistance, 0, 'f', 2));
  }
  if (selected >= 0 && selected < waypoints_->rowCount())
    waypoints_->selectRow(selected);
  refreshRemoteEditControls();
  updateWaypointGizmo();
}
void MissionController::addWaypoint(const QVector3D &p) {
  if (mission_.waypoints_size() >= 1000)
    return;
  if (mission_.waypoints_size() > 0 &&
      mission_.waypoints(mission_.waypoints_size() - 1).pointmode() == "Land_point") {
    log("最后一个航点为降落点；请先修改其类型，再添加航点");
    return;
  }
  auto *w = mission_.add_waypoints();
  put(w->mutable_position(), p);
  w->set_speed(mission_.waypoints_size() > 1
                   ? mission_.waypoints(0).speed() : 1);
  w->set_pointmode("Pass_through");
  w->set_inspection_zoom_ratio(1);
  rebuildRoute();
}
void MissionController::createWaypoint() {
  if (mission_.waypoints_size() >= 1000) {
    QMessageBox::warning(window_, "无法添加航点", "航点已达到 1000 个");
    return;
  }
  if (mission_.waypoints_size() > 0 &&
      mission_.waypoints(mission_.waypoints_size() - 1).pointmode() == "Land_point") {
    QMessageBox::warning(window_, "无法添加航点",
                         "最后一个航点为降落点；请先修改其类型，再添加航点");
    return;
  }
  const bool fresh = lastVehicle_.pose_valid() && vehicleAge_.isValid() &&
                     vehicleAge_.elapsed() <= 3000;
  const QVector3D aircraft = v(lastVehicle_.position());
  const bool validPosition = fresh && std::isfinite(aircraft.x()) &&
                             std::isfinite(aircraft.y()) &&
                             std::isfinite(aircraft.z());
  const QVector3D position = validPosition
      ? aircraft : QVector3D(0, 0, altitude_ ? float(altitude_->value()) : 3.0f);
  const mission::MissionPlan original = mission_;
  const bool originalSynced = routeSyncedToOnboard_;
  const int row = mission_.waypoints_size();
  addWaypoint(position);
  waypoints_->selectRow(row);
  setEditorCollapsed(false);
  if (!editWaypoint(!validPosition)) {
    mission_ = original;
    rebuildRoute();
    mission_.set_revision(original.revision());
    routeSyncedToOnboard_ = originalSynced;
    refreshMapInfo();
    refreshWaypointTable();
    refreshRouteStrip();
  }
}
void MissionController::onItemPicked(const PickedItem &i) {
  if (picking_->isChecked())
    addWaypoint({i.P3D.x, i.P3D.y, float(altitude_->value())});
}
void MissionController::rebuildRoute() {
  routeSyncedToOnboard_ = false;
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
bool MissionController::editWaypoint(bool manualCoordinates) {
  int row = waypoints_->currentRow();
  if (row < 0 || row >= mission_.waypoints_size())
    return false;
  auto *w = mission_.mutable_waypoints(row);
  QDialog d(window_);
  d.setWindowTitle("编辑三维航点");
  QFormLayout f(&d);
  QLabel coordinateHint("当前没有有效的飞机位置。请手动填写并核对 X、Y、Z；"
                        "初始坐标不是飞机当前位置。", &d);
  QCheckBox coordinateConfirmed("我已核对手动坐标", &d);
  if (manualCoordinates) {
    coordinateHint.setWordWrap(true);
    f.addRow(&coordinateHint);
    f.addRow(&coordinateConfirmed);
  }
  QDoubleSpinBox x, y, z, yaw, speed, dwell, pitch, zoom;
  x.setObjectName("waypointX");
  y.setObjectName("waypointY");
  z.setObjectName("waypointZ");
  for (auto *s : {&x, &y, &z}) {
    s->setRange(-10000, 10000);
    s->setDecimals(3);
  }
  x.setValue(w->position().x());
  y.setValue(w->position().y());
  z.setValue(w->position().z());
  yaw.setRange(-180, 180);
  yaw.setDecimals(2);
  yaw.setValue(w->yaw() * 180 / M_PI);
  speed.setRange(.1, 20);
  speed.setValue(w->speed());
  QComboBox mode;
  mode.addItems({"Pass_through", "Detect_point", "Land_point"});
  mode.setCurrentText(w->pointmode().empty()
                          ? "Pass_through" : QString::fromStdString(w->pointmode()));
  dwell.setRange(0, 3600);
  dwell.setDecimals(2);
  dwell.setValue(w->hold_s());
  pitch.setRange(-180, 180);
  pitch.setDecimals(2);
  pitch.setValue(w->inspection_pitch_deg());
  zoom.setRange(0, 100);
  zoom.setDecimals(2);
  zoom.setValue(w->has_inspection_zoom_ratio() ? w->inspection_zoom_ratio() : 1);
  f.addRow("X / m", &x);
  f.addRow("Y / m", &y);
  f.addRow("Z / m", &z);
  f.addRow("航向 / °", &yaw);
  f.addRow("全局巡航速度 / m/s", &speed);
  f.addRow("类型", &mode);
  f.addRow("停留 / s", &dwell);
  f.addRow("云台俯仰 / °", &pitch);
  f.addRow("相机变焦", &zoom);
  speed.setToolTip("控制器只有一个全局巡航速度；修改后同步应用到所有本地航点");
  mode.setToolTip("降落点只能设在最后；直通点停留为 0，检测点停留须大于 0");
  QDialogButtonBox b(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  f.addRow(&b);
  connect(&b, &QDialogButtonBox::accepted, &d, [&] {
    if (manualCoordinates && !coordinateConfirmed.isChecked()) {
      QMessageBox::warning(&d, "请核对坐标", "请填写 X、Y、Z 并确认手动坐标");
      return;
    }
    if (mode.currentText() == "Pass_through" && dwell.value() > 0) {
      QMessageBox::warning(&d, "航点属性无效", "直通点的停留时间须为 0 秒");
      return;
    }
    if (mode.currentText() == "Detect_point" && dwell.value() == 0) {
      QMessageBox::warning(&d, "航点属性无效", "检测点的停留时间须大于 0 秒");
      return;
    }
    if (mode.currentText() == "Land_point" && row != mission_.waypoints_size() - 1) {
      QMessageBox::warning(&d, "航点属性无效", "降落点只能是最后一个航点");
      return;
    }
    d.accept();
  });
  connect(&b, &QDialogButtonBox::rejected, &d, &QDialog::reject);
  const bool accepted = d.exec() == QDialog::Accepted;
  if (accepted) {
    put(w->mutable_position(),
        {float(x.value()), float(y.value()), float(z.value())});
    w->set_yaw(yaw.value() * M_PI / 180);
    for (auto &point : *mission_.mutable_waypoints())
      point.set_speed(speed.value());
    w->set_pointmode(mode.currentText().toStdString());
    w->set_hold_s(dwell.value());
    w->set_inspection_pitch_deg(pitch.value());
    w->set_inspection_zoom_ratio(zoom.value());
    rebuildRoute();
  }
  return accepted;
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
  double routeSpeed = -1;
  for (int i = 0; i < m.waypoints_size(); ++i) {
    auto &w = *m.mutable_waypoints(i);
    auto &a = w.position();
    if (w.pointmode().empty()) {
      w.set_pointmode("Pass_through");
      if (!w.has_inspection_zoom_ratio())
        w.set_inspection_zoom_ratio(1);
    }
    const auto &mode = w.pointmode();
    const double zoom = w.has_inspection_zoom_ratio()
                            ? w.inspection_zoom_ratio() : 1;
    if (!std::isfinite(a.x()) || !std::isfinite(a.y()) ||
        !std::isfinite(a.z()) || std::abs(a.x()) > 10000 ||
        std::abs(a.y()) > 10000 || std::abs(a.z()) > 10000 ||
        !std::isfinite(w.speed()) || w.speed() <= 0 || w.speed() > 20 ||
        !std::isfinite(w.yaw()) || std::abs(w.yaw()) > M_PI ||
        !std::isfinite(w.hold_s()) || w.hold_s() < 0 || w.hold_s() > 3600 ||
        !std::isfinite(w.inspection_pitch_deg()) ||
        std::abs(w.inspection_pitch_deg()) > 180 ||
        !std::isfinite(zoom) || zoom < 0 || zoom > 100 ||
        (mode != "Pass_through" && mode != "Detect_point" && mode != "Land_point") ||
        (mode == "Pass_through" && w.hold_s() != 0) ||
        (mode == "Detect_point" && w.hold_s() == 0) ||
        (mode == "Land_point" && i != m.waypoints_size() - 1) ||
        (routeSpeed >= 0 && std::abs(w.speed() - routeSpeed) > .001))
      return false;
    routeSpeed = w.speed();
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
  if ((accumulationDirty_ || colorAccumulationDirty_) &&
      !saveAccumulatedMap()) {
    log("旧累积地图保存失败，未切换到参考地图");
    return false;
  }
  mapPoints_ = points;
  referenceMap_ = true;
  voxels_.clear();
  colorVoxels_.clear();
  colorMapPoints_.clear();
  colorAccumulationDirty_ = false;
  accumulate_->setChecked(false);
  if (ordinaryMapButton_)
    ordinaryMapButton_->setChecked(true);
  mapDirty_ = colorMapDirty_ = dirty_ = true;
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
QString MissionController::accumulatedMapPath(bool colored) const {
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
  return directory + "/accumulated-" + id +
         (colored ? "-color.pcd" : ".pcd");
}
bool MissionController::saveAccumulatedMap() {
  auto saveOne = [this](bool colored, const QHash<QString, MissionPoint> &voxels,
                        bool &dirty, bool &logged) {
    if (!dirty)
      return true;
    const QString path = accumulatedMapPath(colored);
    if (voxels.isEmpty()) {
      const bool pcdRemoved = !QFile::exists(path) || QFile::remove(path);
      const bool metaRemoved = !QFile::exists(path + ".meta.json") ||
                               QFile::remove(path + ".meta.json");
      if (!pcdRemoved || !metaRemoved) {
        log("自动保存文件清理失败：" + path);
        return false;
      }
      dirty = false;
      return true;
    }
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
      log("无法创建累积点云目录：" + QFileInfo(path).absolutePath());
      return false;
    }
    const auto points = voxels.values().toVector();
    QString error;
    if (!CloudIO::writePCD(path, points, error)) {
      log("累积点云自动保存失败：" + error);
      return false;
    }
    const QByteArray data = QJsonDocument(QJsonObject{
        {"frame_id", frame_}, {"map_id", mapId_}, {"voxel_m", voxel_},
        {"mode", demo_ ? "demo" : "live"}, {"point_count", points.size()},
        {"source", colored ? "cloud_slam_rgb" : "cloud_slam"},
        {"unit", "m"}, {"format", "XYZRGB"}}).toJson();
    QSaveFile metadata(path + ".meta.json");
    if (!metadata.open(QIODevice::WriteOnly) ||
        metadata.write(data) != data.size() || !metadata.commit()) {
      log("累积点云已写入，但元数据自动保存失败：" + metadata.errorString());
      return false;
    }
    dirty = false;
    if (!logged) {
      log(QString("%1累积点云自动保存到 %2")
              .arg(colored ? "彩色" : "普通", path));
      logged = true;
    }
    return true;
  };
  if (referenceMap_)
    return true;
  const bool ordinarySaved =
      saveOne(false, voxels_, accumulationDirty_, autosavePathLogged_);
  const bool colorSaved = saveOne(true, colorVoxels_, colorAccumulationDirty_,
                                 colorAutosavePathLogged_);
  return ordinarySaved && colorSaved;
}
bool MissionController::restoreAccumulatedMap() {
  auto restoreOne = [this](bool colored, QHash<QString, MissionPoint> &voxels,
                           QVector<MissionPoint> &display, bool &dirty,
                           bool &logged) {
    const QString path = accumulatedMapPath(colored);
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
        object.value("mode").toString() != (demo_ ? "demo" : "live") ||
        (colored && object.value("source").toString() != "cloud_slam_rgb") ||
        (!colored && !object.value("source").toString().isEmpty() &&
         object.value("source").toString() != "cloud_slam")) {
      log("累积点云坐标系、地图 ID、体素、模式或来源不一致，未自动加载：" + path);
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
    voxels.clear();
    for (const auto &point : points) {
      const QString key = QString("%1/%2/%3")
                              .arg(qint64(std::floor(point.x / voxel_)))
                              .arg(qint64(std::floor(point.y / voxel_)))
                              .arg(qint64(std::floor(point.z / voxel_)));
      voxels.insert(key, point);
    }
    display = voxels.values().toVector();
    dirty = false;
    logged = true;
    log(QString("已自动加载%1累积点云 %2 点：%3")
            .arg(colored ? "彩色" : "普通")
            .arg(display.size()).arg(path));
    return true;
  };
  const bool ordinary = restoreOne(false, voxels_, mapPoints_,
                                    accumulationDirty_, autosavePathLogged_);
  const bool colored = restoreOne(true, colorVoxels_, colorMapPoints_,
                                   colorAccumulationDirty_, colorAutosavePathLogged_);
  if (ordinary || colored) {
    referenceMap_ = false;
    mapDirty_ = ordinary;
    colorMapDirty_ = colored;
    dirty_ = true;
    refreshMapInfo();
  }
  return ordinary || colored;
}
void MissionController::clearAccumulatedMap() {
  voxels_.clear();
  mapPoints_.clear();
  colorVoxels_.clear();
  colorMapPoints_.clear();
  referenceMap_ = false;
  accumulationDirty_ = colorAccumulationDirty_ = true;
  const bool removed = saveAccumulatedMap();
  autosavePathLogged_ = colorAutosavePathLogged_ = false;
  mapDirty_ = colorMapDirty_ = dirty_ = true;
  refreshMapInfo();
  updateScene();
  log(removed ? "累积地图已清空，本地自动保存文件已删除"
              : "累积地图已清空，但本地自动保存文件删除失败");
}
void MissionController::receiveColorCloud(const QVector<MissionPoint> &points,
                                          bool snapshot) {
  if (!accumulate_ || !accumulate_->isChecked() || referenceMap_)
    return;
  if (snapshot) {
    colorVoxels_.clear();
    colorDrawStep_ = 0;
  }
  for (const auto &p : points) {
    const QString key = QString("%1/%2/%3")
                            .arg(qint64(std::floor(p.x / voxel_)))
                            .arg(qint64(std::floor(p.y / voxel_)))
                            .arg(qint64(std::floor(p.z / voxel_)));
    if (colorVoxels_.size() < 400000 || colorVoxels_.contains(key))
      colorVoxels_[key] = p;
  }
  colorAccumulationDirty_ = true;
  ++colorDrawStep_;
  if (colorMapPoints_.isEmpty() || snapshot || colorDrawStep_ % 5 == 0) {
    colorMapPoints_ = colorVoxels_.values().toVector();
    colorMapDirty_ = dirty_ = true;
  }
  refreshMapInfo();
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
    refreshMediaButtons();
    return;
  }
  const QUrl url = QUrl::fromUserInput(videoUrl_.trimmed());
  if (player_->error() != QMediaPlayer::NoError ||
      player_->mediaStatus() == QMediaPlayer::InvalidMedia ||
      player_->mediaStatus() == QMediaPlayer::StalledMedia ||
      player_->mediaStatus() == QMediaPlayer::EndOfMedia)
    player_->setMedia(QMediaContent());
  if (player_->currentMedia().canonicalUrl() != url)
    player_->setMedia(url);
  if (player_->state() != QMediaPlayer::PlayingState)
    player_->play();
  refreshMediaButtons();
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
  const auto sizeParts = qEnvironmentVariable("THREEDG_SMOKE_SIZE").split('x');
  if (sizeParts.size() == 2 && sizeParts[0].toInt() >= 850 &&
      sizeParts[1].toInt() >= 650) {
    window_->showNormal();
    window_->setWindowState(Qt::WindowNoState);
    window_->resize(sizeParts[0].toInt(), sizeParts[1].toInt());
    QApplication::processEvents();
  }
  demoTimer_.stop();
  updateScene();
  window_->screen()->grabWindow(window_->winId()).save(p + "/3dg.png");
  window_->grab().save(p + "/window-widget.png");
  mapDetailsButton_->click();
  QApplication::processEvents();
  mapPopover_->grab().save(p + "/connection-popup.png");
  mapDetailsButton_->click();
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
  const QString smokeVideoUrl = videoUrl_;
  const bool smokeVideoUrlFromAgent = videoUrlFromAgent_;
  const bool smokeVideoRequested = localVideoRequested();
  if (smokeVideoRequested) {
    videoRetry_.stop();
    player_->stop();
    video_->hide();
  }
  videoUrl_.clear();
  refreshMediaButtons();
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
  videoUrl_ = smokeVideoUrl;
  videoUrlFromAgent_ = smokeVideoUrlFromAgent;
  if (smokeVideoRequested)
    showVideo();
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
  const int savedGridSizePercent = gridSizePercent_;
  gridSizePercent_ = 100;
  gridDirty_ = dirty_ = true;
  updateScene();
  const int gridCells = gridCenters_.size();
  auto *gridMesh = grid_->getChildrenNumber()
                       ? dynamic_cast<ccMesh *>(grid_->getChild(0)) : nullptr;
  // The demo's 316-cell hollow box exposes 664 faces. Hidden shared faces
  // must not be emitted, otherwise dense walls show dark voxel seams.
  const bool gridGeometry = gridCells == 316 && gridMesh &&
                            gridMesh->size() == 1328;
  const CCVector3 fullSizeCorner = gridGeometry
      ? *gridMesh->getAssociatedCloud()->getPoint(0) : CCVector3(0, 0, 0);
  gridSizePercent_ = 50;
  gridDirty_ = dirty_ = true;
  updateScene();
  auto *smallGridMesh = grid_->getChildrenNumber()
                            ? dynamic_cast<ccMesh *>(grid_->getChild(0)) : nullptr;
  const CCVector3 smallSizeCorner = smallGridMesh
      ? *smallGridMesh->getAssociatedCloud()->getPoint(0) : CCVector3(0, 0, 0);
  const bool gridSizeGeometry = smallGridMesh &&
      smallGridMesh->size() == unsigned(gridCells * 12) &&
      std::abs((smallSizeCorner.x - fullSizeCorner.x) - 0.1f) < 1e-4f;
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/ego-grid-half-size.png");
  gridSizePercent_ = savedGridSizePercent;
  gridDirty_ = dirty_ = true;
  updateScene();
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
  gridSnapshot->set_version(1);
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
  bool gridDeltaApplied = false;
  bool gridDeltaMismatchClears = false;
  if (!gridCells_.empty()) {
    const auto oldCell = *gridCells_.begin();
    const GridCell newCell{std::get<0>(oldCell) + 1000.0,
                           std::get<1>(oldCell), std::get<2>(oldCell)};
    auto patch = gridFrame;
    auto *delta = patch.mutable_grid();
    delta->clear_centers();
    delta->set_delta(true);
    delta->set_base_version(1);
    delta->set_version(2);
    auto *removed = delta->add_removed();
    removed->set_x(std::get<0>(oldCell));
    removed->set_y(std::get<1>(oldCell));
    removed->set_z(std::get<2>(oldCell));
    auto *added = delta->add_added();
    added->set_x(std::get<0>(newCell));
    added->set_y(std::get<1>(newCell));
    added->set_z(std::get<2>(newCell));
    sendGrid(patch);
    gridDeltaApplied = gridVersion_ == 2 && gridCells_.size() == size_t(gridCells) &&
                       !gridCells_.count(oldCell) && gridCells_.count(newCell);
    patch.mutable_grid()->set_version(3); // Base is still 1, but the client is at 2.
    sendGrid(patch);
    gridDeltaMismatchClears = gridCenters_.isEmpty() && gridVersion_ == 0 &&
                              !grid_->isVisible();
    sendGrid(gridFrame);
  }
  expireGrid();
  const bool gridExpired = !grid_->isVisible() && !grid_->getChildrenNumber();
  sendGrid(gridFrame);
  // Exercise the normal startup setting and incremental cloud path: the demo
  // itself uses full snapshots, which cannot prove that history is retained.
  const bool accumulationDefault = accumulate_->isChecked();
  const auto savedMap = mapPoints_, savedLive = livePoints_;
  const auto savedColorMap = colorMapPoints_;
  const auto savedVoxels = voxels_, savedColorVoxels = colorVoxels_;
  const bool savedReference = referenceMap_;
  const int savedDrawStep = drawStep_, savedColorDrawStep = colorDrawStep_;
  mapPoints_.clear();
  voxels_.clear();
  colorMapPoints_.clear();
  colorVoxels_.clear();
  referenceMap_ = false;
  drawStep_ = colorDrawStep_ = 0;
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
  auto displayedColorSize = [this]() -> unsigned {
    const auto *cloud = colorMap_->getChildrenNumber()
                            ? dynamic_cast<ccPointCloud *>(colorMap_->getChild(0))
                            : nullptr;
    return cloud && colorMap_->isEnabled() ? cloud->size() : 0;
  };
  QToolButton *colorToggle = nullptr;
  for (auto *tool : left_->findChildren<QToolButton *>())
    if (tool->accessibleName() == "彩色累积点云")
      colorToggle = tool;
  sendCloud({MissionPoint{0, 0, 0, 255, 10, 20}}, false);
  const bool accumulationFirstFrame = displayedMapSize() == 1;
  const bool colorFirstFrame = colorMapPoints_.size() == 1 &&
      colorVoxels_.value("0/0/0").r == 255 && displayedColorSize() == 0;
  for (int i = 1; i < 5; ++i)
    sendCloud({MissionPoint{float(i * voxel_ * 2), 0, 0}}, false);
  const bool accumulationHistory = livePoints_.size() == 1 &&
                                   mapPoints_.size() == 5 && displayedMapSize() == 5;
  const bool colorHistory = colorMapPoints_.size() == 5 &&
                            colorVoxels_.size() == 5;
  if (colorToggle)
    colorToggle->click();
  const bool colorLayerToggle = colorToggle && colorToggle->isChecked() &&
      displayedColorSize() == 5 && displayedMapSize() == 0 &&
      mapPoints_.size() == 5;
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId())
      .save(p + "/color-accumulated-visible.png");
  if (colorToggle)
    colorToggle->click();
  if (ordinaryMapButton_ && !ordinaryMapButton_->isChecked())
    ordinaryMapButton_->click();
  const bool colorHiddenKeepsData = displayedColorSize() == 0 &&
      colorMapPoints_.size() == 5 && displayedMapSize() == 5;
  const auto heightTestMap = mapPoints_;
  const auto heightTestColorMap = colorMapPoints_;
  mapPoints_ = {MissionPoint{0, 0, 0}, MissionPoint{0, 0, 1},
                MissionPoint{0, 0, 2}};
  colorMapPoints_ = mapPoints_;
  mapDirty_ = colorMapDirty_ = dirty_ = true;
  updateScene();
  heightFilterButton_->click();
  const bool heightPanelShown = heightFilterPanel_->isVisible();
  heightSlider_->setValue(500);
  updateScene();
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId())
      .save(p + "/height-filter.png");
  const bool heightVerticalRight =
      heightSlider_->orientation() == Qt::Vertical &&
      heightSlider_->height() > heightSlider_->width() * 2 &&
      heightSlider_->width() * 5 == heightFilterPanel_->width() * 4 &&
      heightFilterPanel_->x() > canvas_->width() / 2;
  const QPoint filterStart = heightFilterPanel_->pos();
  const QPoint pressGlobal = heightFilterTitle_->mapToGlobal(QPoint(4, 4));
  QMouseEvent filterPress(QEvent::MouseButtonPress, QPointF(4, 4),
                          QPointF(pressGlobal), Qt::LeftButton,
                          Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(heightFilterTitle_, &filterPress);
  QMouseEvent filterMove(QEvent::MouseMove, QPointF(4, 4),
                         QPointF(pressGlobal + QPoint(-40, 30)),
                         Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(heightFilterTitle_, &filterMove);
  QMouseEvent filterRelease(QEvent::MouseButtonRelease, QPointF(4, 4),
                            QPointF(pressGlobal + QPoint(-40, 30)),
                            Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(heightFilterTitle_, &filterRelease);
  const bool heightPanelDraggable =
      heightFilterPanel_->pos() == filterStart + QPoint(-40, 30);
  heightFilterPanel_->move(filterStart);
  heightFilterPlaced_ = false;
  const auto heightLayerSize = [](ccHObject *layer) -> unsigned {
    const auto *cloud = layer->getChildrenNumber()
                            ? dynamic_cast<ccPointCloud *>(layer->getChild(0))
                            : nullptr;
    return cloud ? cloud->size() : 0;
  };
  const bool heightFiltered = heightLayerSize(map_) == 2 &&
                              heightLayerSize(colorMap_) == 2 &&
                              mapPoints_.size() == 3 && colorMapPoints_.size() == 3;
  heightFilterButton_->click();
  const bool heightPanelHidden = !heightFilterPanel_->isVisible() &&
                                 heightLayerSize(map_) == 2;
  heightFilterEnabled_->setChecked(false);
  const bool heightFilterDisabled = heightLayerSize(map_) == 3 &&
                                    heightLayerSize(colorMap_) == 3;
  heightFilterEnabled_->setChecked(true);
  heightSlider_->setValue(heightSlider_->maximum());
  mapPoints_ = heightTestMap;
  colorMapPoints_ = heightTestColorMap;
  mapDirty_ = colorMapDirty_ = dirty_ = true;
  updateScene();
  const bool heightDisplayFilter = heightPanelShown && heightFiltered &&
                                   heightPanelHidden && heightFilterDisabled &&
                                   heightVerticalRight && heightPanelDraggable;
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
  const bool colorRestored = colorMapPoints_.size() == 5 &&
                             colorVoxels_.size() == 5 &&
                             colorVoxels_.value("0/0/0").r == 255;
  updateScene();
  const bool accumulationRestoreVisible = displayedMapSize() == 5;
  sendCloud({MissionPoint{0, 0, 0}}, true);
  const bool accumulationSnapshot = displayedMapSize() == 1 && voxels_.size() == 1;
  sendCloud({}, true);
  const bool accumulationEmptySnapshot = mapPoints_.isEmpty() &&
                                         !map_->getChildrenNumber();
  const bool colorEmptySnapshot = colorMapPoints_.isEmpty() &&
                                  !colorMap_->getChildrenNumber();
  const bool accumulationDeleted = saveAccumulatedMap() &&
      !QFile::exists(accumulatedMapPath()) &&
      !QFile::exists(accumulatedMapPath(true));
  for (int i = 0; i < 5; ++i)
    sendCloud({MissionPoint{float(i * voxel_ * 2), 0, 0}}, false);
  const bool accumulationResaved = saveAccumulatedMap();
  mapPoints_ = savedMap;
  colorMapPoints_ = savedColorMap;
  livePoints_ = savedLive;
  liveDirty_ = true;
  voxels_ = savedVoxels;
  colorVoxels_ = savedColorVoxels;
  referenceMap_ = savedReference;
  drawStep_ = savedDrawStep;
  colorDrawStep_ = savedColorDrawStep;
  accumulationDirty_ = colorAccumulationDirty_ = false; // Isolated smoke directory.
  mapDirty_ = colorMapDirty_ = dirty_ = true;
  refreshMapInfo();
  updateScene();
  setEditorCollapsed(false);
  QPushButton *addLocal = nullptr;
  for (auto *button : routeDialog_->findChildren<QPushButton *>()) {
    if (button->text() == "添加航点") {
      addLocal = button;
      break;
    }
  }
  const int countBeforeAdd = mission_.waypoints_size();
  const QVector3D aircraftPosition = v(lastVehicle_.position());
  bool waypointDefaultsToAircraft = false, waypointManualWithoutPose = false;
  bool waypointManualCanSave = false;
  if (addLocal) {
    QTimer::singleShot(0, this, [&] {
      if (auto *dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget())) {
        waypointDefaultsToAircraft = mission_.waypoints_size() == countBeforeAdd + 1 &&
            (v(mission_.waypoints(countBeforeAdd).position()) - aircraftPosition).length() < 1e-5f;
        dialog->reject();
      }
    });
    addLocal->click();
    waypointDefaultsToAircraft &= mission_.waypoints_size() == countBeforeAdd;
    auto invalidPose = lastVehicle_;
    invalidPose.set_pose_valid(false);
    updateVehicle(invalidPose);
    QTimer::singleShot(0, this, [&] {
      if (auto *dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget())) {
        bool hasManualHint = false, hasConfirmation = false;
        for (const auto *label : dialog->findChildren<QLabel *>())
          hasManualHint |= label->text().contains("请手动填写并核对");
        for (const auto *check : dialog->findChildren<QCheckBox *>())
          hasConfirmation |= check->text() == "我已核对手动坐标" && !check->isChecked();
        if (mission_.waypoints_size() == countBeforeAdd + 1) {
          const auto &position = mission_.waypoints(countBeforeAdd).position();
          waypointManualWithoutPose = hasManualHint && hasConfirmation &&
              position.x() == 0 && position.y() == 0 &&
              std::abs(position.z() - altitude_->value()) < 1e-5;
        }
        dialog->reject();
      }
    });
    addLocal->click();
    waypointManualWithoutPose &= mission_.waypoints_size() == countBeforeAdd;
    QTimer::singleShot(0, this, [&] {
      auto *dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget());
      if (!dialog)
        return;
      auto *x = dialog->findChild<QDoubleSpinBox *>("waypointX");
      auto *y = dialog->findChild<QDoubleSpinBox *>("waypointY");
      auto *z = dialog->findChild<QDoubleSpinBox *>("waypointZ");
      auto *buttons = dialog->findChild<QDialogButtonBox *>();
      QCheckBox *confirmed = nullptr;
      for (auto *check : dialog->findChildren<QCheckBox *>())
        if (check->text() == "我已核对手动坐标")
          confirmed = check;
      if (!x || !y || !z || !buttons || !confirmed) {
        dialog->reject();
        return;
      }
      x->setValue(2.5);
      y->setValue(-1);
      z->setValue(4);
      confirmed->setChecked(true);
      buttons->button(QDialogButtonBox::Ok)->click();
    });
    addLocal->click();
    if (mission_.waypoints_size() == countBeforeAdd + 1) {
      const auto &position = mission_.waypoints(countBeforeAdd).position();
      waypointManualCanSave = position.x() == 2.5 && position.y() == -1 &&
                              position.z() == 4;
      mission_.mutable_waypoints()->DeleteSubrange(countBeforeAdd, 1);
      rebuildRoute();
    }
    updateVehicle(savedVehicle);
  }
  mission::YamlDocument routeSample;
  routeSample.set_usage("GUI 自检样例");
  routeSample.set_content_json(
      R"({"ego":{"max_vel":0.8},"waypoints":[{"x":4.75,"y":0.22,"z":1.39,"yaw":0,"pointmode":"Pass_through","dwell_time":0,"inspection_pitch_deg":0,"inspection_zoom_ratio":1},{"x":6.75,"y":-0.53,"z":1.33,"yaw":45,"pointmode":"Detect_point","dwell_time":3,"inspection_pitch_deg":-30,"inspection_zoom_ratio":2}]})");
  importRemoteOnReceive_ = true;
  routeImportMissionRevision_ = mission_.revision();
  receiveRemoteRoute(routeSample);
  const bool waypointRefreshBecomesLocal =
      waypoints_->rowCount() == 2 &&
      waypoints_->item(0, 0)->text() == "本地编辑" &&
      !routeSyncedToOnboard_;
  importRemoteOnReceive_ = true;
  importRemoteAsUploaded_ = true;
  routeImportMissionRevision_ = mission_.revision();
  receiveRemoteRoute(routeSample);
  const bool waypointPopupUnified =
      waypoints_->rowCount() == 2 &&
      waypoints_->item(0, 0)->text() == "机载已同步" &&
      waypoints_->item(1, 7)->text() == "检测点" &&
      mission_.waypoints(1).inspection_pitch_deg() == -30 &&
      mission_.waypoints(1).inspection_zoom_ratio() == 2;
  const bool waypointColumnsComplete =
      waypoints_->columnCount() == 13 &&
      waypoints_->item(0, 6)->text() == "0.80" &&
      waypoints_->item(0, 11)->text() == "—" &&
      waypoints_->item(0, 12)->text() == "0.00" &&
      waypoints_->item(1, 11)->text() == "2.14" &&
      waypoints_->item(1, 12)->text() == "2.14";
  waypoints_->selectRow(1);
  QTimer::singleShot(0, this, [] {
    if (auto *dialog = qobject_cast<QDialog *>(QApplication::activeModalWidget()))
      dialog->reject();
  });
  editWaypoint();
  const bool waypointImportedEditable = mission_.waypoints_size() == 2 &&
                                        routeSyncedToOnboard_;
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
  openOdinStart();
  QApplication::processEvents();
  auto *odinMapBox = odinStartDialog_
                         ? odinStartDialog_->findChild<QGroupBox *>("odinMapBox")
                         : nullptr;
  const bool odinDialogInitial = odinStartDialog_ && odinModeGroup_ &&
                                 odinModeGroup_->checkedId() == 0 &&
                                 odinMapBox && !odinMapBox->isVisible() &&
                                 odinStartButton_ && !odinStartButton_->isEnabled();
  if (odinModeGroup_)
    odinModeGroup_->button(2)->setChecked(true);
  QApplication::processEvents();
  const bool odinStartDialogOk = odinDialogInitial && odinMapBox &&
                                  odinMapBox->isVisible() &&
                                  odinStartButton_ && !odinStartButton_->isEnabled();
  if (odinStartDialog_) {
    odinStartDialog_->grab().save(p + "/odin-start-dialog.png");
    odinStartDialog_->reject();
  }
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
  auto *workspaceMenu = window_->menuBar()->findChild<QMenu *>("missionWorkspaceMenu");
  QAction *firstVisibleMenu = nullptr;
  for (auto *action : window_->menuBar()->actions()) {
    if (action->isVisible()) {
      firstVisibleMenu = action;
      break;
    }
  }
  QStringList workspaceProblems;
  auto checkWorkspace = [&workspaceProblems](const QString &stage, bool ok) {
    if (!ok)
      workspaceProblems.append(stage);
  };
  checkWorkspace("menu", workspaceMenu &&
                             firstVisibleMenu == workspaceMenu->menuAction() &&
                             workspaceMenu->actions().size() == 3 &&
                             !window_->menuBar()->actions().contains(
                                 workspaceActions_.value(static_cast<int>(Workspace::Monitor))));
  checkWorkspace("menu_height", window_->menuBar()->height() <= 36 &&
                                    latest_->height() <= window_->menuBar()->height() &&
                                    latest_->y() >= 0);
  if (workspaceMenu) {
    workspaceMenu->popup(window_->menuBar()->mapToGlobal(
        QPoint(8, window_->menuBar()->height())));
    QApplication::processEvents();
    checkWorkspace("dropdown", workspaceMenu->isVisible() &&
                                   workspaceActions_.value(static_cast<int>(Workspace::Monitor))->isChecked());
    workspaceMenu->grab().save(p + "/workspace-menu.png");
    workspaceMenu->hide();
  }
  const unsigned dbCountBeforeProcessing =
      window_->dbRootObject()->getChildrenNumber();
  workspaceActions_[static_cast<int>(Workspace::Processing)]->trigger();
  QApplication::processEvents();
  checkWorkspace("processing", workspace_ == Workspace::Processing &&
                                   objectsCard_->isVisible() &&
                                   processingCard_->isVisible() &&
                                   !right_->isVisible());
  const auto &processingSelection = window_->getSelectedEntities();
  const auto *currentCloud = processingSelection.size() == 1
                                 ? dynamic_cast<ccPointCloud *>(processingSelection.front())
                                 : nullptr;
  checkWorkspace("current_cloud_default",
                 window_->dbRootObject()->getChildrenNumber() ==
                         dbCountBeforeProcessing + 1 &&
                     currentCloud &&
                     currentCloud->size() == unsigned(mapPoints_.size()));
  const unsigned dbCountBefore = window_->dbRootObject()->getChildrenNumber();
  snapshotMap();
  QApplication::processEvents();
  checkWorkspace("snapshot", window_->dbRootObject()->getChildrenNumber() ==
                                 dbCountBefore + 1);
  QListWidgetItem *snapshotItem = nullptr;
  for (int row = 0; row < processingObjects_->count(); ++row) {
    auto *item = processingObjects_->item(row);
    if (item->text().contains("地图快照"))
      snapshotItem = item;
  }
  checkWorkspace("snapshot_list", snapshotItem != nullptr);
  if (snapshotItem) {
    processingObjects_->setCurrentItem(snapshotItem,
                                       QItemSelectionModel::ClearAndSelect);
    QApplication::processEvents();
    auto *selected = window_->dbRootObject()->find(
        snapshotItem->data(Qt::UserRole).toUInt());
    checkWorkspace("selection", selected &&
                                    !window_->getSelectedEntities().empty() &&
                                    window_->getSelectedEntities().back() == selected);
    const unsigned beforeClone = window_->dbRootObject()->getChildrenNumber();
    runCloudCompareAction("actionClone");
    QApplication::processEvents();
    checkWorkspace("clone", window_->dbRootObject()->getChildrenNumber() ==
                                beforeClone + 1);
    refreshProcessingObjects();
    checkWorkspace("clone_list", processingObjects_->count() >= 2);
  }
  window_->grab().save(p + "/processing.png");
  workspaceActions_[static_cast<int>(Workspace::Comparison)]->trigger();
  QApplication::processEvents();
  const int comparisonSelectedCount = static_cast<int>(window_->getSelectedEntities().size());
  const bool comparisonCardVisible = comparisonOnly_->isVisible() &&
                                      !processingOnly_->isVisible();
  const bool comparisonRolesDiffer = referenceObject_->currentData().toUInt() !=
                                     targetObject_->currentData().toUInt();
  checkWorkspace("comparison", workspace_ == Workspace::Comparison &&
                                   objectsCard_->isVisible() &&
                                   processingCard_->isVisible() &&
                                   comparisonCardVisible &&
                                   comparisonRolesDiffer &&
                                   comparisonSelectedCount == 2);
  processingCard_->grab().save(p + "/comparison-card.png");
  window_->grab().save(p + "/comparison.png");
  bool poissonOk = true;
  if (qEnvironmentVariableIsSet("THREEDG_SMOKE_POISSON")) {
    setWorkspace(Workspace::Processing, false);
    auto *sample = new ccPointCloud("泊松测试球");
    constexpr double pi = 3.14159265358979323846;
    for (int latitude = 1; latitude < 24; ++latitude) {
      const double polar = pi * latitude / 24;
      for (int longitude = 0; longitude < 48; ++longitude) {
        const double azimuth = 2.0 * pi * longitude / 48;
        sample->addPoint(CCVector3(float(std::sin(polar) * std::cos(azimuth)),
                                   float(std::sin(polar) * std::sin(azimuth)),
                                   float(std::cos(polar))));
      }
    }
    sample->setDisplay(gl_);
    window_->addToDB(sample, false, true, false, true);
    window_->db()->selectEntities({sample});
    QApplication::processEvents();
    refreshProcessingObjects();
    quickReconstruct();
    const auto &poissonResult = window_->getSelectedEntities();
    auto *mesh = poissonResult.size() == 1
                     ? dynamic_cast<ccMesh *>(poissonResult.front()) : nullptr;
    poissonOk = mesh && mesh->size() > 0 && mesh->isVisible() &&
                !sample->isVisible() && sample->hasNormals();
    checkWorkspace("poisson_mesh", poissonOk);
    window_->grab().save(p + "/poisson-mesh.png");
  }
  workspaceActions_[static_cast<int>(Workspace::Monitor)]->trigger();
  QApplication::processEvents();
  checkWorkspace("return_monitor", workspace_ == Workspace::Monitor &&
                                       right_->isVisible() &&
                                       !objectsCard_->isVisible());
  const bool workspaceUi = workspaceProblems.isEmpty();
  bool missionRead = importMission(p + "/mission.3dg.json");
  bool videoOk = player_->state() == QMediaPlayer::PlayingState &&
                 player_->isVideoAvailable() && player_->position() > 0;
  bool localVideoControls = true;
  if (videoOk) {
    const bool savedDemo = demo_, savedReady = ready_;
    const auto savedMediaStatus = lastStatus_;
    lastStatus_.set_video_running(false);
    refreshMediaButtons();
    auto *button = commands_.value(mission::Command::VIDEO_START);
    const int pendingCount = pendingKinds_.size();
    localVideoControls = button->isEnabled() && button->text() == "停止播放" &&
                         mediaStates_[mission::Command::VIDEO_START]->text().contains("本机播放中") &&
                         statusCells_[8]->text() == "视频 本机播放中";
    mission::Envelope unrelatedVideoStop;
    unrelatedVideoStop.mutable_video()->set_running(false);
    receive(unrelatedVideoStop);
    localVideoControls &= localVideoPlaying() && videoUrl_ == smokeVideoUrl &&
                          button->text() == "停止播放";
    right_->grab().save(p + "/media-local-running.png");
    demo_ = ready_ = false;
    refreshMediaButtons();
    localVideoControls &= button->isEnabled() && button->text() == "停止播放";
    button->click();
    localVideoControls &= button->text() == "播放视频" &&
                          !localVideoRequested() && pendingKinds_.size() == pendingCount;
    button->click();
    localVideoControls &= localVideoRequested() && pendingKinds_.size() == pendingCount;
    demo_ = savedDemo;
    ready_ = savedReady;
    lastStatus_ = savedMediaStatus;
    refreshMediaButtons();
  }
  QSaveFile f(p + "/result.json");
  f.open(QIODevice::WriteOnly);
  f.write(QJsonDocument(QJsonObject{{"pcd_write", a},
                                    {"waypoint_defaults_to_aircraft", waypointDefaultsToAircraft},
                                    {"waypoint_manual_without_pose", waypointManualWithoutPose},
                                    {"waypoint_manual_can_save", waypointManualCanSave},
                                    {"waypoint_popup_unified", waypointPopupUnified},
                                    {"waypoint_columns_complete", waypointColumnsComplete},
                                    {"waypoint_refresh_becomes_local", waypointRefreshBecomesLocal},
                                    {"waypoint_imported_editable", waypointImportedEditable},
                                    {"media_controls", mediaControls},
                                    {"odin_start_dialog", odinStartDialogOk},
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
                                    {"color_first_frame", colorFirstFrame},
                                    {"color_history", colorHistory},
                                    {"color_layer_toggle", colorLayerToggle},
                                    {"color_hidden_keeps_data", colorHiddenKeepsData},
                                    {"height_display_filter", heightDisplayFilter},
                                    {"color_restored", colorRestored},
                                    {"color_empty_snapshot", colorEmptySnapshot},
                                    {"ego_grid_cells", gridCells},
                                    {"ego_grid_geometry", gridGeometry},
                                    {"ego_grid_size_geometry", gridSizeGeometry},
                                    {"ego_grid_toggle", gridToggle},
                                    {"ego_grid_empty_clears", gridCleared},
                                    {"ego_grid_frame_guard", gridFrameGuard},
                                    {"ego_grid_delta", gridDeltaApplied},
                                    {"ego_grid_delta_resync", gridDeltaMismatchClears},
                                    {"ego_grid_expiry", gridExpired},
                                    {"mission_write", b},
                                    {"pcd_read", c},
                                    {"pcd_loaded", pcdLoaded},
                                    {"pcd_loaded_points", mapPoints_.size()},
                                    {"pcd_scene_visible", pcdVisible},
                                    {"workspace_ui", workspaceUi},
                                    {"poisson_mesh", poissonOk},
                                    {"workspace_problems", workspaceProblems.join(",")},
                                    {"window_size", QString("%1x%2").arg(window_->width()).arg(window_->height())},
                                    {"canvas_size", QString("%1x%2").arg(canvas_->width()).arg(canvas_->height())},
                                    {"menu_height", window_->menuBar()->height()},
                                    {"comparison_selected_count", comparisonSelectedCount},
                                    {"comparison_card_visible", comparisonCardVisible},
                                    {"comparison_roles_differ", comparisonRolesDiffer},
                                    {"pcd_load_file", loadPath},
                                    {"mission_read", missionRead},
                                    {"video_playing", videoOk},
                                    {"local_video_controls", localVideoControls},
                                    {"video_empty_visible", emptyVideoVisible},
                                    {"points", pts.size()},
                                    {"trajectory_points", history_.size()},
                                    {"ego_points", egoPoints_.size()}})
              .toJson());
  f.commit();
  QApplication::exit(
      a && b && c && pcdLoaded && pcdVisible && workspaceUi && poissonOk && missionRead && mediaControls &&
              waypointDefaultsToAircraft && waypointManualWithoutPose &&
              waypointManualCanSave &&
              waypointPopupUnified && waypointColumnsComplete &&
              waypointRefreshBecomesLocal &&
              waypointImportedEditable &&
              aircraftGeometry && aircraftPose && aircraftInvalidHidden && aircraftRecovered &&
              aircraftNoGeometryRebuild &&
              accumulationDefault && accumulationFirstFrame && accumulationHistory &&
              accumulationExport && accumulationSaved && accumulationRestored &&
              accumulationRestoreVisible && accumulationDeleted && accumulationResaved &&
              accumulationSnapshot && accumulationEmptySnapshot &&
              colorFirstFrame && colorHistory && colorLayerToggle &&
              colorHiddenKeepsData && colorRestored && colorEmptySnapshot &&
              heightDisplayFilter &&
              gridGeometry && gridSizeGeometry && gridToggle && gridCleared &&
              gridFrameGuard && gridDeltaApplied && gridDeltaMismatchClears && gridExpired &&
              emptyVideoVisible &&
              (!qEnvironmentVariableIsSet("THREEDG_SMOKE_VIDEO") ||
               (videoOk && localVideoControls))
          ? 0
          : 1);
}
