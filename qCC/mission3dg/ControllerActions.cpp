#include "MissionController.h"
#include "YamlSettingsPanel.h"
#include "mainwindow.h"
#include <FileIOFilter.h>
#include <QMediaPlayer>
#include <QQuaternion>
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
} // namespace
void MissionController::setDemo(bool enabled) {
  client_.stop();
  demo_ = enabled;
  demoTimer_.stop();
  resetSession();
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
  accumulate_->setChecked(true);
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
  double t = ++demoStep_ * .025;
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
  if (demoStep_ % 3 == 1) {
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
  if (demoStep_ % 30 == 1)
    log(QString("模拟遥测正常 · 实际轨迹 %1 点 · EGO 规划 60 点")
            .arg(history_.size()),
        "模拟器");
  if (demoStep_ % 10 == 1) {
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
  d.setWindowTitle("任务机参数配置");
  d.resize(900, 650);
  QVBoxLayout outer(&d);
  QTabWidget tabs(&d);
  QWidget local(&tabs);
  QFormLayout f(&local);
  QLineEdit address(baseUrl_), frame(frame_), map(mapId_), video(videoUrl_);
  QDoubleSpinBox voxel, hz, clearance;
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
  QDialogButtonBox buttons(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
  buttons.button(QDialogButtonBox::Save)->setText("保存连接与显示");
  buttons.button(QDialogButtonBox::Cancel)->setText("关闭");
  auto *applyRemote = new QPushButton("发送当前本地配置到任务机");
  applyRemote->setEnabled(client_.ready() && !demo_);
  f.addRow(applyRemote);
  connect(applyRemote, &QPushButton::clicked, this,
          [this] { command(mission::Command::SET_CONFIG); });
  tabs.addTab(&local, "连接与显示");
  YamlSettingsPanel yaml(&client_, &tabs);
  tabs.addTab(&yaml, "任务机 YAML");
  outer.addWidget(&tabs, 1);
  outer.addWidget(&buttons);
  yaml.refresh();
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
  setDemo(false);
  baseUrl_ = address.text();
  frame_ = frame.text();
  mapId_ = map.text();
  videoUrl_ = video.text();
  voxel_ = voxel.value();
  cloudHz_ = hz.value();
  clearance_ = clearance.value();
  if (changed) {
    resetSession();
    mapPoints_.clear();
    voxels_.clear();
    mission_.clear_waypoints();
    rebuildRoute();
    mapDirty_ = dirty_ = true;
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
  log("本地参数已保存；重新连接后才能发送任务。坐标系变化会清除旧地图和航线。");
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
  expireGrid();
  lastVehicle_.Clear();
  vehicleAge_.invalidate();
  plannerAge_.invalidate();
  pendingKinds_.clear();
  voxels_.clear();
  if (!referenceMap_)
    mapPoints_.clear();
  mapDirty_ = dirty_ = true;
  refreshMapInfo();
  if (aircraft_)
    aircraft_->removeAllChildren();
  log("新会话：清除旧遥测、实时累积和轨迹；本地参考地图及航线保留，请核对地图 "
      "ID。");
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
      if (++drawStep_ % 5 == 0) {
        mapPoints_ = voxels_.values().toVector();
        mapDirty_ = true;
      }
    }
    dirty_ = true;
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
    dirty_ = true;
  } else if (e.has_status()) {
    statusAge_.restart();
    lastStatus_ = e.status();
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
  } else if (e.has_log())
    log(QString::fromStdString(e.log().text()),
        QString::fromStdString(e.log().source()));
  else if (e.has_result()) {
    log(QString("命令 %1：%2")
            .arg(QString::fromStdString(e.request_id()),
                 QString::fromStdString(e.result().detail())),
        "任务机");
    if (e.result().state() != mission::CommandResult::ACCEPTED)
      pendingKinds_.remove(QString::fromStdString(e.request_id()));
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
void MissionController::command(mission::Command::Kind k) {
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
  if (!id.isEmpty())
    pendingKinds_[id] = k;
}
void MissionController::addWaypoint(const QVector3D &p) {
  if (mission_.waypoints_size() >= 1000)
    return;
  auto *w = mission_.add_waypoints();
  put(w->mutable_position(), p);
  w->set_speed(1);
  rebuildRoute();
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
  waypoints_->setRowCount(mission_.waypoints_size());
  QVector<QVector3D> pts;
  for (int i = 0; i < mission_.waypoints_size(); ++i) {
    auto &w = mission_.waypoints(i);
    pts << v(w.position());
    QStringList cells{QString::number(i + 1),
                      QString::number(w.position().x(), 'f', 2),
                      QString::number(w.position().y(), 'f', 2),
                      QString::number(w.position().z(), 'f', 2)};
    for (int j = 0; j < 4; ++j)
      waypoints_->setItem(i, j, new QTableWidgetItem(cells[j]));
  }
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
  if (row < 0)
    return;
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
  setEditorCollapsed(false);
  QApplication::processEvents();
  window_->screen()->grabWindow(window_->winId()).save(p + "/editor.png");
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
      if (auto *tabs = d->findChild<QTabWidget *>()) {
        tabs->setCurrentIndex(1);
        QApplication::processEvents();
        d->grab().save(p + "/yaml-settings.png");
      }
      d->reject();
    }
  });
  settings();
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
      a && b && c && pcdLoaded && pcdVisible && missionRead &&
              gridGeometry && gridToggle && gridCleared && gridFrameGuard && gridExpired &&
              emptyVideoVisible &&
              (!qEnvironmentVariableIsSet("THREEDG_SMOKE_VIDEO") || videoOk)
          ? 0
          : 1);
}
