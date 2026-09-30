#include "FlightReplayDialog.h"
#ifdef ERROR
#undef ERROR
#endif
#include "mission.pb.h"

#include <QDialogButtonBox>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QListWidget>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSettings>
#include <QSlider>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <memory>

namespace {
const QRegularExpression kId("^(?:flight_[0-9]{6,}|[0-9]{8}_[0-9]{6}_[a-f0-9]{8})$");
constexpr qint64 kMaxArchiveBytes = 512LL * 1024 * 1024;
constexpr qint64 kMaxReplayFileBytes = 256LL * 1024 * 1024;
constexpr double kRadiansToDegrees = 57.29577951308232;

QByteArray replayStreamFromTar(const QByteArray &archive, QString &error) {
  qint64 offset = 0;
  while (offset + 512 <= archive.size()) {
    const QByteArray header = archive.mid(int(offset), 512);
    if (header.at(0) == '\0')
      break;
    const QByteArray name = header.left(100).split('\0').first();
    bool ok = false;
    const qint64 size = header.mid(124, 12).trimmed().toLongLong(&ok, 8);
    const qint64 dataAt = offset + 512;
    if (!ok || size < 0 || size > kMaxReplayFileBytes || dataAt + size > archive.size()) {
      error = "任务机返回的飞行记录归档无效或过大";
      return {};
    }
    if (name == "3dg_replay_stream.jsonl")
      return archive.mid(int(dataAt), int(size));
    offset = dataAt + ((size + 511) / 512) * 512;
  }
  error = "该飞行记录没有 3DG 回放流，请使用更新后的任务机记录器录制新任务";
  return {};
}

QJsonArray parseReplayStream(const QByteArray &stream, QString &error) {
  QJsonArray frames;
  bool headerSeen = false;
  int offset = 0;
  while (offset < stream.size()) {
    int end = stream.indexOf('\n', offset);
    if (end < 0)
      end = stream.size();
    const QByteArray line = stream.mid(offset, end - offset).trimmed();
    offset = end + 1;
    if (line.isEmpty())
      continue;
    if (line.size() > 4 * 1024 * 1024) {
      error = "回放流包含过大的单帧";
      return {};
    }
    QJsonParseError parseError;
    const auto doc = QJsonDocument::fromJson(line, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
      error = "3DG 回放流 JSON 无效";
      return {};
    }
    const QJsonObject row = doc.object();
    if (!headerSeen) {
      if (row.value("format").toString() != "3dg-replay-stream-v1" ||
          row.value("encoding").toString() != "qz-base64") {
        error = "3DG 回放流版本不受支持";
        return {};
      }
      headerSeen = true;
      continue;
    }
    const QString kind = row.value("kind").toString();
    if ((kind != "vehicle" && kind != "cloud" && kind != "grid") ||
        !row.value("mono_ns").isDouble() ||
        row.value("payload_qz_b64").toString().isEmpty() || frames.size() >= 50000) {
      error = "3DG 回放流帧格式无效或数量超限";
      return {};
    }
    frames.append(row);
  }
  if (!headerSeen)
    error = "3DG 回放流为空";
  return frames;
}

QByteArray replayPayload(const QJsonObject &row) {
  const QByteArray compressed = QByteArray::fromBase64(
      row.value("payload_qz_b64").toString().toLatin1());
  if (compressed.size() < 6)
    return {};
  const auto *prefix = reinterpret_cast<const unsigned char *>(compressed.constData());
  const quint32 plainSize = (quint32(prefix[0]) << 24) |
                            (quint32(prefix[1]) << 16) |
                            (quint32(prefix[2]) << 8) | quint32(prefix[3]);
  if (plainSize == 0 || plainSize > 2 * 1024 * 1024)
    return {};
  const QByteArray payload = qUncompress(compressed);
  return payload.size() == int(plainSize) ? payload : QByteArray();
}

QString recordStartTime(const QJsonObject &row) {
  // The recorder can start before the task machine's wall clock is set.
  const QJsonValue clockValid = row.value("wall_clock_valid");
  if (clockValid.isBool() && !clockValid.toBool())
    return {};
  const QJsonValue timestamp = row.value("started_ts_ns");
  if (!timestamp.isDouble())
    return {};
  const double nanoseconds = timestamp.toDouble();
  if (!std::isfinite(nanoseconds) || nanoseconds < 1704067200e9 ||
      nanoseconds >= 253402300800e9)
    return {};
  return QDateTime::fromMSecsSinceEpoch(qint64(nanoseconds / 1e6))
      .toLocalTime().toString("yyyy-MM-dd HH:mm:ss");
}

bool coordinate(const QJsonObject &row, QVector3D &point) {
  const auto x = row.value("x"), y = row.value("y"), z = row.value("z");
  if (!x.isDouble() || !y.isDouble() || !z.isDouble())
    return false;
  if (!std::isfinite(x.toDouble()) || !std::isfinite(y.toDouble()) ||
      !std::isfinite(z.toDouble()) || std::abs(x.toDouble()) > 1e6 ||
      std::abs(y.toDouble()) > 1e6 || std::abs(z.toDouble()) > 1e6)
    return false;
  point = QVector3D(float(x.toDouble()), float(y.toDouble()), float(z.toDouble()));
  return true;
}
}

FlightReplayDialog::FlightReplayDialog(const QUrl &taskSocket, Render render,
                                       QWidget *parent)
    : QDialog(parent), render_(std::move(render)) {
  setWindowTitle("飞行记录回放 · 历史数据");
  resize(470, 480);
  network_.setProxy(QNetworkProxy(QNetworkProxy::NoProxy));
  origin_.setScheme("http");
  origin_.setHost(taskSocket.host());
  origin_.setPort(8000);
  QSettings settings("3DG", "Mission");
  downloadDir_ = settings.value(
      "replayDownloadDir",
      QStandardPaths::writableLocation(QStandardPaths::DownloadLocation)).toString();
  if (downloadDir_.isEmpty())
    downloadDir_ = QDir::homePath();

  auto *layout = new QVBoxLayout(this);
  auto *intro = new QLabel("选择任务机新飞行记录，或打开已下载的 3DG 回放数据。"
                           "历史 HUD、Odin 点云和 EGO 栅格随时间轴显示；栅格使用完整帧与增量帧。", this);
  intro->setWordWrap(true);
  layout->addWidget(intro);
  records_ = new QListWidget(this);
  layout->addWidget(records_, 1);
  auto *row = new QHBoxLayout;
  auto *refresh = new QPushButton("刷新记录", this);
  auto *load = new QPushButton("加载选中记录", this);
  row->addWidget(refresh);
  row->addWidget(load);
  layout->addLayout(row);
  auto *localRow = new QHBoxLayout;
  auto *download = new QPushButton("下载选中回放数据…", this);
  auto *openLocal = new QPushButton("打开本地回放数据…", this);
  localRow->addWidget(download);
  localRow->addWidget(openLocal);
  layout->addLayout(localRow);
  auto *directoryRow = new QHBoxLayout;
  downloadDirLabel_ = new QLabel("下载目录：" + QDir::toNativeSeparators(downloadDir_), this);
  downloadDirLabel_->setWordWrap(true);
  directoryRow->addWidget(downloadDirLabel_, 1);
  auto *chooseDirectory = new QPushButton("更改目录…", this);
  directoryRow->addWidget(chooseDirectory);
  layout->addLayout(directoryRow);
  status_ = new QLabel(this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setEnabled(false);
  layout->addWidget(slider_);
  auto *controls = new QHBoxLayout;
  play_ = new QPushButton("播放", this);
  play_->setEnabled(false);
  clock_ = new QLabel("00:00.0 / 00:00.0", this);
  controls->addWidget(play_);
  controls->addWidget(clock_, 1);
  layout->addLayout(controls);
  event_ = new QLabel("事件：—", this);
  event_->setWordWrap(true);
  layout->addWidget(event_);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(refresh, &QPushButton::clicked, this, [this] { listRecords(); });
  connect(load, &QPushButton::clicked, this, [this] { loadRecord(); });
  connect(download, &QPushButton::clicked, this, [this] { downloadReplay(); });
  connect(chooseDirectory, &QPushButton::clicked, this,
          [this] { chooseDownloadDir(); });
  connect(openLocal, &QPushButton::clicked, this, [this] { openLocalReplay(); });
  connect(records_, &QListWidget::itemDoubleClicked, this, [this] { loadRecord(); });
  connect(slider_, &QSlider::valueChanged, this, [this](int value) { seek(value); });
  connect(play_, &QPushButton::clicked, this, [this] {
    if (timer_.isActive()) {
      timer_.stop();
      play_->setText("播放");
    } else {
      if (slider_->value() >= durationMs_)
        slider_->setValue(0);
      timer_.start();
      play_->setText("暂停");
    }
  });
  timer_.setInterval(50);
  connect(&timer_, &QTimer::timeout, this, [this] {
    slider_->setValue(qMin(durationMs_, slider_->value() + 50));
    if (slider_->value() >= durationMs_) {
      timer_.stop();
      play_->setText("播放");
    }
  });
  if (origin_.host().isEmpty())
    status_->setText("请先在“设置 → 3DG 配置”填写任务机 WebSocket 地址。");
  else
    listRecords();
}

void FlightReplayDialog::request(
    const QString &path,
    std::function<void(const QJsonDocument &, const QString &)> done) {
  if (pending_) {
    auto *previous = pending_.data();
    pending_.clear();
    previous->abort();
  }
  QUrl url = origin_;
  const int query = path.indexOf('?');
  url.setPath(query < 0 ? path : path.left(query));
  if (query >= 0)
    url.setQuery(path.mid(query + 1));
  auto *reply = network_.get(QNetworkRequest(url));
  pending_ = reply;
  QTimer::singleShot(20000, reply, [reply] {
    if (reply->isRunning())
      reply->abort();
  });
  connect(reply, &QNetworkReply::finished, this, [this, reply, done = std::move(done)] {
    if (pending_ != reply) {
      reply->deleteLater();
      return;
    }
    pending_.clear();
    const QByteArray body = reply->readAll();
    QJsonParseError parseError;
    const auto doc = QJsonDocument::fromJson(body, &parseError);
    QString error;
    const int http = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    if (reply->error() != QNetworkReply::NoError || http != 200) {
      error = QJsonDocument::fromJson(body).object().value("detail").toString();
      if (error.isEmpty())
        error = reply->error() == QNetworkReply::NoError
                    ? QString("HTTP %1").arg(http) : reply->errorString();
    } else if (parseError.error != QJsonParseError::NoError) {
      error = "任务机返回的 JSON 无效";
    }
    reply->deleteLater();
    done(doc, error);
  });
}

void FlightReplayDialog::requestArchive(
    const QString &path,
    std::function<void(const QByteArray &, const QString &)> done) {
  if (pending_) {
    auto *previous = pending_.data();
    pending_.clear();
    previous->abort();
  }
  QUrl url = origin_;
  url.setPath(path);
  auto *reply = network_.get(QNetworkRequest(url));
  pending_ = reply;
  auto body = std::make_shared<QByteArray>();
  connect(reply, &QIODevice::readyRead, this, [reply, body] {
    body->append(reply->readAll());
    if (body->size() > kMaxArchiveBytes)
      reply->abort();
  });
  QTimer::singleShot(120000, reply, [reply] {
    if (reply->isRunning())
      reply->abort();
  });
  connect(reply, &QNetworkReply::finished, this,
          [this, reply, body, done = std::move(done)] {
    if (pending_ != reply) {
      reply->deleteLater();
      return;
    }
    pending_.clear();
    body->append(reply->readAll());
    QString error;
    const int http = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    if (body->size() > kMaxArchiveBytes)
      error = "飞行记录归档超过 512 MB";
    else if (reply->error() != QNetworkReply::NoError || http != 200) {
      error = QJsonDocument::fromJson(*body).object().value("detail").toString();
      if (error.isEmpty())
        error = reply->error() == QNetworkReply::NoError
                    ? QString("HTTP %1").arg(http) : reply->errorString();
    }
    reply->deleteLater();
    done(error.isEmpty() ? *body : QByteArray(), error);
  });
}

void FlightReplayDialog::listRecords() {
  timer_.stop();
  play_->setText("播放");
  status_->setText("正在读取飞行记录…");
  request("/api/flight-records", [this](const QJsonDocument &doc, const QString &error) {
    records_->clear();
    if (!error.isEmpty() || !doc.isArray()) {
      status_->setText("读取失败：" + (error.isEmpty() ? "数据格式无效" : error));
      return;
    }
    for (const auto &entry : doc.array()) {
      const auto row = entry.toObject();
      const QString id = row.value("session_id").toString();
      if (!kId.match(id).hasMatch())
        continue;
      const QString state = row.value("status").toString();
      const QString started = recordStartTime(row);
      const QString recordedAt = started.isEmpty()
                                     ? QStringLiteral("记录时间未校准")
                                     : QStringLiteral("记录于 ") + started;
      const bool replayReady = row.value("replay_stream_format").toString() ==
                               "3dg-replay-stream-v1";
      if (!replayReady)
        continue;
      auto *item = new QListWidgetItem(
          id + " · " + recordedAt +
              (state == "recording" ? " · 记录中" : ""), records_);
      item->setData(Qt::UserRole, id);
      item->setToolTip(QString("%1 · %2 秒 · %3")
                           .arg(state).arg(row.value("duration_sec").toDouble(), 0, 'f', 1)
                           .arg(started.isEmpty() ? "任务机时钟未校准，无法确定记录日期时间"
                                                  : "开始时间按本机时区显示"));
      if (!row.value("database_ready").toBool() || state == "recording")
        item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
    status_->setText(records_->count() ? "请选择一条记录。" : "任务机还没有新格式的 3DG 飞行记录。");
  });
}

void FlightReplayDialog::loadRecord() {
  const auto *item = records_->currentItem();
  if (!item || !(item->flags() & Qt::ItemIsEnabled))
    return;
  const QString id = item->data(Qt::UserRole).toString();
  if (!kId.match(id).hasMatch())
    return;
  timer_.stop();
  play_->setText("播放");
  slider_->setEnabled(false);
  play_->setEnabled(false);
  status_->setText("正在加载 " + id + "…");
  loadRemoteBundle(id, {});
}

void FlightReplayDialog::downloadReplay() {
  const auto *item = records_->currentItem();
  if (!item || !(item->flags() & Qt::ItemIsEnabled))
    return;
  const QString id = item->data(Qt::UserRole).toString();
  if (!kId.match(id).hasMatch())
    return;
  const QFileInfo directory(downloadDir_);
  if (!directory.isDir() || !directory.isWritable()) {
    status_->setText("下载目录不可写，请先选择其他目录：" + downloadDir_);
    return;
  }
  QString path = QDir(downloadDir_).filePath(id + ".3dg-replay.json");
  for (int copy = 2; QFileInfo::exists(path); ++copy) {
    if (copy > 1000) {
      status_->setText("下载目录中同一记录的副本过多，请更改目录。");
      return;
    }
    path = QDir(downloadDir_).filePath(
        id + "-" + QString::number(copy) + ".3dg-replay.json");
  }
  status_->setText("正在下载 " + id + " 的完整 3DG 回放数据…");
  loadRemoteBundle(id, path);
}

void FlightReplayDialog::loadRemoteBundle(const QString &id,
                                          const QString &savePath) {
  request("/api/flight-records/" + id + "/replay?max_points=4000",
          [this, id, savePath](const QJsonDocument &doc, const QString &error) {
    if (!error.isEmpty() || !doc.isObject()) {
      status_->setText("读取回放失败：" + (error.isEmpty() ? "数据格式无效" : error));
      return;
    }
    status_->setText("正在读取任务机记录中的点云、HUD 和栅格增量…");
    requestArchive("/api/flight-records/" + id + "/download",
                   [this, id, savePath, summary = doc.object()]
                   (const QByteArray &archive, const QString &archiveError) {
      if (!archiveError.isEmpty()) {
        status_->setText("读取回放归档失败：" + archiveError);
        return;
      }
      QString parseError;
      const QByteArray stream = replayStreamFromTar(archive, parseError);
      if (!parseError.isEmpty()) {
        status_->setText(parseError);
        return;
      }
      const QJsonArray frames = parseReplayStream(stream, parseError);
      if (!parseError.isEmpty()) {
        status_->setText(parseError);
        return;
      }
      QJsonObject data;
      for (const QString &key : {"manifest", "telemetry", "setpoints", "events"})
        data.insert(key, summary.value(key));
      data.insert("recorded_frames", frames);
      if (!savePath.isEmpty()) {
        const QByteArray bytes = QJsonDocument(data).toJson(QJsonDocument::Compact);
        if (bytes.size() > kMaxReplayFileBytes) {
          status_->setText("3DG 回放文件超过 256 MB，未保存");
          return;
        }
        QSaveFile file(savePath);
        if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() ||
            !file.commit()) {
          status_->setText("本地文件保存失败：" + savePath);
          return;
        }
      }
      applyReplayData(data, id);
      if (slider_->isEnabled() && !savePath.isEmpty())
        status_->setText(status_->text() + "\n已保存到：" + savePath);
    });
  });
}

void FlightReplayDialog::chooseDownloadDir() {
  const QString path = QFileDialog::getExistingDirectory(
      this, "选择 3DG 回放数据下载目录", downloadDir_);
  if (path.isEmpty())
    return;
  const QFileInfo directory(path);
  if (!directory.isDir() || !directory.isWritable()) {
    status_->setText("所选目录不可写：" + path);
    return;
  }
  downloadDir_ = directory.absoluteFilePath();
  QSettings("3DG", "Mission").setValue("replayDownloadDir", downloadDir_);
  downloadDirLabel_->setText("下载目录：" + QDir::toNativeSeparators(downloadDir_));
}

void FlightReplayDialog::openLocalReplay() {
  const QString path = QFileDialog::getOpenFileName(
      this, "打开本地 3DG 回放数据", downloadDir_,
      "3DG 回放数据 (*.3dg-replay.json)");
  if (path.isEmpty())
    return;
  if (pending_) {
    auto *previous = pending_.data();
    pending_.clear();
    previous->abort();
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly) || file.size() > kMaxReplayFileBytes) {
    status_->setText("无法读取回放文件，或文件超过 256 MB：" + path);
    return;
  }
  QJsonParseError parseError;
  const auto doc = QJsonDocument::fromJson(file.readAll(), &parseError);
  if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
    status_->setText("回放文件 JSON 无效：" + path);
    return;
  }
  applyReplayData(doc.object(), QFileInfo(path).fileName());
}

void FlightReplayDialog::applyReplayData(const QJsonObject &data,
                                          const QString &id) {
    const auto telemetry = data.value("telemetry").toArray();
    if (telemetry.isEmpty()) {
      status_->setText("记录没有可回放的遥测位置。");
      return;
    }
    if (!data.value("recorded_frames").isArray()) {
      status_->setText("该文件没有新格式的 3DG 回放流。");
      return;
    }
    if (telemetry.size() > 10000 || data.value("setpoints").toArray().size() > 10000 ||
        data.value("events").toArray().size() > 3000 ||
        data.value("recorded_frames").toArray().size() > 50000) {
      status_->setText("回放数据超出 3DG 的数量限制。");
      return;
    }
    monotonic_ = telemetry.first().toObject().value("mono_ns").toDouble() > 0;
    startNs_ = timeNs(telemetry.first().toObject());
    const double endNs = timeNs(telemetry.last().toObject());
    if (startNs_ <= 0 || endNs < startNs_) {
      status_->setText("记录时间戳无效。");
      return;
    }
    telemetry_ = telemetry;
    setpoints_ = data.value("setpoints").toArray();
    events_ = data.value("events").toArray();
    vehicles_ = {}; clouds_ = {}; grids_ = {};
    for (const auto &entry : data.value("recorded_frames").toArray()) {
      const auto row = entry.toObject();
      const QString kind = row.value("kind").toString();
      if (kind == "vehicle")
        vehicles_.append(row);
      else if (kind == "cloud")
        clouds_.append(row);
      else if (kind == "grid")
        grids_.append(row);
    }
    cachedCloud_.clear(); cachedGrid_.clear(); gridCells_.clear();
    cloudIndex_ = gridIndex_ = -1;
    gridVersion_ = 0;
    gridTimeNs_ = cloudTimeNs_ = gridResolution_ = 0;
    gridFrame_.clear();
    recordId_ = id;
    durationMs_ = int(qBound(1.0, (endNs - startNs_) / 1e6, 24.0 * 3600 * 1000));
    slider_->setRange(0, durationMs_);
    slider_->setEnabled(true);
    play_->setEnabled(true);
    slider_->setValue(0);
    seek(0);
    status_->setText(QString("回放：%1 · 遥测 %2 点 · HUD %3 帧 · 点云 %4 帧 · 栅格增量 %5 帧 · 历史数据")
                         .arg(id).arg(telemetry_.size()).arg(vehicles_.size())
                         .arg(clouds_.size()).arg(grids_.size()));
}

double FlightReplayDialog::timeNs(const QJsonObject &row) const {
  return row.value(monotonic_ ? "mono_ns" : "ts_ns").toDouble();
}

void FlightReplayDialog::seek(int milliseconds) {
  if (telemetry_.isEmpty() || startNs_ <= 0)
    return;
  const double now = startNs_ + double(milliseconds) * 1e6;
  FlightReplayFrame frame;
  QJsonObject latestTelemetry;
  for (const auto &entry : telemetry_) {
    const auto row = entry.toObject();
    if (timeNs(row) > now)
      break;
    QVector3D point;
    if (coordinate(row, point)) {
      frame.position = point;
      frame.poseValid = true;
      frame.trail.append(point);
      frame.yaw = row.value("yaw").toDouble();
      latestTelemetry = row;
    }
  }
  if (!latestTelemetry.isEmpty()) {
    frame.hudPoseValid = true;
    frame.hudPosition = frame.position;
    frame.yawDeg = frame.yaw * kRadiansToDegrees;
    const double vx = latestTelemetry.value("vx").toDouble();
    const double vy = latestTelemetry.value("vy").toDouble();
    const double vz = latestTelemetry.value("vz").toDouble();
    frame.hudVelocityValid = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz);
    frame.speedMps = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double battery = latestTelemetry.value("battery_pct").toDouble(-1);
    frame.hudBatteryValid = battery >= 0 && battery <= 1;
    frame.batteryPercent = battery * 100.0;
    const auto connected = latestTelemetry.value("connected");
    const auto armed = latestTelemetry.value("armed");
    const bool fcuConnected = connected.isBool() ? connected.toBool()
                                                      : connected.toInt(-1) == 1;
    frame.fcuStateValid = fcuConnected &&
        (armed.isBool() || armed.toInt(-1) == 0 || armed.toInt(-1) == 1);
    frame.armed = armed.isBool() ? armed.toBool() : armed.toInt() == 1;
    frame.flightMode = latestTelemetry.value("mode").toString().trimmed().left(32);
  }
  QJsonObject latestVehicle;
  for (const auto &entry : vehicles_) {
    const auto row = entry.toObject();
    if (timeNs(row) > now)
      break;
    latestVehicle = row;
  }
  if (!latestVehicle.isEmpty()) {
    const QByteArray payload = replayPayload(latestVehicle);
    mission::VehicleState vehicle;
    if (!payload.isEmpty() && vehicle.ParseFromArray(payload.constData(), payload.size())) {
      if (vehicle.pose_valid()) {
        const auto &p = vehicle.position();
        const auto &q = vehicle.orientation();
        const double norm = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
        if (std::isfinite(norm) && std::abs(norm - 1.0) <= .02 &&
            std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z())) {
          const double roll = std::atan2(2 * (q.w()*q.x() + q.y()*q.z()),
                                         1 - 2 * (q.x()*q.x() + q.y()*q.y()));
          const double pitch = std::asin(qBound(-1.0, 2 * (q.w()*q.y() - q.z()*q.x()), 1.0));
          const double yaw = std::atan2(2 * (q.w()*q.z() + q.x()*q.y()),
                                        1 - 2 * (q.y()*q.y() + q.z()*q.z()));
          frame.hudPoseValid = frame.poseValid = true;
          frame.hudPosition = frame.position = QVector3D(float(p.x()), float(p.y()), float(p.z()));
          frame.rollDeg = roll * kRadiansToDegrees;
          frame.pitchDeg = pitch * kRadiansToDegrees;
          frame.yawDeg = yaw * kRadiansToDegrees;
          frame.yaw = yaw;
        }
      }
      if (vehicle.velocity_valid()) {
        const auto &v = vehicle.velocity();
        frame.hudVelocityValid = std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        frame.speedMps = std::sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
      }
      frame.hudBatteryValid = vehicle.battery_valid() &&
                              std::isfinite(vehicle.battery_percent()) &&
                              vehicle.battery_percent() >= 0 && vehicle.battery_percent() <= 100;
      frame.batteryPercent = vehicle.battery_percent();
      if (vehicle.fcu_state_valid()) {
        frame.fcuStateValid = true;
        frame.armed = vehicle.armed();
        frame.flightMode = QString::fromStdString(vehicle.flight_mode());
      }
    }
  }
  QString setpointSource;
  for (const QString &candidate : {"astra_position", "astra_raw", "planner_position"}) {
    for (const auto &entry : setpoints_)
      if (entry.toObject().value("source").toString() == candidate &&
          (candidate != "astra_raw" ||
           (entry.toObject().value("type_mask").toInt() & 7) == 0)) {
        setpointSource = candidate;
        break;
      }
    if (!setpointSource.isEmpty())
      break;
  }
  for (const auto &entry : setpoints_) {
    const auto row = entry.toObject();
    if (timeNs(row) > now)
      break;
    if (row.value("source").toString() != setpointSource)
      continue;
    if (setpointSource == "astra_raw" &&
        (row.value("type_mask").toInt() & 7) != 0)
      continue;
    QVector3D point;
    if (coordinate(row, point))
      frame.setpoints.append(point);
  }
  int cloudTarget = -1;
  for (int i = 0; i < clouds_.size() && timeNs(clouds_.at(i).toObject()) <= now; ++i)
    cloudTarget = i;
  if (cloudTarget != cloudIndex_) {
    cachedCloud_.clear(); cloudTimeNs_ = 0;
    if (cloudTarget >= 0) {
      const QByteArray payload = replayPayload(clouds_.at(cloudTarget).toObject());
      mission::CloudFrame cloud;
      if (!payload.isEmpty() && cloud.ParseFromArray(payload.constData(), payload.size()) &&
          cloud.point_count() <= 5000 && cloud.point_data().size() == cloud.point_count() * 16) {
        QString decodeError;
        cachedCloud_ = CloudIO::decode(QByteArray::fromStdString(cloud.point_data()), decodeError);
        if (decodeError.isEmpty())
          cloudTimeNs_ = timeNs(clouds_.at(cloudTarget).toObject());
        else
          cachedCloud_.clear();
      }
    }
    cloudIndex_ = cloudTarget;
  }
  frame.cloud = cachedCloud_;
  frame.cloudTimeNs = cloudTimeNs_;

  int gridTarget = -1;
  for (int i = 0; i < grids_.size() && timeNs(grids_.at(i).toObject()) <= now; ++i)
    gridTarget = i;
  if (gridTarget < gridIndex_) {
    gridCells_.clear(); cachedGrid_.clear(); gridVersion_ = 0;
    gridResolution_ = gridTimeNs_ = 0; gridFrame_.clear(); gridIndex_ = -1;
  }
  if (gridTarget != gridIndex_) {
    for (int i = gridIndex_ + 1; i <= gridTarget; ++i) {
      const auto row = grids_.at(i).toObject();
      const QByteArray payload = replayPayload(row);
      mission::VoxelGrid grid;
      if (payload.isEmpty() || !grid.ParseFromArray(payload.constData(), payload.size()) ||
          grid.frame_id().empty() || !std::isfinite(grid.resolution_m()) ||
          grid.resolution_m() < .02 || grid.resolution_m() > 5 ||
          grid.centers_size() > 50000 || grid.added_size() > 50000 ||
          grid.removed_size() > 50000 || grid.version() == 0) {
        gridCells_.clear(); gridVersion_ = 0; gridTimeNs_ = 0;
        continue;
      }
      if (grid.delta()) {
        if (gridVersion_ != grid.base_version() || gridFrame_ != QString::fromStdString(grid.frame_id()) ||
            gridResolution_ != grid.resolution_m()) {
          gridCells_.clear(); gridVersion_ = 0; gridTimeNs_ = 0;
          continue;
        }
        auto next = gridCells_;
        bool valid = true;
        for (const auto &p : grid.removed())
          valid = (next.erase({p.x(), p.y(), p.z()}) == 1) && valid;
        for (const auto &p : grid.added())
          valid = next.insert({p.x(), p.y(), p.z()}).second && valid;
        if (!valid || next.size() > 50000) {
          gridCells_.clear(); gridVersion_ = 0; gridTimeNs_ = 0;
          continue;
        }
        gridCells_.swap(next);
      } else {
        gridCells_.clear();
        for (const auto &p : grid.centers())
          gridCells_.insert({p.x(), p.y(), p.z()});
        gridFrame_ = QString::fromStdString(grid.frame_id());
        gridResolution_ = grid.resolution_m();
      }
      gridVersion_ = grid.version();
      gridTimeNs_ = timeNs(row);
    }
    gridIndex_ = gridTarget;
    cachedGrid_.clear();
    cachedGrid_.reserve(int(gridCells_.size()));
    for (const auto &cell : gridCells_)
      cachedGrid_.append(QVector3D(float(std::get<0>(cell)), float(std::get<1>(cell)),
                                   float(std::get<2>(cell))));
  }
  frame.voxels = cachedGrid_;
  frame.gridTimeNs = gridTimeNs_;
  frame.resolution = gridResolution_;
  frame.gridFrame = gridFrame_;
  QString latestEvent = "—";
  for (const auto &entry : events_) {
    const auto row = entry.toObject();
    const double age = now - timeNs(row);
    if (age < 0)
      break;
    if (age <= 2e9)
      latestEvent = row.value("category").toString() + " / " + row.value("name").toString();
  }
  event_->setText("事件：" + latestEvent +
                  (gridIndex_ >= 0 && gridVersion_ == 0 ? " · 栅格增量断链，等待完整帧" : ""));
  const auto format = [](int ms) {
    return QString("%1:%2.%3")
        .arg(ms / 60000, 2, 10, QChar('0'))
        .arg((ms / 1000) % 60, 2, 10, QChar('0'))
        .arg((ms / 100) % 10);
  };
  clock_->setText(format(milliseconds) + " / " + format(durationMs_));
  render_(frame);
}
