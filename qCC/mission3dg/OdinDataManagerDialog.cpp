#include "OdinDataManagerDialog.h"

#include <QDateTime>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QLabel>
#include <QMessageBox>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPushButton>
#include <QRegularExpression>
#include <QSharedPointer>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace {
const QRegularExpression kRecordId(QStringLiteral("^Odin1_Data_[0-9]{6}$"));
const QRegularExpression kBagId(QStringLiteral("^mission3dg-[0-9]{8}-[0-9]{6}$"));
const QRegularExpression kFlightId(QStringLiteral("^(?:flight_[0-9]{6,}|[0-9]{8}_[0-9]{6}_[a-f0-9]{8})$"));

QString recordDateTime(const QJsonObject &record) {
  const QJsonValue value = record.value("modified_ns");
  if (!value.isDouble())
    return QStringLiteral("未知");
  const double nanoseconds = value.toDouble();
  if (!std::isfinite(nanoseconds) || nanoseconds < 1704067200e9 ||
      nanoseconds >= 253402300800e9)
    return QStringLiteral("未知");
  return QDateTime::fromMSecsSinceEpoch(qint64(nanoseconds / 1e6))
      .toLocalTime().toString("yyyy-MM-dd HH:mm:ss");
}

QString flightDateTime(const QJsonObject &record) {
  if (record.value("wall_clock_valid").isBool() &&
      !record.value("wall_clock_valid").toBool())
    return QStringLiteral("时间未校准");
  QJsonObject timestamp;
  timestamp.insert("modified_ns", record.value("started_ts_ns"));
  const QString value = recordDateTime(timestamp);
  return value == QStringLiteral("未知") ? QStringLiteral("时间未校准") : value;
}

QString responseError(QNetworkReply *reply, const QByteArray &body) {
  const auto json = QJsonDocument::fromJson(body).object();
  QString message = json.value("detail").toString();
  if (message.isEmpty())
    message = json.value("message").toString();
  if (message.isEmpty())
    message = reply->error() == QNetworkReply::NoError
                  ? QStringLiteral("HTTP %1").arg(
                        reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt())
                  : reply->errorString();
  return message;
}
} // namespace

OdinDataManagerDialog::OdinDataManagerDialog(const QUrl &taskWebSocket,
                                             QWidget *parent,
                                             InitialAction initialAction,
                                             ProtocolClient *client)
    : QDialog(parent), client_(client) {
  setWindowTitle("数据管理");
  resize(990, 500);
  network_.setProxy(QNetworkProxy(QNetworkProxy::NoProxy));
  origin_.setScheme("http");
  origin_.setHost(taskWebSocket.host());
  origin_.setPort(8000); // WebGroundStation's task-machine API port.

  auto *layout = new QVBoxLayout(this);
  auto *intro = new QLabel("按数据分类查看任务机上的 Odin1 原始数据、3DG bag 和飞行记录。", this);
  intro->setWordWrap(true);
  layout->addWidget(intro);
  auto *address = new QLabel("任务机数据接口：" + origin_.toString(), this);
  address->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(address);

  auto *toolbar = new QHBoxLayout;
  toolbar->addWidget(new QLabel("数据分类：", this));
  category_ = new QComboBox(this);
  category_->addItems({"Odin1 原始数据", "3DG bag", "飞行记录"});
  toolbar->addWidget(category_);
  summary_ = new QLabel("正在读取数据…", this);
  toolbar->addWidget(summary_, 1);
  bagSummary_ = new QLabel("bag 正在读取…", this);
  toolbar->addWidget(bagSummary_, 1);
  flightSummary_ = new QLabel("飞行记录正在读取…", this);
  toolbar->addWidget(flightSummary_, 1);
  recordButton_ = new QPushButton("开始录制", this);
  toolbar->addWidget(recordButton_);
  refreshButton_ = new QPushButton("刷新", this);
  toolbar->addWidget(refreshButton_);
  layout->addLayout(toolbar);

  table_ = new QTableWidget(0, 5, this);
  table_->setHorizontalHeaderLabels({"类型", "数据编号", "记录日期和时间", "占用空间", "操作"});
  table_->horizontalHeaderItem(2)->setToolTip(
      "Odin1/bag 显示最后修改时间；飞行记录显示开始时间。按本机时区显示。");
  table_->verticalHeader()->hide();
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
  layout->addWidget(table_, 1);

  note_ = new QLabel(this);
  note_->setWordWrap(true);
  layout->addWidget(note_);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(refreshButton_, &QPushButton::clicked, this, [this] { refresh(); });
  connect(category_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int) { refresh(); });
  connect(recordButton_, &QPushButton::clicked, this,
          [this] { changeRecording(!recordingActive_); });
  if (client_) {
    connect(client_, &ProtocolClient::message, this,
            [this](const mission::Envelope &e) {
              if (e.has_replay_status())
                updateReplay(e.replay_status());
              else if (e.has_result() &&
                       QString::fromStdString(e.request_id()) == replayRequest_ &&
                       e.result().state() != mission::CommandResult::ACCEPTED) {
                replayRequest_.clear();
                if (e.result().state() == mission::CommandResult::SUCCEEDED)
                  accept();
                else
                  note_->setText(QString::fromStdString(e.result().detail()));
              }
            });
    connect(client_, &ProtocolClient::stateChanged, this,
            [this](const QString &, bool ready) {
              if (ready && client_->bagReplaySupported()) {
                mission::Command request;
                request.set_kind(mission::Command::GET_REPLAY_STATUS);
                client_->sendCommand(request);
              }
              refresh(true);
            });
    if (client_->connected() && client_->bagReplaySupported()) {
      mission::Command request;
      request.set_kind(mission::Command::GET_REPLAY_STATUS);
      client_->sendCommand(request);
    }
  }
  updateReplay(mission::ReplayStatus());
  updateRecordingControls();

  if (origin_.host().isEmpty()) {
    refreshButton_->setEnabled(false);
    note_->setText("请先在“设置 → 3DG 配置”填写任务机 WebSocket 地址。");
  } else {
    refresh();
    if (initialAction != InitialAction::None)
      QTimer::singleShot(0, this, [this, initialAction] {
        changeRecording(initialAction == InitialAction::Start);
      });
  }
}

QUrl OdinDataManagerDialog::apiUrl(const QString &path) const {
  QUrl url = origin_;
  url.setPath(path);
  return url;
}

QString OdinDataManagerDialog::sizeText(qint64 bytes) {
  if (bytes >= 1024LL * 1024 * 1024)
    return QString::number(bytes / double(1024LL * 1024 * 1024), 'f', 2) + " GB";
  if (bytes >= 1024LL * 1024)
    return QString::number(bytes / double(1024LL * 1024), 'f', 1) + " MB";
  return QString::number(qMax<qint64>(1, qRound64(bytes / 1024.0))) + " KB";
}

void OdinDataManagerDialog::requestJson(
    const QString &path, const QByteArray &method,
    std::function<void(const QJsonObject &, const QString &)> done,
    const QByteArray &body) {
  QNetworkRequest request(apiUrl(path));
  request.setRawHeader("Accept", "application/json");
  if (!body.isEmpty())
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  QNetworkReply *reply = nullptr;
  if (method == "GET")
    reply = network_.get(request);
  else if (method == "POST")
    reply = network_.post(request, body);
  else
    reply = network_.sendCustomRequest(request, method);
  QTimer::singleShot(method == "DELETE" ? 30000 : 15000, reply, [reply] {
    if (reply->isRunning())
      reply->abort();
  });
  connect(reply, &QNetworkReply::finished, this,
          [reply, done = std::move(done)] {
            const QByteArray body = reply->readAll();
            const int status = reply->attribute(
                                        QNetworkRequest::HttpStatusCodeAttribute)
                                   .toInt();
            QJsonParseError parseError;
            const QJsonDocument document = QJsonDocument::fromJson(body, &parseError);
            QString error = reply->error() != QNetworkReply::NoError ||
                                    status < 200 || status >= 300
                                ? responseError(reply, body)
                                : QString();
            if (error.isEmpty() && (parseError.error != QJsonParseError::NoError ||
                                    !document.isObject()))
              error = "任务机返回了无效数据";
            const QJsonObject json = document.object();
            reply->deleteLater();
            done(json, error);
          });
}

void OdinDataManagerDialog::requestFlightRecords(
    std::function<void(const QJsonArray &, const QString &)> done) {
  auto *reply = network_.get(QNetworkRequest(apiUrl("/api/flight-records")));
  QTimer::singleShot(15000, reply, [reply] {
    if (reply->isRunning())
      reply->abort();
  });
  connect(reply, &QNetworkReply::finished, this,
          [reply, done = std::move(done)] {
            const QByteArray body = reply->readAll();
            const int status = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            QJsonParseError parseError;
            const auto document = QJsonDocument::fromJson(body, &parseError);
            QString error = reply->error() != QNetworkReply::NoError ||
                                    status < 200 || status >= 300
                                ? responseError(reply, body) : QString();
            if (error.isEmpty() && (parseError.error != QJsonParseError::NoError ||
                                    !document.isArray()))
              error = "任务机返回了无效的飞行记录列表";
            reply->deleteLater();
            done(document.array(), error);
          });
}

void OdinDataManagerDialog::refresh(bool preserveNote) {
  if (downloadReply_ || deleting_ || recordingBusy_ || flightUpdateBusy_)
    return;
  const quint64 generation = ++refreshGeneration_;
  const Category category = static_cast<Category>(category_->currentIndex());
  refreshButton_->setEnabled(false);
  table_->setRowCount(0);
  pendingLists_ = 1;
  summary_->setVisible(category == Category::Odin);
  bagSummary_->setVisible(category == Category::Bag);
  flightSummary_->setVisible(category == Category::Flight);
  recordButton_->setVisible(category == Category::Odin);
  if (!preserveNote)
    note_->clear();
  if (category == Category::Flight) {
    flightSummary_->setText("飞行记录正在读取…");
    requestFlightRecords([this, generation](const QJsonArray &records,
                                          const QString &error) {
      if (generation != refreshGeneration_)
        return;
      refreshButton_->setEnabled(true);
      if (!error.isEmpty()) {
        flightSummary_->setText("飞行记录读取失败");
        note_->setText(error);
        return;
      }
      flightSummary_->setText(QString("飞行记录 %1 条").arg(records.size()));
      for (const auto &record : records)
        addFlightRecord(record.toObject());
      if (records.isEmpty())
        note_->setText("任务机尚无飞行记录。");
    });
    return;
  }
  if (category == Category::Bag) {
    bagSummary_->setText("bag 正在读取…");
    requestJson("/api/3dg/bag/records", "GET",
                [this, generation](const QJsonObject &data, const QString &error) {
      if (generation != refreshGeneration_)
        return;
      refreshButton_->setEnabled(true);
      if (!error.isEmpty()) {
        bagSummary_->setText("bag 列表读取失败");
        note_->setText("bag 列表读取失败：" + error);
        return;
      }
      const auto records = data.value("records").toArray();
      bagSummary_->setText(QString("bag %1 条").arg(records.size()));
      for (const auto &record : records)
        addRecord(record.toObject(), true);
      if (records.isEmpty())
        note_->setText("任务机尚无 3DG bag 记录。");
    });
    return;
  }
  summary_->setText("Odin1 正在读取…");
  requestJson("/api/odin/record/status", "GET",
              [this, generation](const QJsonObject &status, const QString &error) {
                if (generation != refreshGeneration_ || recordingBusy_)
                  return;
                if (!error.isEmpty()) {
                  summary_->setText("Odin1 状态读取失败：" + error);
                  driverRunning_ = recordingActive_ = false;
                  updateRecordingControls();
                  return;
                }
                driverRunning_ = status.value("driver_running").toBool();
                recordingActive_ = status.value("recording_active").toBool();
                updateRecordingControls();
                summary_->setText(
                    QString("Odin1 %1 · %2 条 · 已用 %3 · 可用 %4")
                        .arg(recordingActive_ ? "录制中" : "未录制")
                        .arg(status.value("recordings_count").toInt())
                        .arg(sizeText(status.value("record_root_size_bytes").toVariant().toLongLong()))
                        .arg(sizeText(status.value("available_bytes").toVariant().toLongLong())));
              });
  requestJson("/api/odin/record/records", "GET",
              [this, generation](const QJsonObject &data, const QString &error) {
                if (generation != refreshGeneration_ || recordingBusy_)
                  return;
                if (--pendingLists_ == 0)
                  refreshButton_->setEnabled(true);
                if (!error.isEmpty()) {
                  note_->setText("Odin1 列表读取失败：" + error);
                  return;
                }
                const auto records = data.value("records").toArray();
                for (const auto &record : records)
                  addRecord(record.toObject(), false);
              });
}

void OdinDataManagerDialog::addRecord(const QJsonObject &record, bool bag) {
  const QString id = record.value("id").toString();
  const qint64 size = record.value("size_bytes").toVariant().toLongLong();
  const bool active = record.value("recording_active").toBool();
  const int row = table_->rowCount();
  table_->insertRow(row);
  table_->setItem(row, 0, new QTableWidgetItem(bag ? "3DG bag" : "Odin1"));
  table_->setItem(row, 1, new QTableWidgetItem(id));
  table_->setItem(row, 2, new QTableWidgetItem(recordDateTime(record)));
  table_->setItem(row, 3, new QTableWidgetItem(sizeText(size) +
                                 (active ? (bag ? " · 未完成/录制中" : " · 正在录制") : "")));
  auto *actions = new QWidget(table_);
  auto *buttons = new QHBoxLayout(actions);
  buttons->setContentsMargins(2, 2, 2, 2);
  auto *get = new QPushButton("下载", actions);
  auto *del = new QPushButton("删除", actions);
  const bool allowed = (bag ? kBagId : kRecordId).match(id).hasMatch() && !active;
  get->setEnabled(allowed);
  del->setEnabled(allowed && (!bag || id != replayBagId_));
  buttons->addWidget(get);
  if (bag) {
    auto *play = new QPushButton("回放", actions);
    play->setObjectName("bagReplayStart");
    play->setCursor(Qt::PointingHandCursor);
    play->setEnabled(allowed && replayState_ == mission::ReplayStatus::LIVE &&
                     client_ && client_->connected() &&
                     client_->bagReplaySupported());
    buttons->addWidget(play);
    connect(play, &QPushButton::clicked, this,
            [this, id] { startReplay(id); });
  }
  buttons->addWidget(del);
  table_->setCellWidget(row, 4, actions);
  connect(get, &QPushButton::clicked, this,
          [this, id, size, bag] { download(id, size, bag ? Category::Bag : Category::Odin); });
  connect(del, &QPushButton::clicked, this,
          [this, id, size, bag] { remove(id, size, bag); });
}

void OdinDataManagerDialog::addFlightRecord(const QJsonObject &record) {
  const QString id = record.value("session_id").toString();
  if (!kFlightId.match(id).hasMatch())
    return;
  const qint64 size = record.value("size_bytes").toVariant().toLongLong();
  const bool active = record.value("status").toString() == "recording";
  const bool protectedValue = record.value("protected").toBool();
  const bool replayReady = !active && record.value("database_ready").toBool() &&
      record.value("replay_stream_format").toString() == "3dg-replay-stream-v1";
  const int row = table_->rowCount();
  table_->insertRow(row);
  table_->setItem(row, 0, new QTableWidgetItem("飞行记录"));
  table_->setItem(row, 1, new QTableWidgetItem(id));
  table_->setItem(row, 2, new QTableWidgetItem(flightDateTime(record)));
  table_->setItem(row, 3, new QTableWidgetItem(
      sizeText(size) + (active ? " · 记录中" : "") +
      (protectedValue ? " · 已保护" : "")));
  auto *actions = new QWidget(table_);
  auto *buttons = new QHBoxLayout(actions);
  buttons->setContentsMargins(2, 2, 2, 2);
  auto *get = new QPushButton("下载", actions);
  get->setEnabled(!active);
  buttons->addWidget(get);
  auto *play = new QPushButton("回放", actions);
  play->setEnabled(replayReady);
  buttons->addWidget(play);
  auto *protect = new QPushButton(protectedValue ? "取消保护" : "保护", actions);
  protect->setEnabled(!active);
  buttons->addWidget(protect);
  table_->setCellWidget(row, 4, actions);
  connect(get, &QPushButton::clicked, this,
          [this, id, size] { download(id, size, Category::Flight); });
  connect(play, &QPushButton::clicked, this, [this, id] {
    selectedFlightReplayId_ = id;
    accept();
  });
  connect(protect, &QPushButton::clicked, this,
          [this, id, protectedValue] { setFlightProtected(id, !protectedValue); });
}

void OdinDataManagerDialog::setFlightProtected(const QString &id,
                                                bool protectedValue) {
  if (flightUpdateBusy_ || downloadReply_ || deleting_ ||
      !kFlightId.match(id).hasMatch())
    return;
  flightUpdateBusy_ = true;
  table_->setEnabled(false);
  category_->setEnabled(false);
  refreshButton_->setEnabled(false);
  note_->setText((protectedValue ? QStringLiteral("正在保护 ")
                                 : QStringLiteral("正在取消保护 ")) + id);
  const QByteArray body = QJsonDocument(QJsonObject{
      {"protected", protectedValue}}).toJson(QJsonDocument::Compact);
  requestJson("/api/flight-records/" + id + "/protect", "POST",
              [this, id](const QJsonObject &, const QString &error) {
    flightUpdateBusy_ = false;
    table_->setEnabled(true);
    category_->setEnabled(true);
    if (!error.isEmpty()) {
      refreshButton_->setEnabled(true);
      note_->setText("更新保护状态失败：" + error);
      return;
    }
    refresh(true);
    note_->setText("已更新保护状态：" + id);
  }, body);
}

void OdinDataManagerDialog::startReplay(const QString &bagId) {
  if (!client_ || !client_->connected() || !client_->bagReplaySupported()) {
    note_->setText("任务机代理未连接或不支持 bag 回放；请更新代理。");
    return;
  }
  if (!replayRequest_.isEmpty()) {
    note_->setText("上一条回放命令仍在处理，请等待结果。");
    return;
  }
  mission::Command command;
  command.set_kind(mission::Command::REPLAY_START);
  command.set_replay_bag_id(bagId.toStdString());
  command.set_replay_speed(1.0);
  replayRequest_ = client_->sendCommand(command);
  if (!replayRequest_.isEmpty()) {
    note_->setText("回放命令已发送，等待任务机确认…");
    const QString request = replayRequest_;
    QTimer::singleShot(300000, this, [this, request] {
      if (replayRequest_ == request) {
        replayRequest_.clear();
        note_->setText("回放命令等待超时；请查看当前回放状态再操作。");
      }
    });
  }
}

void OdinDataManagerDialog::updateReplay(const mission::ReplayStatus &status) {
  const auto previousState = replayState_;
  const auto previousBag = replayBagId_;
  replayState_ = status.state();
  replayBagId_ = QString::fromStdString(status.bag_id());
  if ((previousState != replayState_ || previousBag != replayBagId_) &&
      category_->currentIndex() == int(Category::Bag))
    refresh(true);
}

void OdinDataManagerDialog::updateRecordingControls() {
  const bool available = !origin_.host().isEmpty() && !recordingBusy_ &&
                         !deleting_ && !downloadReply_;
  recordButton_->setText(recordingActive_ ? "停止 Odin1 录制" : "开始 Odin1 录制");
  recordButton_->setEnabled(available && (recordingActive_ || driverRunning_));
}

void OdinDataManagerDialog::changeRecording(bool start) {
  if (origin_.host().isEmpty() || recordingBusy_ || deleting_ || downloadReply_)
    return;
  ++refreshGeneration_; // Ignore status/list responses started before this operation.
  recordingBusy_ = true;
  category_->setEnabled(false);
  updateRecordingControls();
  refreshButton_->setEnabled(false);
  note_->setText("正在检查录制状态…");
  requestJson("/api/odin/record/status", "GET",
              [this, start](const QJsonObject &status, const QString &error) {
                auto finish = [this](const QString &message) {
                  recordingBusy_ = false;
                  category_->setEnabled(true);
                  note_->setText(message);
                  updateRecordingControls();
                  refreshButton_->setEnabled(true);
                };
                if (!error.isEmpty()) {
                  finish("读取录制状态失败：" + error);
                  return;
                }
                driverRunning_ = status.value("driver_running").toBool();
                recordingActive_ = status.value("recording_active").toBool();
                if (start && recordingActive_) {
                  finish("Odin1 已在录制");
                  refresh(true);
                  return;
                }
                if (!start && !recordingActive_) {
                  finish("Odin1 当前未录制");
                  refresh(true);
                  return;
                }
                if (start && !driverRunning_) {
                  finish("Odin1 驱动未运行，请先启动 Odin 驱动及 MAVROS");
                  QMessageBox::warning(this, "无法开始录制", note_->text());
                  return;
                }
                if (start) {
                  const QString prompt =
                      "开始录制 Odin1 的 OLX、位姿、IMU、点云和图像数据？\n\n"
                      "录制数据量约为 9.5 GB / 10 分钟，请留意剩余空间。";
                  if (QMessageBox::question(this, "开始 Odin1 录制", prompt,
                                            QMessageBox::Yes | QMessageBox::No,
                                            QMessageBox::No) != QMessageBox::Yes) {
                    finish("已取消开始录制");
                    return;
                  }
                }
                note_->setText(start ? "正在启动 Odin1 录制…"
                                     : "正在停止 Odin1 录制…");
                requestJson(start ? "/api/odin/record/start"
                                  : "/api/odin/record/stop", "POST",
                            [this, start, finish](const QJsonObject &result,
                                                   const QString &requestError) {
                              QString failure = requestError;
                              if (failure.isEmpty() &&
                                  result.value("status").toString() != "ok")
                                failure = result.value("message").toString(
                                    "任务机未确认录制操作");
                              if (failure.isEmpty())
                                recordingActive_ = start;
                              finish(failure.isEmpty()
                                         ? result.value("message").toString(
                                               start ? "Odin1 录制已开始"
                                                     : "Odin1 录制已停止")
                                         : (start ? "启动失败：" : "停止失败：") + failure);
                              if (!failure.isEmpty())
                                QMessageBox::warning(this, "Odin1 录制失败", failure);
                              refresh(true);
                            });
              });
}

void OdinDataManagerDialog::download(const QString &id, qint64 size,
                                     Category category) {
  const bool bag = category == Category::Bag;
  const bool flight = category == Category::Flight;
  if (downloadReply_ || deleting_ || flightUpdateBusy_ ||
      !(flight ? kFlightId : bag ? kBagId : kRecordId).match(id).hasMatch())
    return;
  requestJson("/api/network/access", "GET",
              [this, id, size, bag, flight](const QJsonObject &access, const QString &error) {
                if (!error.isEmpty()) {
                  QMessageBox::warning(this, "下载受阻", error);
                  return;
                }
                if (!access.value("download_allowed").toBool()) {
                  QMessageBox::warning(this, "下载受阻",
                                       access.value("reason").toString());
                  return;
                }
                const QString link = origin_.host().startsWith("192.168.144.")
                                         ? "当前为有线链路。"
                                         : "大文件下载建议使用有线链路。";
                const QString title = flight ? "下载飞行记录" :
                                      bag ? "下载 3DG bag" : "下载 Odin1 数据";
                if (QMessageBox::question(this, title,
                      QString("通过 %1 下载 %2（%3）？\n%4")
                          .arg(origin_.toString(), id, sizeText(size), link))
                    != QMessageBox::Yes)
                  return;
                const QString suffix = bag ? ".bag" : ".tar";
                const QString path = QFileDialog::getSaveFileName(
                    this, flight ? "保存飞行记录" : bag ? "保存 3DG bag" : "保存 Odin1 数据", id + suffix,
                    bag ? "ROS bag (*.bag)" : "Tar 归档 (*.tar)");
                if (path.isEmpty())
                  return;
                downloadFile_ = std::make_unique<QSaveFile>(path);
                if (!downloadFile_->open(QIODevice::WriteOnly)) {
                  QMessageBox::warning(this, "下载失败", "无法创建本地文件：" + path);
                  downloadFile_.reset();
                  return;
                }
                QString endpoint;
                if (flight)
                  endpoint = QStringLiteral("/api/flight-records/") + id + "/download";
                else
                  endpoint = (bag ? QStringLiteral("/api/3dg/bag/download/")
                                  : QStringLiteral("/api/odin/record/download/")) + id;
                QNetworkRequest request(apiUrl(endpoint));
                auto *reply = network_.get(request);
                auto errorBody = QSharedPointer<QByteArray>::create();
                auto writeError = QSharedPointer<QString>::create();
                downloadReply_ = reply;
                refreshButton_->setEnabled(false);
                category_->setEnabled(false);
                table_->setEnabled(false);
                updateRecordingControls();
                note_->setText("下载中：" + id);
                connect(reply, &QIODevice::readyRead, this,
                        [this, reply, errorBody, writeError] {
                  if (!downloadFile_)
                    return;
                  const QByteArray data = reply->readAll();
                  if (reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() != 200) {
                    if (errorBody->size() < 65536)
                      errorBody->append(data.left(65536 - errorBody->size()));
                  } else if (downloadFile_->write(data) != data.size()) {
                    *writeError = "本地文件写入失败";
                    note_->setText(*writeError + "，下载已取消");
                    reply->abort();
                  }
                });
                connect(reply, &QNetworkReply::downloadProgress, this,
                        [this, id](qint64 received, qint64) {
                          note_->setText("下载中：" + id + " · " + sizeText(received));
                        });
                connect(reply, &QNetworkReply::finished, this,
                        [this, reply, id, errorBody, writeError] {
                  const int status = reply->attribute(
                                             QNetworkRequest::HttpStatusCodeAttribute)
                                         .toInt();
                  const QByteArray tail = reply->readAll();
                  QString error = *writeError;
                  if (error.isEmpty() &&
                      (reply->error() != QNetworkReply::NoError || status != 200)) {
                    if (errorBody->size() < 65536)
                      errorBody->append(tail.left(65536 - errorBody->size()));
                    error = responseError(reply, *errorBody);
                  }
                  else if (downloadFile_ && downloadFile_->write(tail) != tail.size())
                    error = "本地文件写入失败";
                  if (error.isEmpty() && (!downloadFile_ || !downloadFile_->commit()))
                    error = "本地文件保存失败";
                  downloadFile_.reset(); // QSaveFile discards any incomplete archive.
                  downloadReply_.clear();
                  reply->deleteLater();
                  refreshButton_->setEnabled(true);
                  category_->setEnabled(true);
                  table_->setEnabled(true);
                  updateRecordingControls();
                  note_->setText(error.isEmpty() ? "下载完成：" + id
                                                 : "下载失败：" + error);
                });
              });
}

void OdinDataManagerDialog::remove(const QString &id, qint64 size, bool bag) {
  if (downloadReply_ || deleting_ || !(bag ? kBagId : kRecordId).match(id).hasMatch())
    return;
  if (bag && id == replayBagId_) {
    note_->setText("正在回放的 bag 不能删除；请先停止回放。");
    return;
  }
  if (QMessageBox::warning(this, bag ? "永久删除 3DG bag" : "永久删除 Odin1 数据",
        QString("确认永久删除 %1？\n占用空间：%2\n删除后无法恢复。")
            .arg(id, sizeText(size)), QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No) != QMessageBox::Yes)
    return;
  deleting_ = true;
  refreshButton_->setEnabled(false);
  category_->setEnabled(false);
  table_->setEnabled(false);
  updateRecordingControls();
  note_->setText("删除中：" + id);
  requestJson((bag ? "/api/3dg/bag/records/"
                   : "/api/odin/record/records/") + id, "DELETE",
              [this, id](const QJsonObject &data, const QString &error) {
                deleting_ = false;
                category_->setEnabled(true);
                table_->setEnabled(true);
                updateRecordingControls();
                if (!error.isEmpty()) {
                  refreshButton_->setEnabled(true);
                  note_->setText("删除失败：" + error);
                  QMessageBox::warning(this, "删除失败", error);
                  return;
                }
                refresh(true);
                note_->setText(data.value("message").toString("已删除 " + id));
              });
}

void OdinDataManagerDialog::reject() {
  if (flightUpdateBusy_) {
    QMessageBox::information(this, "正在更新保护状态", "请等待任务机确认后再关闭窗口。");
    return;
  }
  if (recordingBusy_) {
    QMessageBox::information(this, "录制操作进行中",
                             "请等待任务机确认录制状态后再关闭窗口。");
    return;
  }
  if (deleting_) {
    QMessageBox::information(this, "正在删除", "请等待删除结果返回后再关闭窗口。");
    return;
  }
  if (downloadReply_ && QMessageBox::question(
          this, "取消下载", "下载尚未完成，关闭窗口将取消下载。确定关闭？")
                            != QMessageBox::Yes)
    return;
  if (downloadReply_)
    downloadReply_->abort();
  QDialog::reject();
}
