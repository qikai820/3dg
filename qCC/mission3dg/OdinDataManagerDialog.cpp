#include "OdinDataManagerDialog.h"

#include <QDateTime>
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
                                             InitialAction initialAction)
    : QDialog(parent) {
  setWindowTitle("Odin1 数据管理");
  resize(860, 460);
  network_.setProxy(QNetworkProxy(QNetworkProxy::NoProxy));
  origin_.setScheme("http");
  origin_.setHost(taskWebSocket.host());
  origin_.setPort(8000); // WebGroundStation's task-machine API port.

  auto *layout = new QVBoxLayout(this);
  auto *intro = new QLabel("选择指定序号的数据下载或删除。正在录制的数据不能操作。", this);
  intro->setWordWrap(true);
  layout->addWidget(intro);
  auto *address = new QLabel("任务机数据接口：" + origin_.toString(), this);
  address->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(address);

  auto *toolbar = new QHBoxLayout;
  summary_ = new QLabel("正在读取数据…", this);
  toolbar->addWidget(summary_, 1);
  recordButton_ = new QPushButton("开始录制", this);
  toolbar->addWidget(recordButton_);
  refreshButton_ = new QPushButton("刷新", this);
  toolbar->addWidget(refreshButton_);
  layout->addLayout(toolbar);

  table_ = new QTableWidget(0, 4, this);
  table_->setHorizontalHeaderLabels({"数据编号", "记录日期和时间", "占用空间", "操作"});
  table_->horizontalHeaderItem(1)->setToolTip(
      "任务机录制目录的最后修改时间，按本机时区显示；不一定是录制开始时间。");
  table_->verticalHeader()->hide();
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  layout->addWidget(table_, 1);

  note_ = new QLabel(this);
  note_->setWordWrap(true);
  layout->addWidget(note_);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
  layout->addWidget(buttons);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(refreshButton_, &QPushButton::clicked, this, [this] { refresh(); });
  connect(recordButton_, &QPushButton::clicked, this,
          [this] { changeRecording(!recordingActive_); });
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
    std::function<void(const QJsonObject &, const QString &)> done) {
  QNetworkRequest request(apiUrl(path));
  request.setRawHeader("Accept", "application/json");
  QNetworkReply *reply = nullptr;
  if (method == "GET")
    reply = network_.get(request);
  else if (method == "POST")
    reply = network_.post(request, QByteArray());
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

void OdinDataManagerDialog::refresh(bool preserveNote) {
  if (downloadReply_ || deleting_ || recordingBusy_)
    return;
  const quint64 generation = ++refreshGeneration_;
  refreshButton_->setEnabled(false);
  table_->setRowCount(0);
  summary_->setText("正在读取数据列表…");
  if (!preserveNote)
    note_->clear();
  requestJson("/api/odin/record/status", "GET",
              [this, generation](const QJsonObject &status, const QString &error) {
                if (generation != refreshGeneration_ || recordingBusy_)
                  return;
                if (!error.isEmpty()) {
                  summary_->setText("读取录制状态失败：" + error);
                  driverRunning_ = recordingActive_ = false;
                  updateRecordingControls();
                  return;
                }
                driverRunning_ = status.value("driver_running").toBool();
                recordingActive_ = status.value("recording_active").toBool();
                updateRecordingControls();
                summary_->setText(
                    QString("%1 · 共 %2 条 · 已用 %3 · 可用 %4")
                        .arg(recordingActive_ ? "录制中" : "未录制")
                        .arg(status.value("recordings_count").toInt())
                        .arg(sizeText(status.value("record_root_size_bytes").toVariant().toLongLong()))
                        .arg(sizeText(status.value("available_bytes").toVariant().toLongLong())));
              });
  requestJson("/api/odin/record/records", "GET",
              [this, generation](const QJsonObject &data, const QString &error) {
                if (generation != refreshGeneration_ || recordingBusy_)
                  return;
                refreshButton_->setEnabled(true);
                if (!error.isEmpty()) {
                  note_->setText("读取数据列表失败：" + error);
                  return;
                }
                const auto records = data.value("records").toArray();
                table_->setRowCount(records.size());
                for (int row = 0; row < records.size(); ++row) {
                  const auto record = records.at(row).toObject();
                  const QString id = record.value("id").toString();
                  const qint64 size = record.value("size_bytes").toVariant().toLongLong();
                  const bool active = record.value("recording_active").toBool();
                  table_->setItem(row, 0, new QTableWidgetItem(id));
                  table_->setItem(row, 1, new QTableWidgetItem(recordDateTime(record)));
                  table_->setItem(row, 2, new QTableWidgetItem(sizeText(size) +
                                          (active ? " · 正在录制" : "")));
                  auto *actions = new QWidget(table_);
                  auto *buttons = new QHBoxLayout(actions);
                  buttons->setContentsMargins(2, 2, 2, 2);
                  auto *get = new QPushButton("下载", actions);
                  auto *del = new QPushButton("删除", actions);
                  const bool allowed = kRecordId.match(id).hasMatch() && !active;
                  get->setEnabled(allowed);
                  del->setEnabled(allowed);
                  buttons->addWidget(get);
                  buttons->addWidget(del);
                  table_->setCellWidget(row, 3, actions);
                  connect(get, &QPushButton::clicked, this,
                          [this, id, size] { download(id, size); });
                  connect(del, &QPushButton::clicked, this,
                          [this, id, size] { remove(id, size); });
                }
                if (records.isEmpty() && note_->text().isEmpty())
                  note_->setText("暂无录制数据");
              });
}

void OdinDataManagerDialog::updateRecordingControls() {
  const bool available = !origin_.host().isEmpty() && !recordingBusy_ &&
                         !deleting_ && !downloadReply_;
  recordButton_->setText(recordingActive_ ? "停止录制" : "开始录制");
  recordButton_->setEnabled(available && (recordingActive_ || driverRunning_));
}

void OdinDataManagerDialog::changeRecording(bool start) {
  if (origin_.host().isEmpty() || recordingBusy_ || deleting_ || downloadReply_)
    return;
  ++refreshGeneration_; // Ignore status/list responses started before this operation.
  recordingBusy_ = true;
  updateRecordingControls();
  refreshButton_->setEnabled(false);
  note_->setText("正在检查录制状态…");
  requestJson("/api/odin/record/status", "GET",
              [this, start](const QJsonObject &status, const QString &error) {
                auto finish = [this](const QString &message) {
                  recordingBusy_ = false;
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

void OdinDataManagerDialog::download(const QString &id, qint64 size) {
  if (downloadReply_ || deleting_ || !kRecordId.match(id).hasMatch())
    return;
  requestJson("/api/network/access", "GET",
              [this, id, size](const QJsonObject &access, const QString &error) {
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
                if (QMessageBox::question(this, "下载 Odin1 数据",
                      QString("通过 %1 下载 %2（%3）？\n%4")
                          .arg(origin_.toString(), id, sizeText(size), link))
                    != QMessageBox::Yes)
                  return;
                const QString path = QFileDialog::getSaveFileName(
                    this, "保存 Odin1 数据", id + ".tar", "Tar 归档 (*.tar)");
                if (path.isEmpty())
                  return;
                downloadFile_ = std::make_unique<QSaveFile>(path);
                if (!downloadFile_->open(QIODevice::WriteOnly)) {
                  QMessageBox::warning(this, "下载失败", "无法创建本地文件：" + path);
                  downloadFile_.reset();
                  return;
                }
                QNetworkRequest request(apiUrl("/api/odin/record/download/" + id));
                auto *reply = network_.get(request);
                auto errorBody = QSharedPointer<QByteArray>::create();
                auto writeError = QSharedPointer<QString>::create();
                downloadReply_ = reply;
                refreshButton_->setEnabled(false);
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
                  table_->setEnabled(true);
                  updateRecordingControls();
                  note_->setText(error.isEmpty() ? "下载完成：" + id
                                                 : "下载失败：" + error);
                });
              });
}

void OdinDataManagerDialog::remove(const QString &id, qint64 size) {
  if (downloadReply_ || deleting_ || !kRecordId.match(id).hasMatch())
    return;
  if (QMessageBox::warning(this, "永久删除 Odin1 数据",
        QString("确认永久删除 %1？\n占用空间：%2\n删除后无法恢复。")
            .arg(id, sizeText(size)), QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No) != QMessageBox::Yes)
    return;
  deleting_ = true;
  refreshButton_->setEnabled(false);
  table_->setEnabled(false);
  updateRecordingControls();
  note_->setText("删除中：" + id);
  requestJson("/api/odin/record/records/" + id, "DELETE",
              [this, id](const QJsonObject &data, const QString &error) {
                deleting_ = false;
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
