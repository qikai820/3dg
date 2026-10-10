#include "BagReplayToolbar.h"

#include <QComboBox>
#include <QFont>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QSlider>
#include <QTimer>
#include <cmath>

namespace {
QString replayTime(double seconds) {
  const qint64 total = qMax<qint64>(0, qRound64(seconds));
  const qint64 hours = total / 3600;
  const qint64 minutes = (total / 60) % 60;
  const qint64 remainder = total % 60;
  return hours ? QString("%1:%2:%3")
                     .arg(hours).arg(minutes, 2, 10, QLatin1Char('0'))
                     .arg(remainder, 2, 10, QLatin1Char('0'))
               : QString("%1:%2")
                     .arg(total / 60).arg(remainder, 2, 10, QLatin1Char('0'));
}
} // namespace

class ReplayTimeRuler : public QWidget {
public:
  explicit ReplayTimeRuler(QWidget *parent = nullptr) : QWidget(parent) {
    setFixedHeight(14);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  void setDuration(double seconds) {
    duration_ = seconds;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    if (duration_ <= 0)
      return;
    QPainter painter(this);
    QFont font = painter.font();
    font.setPixelSize(10);
    painter.setFont(font);
    painter.setPen(QColor(91, 112, 125));
    const int left = 9;
    const int span = qMax(0, width() - 18);
    int divisions = 6;
    if (width() < 150)
      divisions = 1;
    else if (width() < 360)
      divisions = 2;
    else if (width() < 620)
      divisions = 4;
    for (int i = 0; i <= divisions; ++i) {
      const int x = left + qRound(double(span) * i / divisions);
      painter.drawLine(x, 0, x, i % 2 == 0 ? 3 : 2);
      const QString label = replayTime(duration_ * i / divisions);
      const int labelX = i == 0 ? 0 : i == divisions ? width() - 64 : x - 32;
      const QRect text(labelX, 2, 64, height() - 2);
      painter.drawText(text, Qt::AlignHCenter | Qt::AlignBottom, label);
    }
  }

private:
  double duration_ = 0;
};

class ReplayTimeline : public QWidget {
public:
  explicit ReplayTimeline(QWidget *parent = nullptr) : QWidget(parent) {
    setFixedHeight(40);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }

  void setControls(QSlider *seek, ReplayTimeRuler *ruler) {
    seek_ = seek;
    ruler_ = ruler;
    positionControls();
  }

protected:
  void resizeEvent(QResizeEvent *event) override {
    QWidget::resizeEvent(event);
    positionControls();
  }

private:
  void positionControls() {
    if (ruler_)
      ruler_->setGeometry(0, 0, width(), 14);
    if (seek_)
      seek_->setGeometry(0, 8, width(), 24);
  }

  QSlider *seek_ = nullptr;
  ReplayTimeRuler *ruler_ = nullptr;
};

BagReplayToolbar::BagReplayToolbar(ProtocolClient *client, QWidget *parent)
    : QFrame(parent), client_(client) {
  setObjectName("bagReplayToolbar");
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setFixedHeight(46);
  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(10, 3, 10, 3);
  layout->setSpacing(8);

  leftInfo_ = new QWidget(this);
  leftInfo_->setFixedHeight(22);
  auto *infoLayout = new QHBoxLayout(leftInfo_);
  infoLayout->setContentsMargins(0, 0, 0, 0);
  infoLayout->setSpacing(6);
  status_ = new QLabel(leftInfo_);
  status_->setFixedHeight(22);
  status_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  infoLayout->addWidget(status_);
  time_ = new QLabel("--:-- / --:--", leftInfo_);
  time_->setFixedHeight(22);
  time_->setStyleSheet("color:#587487;font-size:11px;");
  infoLayout->addWidget(time_);
  infoLayout->addStretch(1);
  layout->addWidget(leftInfo_);

  auto *timeline = new ReplayTimeline(this);
  seek_ = new QSlider(Qt::Horizontal, timeline);
  seek_->setFixedHeight(24);
  seek_->setRange(0, 10000);
  seek_->setAccessibleName("bag 回放时间轴");
  seek_->setToolTip("松开滑块后跳转；定位、点云和历史轨迹会重新开始");
  seek_->setStyleSheet(
      "QSlider::groove:horizontal{height:12px;"
      "background:rgba(220,233,237,110);"
      "border:1px solid rgba(184,206,215,140);border-radius:6px;}"
      "QSlider::sub-page:horizontal{background:rgba(101,203,189,150);"
      "border-radius:6px;}"
      "QSlider::handle:horizontal{width:13px;margin:0;"
      "background:rgba(255,255,255,210);"
      "border:1px solid rgba(129,158,170,200);border-radius:6px;}"
      "QSlider:disabled::groove:horizontal{"
      "background:rgba(233,239,241,110);}");
  ruler_ = new ReplayTimeRuler(timeline);
  timeline->setControls(seek_, ruler_);
  layout->addWidget(timeline, 1, Qt::AlignBottom);

  auto *buttons = new QHBoxLayout;
  buttons->setSpacing(6);
  pause_ = new QPushButton("暂停", this);
  stop_ = new QPushButton("停止回放", this);
  for (QPushButton *button : {pause_, stop_}) {
    button->setFixedHeight(22);
    button->setStyleSheet("min-height:14px;padding:1px 6px;border-radius:5px;");
  }
  speed_ = new QComboBox(this);
  speed_->setFixedHeight(22);
  for (double speed : {.25, .5, 1.0, 2.0, 4.0})
    speed_->addItem(QString("%1 倍").arg(speed, 0, 'g', 3), speed);
  speed_->setCurrentIndex(2);
  buttons->addWidget(pause_);
  buttons->addWidget(stop_);
  buttons->addWidget(speed_);
  layout->addLayout(buttons);
  updateStatusText();

  connect(pause_, &QPushButton::clicked, this, [this] {
    sendReplay(replay_.state() == mission::ReplayStatus::PAUSED
                   ? mission::Command::REPLAY_RESUME
                   : mission::Command::REPLAY_PAUSE);
  });
  connect(stop_, &QPushButton::clicked, this,
          [this] { sendReplay(mission::Command::REPLAY_STOP); });
  connect(seek_, &QSlider::sliderReleased, this, [this] {
    sendReplay(mission::Command::REPLAY_SEEK,
               replay_.duration_seconds() * seek_->value() / 10000.0);
  });
  connect(seek_, &QSlider::sliderMoved, this, [this](int value) {
    time_->setText(replayTime(replay_.duration_seconds() * value / 10000.0) +
                   " / " + replayTime(replay_.duration_seconds()));
  });
  connect(speed_, QOverload<int>::of(&QComboBox::activated), this,
          [this](int) { sendReplay(mission::Command::REPLAY_SPEED); });
  if (client_) {
    connect(client_, &ProtocolClient::message, this,
            [this](const mission::Envelope &envelope) {
              if (envelope.has_replay_status())
                setReplayStatus(envelope.replay_status());
              else if (envelope.has_result() &&
                       QString::fromStdString(envelope.request_id()) ==
                           pendingRequest_ &&
                       envelope.result().state() !=
                           mission::CommandResult::ACCEPTED) {
                pendingRequest_.clear();
                refreshControls();
                if (envelope.result().state() == mission::CommandResult::FAILED)
                  status_->setToolTip(
                      QString::fromStdString(envelope.result().detail()));
              }
            });
    connect(client_, &ProtocolClient::stateChanged, this,
            [this](const QString &, bool ready) {
              pendingRequest_.clear();
              if (ready && client_->bagReplaySupported()) {
                mission::Command command;
                command.set_kind(mission::Command::GET_REPLAY_STATUS);
                client_->sendCommand(command);
              } else if (!ready) {
                replay_.Clear();
                fullStatus_ = QStringLiteral("bag 回放：任务机未连接");
                updateStatusText();
                status_->setToolTip(fullStatus_);
                time_->setText("--:-- / --:--");
                seek_->setValue(0);
                ruler_->setDuration(0);
              }
              refreshControls();
            });
  }
  refreshControls();
}

void BagReplayToolbar::setWindowWidth(int width) {
  const int barWidth = qMax(0, width * 2 / 3);
  setFixedWidth(barWidth);
  leftInfo_->setFixedWidth(qMin(barWidth * 2 / 5, 440));
  updateStatusText();
}

void BagReplayToolbar::updateStatusText() {
  const int available = leftInfo_->width() - 6 - time_->sizeHint().width();
  status_->setText(status_->fontMetrics().elidedText(
      fullStatus_, Qt::ElideMiddle, qMax(0, available)));
}

void BagReplayToolbar::setReplayStatus(const mission::ReplayStatus &status) {
  replay_ = status;
  const bool active = replay_.state() != mission::ReplayStatus::LIVE;
  const QString state = replay_.state() == mission::ReplayStatus::PLAYING
                            ? QStringLiteral("播放中")
                        : replay_.state() == mission::ReplayStatus::PAUSED
                            ? QStringLiteral("已暂停")
                        : replay_.state() == mission::ReplayStatus::ENDED
                            ? QStringLiteral("已结束")
                            : QStringLiteral("出错");
  fullStatus_ = active
                    ? QString("bag 回放：%1 · %2")
                          .arg(QString::fromStdString(replay_.bag_id()), state)
                    : QStringLiteral("bag 回放：实时数据");
  status_->setToolTip(fullStatus_ + "\n" +
                      QString::fromStdString(replay_.detail()));
  time_->setText(active ? QString("%1 / %2")
                              .arg(replayTime(replay_.position_seconds()),
                                   replayTime(replay_.duration_seconds()))
                        : QStringLiteral("--:-- / --:--"));
  updateStatusText();
  ruler_->setDuration(active ? replay_.duration_seconds() : 0);
  if (!seek_->isSliderDown())
    seek_->setValue(active && replay_.duration_seconds() > 0
                        ? qBound(0, int(replay_.position_seconds() /
                                            replay_.duration_seconds() * 10000), 10000)
                        : 0);
  const int speedIndex = speed_->findData(replay_.speed());
  if (speedIndex >= 0)
    speed_->setCurrentIndex(speedIndex);
  refreshControls();
}

void BagReplayToolbar::refreshControls() {
  const bool connected = client_ && client_->connected() &&
                         client_->bagReplaySupported();
  const bool idle = pendingRequest_.isEmpty();
  const auto state = replay_.state();
  pause_->setText(state == mission::ReplayStatus::PAUSED ? "继续" : "暂停");
  pause_->setEnabled(connected && idle &&
                     (state == mission::ReplayStatus::PLAYING ||
                      state == mission::ReplayStatus::PAUSED));
  stop_->setEnabled(connected && idle && state != mission::ReplayStatus::LIVE);
  speed_->setEnabled(connected && idle &&
                      (state == mission::ReplayStatus::PLAYING ||
                       state == mission::ReplayStatus::PAUSED));
  seek_->setEnabled(connected && idle && replay_.duration_seconds() > 0 &&
                    state != mission::ReplayStatus::LIVE &&
                    state != mission::ReplayStatus::ERROR);
}

void BagReplayToolbar::sendReplay(mission::Command::Kind kind, double seconds) {
  if (!client_ || !client_->connected() || !client_->bagReplaySupported() ||
      !pendingRequest_.isEmpty())
    return;
  mission::Command command;
  command.set_kind(kind);
  if (kind == mission::Command::REPLAY_SEEK)
    command.set_replay_seconds(seconds);
  else if (kind == mission::Command::REPLAY_SPEED)
    command.set_replay_speed(speed_->currentData().toDouble());
  pendingRequest_ = client_->sendCommand(command);
  refreshControls();
  if (!pendingRequest_.isEmpty()) {
    const QString request = pendingRequest_;
    QTimer::singleShot(30000, this, [this, request] {
      if (pendingRequest_ == request) {
        pendingRequest_.clear();
        status_->setToolTip("回放命令等待超时；请查看任务机日志");
        refreshControls();
      }
    });
  }
}
