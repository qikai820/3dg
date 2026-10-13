#include "BagReplayToolbar.h"

#include <QActionGroup>
#include <QFont>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMenu>
#include <QPainter>
#include <QPixmap>
#include <QSlider>
#include <QTimer>
#include <QToolButton>

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

QString replaySpeed(double speed) {
  return QString("X%1").arg(speed, 0, 'g', 3);
}

enum class ReplayGlyph { Play, Pause, Stop };

QPixmap glyphPixmap(ReplayGlyph glyph, const QColor &color) {
  QPixmap pixmap(40, 40);
  pixmap.fill(Qt::transparent);
  pixmap.setDevicePixelRatio(2);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.scale(2, 2);
  painter.setPen(Qt::NoPen);
  painter.setBrush(color);
  if (glyph == ReplayGlyph::Pause) {
    painter.drawRoundedRect(QRectF(4, 4, 4, 12), 1, 1);
    painter.drawRoundedRect(QRectF(12, 4, 4, 12), 1, 1);
  } else if (glyph == ReplayGlyph::Stop) {
    painter.drawRoundedRect(QRectF(5, 5, 10, 10), 1.5, 1.5);
  } else {
    const QPointF triangle[] = {{5, 3}, {5, 17}, {17, 10}};
    painter.drawPolygon(triangle, 3);
  }
  return pixmap;
}

QIcon playbackIcon(ReplayGlyph glyph) {
  QIcon icon;
  icon.addPixmap(glyphPixmap(glyph, QColor(20, 111, 98)), QIcon::Normal);
  icon.addPixmap(glyphPixmap(glyph, QColor(117, 161, 154)), QIcon::Disabled);
  return icon;
}
} // namespace

BagReplayToolbar::BagReplayToolbar(ProtocolClient *client, QWidget *parent)
    : QFrame(parent), client_(client) {
  setObjectName("bagReplayToolbar");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setFixedHeight(38);
  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(8, 3, 8, 3);
  layout->setSpacing(6);

  time_ = new QLabel("--:-- / --:--", this);
  time_->setObjectName("replayTimeLabel");
  time_->setFixedSize(138, 24);
  time_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  QFont timeFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  timeFont.setPixelSize(11);
  timeFont.setWeight(QFont::DemiBold);
  time_->setFont(timeFont);

  seek_ = new QSlider(Qt::Horizontal, this);
  seek_->setFixedHeight(28);
  seek_->setRange(0, 10000);
  seek_->setAccessibleName("bag 回放时间轴");
  seek_->setToolTip("松开滑块后跳转；定位、点云和历史轨迹会重新开始");
  seek_->setStyleSheet(
      "QSlider{background:transparent;}"
      "QSlider::groove:horizontal{height:6px;"
      "background:#d7e7ec;"
      "border:1px solid #c6dbe2;border-radius:3px;}"
      "QSlider::sub-page:horizontal{background:#3ac4ad;"
      "border-radius:3px;}"
      "QSlider::add-page:horizontal{background:#d7e7ec;"
      "border-radius:3px;}"
      "QSlider::handle:horizontal{width:26px;margin:-10px 0;"
      "background:#ffffff;border:2px solid #168778;border-radius:13px;}"
      "QSlider::handle:horizontal:hover{background:#e6fbf6;"
      "border-color:#0f7468;}"
      "QSlider::handle:horizontal:pressed{background:#c8eee4;}");
  layout->addWidget(seek_, 1);

  auto *buttons = new QHBoxLayout;
  buttons->setSpacing(2);
  playback_ = new QToolButton(this);
  playback_->setObjectName("replayPlaybackButton");
  playback_->setIconSize(QSize(18, 18));
  playback_->setFixedWidth(50);
  playback_->setFixedHeight(28);
  playback_->setCursor(Qt::PointingHandCursor);
  playback_->setPopupMode(QToolButton::MenuButtonPopup);
  playback_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  auto *playbackMenu = new QMenu(playback_);
  auto *stopAction = playbackMenu->addAction(
      QIcon(glyphPixmap(ReplayGlyph::Stop, QColor(164, 84, 60))),
      "停止回放");
  playback_->setMenu(playbackMenu);
  connect(stopAction, &QAction::triggered, this,
          [this] { sendReplay(mission::Command::REPLAY_STOP); });
  speed_ = new QToolButton(this);
  speed_->setObjectName("replaySpeedButton");
  speed_->setText(replaySpeed(selectedSpeed_));
  speed_->setFixedSize(44, 28);
  speed_->setCursor(Qt::PointingHandCursor);
  speed_->setToolButtonStyle(Qt::ToolButtonTextOnly);
  speed_->setAccessibleName("回放倍率 " + replaySpeed(selectedSpeed_));
  speedMenu_ = new QMenu(speed_);
  auto *speedGroup = new QActionGroup(speedMenu_);
  speedGroup->setExclusive(true);
  for (double speed : {.25, .5, 1.0, 2.0, 4.0}) {
    auto *action = speedMenu_->addAction(replaySpeed(speed));
    action->setData(speed);
    action->setCheckable(true);
    action->setActionGroup(speedGroup);
    action->setChecked(speed == selectedSpeed_);
    connect(action, &QAction::triggered, this, [this, speed] {
      selectedSpeed_ = speed;
      speed_->setText(replaySpeed(speed));
      speed_->setToolTip("播放倍率：" + replaySpeed(speed));
      speed_->setAccessibleName("回放倍率 " + replaySpeed(speed));
      sendReplay(mission::Command::REPLAY_SPEED);
    });
  }
  speed_->setToolTip("播放倍率：" + replaySpeed(selectedSpeed_));
  connect(speed_, &QToolButton::clicked, this, [this] {
    speedMenu_->popup(speed_->mapToGlobal(QPoint(0, speed_->height())));
  });
  buttons->addWidget(playback_);
  buttons->addWidget(speed_);
  layout->insertLayout(0, buttons);
  layout->addWidget(time_);
  connect(playback_, &QToolButton::clicked, this, [this] {
    const auto state = replay_.state();
    sendReplay(state == mission::ReplayStatus::PAUSED
                   ? mission::Command::REPLAY_RESUME
               : state == mission::ReplayStatus::PLAYING
                   ? mission::Command::REPLAY_PAUSE
                   : mission::Command::REPLAY_STOP);
  });
  connect(seek_, &QSlider::sliderReleased, this, [this] {
    sendReplay(mission::Command::REPLAY_SEEK,
               replay_.duration_seconds() * seek_->value() / 10000.0);
  });
  connect(seek_, &QSlider::sliderMoved, this, [this](int value) {
    time_->setText(replayTime(replay_.duration_seconds() * value / 10000.0) +
                   " / " + replayTime(replay_.duration_seconds()));
  });
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
                  time_->setToolTip(
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
                time_->setText("--:-- / --:--");
                time_->setToolTip(QString());
                seek_->setValue(0);
              }
              refreshControls();
            });
  }
  refreshControls();
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
  time_->setToolTip(active ? QString("%1 · %2\n%3")
                                  .arg(QString::fromStdString(replay_.bag_id()),
                                       state,
                                       QString::fromStdString(replay_.detail()))
                            : QString());
  time_->setText(active ? QString("%1 / %2")
                              .arg(replayTime(replay_.position_seconds()),
                                   replayTime(replay_.duration_seconds()))
                        : QStringLiteral("--:-- / --:--"));
  if (!seek_->isSliderDown())
    seek_->setValue(active && replay_.duration_seconds() > 0
                        ? qBound(0, int(replay_.position_seconds() /
                                            replay_.duration_seconds() * 10000), 10000)
                        : 0);
  for (QAction *action : speedMenu_->actions()) {
    if (action->data().toDouble() == replay_.speed()) {
      selectedSpeed_ = replay_.speed();
      action->setChecked(true);
      speed_->setText(replaySpeed(selectedSpeed_));
      speed_->setToolTip("播放倍率：" + replaySpeed(selectedSpeed_));
      speed_->setAccessibleName("回放倍率 " + replaySpeed(selectedSpeed_));
      break;
    }
  }
  refreshControls();
}

void BagReplayToolbar::refreshControls() {
  const bool connected = client_ && client_->connected() &&
                         client_->bagReplaySupported();
  const bool idle = pendingRequest_.isEmpty();
  const auto state = replay_.state();
  const auto glyph = state == mission::ReplayStatus::PLAYING
                         ? ReplayGlyph::Pause
                     : state == mission::ReplayStatus::ENDED ||
                               state == mission::ReplayStatus::ERROR
                         ? ReplayGlyph::Stop
                         : ReplayGlyph::Play;
  playback_->setIcon(playbackIcon(glyph));
  playback_->setAccessibleName(state == mission::ReplayStatus::PLAYING ? "暂停回放" :
                               state == mission::ReplayStatus::PAUSED ? "继续回放" :
                               state == mission::ReplayStatus::LIVE ? "播放" :
                               "停止回放");
  playback_->setEnabled(connected && idle && state != mission::ReplayStatus::LIVE);
  playback_->setToolTip(!connected
                            ? "连接任务机后可回放 bag"
                        : state == mission::ReplayStatus::LIVE
                            ? "请先从数据管理选择已完成的 bag"
                        : state == mission::ReplayStatus::PLAYING ||
                                  state == mission::ReplayStatus::PAUSED
                            ? "点击暂停或继续；点右侧箭头可停止回放"
                            : "停止回放并恢复实时数据");
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
    command.set_replay_speed(selectedSpeed_);
  pendingRequest_ = client_->sendCommand(command);
  refreshControls();
  if (!pendingRequest_.isEmpty()) {
    const QString request = pendingRequest_;
    QTimer::singleShot(30000, this, [this, request] {
      if (pendingRequest_ == request) {
        pendingRequest_.clear();
        time_->setToolTip("回放命令等待超时；请查看任务机日志");
        refreshControls();
      }
    });
  }
}
