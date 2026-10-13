#pragma once

#include "ProtocolClient.h"

#include <QFrame>

class QLabel;
class QMenu;
class QSlider;
class QToolButton;

// Main-window controls for the task-machine bag source. The data manager only
// selects a recording; playback remains accessible after that dialog closes.
class BagReplayToolbar : public QFrame {
public:
  explicit BagReplayToolbar(ProtocolClient *client, QWidget *parent = nullptr);
  void setReplayStatus(const mission::ReplayStatus &status);

private:
  void sendReplay(mission::Command::Kind kind, double seconds = 0);
  void refreshControls();

  ProtocolClient *client_ = nullptr;
  QLabel *time_ = nullptr;
  QToolButton *playback_ = nullptr;
  QToolButton *speed_ = nullptr;
  QMenu *speedMenu_ = nullptr;
  QSlider *seek_ = nullptr;
  double selectedSpeed_ = 1.0;
  QString pendingRequest_;
  mission::ReplayStatus replay_;
};
