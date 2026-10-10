#pragma once

#include "ProtocolClient.h"

#include <QFrame>

class QLabel;
class QPushButton;
class QComboBox;
class QSlider;
class ReplayTimeRuler;
class QWidget;

// Main-window controls for the task-machine bag source. The data manager only
// selects a recording; playback remains accessible after that dialog closes.
class BagReplayToolbar : public QFrame {
public:
  explicit BagReplayToolbar(ProtocolClient *client, QWidget *parent = nullptr);
  void setReplayStatus(const mission::ReplayStatus &status);
  void setWindowWidth(int width);

private:
  void sendReplay(mission::Command::Kind kind, double seconds = 0);
  void refreshControls();
  void updateStatusText();

  ProtocolClient *client_ = nullptr;
  QWidget *leftInfo_ = nullptr;
  QLabel *status_ = nullptr;
  QLabel *time_ = nullptr;
  QPushButton *pause_ = nullptr;
  QPushButton *stop_ = nullptr;
  QComboBox *speed_ = nullptr;
  QSlider *seek_ = nullptr;
  ReplayTimeRuler *ruler_ = nullptr;
  QString fullStatus_ = QStringLiteral("bag 回放：实时数据");
  QString pendingRequest_;
  mission::ReplayStatus replay_;
};
