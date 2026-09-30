#pragma once

#include "CloudIO.h"
#include <QDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QTimer>
#include <QUrl>
#include <QVector3D>
#include <functional>
#include <set>
#include <tuple>

class QLabel;
class QListWidget;
class QPushButton;
class QSlider;
class QNetworkReply;

struct FlightReplayFrame {
  QVector<QVector3D> trail;
  QVector<QVector3D> setpoints;
  QVector<QVector3D> voxels;
  QVector<MissionPoint> cloud;
  QVector3D position;
  double yaw = 0;
  double resolution = 0;
  double gridTimeNs = 0;
  double cloudTimeNs = 0;
  bool poseValid = false;
  QString gridFrame;
  bool hudPoseValid = false, hudVelocityValid = false, hudBatteryValid = false;
  bool fcuStateValid = false, armed = false;
  QString flightMode;
  QVector3D hudPosition;
  double rollDeg = 0, pitchDeg = 0, yawDeg = 0;
  double speedMps = 0, batteryPercent = 0;
};

// Reads the existing task-machine flight-record API; it never sends commands.
class FlightReplayDialog : public QDialog {
public:
  using Render = std::function<void(const FlightReplayFrame &)>;
  FlightReplayDialog(const QUrl &taskSocket, Render render, QWidget *parent = nullptr);

private:
  void request(const QString &path, std::function<void(const QJsonDocument &, const QString &)> done);
  void requestArchive(const QString &path,
                      std::function<void(const QByteArray &, const QString &)> done);
  void loadRemoteBundle(const QString &id, const QString &savePath);
  void listRecords();
  void loadRecord();
  void downloadReplay();
  void chooseDownloadDir();
  void openLocalReplay();
  void applyReplayData(const QJsonObject &data, const QString &id);
  void seek(int milliseconds);
  double timeNs(const QJsonObject &row) const;

  QUrl origin_;
  QNetworkAccessManager network_;
  QPointer<QNetworkReply> pending_;
  Render render_;
  QJsonArray telemetry_, setpoints_, events_, vehicles_, clouds_, grids_;
  QVector<MissionPoint> cachedCloud_;
  QVector<QVector3D> cachedGrid_;
  std::set<std::tuple<double, double, double>> gridCells_;
  int cloudIndex_ = -1, gridIndex_ = -1;
  quint64 gridVersion_ = 0;
  double gridResolution_ = 0, gridTimeNs_ = 0, cloudTimeNs_ = 0;
  QString gridFrame_;
  QListWidget *records_ = nullptr;
  QLabel *status_ = nullptr;
  QLabel *clock_ = nullptr;
  QLabel *event_ = nullptr;
  QLabel *downloadDirLabel_ = nullptr;
  QSlider *slider_ = nullptr;
  QPushButton *play_ = nullptr;
  QTimer timer_;
  QString recordId_;
  QString downloadDir_;
  double startNs_ = 0;
  int durationMs_ = 0;
  bool monotonic_ = false;
};
