#pragma once
#include "ProtocolClient.h"

#include <QDialog>
#include <QJsonArray>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QSaveFile>
#include <QUrl>
#include <functional>
#include <memory>

class QLabel;
class QPushButton;
class QComboBox;
class QTableWidget;
class QNetworkReply;

// Uses the same task-machine WebGroundStation API as its Odin1 data manager.
class OdinDataManagerDialog : public QDialog {
public:
  enum class InitialAction { None, Start, Stop };
  explicit OdinDataManagerDialog(const QUrl &taskWebSocket, QWidget *parent = nullptr,
                                 InitialAction initialAction = InitialAction::None,
                                 ProtocolClient *client = nullptr);
  QString selectedFlightReplayId() const { return selectedFlightReplayId_; }

protected:
  void reject() override;

private:
  enum class Category { Odin, Bag, Flight };
  void refresh(bool preserveNote = false);
  void requestFlightRecords(std::function<void(const QJsonArray &, const QString &)> done);
  void changeRecording(bool start);
  void updateRecordingControls();
  void addRecord(const QJsonObject &record, bool bag);
  void addFlightRecord(const QJsonObject &record);
  void download(const QString &id, qint64 size, Category category);
  void remove(const QString &id, qint64 size, bool bag);
  void setFlightProtected(const QString &id, bool protectedValue);
  void startReplay(const QString &bagId);
  void updateReplay(const mission::ReplayStatus &status);
  void requestJson(const QString &path, const QByteArray &method,
                   std::function<void(const QJsonObject &, const QString &)> done,
                   const QByteArray &body = {});
  QUrl apiUrl(const QString &path) const;
  static QString sizeText(qint64 bytes);

  QUrl origin_;
  QNetworkAccessManager network_;
  ProtocolClient *client_ = nullptr;
  QComboBox *category_ = nullptr;
  QTableWidget *table_ = nullptr;
  QLabel *summary_ = nullptr;
  QLabel *bagSummary_ = nullptr;
  QLabel *flightSummary_ = nullptr;
  QLabel *note_ = nullptr;
  QPushButton *refreshButton_ = nullptr;
  QPushButton *recordButton_ = nullptr;
  QString replayBagId_;
  QString replayRequest_;
  QString selectedFlightReplayId_;
  mission::ReplayStatus::State replayState_ = mission::ReplayStatus::LIVE;
  QPointer<QNetworkReply> downloadReply_;
  std::unique_ptr<QSaveFile> downloadFile_;
  bool deleting_ = false;
  bool recordingBusy_ = false;
  bool flightUpdateBusy_ = false;
  bool driverRunning_ = false;
  bool recordingActive_ = false;
  quint64 refreshGeneration_ = 0;
  int pendingLists_ = 0;
};
