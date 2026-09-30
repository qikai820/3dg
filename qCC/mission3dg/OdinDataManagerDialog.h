#pragma once

#include <QDialog>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QSaveFile>
#include <QUrl>
#include <functional>
#include <memory>

class QLabel;
class QPushButton;
class QTableWidget;
class QNetworkReply;

// Uses the same task-machine WebGroundStation API as its Odin1 data manager.
class OdinDataManagerDialog : public QDialog {
public:
  enum class InitialAction { None, Start, Stop };
  explicit OdinDataManagerDialog(const QUrl &taskWebSocket, QWidget *parent = nullptr,
                                 InitialAction initialAction = InitialAction::None);

protected:
  void reject() override;

private:
  void refresh(bool preserveNote = false);
  void changeRecording(bool start);
  void updateRecordingControls();
  void download(const QString &id, qint64 size);
  void remove(const QString &id, qint64 size);
  void requestJson(const QString &path, const QByteArray &method,
                   std::function<void(const QJsonObject &, const QString &)> done);
  QUrl apiUrl(const QString &path) const;
  static QString sizeText(qint64 bytes);

  QUrl origin_;
  QNetworkAccessManager network_;
  QTableWidget *table_ = nullptr;
  QLabel *summary_ = nullptr;
  QLabel *note_ = nullptr;
  QPushButton *refreshButton_ = nullptr;
  QPushButton *recordButton_ = nullptr;
  QPointer<QNetworkReply> downloadReply_;
  std::unique_ptr<QSaveFile> downloadFile_;
  bool deleting_ = false;
  bool recordingBusy_ = false;
  bool driverRunning_ = false;
  bool recordingActive_ = false;
  quint64 refreshGeneration_ = 0;
};
