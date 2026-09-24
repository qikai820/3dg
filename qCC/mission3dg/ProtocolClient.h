#pragma once
#include "mission.pb.h"
#include <QElapsedTimer>
#include <QHash>
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <QWebSocket>

class ProtocolClient : public QObject {
  Q_OBJECT
public:
  explicit ProtocolClient(QObject *parent = nullptr);
  void open(const QUrl &base);
  void stop();
  bool ready() const { return ready_; }
  bool connected() const {
    return ready_ && control_.state() == QAbstractSocket::ConnectedState;
  }
  QString sendCommand(const mission::Command &command);
  void ingest(const QByteArray &bytes, int channel = 0);
  static QString validate(const mission::Envelope &e);
  static QByteArray encode(const mission::Envelope &e);
Q_SIGNALS:
  void message(const mission::Envelope &e);
  void stateChanged(const QString &state, bool ready);
  void diagnostic(const QString &text);
  void sessionChanged();

private:
  void connectSockets();
  void transmit(mission::Envelope e);
  QWebSocket control_, cloud_, files_;
  QTimer heartbeat_, reconnect_, cloudDispatch_;
  QElapsedTimer lastRx_;
  QHash<int, quint64> sequences_;
  QHash<QString, qint64> pending_;
  mission::Envelope latestCloud_, latestGrid_;
  QUrl base_;
  QString remoteSession_, localSession_;
  quint64 sequence_ = 0;
  bool desired_ = false, ready_ = false;
};
