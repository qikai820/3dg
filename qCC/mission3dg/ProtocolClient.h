#pragma once
#ifdef ERROR
#undef ERROR // Windows headers define ERROR and collide with LogEntry::ERROR.
#endif
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
  bool routeEditSupported() const { return routeEditSupported_; }
  bool routeDeleteSupported() const { return routeDeleteSupported_; }
  quint64 receivedMessages() const { return receivedMessages_; }
  quint64 invalidMessages() const { return invalidMessages_; }
  double receiveMbps() const { return receiveMbps_; }
  double transmitMbps() const { return transmitMbps_; }
  QString sendCommand(const mission::Command &command);
  void requestGridResync();
  void ingest(const QByteArray &bytes, int channel = 0);
  static QString validate(const mission::Envelope &e);
  static QByteArray encode(const mission::Envelope &e);
Q_SIGNALS:
  void message(const mission::Envelope &e);
  void stateChanged(const QString &state, bool ready);
  void diagnostic(const QString &text);
  void sessionChanged();
  void linkStatsChanged();

private:
  void connectSockets();
  void transmit(mission::Envelope e);
  void probeTime();
  void resetTimeSync();
  void handleTimeResult(const mission::Envelope &e);
  void resetLinkStats();
  QWebSocket control_, cloud_, files_;
  QTimer heartbeat_, reconnect_, cloudDispatch_, linkStatsTimer_;
  QElapsedTimer lastRx_;
  QElapsedTimer linkStatsAge_;
  QElapsedTimer timeProbeAge_, lastTimeProbe_, lastTimeSync_;
  QHash<int, quint64> sequences_;
  QHash<QString, qint64> pending_;
  mission::Envelope latestCloud_;
  QUrl base_;
  QString remoteSession_, localSession_;
  QString timeProbeRequest_, setTimeRequest_;
  qint64 timeProbeSentMs_ = 0;
  quint64 sequence_ = 0;
  quint64 receivedMessages_ = 0, invalidMessages_ = 0;
  quint64 receivedBytes_ = 0, transmittedBytes_ = 0;
  quint64 previousReceivedBytes_ = 0, previousTransmittedBytes_ = 0;
  double receiveMbps_ = 0, transmitMbps_ = 0;
  bool desired_ = false, ready_ = false;
  bool timeSyncSupported_ = false, timeChecked_ = false;
  bool routeEditSupported_ = false;
  bool routeDeleteSupported_ = false;
};
