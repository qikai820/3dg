#include "CloudIO.h"
#include "ProtocolClient.h"
#include <QCoreApplication>
#include <QDateTime>
#include <QEventLoop>
#include <QFile>
#include <QTemporaryDir>
#include <QWebSocketServer>
#include <QtEndian>
#include <iostream>
#include <limits>
#define CHECK(x)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(x)) {                                                                \
      std::cerr << "FAILED line " << __LINE__ << ": " << #x << "\n";           \
      return 1;                                                                \
    }                                                                          \
  } while (false)
int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  int checks = 0;
  QTemporaryDir dir;
  QString error;
  QVector<MissionPoint> pts{{1, 2, 3, 255, 80, 12}, {-4, 5, 6, 10, 20, 30}},
      out;
  auto bytes = CloudIO::encode(pts);
  CHECK(bytes.size() == 32);
  out = CloudIO::decode(bytes, error);
  CHECK(out.size() == 2 && out[0].r == 255 && out[1].x == -4);
  CHECK(CloudIO::writePCD(dir.path() + "/map.pcd", pts, error));
  CHECK(CloudIO::readPCD(dir.path() + "/map.pcd", out, error));
  CHECK(out.size() == 2 && out[0].b == 12 && out[1].x == -4);
  QFile f(dir.path() + "/ascii.pcd");
  CHECK(f.open(QIODevice::WriteOnly));
  f.write("FIELDS z x y rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH "
          "1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n3 1 2 16711680\n");
  f.close();
  CHECK(CloudIO::readPCD(f.fileName(), out, error));
  CHECK(out.size() == 1 && out[0].r == 255 && out[0].x == 1);
  QFile compressed(dir.path() + "/compressed.pcd");
  CHECK(compressed.open(QIODevice::WriteOnly));
  compressed.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 "
                   "1\nPOINTS 2\nDATA binary_compressed\n");
  QByteArray aos = CloudIO::encode(pts), soa;
  for (int field = 0; field < 4; ++field)
    for (int point = 0; point < 2; ++point) {
      if (field == 3) {
        char rgb[4];
        qToLittleEndian<quint32>((quint32(pts[point].r) << 16) |
                                     (quint32(pts[point].g) << 8) |
                                     pts[point].b,
                                 rgb);
        soa.append(rgb, 4);
      } else
        soa.append(aos.mid(point * 16 + field * 4, 4));
    }
  QByteArray lzf(1, char(31));
  lzf += soa;
  char header[8];
  qToLittleEndian<quint32>(lzf.size(), header);
  qToLittleEndian<quint32>(soa.size(), header + 4);
  compressed.write(header, 8);
  compressed.write(lzf);
  compressed.close();
  CHECK(CloudIO::readPCD(compressed.fileName(), out, error));
  CHECK(out.size() == 2 && out[1].r == 10 && out[1].z == 6);
  CHECK(f.open(QIODevice::WriteOnly | QIODevice::Truncate));
  f.write(
      "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 999999999\nDATA binary\n");
  f.close();
  CHECK(!CloudIO::readPCD(f.fileName(), out, error));
  const QString largePcd = qEnvironmentVariable("THREEDG_TEST_LARGE_PCD");
  if (!largePcd.isEmpty()) {
    quint64 sourcePoints = 0;
    CHECK(CloudIO::readPCD(largePcd, out, error, &sourcePoints));
    CHECK(sourcePoints > 2000000 && !out.isEmpty() &&
          out.size() <= 2000000);
  }
  mission::Envelope e;
  e.set_protocol_version(1);
  e.set_session_id("test");
  e.set_sequence(1);
  e.mutable_hello()->set_name("test");
  CHECK(ProtocolClient::validate(e).isEmpty());
  e.set_protocol_version(2);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  e.set_protocol_version(1);
  auto *c = e.mutable_cloud();
  c->set_frame_id("odom");
  c->set_map_id("map");
  c->set_point_count(2);
  c->set_point_data(bytes.toStdString());
  CHECK(ProtocolClient::validate(e).isEmpty());
  c->set_point_count(3);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  auto *v = e.mutable_vehicle();
  v->set_frame_id("odom");
  v->set_map_id("map");
  v->set_pose_valid(true);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  v->mutable_orientation()->set_w(1);
  CHECK(ProtocolClient::validate(e).isEmpty());
  v->set_fcu_state_valid(true);
  v->set_armed(true);
  v->set_flight_mode("OFFBOARD");
  CHECK(ProtocolClient::validate(e).isEmpty());
  mission::Envelope decoded;
  CHECK(decoded.ParseFromString(e.SerializeAsString()));
  CHECK(decoded.vehicle().fcu_state_valid() && decoded.vehicle().armed() &&
        decoded.vehicle().flight_mode() == "OFFBOARD");
  v->set_flight_mode("OFFBOARD\nspoofed");
  CHECK(!ProtocolClient::validate(e).isEmpty());
  v->set_flight_mode("OFFBOARD");
  v->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
  CHECK(!ProtocolClient::validate(e).isEmpty());
  ProtocolClient client;
  CHECK(!client.connected());
  auto *grid = e.mutable_grid();
  grid->set_frame_id("odom");
  grid->set_map_id("map");
  grid->set_resolution_m(.1);
  grid->add_centers()->set_x(1);
  CHECK(ProtocolClient::validate(e).isEmpty());
  grid->set_resolution_m(0);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->set_resolution_m(std::numeric_limits<double>::quiet_NaN());
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->set_resolution_m(.1);
  grid->mutable_centers(0)->set_z(std::numeric_limits<double>::infinity());
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->mutable_centers(0)->set_z(0);
  grid->clear_map_id();
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->set_map_id("map");
  for (int i = 1; i <= 50000; ++i)
    grid->add_centers();
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->clear_centers();
  CHECK(ProtocolClient::validate(e).isEmpty()); // Empty snapshot clears grid.
  grid->set_delta(true);
  grid->set_base_version(1);
  grid->set_version(2);
  grid->add_added()->set_x(2);
  grid->add_removed()->set_x(1);
  CHECK(ProtocolClient::validate(e).isEmpty());
  grid->set_base_version(2);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->set_base_version(1);
  grid->add_centers()->set_x(3);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  grid->clear_centers();
  grid->mutable_added(0)->set_x(std::numeric_limits<double>::infinity());
  CHECK(!ProtocolClient::validate(e).isEmpty());
  auto *yamlDocument = e.mutable_yaml_document();
  yamlDocument->set_id("ego");
  yamlDocument->set_revision(std::string(64, 'a'));
  yamlDocument->set_content_json("{\"ego\":{\"max_vel\":0.8}}");
  yamlDocument->set_metadata_json("{\"order\":[\"/ego/max_vel\"]}");
  CHECK(ProtocolClient::validate(e).isEmpty());
  yamlDocument->set_revision("stale");
  CHECK(!ProtocolClient::validate(e).isEmpty());
  yamlDocument->set_revision(std::string(64, 'a'));
  mission::Command yamlPatch;
  yamlPatch.set_kind(mission::Command::PATCH_YAML_DOCUMENT);
  yamlPatch.set_yaml_document_id("ego");
  yamlPatch.set_yaml_revision(std::string(64, 'a'));
  yamlPatch.set_yaml_patch_json("{\"/ego/max_vel\":0.6}");
  mission::Command decodedPatch;
  CHECK(decodedPatch.ParseFromString(yamlPatch.SerializeAsString()));
  CHECK(decodedPatch.yaml_document_id() == "ego" &&
        decodedPatch.kind() == mission::Command::PATCH_YAML_DOCUMENT);
  mission::Command odinStart;
  odinStart.set_kind(mission::Command::START_ODIN);
  odinStart.mutable_odin_start()->set_mode(mission::OdinStart::RELOCALIZATION);
  odinStart.mutable_odin_start()->set_map_id(std::string(64, 'b'));
  mission::Command decodedStart;
  CHECK(decodedStart.ParseFromString(odinStart.SerializeAsString()));
  CHECK(decodedStart.has_odin_start() &&
        decodedStart.odin_start().mode() == mission::OdinStart::RELOCALIZATION);
  auto *odinMap = e.mutable_odin_maps()->add_maps();
  odinMap->set_id(std::string(64, 'b'));
  odinMap->set_name("map_20260929.bin");
  odinMap->set_size_bytes(1024);
  CHECK(ProtocolClient::validate(e).isEmpty());
  odinMap->set_id("invalid");
  CHECK(!ProtocolClient::validate(e).isEmpty());
  int messages = 0;
  QObject::connect(&client, &ProtocolClient::message,
                   [&](const mission::Envelope &) { ++messages; });
  e.mutable_heartbeat();
  client.ingest(ProtocolClient::encode(e));
  CHECK(messages == 0);
  e.mutable_hello();
  CHECK(!client.routeEditSupported());
  CHECK(!client.routeDeleteSupported());
  e.mutable_hello()->add_capabilities("route-edit-v1");
  e.mutable_hello()->add_capabilities("route-delete-v1");
  client.ingest(ProtocolClient::encode(e));
  CHECK(messages == 1);
  CHECK(client.routeEditSupported());
  CHECK(client.routeDeleteSupported());
  CHECK(!client.connected());
  mission::Command offlineYaml;
  offlineYaml.set_kind(mission::Command::GET_YAML_CATALOG);
  CHECK(client.sendCommand(offlineYaml).isEmpty());
  e.mutable_heartbeat();
  client.ingest(ProtocolClient::encode(e));
  CHECK(messages == 1);
  e.set_sequence(2);
  client.ingest(ProtocolClient::encode(e));
  CHECK(messages == 2);
  e.set_session_id("other");
  e.set_sequence(3);
  client.ingest(ProtocolClient::encode(e));
  CHECK(messages == 2);
  client.ingest(QByteArray(1, char(0xff)));
  CHECK(client.receivedMessages() == 6 && client.invalidMessages() == 4);
  if (client.receiveMbps() <= 0) {
    QEventLoop rateLoop;
    QObject::connect(&client, &ProtocolClient::linkStatsChanged, &rateLoop,
                     [&] {
                       if (client.receiveMbps() > 0)
                         rateLoop.quit();
                     });
    QTimer::singleShot(2500, &rateLoop, &QEventLoop::quit);
    rateLoop.exec();
  }
  CHECK(client.receiveMbps() > 0);
  client.stop();
  CHECK(!client.routeEditSupported());
  CHECK(!client.routeDeleteSupported());
  CHECK(client.receivedMessages() == 0 && client.invalidMessages() == 0);
  CHECK(client.receiveMbps() == 0 && client.transmitMbps() == 0);
  QWebSocketServer server("test", QWebSocketServer::NonSecureMode);
  CHECK(server.listen(QHostAddress::LocalHost, 0));
  int commands = 0, results = 0, clouds = 0, grids = 0, files = 0;
  bool latestGridReceived = false;
  quint64 sequence = 0;
  QList<QWebSocket *> peers;
  QObject::connect(&server, &QWebSocketServer::newConnection, [&] {
    auto *peer = server.nextPendingConnection();
    peers << peer;
    if (peer->requestUrl().path() == "/cloud") {
      mission::Envelope push;
      push.set_protocol_version(1);
      push.set_session_id("server");
      push.set_sequence(++sequence);
      auto *c = push.mutable_cloud();
      c->set_frame_id("odom");
      c->set_map_id("map");
      c->set_point_count(pts.size());
      c->set_point_data(bytes.toStdString());
      peer->sendBinaryMessage(ProtocolClient::encode(push));
      auto cloudPush = push;
      auto *grid = push.mutable_grid();
      grid->set_frame_id("odom");
      grid->set_map_id("map");
      grid->set_resolution_m(.2);
      grid->add_centers()->set_x(1);
      push.set_sequence(++sequence);
      peer->sendBinaryMessage(ProtocolClient::encode(push));
      cloudPush.set_sequence(++sequence);
      peer->sendBinaryMessage(ProtocolClient::encode(cloudPush));
      grid->mutable_centers(0)->set_x(2);
      push.set_sequence(++sequence);
      peer->sendBinaryMessage(ProtocolClient::encode(push));
      // A duplicate sequence must not replace the newer accepted snapshot.
      grid->mutable_centers(0)->set_x(99);
      peer->sendBinaryMessage(ProtocolClient::encode(push));
    }
    if (peer->requestUrl().path() == "/files") {
      mission::Envelope push;
      push.set_protocol_version(1);
      push.set_session_id("server");
      push.set_sequence(++sequence);
      auto *f = push.mutable_file();
      f->set_file_id("test");
      f->set_total_size(3);
      f->set_data("pcd");
      peer->sendBinaryMessage(ProtocolClient::encode(push));
    }
    QObject::connect(peer, &QWebSocket::binaryMessageReceived,
                     [&, peer](const QByteArray &b) {
                       mission::Envelope request;
                       if (!request.ParseFromArray(b.data(), b.size()))
                         return;
                       mission::Envelope reply;
                       reply.set_protocol_version(1);
                       reply.set_session_id("server");
                       reply.set_sequence(++sequence);
                       if (request.has_hello())
                         reply.mutable_hello()->set_name("test server");
                       else if (request.has_command()) {
                         ++commands;
                         reply.set_request_id(request.request_id());
                         reply.mutable_result()->set_state(
                             mission::CommandResult::SUCCEEDED);
                       } else
                         reply.mutable_heartbeat();
                       peer->sendBinaryMessage(ProtocolClient::encode(reply));
                     });
  });
  QEventLoop loop;
  QObject::connect(&client, &ProtocolClient::stateChanged,
                   [&](const QString &, bool ready) {
                     if (ready) {
                       mission::Command cmd;
                       cmd.set_kind(mission::Command::START_ODIN);
                       client.sendCommand(cmd);
                     }
                   });
  QObject::connect(&client, &ProtocolClient::message,
                   [&](const mission::Envelope &r) {
                     if (r.has_result()) {
                       ++results;
                     }
                     if (r.has_cloud())
                       ++clouds;
                     if (r.has_grid()) {
                       ++grids;
                       latestGridReceived = r.grid().centers_size() == 1 &&
                                            r.grid().centers(0).x() == 2;
                     }
                     if (r.has_file())
                       ++files;
                     if (results && clouds && latestGridReceived && files)
                       loop.quit();
                   });
  client.open(QUrl(QString("ws://127.0.0.1:%1").arg(server.serverPort())));
  QTimer::singleShot(3000, &loop, &QEventLoop::quit);
  loop.exec();
  CHECK(commands == 1 && results == 1);
  CHECK(clouds == 1 && files == 1);
  CHECK(grids == 2 && latestGridReceived);
  auto *file = e.mutable_file();
  file->set_total_size(5);
  file->set_offset(6);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  auto *traj = e.mutable_trajectory();
  traj->set_frame_id("odom");
  traj->set_map_id("map");
  traj->add_points()->set_time_from_start_s(1);
  traj->add_points()->set_time_from_start_s(0);
  CHECK(!ProtocolClient::validate(e).isEmpty());
  client.stop();
  qDeleteAll(peers);
  // Exercise automatic clock correction through the real WebSocket client.
  QWebSocketServer timeServer("clock-test", QWebSocketServer::NonSecureMode);
  CHECK(timeServer.listen(QHostAddress::LocalHost, 0));
  QList<QWebSocket *> timePeers;
  int probes = 0, syncs = 0;
  bool targetCurrent = false;
  qint64 clockOffsetMs = 20000;
  quint64 timeSequence = 0;
  QObject::connect(&timeServer, &QWebSocketServer::newConnection, [&] {
    auto *peer = timeServer.nextPendingConnection();
    timePeers << peer;
    QObject::connect(peer, &QWebSocket::binaryMessageReceived,
                     [&, peer](const QByteArray &b) {
                       mission::Envelope request;
                       if (!request.ParseFromArray(b.data(), b.size()) ||
                           peer->requestUrl().path() != "/control")
                         return;
                       mission::Envelope reply;
                       reply.set_protocol_version(1);
                       reply.set_session_id("clock-server");
                       reply.set_sequence(++timeSequence);
                       reply.set_request_id(request.request_id());
                       reply.set_time_domain("unix");
                       reply.set_timestamp_ns(quint64(QDateTime::currentMSecsSinceEpoch() +
                                                     clockOffsetMs) * 1000000);
                       if (request.has_hello()) {
                         reply.mutable_hello()->add_capabilities("time-sync-v1");
                       } else if (request.has_command()) {
                         reply.mutable_result()->set_state(
                             mission::CommandResult::SUCCEEDED);
                         if (request.command().kind() == mission::Command::GET_TIME)
                           ++probes;
                         if (request.command().kind() == mission::Command::SET_TIME) {
                           ++syncs;
                           targetCurrent = std::abs(qint64(request.command().unix_time_ns() /
                                       1000000) - QDateTime::currentMSecsSinceEpoch()) < 2000;
                           clockOffsetMs = 0;
                         }
                       } else {
                         reply.mutable_heartbeat();
                       }
                       peer->sendBinaryMessage(ProtocolClient::encode(reply));
                     });
  });
  ProtocolClient timeClient;
  QEventLoop timeLoop;
  QObject::connect(&timeClient, &ProtocolClient::diagnostic,
                   [&](const QString &text) {
                     if (text.contains("无需校时"))
                       timeLoop.quit();
                   });
  timeClient.open(QUrl(QString("ws://127.0.0.1:%1").arg(timeServer.serverPort())));
  QTimer::singleShot(3000, &timeLoop, &QEventLoop::quit);
  timeLoop.exec();
  CHECK(probes >= 2 && syncs == 1 && targetCurrent);
  timeClient.stop();
  clockOffsetMs = 5000;
  timeClient.open(QUrl(QString("ws://127.0.0.1:%1").arg(timeServer.serverPort())));
  QTimer::singleShot(3000, &timeLoop, &QEventLoop::quit);
  timeLoop.exec();
  CHECK(probes >= 3 && syncs == 1);
  timeClient.stop();
  qDeleteAll(timePeers);
  std::cout << checks
            << " checks passed (PCD ASCII/binary/LZF, malformed data, "
               "session/sequence, WebSocket control/cloud/files)\n";
  return 0;
}
