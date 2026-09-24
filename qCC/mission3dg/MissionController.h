#pragma once
#include "CloudIO.h"
#include "ProtocolClient.h"
#include <QCryptographicHash>
#include <QFile>
#include <QPointer>
#include <QSaveFile>
#include <QVector3D>
#include <ccPickingListener.h>
#include <memory>
class MainWindow;
class ccGLWindowInterface;
class ccHObject;
class ccPointCloud;
class QLabel;
class QFrame;
class QPushButton;
class QAction;
class QTableWidget;
class QDoubleSpinBox;
class QListWidget;
class QMediaPlayer;
class VideoCanvas;
class QCheckBox;
class QMenuBar;
class QProgressBar;
class QToolButton;
class AttitudeWidget;
class WaypointGizmo;
class MissionController : public QObject, public ccPickingListener {
  Q_OBJECT
public:
  explicit MissionController(MainWindow *window);
  ~MissionController() override;
  void onItemPicked(const PickedItem &) override;
  bool eventFilter(QObject *, QEvent *) override;

private:
  void buildUi();
  void arrange();
  void refreshMapInfo();
  void log(const QString &, const QString &source = "3DG");
  void settings();
  void receive(const mission::Envelope &);
  void command(mission::Command::Kind);
  void setDemo(bool);
  void demoTick();
  void updateScene();
  void resetSession();
  void loadMap();
  bool loadMapPath(const QString &);
  void saveMap();
  bool exportMap(const QString &);
  void saveMission();
  void openMission();
  bool exportMission(const QString &);
  bool importMission(const QString &);
  void rebuildRoute();
  void refreshRouteStrip();
  void setEditorCollapsed(bool collapsed);
  void addWaypoint(const QVector3D &);
  void editWaypoint();
  void updateWaypointGizmo();
  void moveWaypointAlongAxis(int axis, double distance);
  void makeLine(ccHObject *, const QVector<QVector3D> &, unsigned char,
                unsigned char, unsigned char, float = 2);
  void makeCloud(ccHObject *, const QVector<MissionPoint> &);
  void makeVoxelGrid();
  void expireGrid();
  void updateVehicle(const mission::VehicleState &);
  bool frameMatches(const std::string &, const std::string &);
  void showVideo();
  void fileChunk(const mission::Envelope &);
  void requestFile();
  void smoke();
  MainWindow *window_;
  ccGLWindowInterface *gl_;
  QPointer<QWidget> canvas_;
  ccHObject *root_ = nullptr, *live_ = nullptr, *map_ = nullptr,
            *route_ = nullptr, *ego_ = nullptr, *grid_ = nullptr, *trail_ = nullptr,
            *aircraft_ = nullptr;
  ProtocolClient client_;
  QTimer demoTimer_, redraw_, staleTimer_;
  QElapsedTimer vehicleAge_, plannerAge_, statusAge_, mismatchAge_,
      downloadAge_, gridAge_;
  bool referenceMap_ = false;
  QFrame *left_ = nullptr, *right_ = nullptr, *top_ = nullptr,
         *editor_ = nullptr, *video_ = nullptr, *logPopup_ = nullptr,
         *mapPopover_ = nullptr;
  QWidget *routeDetails_ = nullptr, *routeStrip_ = nullptr;
  QLabel *hud_ = nullptr, *status_ = nullptr, *mode_ = nullptr,
         *videoState_ = nullptr, *logCount_ = nullptr, *mapInfo_ = nullptr,
         *mapBadge_ = nullptr, *protocolHint_ = nullptr,
         *routeSummary_ = nullptr;
  QProgressBar *batteryBar_ = nullptr;
  QPushButton *latest_ = nullptr;
  QPushButton *connectButton_ = nullptr;
  QToolButton *mapDetailsButton_ = nullptr;
  QToolButton *gridButton_ = nullptr;
  QPushButton *routeFold_ = nullptr;
  QListWidget *logList_ = nullptr;
  QTableWidget *waypoints_ = nullptr;
  QDoubleSpinBox *altitude_ = nullptr;
  QCheckBox *accumulate_ = nullptr, *follow_ = nullptr, *picking_ = nullptr;
  QMediaPlayer *player_ = nullptr;
  VideoCanvas *videoWidget_ = nullptr;
  AttitudeWidget *attitude_ = nullptr;
  WaypointGizmo *gizmo_ = nullptr;
  QHash<int, QPushButton *> commands_;
  QHash<QString, int> pendingKinds_;
  QHash<QString, MissionPoint> voxels_;
  QVector<MissionPoint> mapPoints_, livePoints_;
  QVector<QVector3D> history_, egoPoints_;
  QVector<QVector3D> gridCenters_;
  double gridResolution_ = .1;
  bool gridDirty_ = false;
  mission::MissionPlan mission_;
  mission::CompanionStatus lastStatus_;
  mission::VehicleState lastVehicle_;
  QString baseUrl_ = "ws://127.0.0.1:8765", videoUrl_, frame_ = "odom",
          mapId_ = "local-map";
  double voxel_ = 0.15, clearance_ = 1.0, cloudHz_ = 10;
  bool demo_ = false, dirty_ = false, ready_ = false, mapDirty_ = false;
  bool editorCollapsed_ = true;
  bool videoBeforeExpanded_ = true;
  int demoStep_ = 0, unread_ = 0, drawStep_ = 0;
  quint64 demoSequence_ = 0;
  QPoint videoDrag_;
  bool dragging_ = false;
  bool videoPlaced_ = false;
  QFile logFile_;
  std::unique_ptr<QSaveFile> download_;
  std::unique_ptr<QCryptographicHash> downloadHash_;
  QString downloadRequest_, downloadId_;
  quint64 downloadOffset_ = 0, downloadSize_ = 0;
};
