#pragma once
#include "CloudIO.h"
#include "ProtocolClient.h"
#include <QCryptographicHash>
#include <QColor>
#include <QFile>
#include <QJsonArray>
#include <QPointer>
#include <QSaveFile>
#include <QSize>
#include <QVector3D>
#include <ccPickingListener.h>
#include <memory>
class MainWindow;
class ccGLWindowInterface;
class ccHObject;
class ccPointCloud;
class ccMesh;
class QLabel;
class QDialog;
class QFrame;
class QPushButton;
class QAction;
class QTableWidget;
class QDoubleSpinBox;
class QListWidget;
class QComboBox;
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
  enum class Workspace { Monitor, Processing, Comparison };
  void setWorkspace(Workspace workspace);
  void refreshProcessingObjects();
  void selectComparisonPair();
  void importProcessingFiles();
  void snapshotMap();
  void runCloudCompareAction(const char *objectName);
  void refreshMapInfo();
  void refreshStartupButtons();
  void refreshMediaButtons();
  void toggleMedia(mission::Command::Kind start);
  void log(const QString &, const QString &source = "3DG");
  void settings();
  void yamlSettings();
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
  QString accumulatedMapPath() const;
  bool saveAccumulatedMap();
  bool restoreAccumulatedMap();
  void clearAccumulatedMap();
  void saveMission();
  void openMission();
  bool exportMission(const QString &);
  bool importMission(const QString &);
  void rebuildRoute();
  void requestRemoteRoute();
  void receiveRemoteRoute(const mission::YamlDocument &);
  void clearRemoteRoute();
  void refreshWaypointTable();
  void refreshRouteStrip();
  void setEditorCollapsed(bool collapsed);
  void setToolbarCollapsed(bool collapsed);
  void addWaypoint(const QVector3D &);
  int addWaypointAtAircraft();
  void editWaypoint();
  void updateWaypointGizmo();
  void moveWaypointAlongAxis(int axis, double distance);
  void makeLine(ccHObject *, const QVector<QVector3D> &, unsigned char,
                unsigned char, unsigned char, float = 2);
  void makeCloud(ccHObject *, const QVector<MissionPoint> &);
  void makeVoxelGrid();
  void expireGrid();
  void updateVehicle(const mission::VehicleState &);
  void updateAircraftPose();
  void focusAircraft();
  bool frameMatches(const std::string &, const std::string &);
  void showVideo();
  void fileChunk(const mission::Envelope &);
  void requestFile();
  void smoke();
  MainWindow *window_;
  ccGLWindowInterface *gl_;
  QPointer<QWidget> canvas_;
  ccHObject *root_ = nullptr, *live_ = nullptr, *map_ = nullptr,
            *route_ = nullptr, *remoteRoute_ = nullptr, *ego_ = nullptr,
            *grid_ = nullptr, *trail_ = nullptr,
            *aircraft_ = nullptr;
  ccMesh *aircraftModel_ = nullptr; // Owned by aircraft_; mesh is loaded once.
  ProtocolClient client_;
  QTimer demoTimer_, redraw_, staleTimer_, autosaveTimer_;
  QElapsedTimer vehicleAge_, plannerAge_, statusAge_, mismatchAge_,
      downloadAge_, gridAge_;
  bool referenceMap_ = false;
  QFrame *left_ = nullptr, *right_ = nullptr, *top_ = nullptr,
         *editor_ = nullptr, *video_ = nullptr, *logPopup_ = nullptr,
         *mapPopover_ = nullptr, *objectsCard_ = nullptr,
         *processingCard_ = nullptr, *processingToolbar_ = nullptr;
  QWidget *processingOnly_ = nullptr, *comparisonOnly_ = nullptr;
  QListWidget *processingObjects_ = nullptr;
  QComboBox *referenceObject_ = nullptr, *targetObject_ = nullptr;
  QLabel *processingHint_ = nullptr;
  Workspace workspace_ = Workspace::Monitor;
  QHash<int, QAction *> workspaceActions_;
  QWidget *toolbarContents_ = nullptr;
  QPushButton *toolbarFold_ = nullptr;
  QWidget *routeDetails_ = nullptr, *routeStrip_ = nullptr;
  QDialog *routeDialog_ = nullptr;
  QLabel *hud_ = nullptr, *status_ = nullptr, *mode_ = nullptr,
         *logCount_ = nullptr, *mapInfo_ = nullptr,
         *mapBadge_ = nullptr, *protocolHint_ = nullptr,
         *routeSummary_ = nullptr, *remoteRouteInfo_ = nullptr;
  QProgressBar *batteryBar_ = nullptr;
  QPushButton *latest_ = nullptr;
  QPushButton *connectButton_ = nullptr;
  QPushButton *logRecordButton_ = nullptr;
  QLabel *logRecordState_ = nullptr;
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
  struct MediaOperation {
    QString request;
    bool target = false, succeeded = false;
    QElapsedTimer age;
  };
  QHash<int, MediaOperation> mediaOperations_;
  QHash<int, QString> mediaErrors_;
  QHash<int, QLabel *> mediaStates_;
  QHash<QString, MissionPoint> voxels_;
  QVector<MissionPoint> mapPoints_, livePoints_;
  QVector<QVector3D> history_, egoPoints_;
  QVector<QVector3D> gridCenters_, remotePoints_;
  QJsonArray remoteEntries_;
  QString remoteRouteRequest_, remoteRouteSource_;
  double gridResolution_ = .1;
  QColor gridColor_ = QColor(235, 169, 86);
  double gridOpacity_ = 1.0;
  bool gridDirty_ = false;
  mission::MissionPlan mission_;
  mission::CompanionStatus lastStatus_;
  mission::VehicleState lastVehicle_;
  QString baseUrl_ = "ws://127.0.0.1:8765", videoUrl_, frame_ = "odom",
          mapId_ = "local-map";
  double voxel_ = 0.15, clearance_ = 1.0, cloudHz_ = 10;
  bool demo_ = false, dirty_ = false, ready_ = false, mapDirty_ = false;
  bool accumulationDirty_ = false;
  bool autosavePathLogged_ = false;
  bool liveDirty_ = false, trailDirty_ = false, egoDirty_ = false;
  bool editorCollapsed_ = true;
  bool toolbarCollapsed_ = false;
  int demoStep_ = 0, unread_ = 0, drawStep_ = 0;
  quint64 demoSequence_ = 0;
  QPoint videoDrag_;
  QPoint videoResizeMouse_;
  QPoint videoResizeBottomLeft_;
  QSize videoResizeStart_;
  bool dragging_ = false;
  bool resizingVideo_ = false;
  bool videoPlaced_ = false;
  QFile logFile_;
  std::unique_ptr<QSaveFile> download_;
  std::unique_ptr<QCryptographicHash> downloadHash_;
  QString downloadRequest_, downloadId_;
  quint64 downloadOffset_ = 0, downloadSize_ = 0;
};
