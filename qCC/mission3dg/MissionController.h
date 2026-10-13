#pragma once
#include "CloudIO.h"
#include "ProtocolClient.h"
#include <QCryptographicHash>
#include <QColor>
#include <QElapsedTimer>
#include <QFile>
#include <QHash>
#include <QJsonArray>
#include <QJsonObject>
#include <QPointer>
#include <QSaveFile>
#include <QSet>
#include <QSize>
#include <QStringList>
#include <QVector>
#include <QVector3D>
#include <ccPickingListener.h>
#include <memory>
#include <set>
#include <tuple>
#include <vector>
class MainWindow;
class ccGLWindowInterface;
class ccHObject;
class ccPointCloud;
class ccMesh;
class QLabel;
class QDialog;
class QButtonGroup;
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
class QSlider;
class QToolButton;
class QTabWidget;
class QScrollArea;
class AttitudeWidget;
class WaypointGizmo;
class FlightReplayDialog;
class BagReplayToolbar;
struct FlightReplayFrame;
class MissionController : public QObject, public ccPickingListener {
  Q_OBJECT
public:
  explicit MissionController(MainWindow *window);
  ~MissionController() override;
  void onItemPicked(const PickedItem &) override;
  bool eventFilter(QObject *, QEvent *) override;

private:
  void buildUi();
  void placeLatestLogButton();
  void arrange();
  bool selectCurrentCloud();
  void refreshProcessingObjects();
  QVector<ccHObject *> selectedProcessingObjects() const;
  void showProcessingObjectMenu(const QPoint &position);
  void copyProcessingObjects();
  void pasteProcessingObjects();
  void renameProcessingObject();
  void deleteProcessingObjects();
  bool selectComparisonPair();
  void importProcessingFiles();
  ccPointCloud *captureSceneCloud(const QString &name, bool visible = true);
  void snapshotMap();
  void quickReconstruct();
  void runCloudCompareAction(const char *objectName);
  void configureNoiseDetection();
  void detectCloudNoise();
  void showCloudNoise(bool onlyNoise);
  void restoreCloudNoiseView();
  void refreshMapInfo();
  void refreshLinkStats();
  void resetHud(const QString &message, const QString &context = {});
  void updateFlightTotals(const mission::VehicleState &state);
  void showFlightTotals();
  void refreshStartupButtons();
  void refreshTaskModeControls();
  void switchTaskMode(mission::TaskMode mode);
  void refreshMediaButtons();
  bool localVideoRequested() const;
  bool localVideoPlaying() const;
  bool controlsLocalVideo() const;
  void toggleMedia(mission::Command::Kind start);
  void log(const QString &, const QString &source = "3DG");
  void settings();
  void yamlSettings();
  void openOdinStart();
  void requestOdinMaps();
  void refreshOdinStartDialog();
  void receive(const mission::Envelope &);
  void command(mission::Command::Kind);
  void setDemo(bool);
  void demoTick();
  void updateScene();
  void refreshHeightFilterRange();
  void refreshHeightFilterLabel();
  void resetSession();
  void applyBagReplayStatus(const mission::ReplayStatus &);
  void loadMap();
  bool loadMapPath(const QString &);
  void saveMap();
  bool exportMap(const QString &);
  QString accumulatedMapPath(bool colored = false) const;
  bool saveAccumulatedMap();
  bool restoreAccumulatedMap();
  void clearAccumulatedMap();
  void receiveColorCloud(const QVector<MissionPoint> &points, bool snapshot);
  void saveMission();
  void openMission();
  bool exportMission(const QString &);
  bool importMission(const QString &);
  void rebuildRoute();
  void requestRemoteRoute();
  void loadOnboardRoute();
  void receiveRemoteRoute(const mission::YamlDocument &);
  void clearRemoteRoute();
  void toggleRemoteEdit();
  void editRemoteWaypoint(int row);
  void deleteRemoteWaypoint();
  void saveRemoteRoute();
  void refreshRemoteEditControls();
  void refreshRemoteRoutePreview();
  void refreshWaypointTable();
  void refreshRouteStrip();
  void setEditorCollapsed(bool collapsed);
  void setToolbarCollapsed(bool collapsed);
  void addWaypoint(const QVector3D &);
  void createWaypoint();
  bool editWaypoint(bool manualCoordinates = false);
  void updateWaypointGizmo();
  void moveWaypointAlongAxis(int axis, double distance);
  void makeLine(ccHObject *, const QVector<QVector3D> &, unsigned char,
                unsigned char, unsigned char, float = 2);
  void makeCloud(ccHObject *, const QVector<MissionPoint> &,
                 bool usePointColors = true, bool filterHeight = false);
  void makeVoxelGrid();
  void makeVoxelGrid(ccHObject *target, const QVector<QVector3D> &centers,
                     double resolution);
  void renderFlightReplay(const FlightReplayFrame &frame);
  void clearFlightReplay();
  void expireGrid();
  void updateVehicle(const mission::VehicleState &, bool countFlight = true);
  void setStatusCells(const QStringList &values);
  void updateAircraftPose();
  void updateOdinSensorPose(const mission::OdinSensorPose &);
  void updateOdinProjection(const mission::OdinProjection &);
  void refreshOdinOverlayStatus();
  void rebuildOdinGeometry();
  void clearOdinOverlays();
  void addOverlayLine(ccHObject *, const QVector<QVector3D> &,
                      unsigned char, unsigned char, unsigned char, float = 2);
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
            *colorMap_ = nullptr,
            *route_ = nullptr, *remoteRoute_ = nullptr, *ego_ = nullptr,
            *grid_ = nullptr, *trail_ = nullptr,
            *aircraft_ = nullptr;
  ccHObject *cameraFov_ = nullptr, *lidarFov_ = nullptr,
            *imageProjection_ = nullptr, *aircraftInflation_ = nullptr;
  ccMesh *aircraftModel_ = nullptr; // Owned by aircraft_; mesh is loaded once.
  ccHObject *replay_ = nullptr, *replayTrail_ = nullptr,
            *replaySetpoints_ = nullptr, *replayGrid_ = nullptr,
            *replayCloud_ = nullptr;
  ccMesh *replayAircraft_ = nullptr;
  QHash<ccHObject *, bool> replayVisibility_;
  double replayGridTimeNs_ = 0;
  double replayCloudTimeNs_ = 0;
  ProtocolClient client_;
  QTimer demoTimer_, redraw_, staleTimer_, autosaveTimer_, videoRetry_;
  quint64 videoRetryFrameCount_ = 0;
  QElapsedTimer vehicleAge_, plannerAge_, statusAge_, mismatchAge_,
      downloadAge_, gridAge_, sensorAge_, projectionAge_;
  mission::OdinSensorPose lastSensorPose_;
  QVector<MissionPoint> projectionPoints_;
  bool referenceMap_ = false;
  quint64 mapCloudVersion_ = 0, liveCloudVersion_ = 0;
  quint64 automaticCloudVersion_ = 0;
  unsigned automaticCloudId_ = 0;
  bool automaticCloudFromMap_ = false;
  unsigned noiseSourceId_ = 0, noiseResultId_ = 0;
  int noiseKnn_ = 6, noisePointSize_ = 6;
  double noiseSigma_ = 1.0;
  bool noiseViewActive_ = false;
  QHash<unsigned, bool> noiseDbVisibility_;
  QHash<ccHObject *, bool> noiseSceneVisibility_;
  QAction *noiseOverlayAction_ = nullptr, *noiseOnlyAction_ = nullptr,
          *noiseRestoreAction_ = nullptr;
  QFrame *left_ = nullptr, *right_ = nullptr, *top_ = nullptr,
         *editor_ = nullptr, *video_ = nullptr, *logPopup_ = nullptr,
         *mapPopover_ = nullptr, *objectsCard_ = nullptr,
         *processingCard_ = nullptr,
         *heightFilterPanel_ = nullptr;
  QWidget *processingOnly_ = nullptr, *comparisonOnly_ = nullptr;
  QWidget *objectsContent_ = nullptr;
  QTabWidget *toolsTabs_ = nullptr;
  QScrollArea *detailsScroll_ = nullptr;
  QListWidget *processingObjects_ = nullptr;
  QComboBox *referenceObject_ = nullptr, *targetObject_ = nullptr;
  QLabel *processingHint_ = nullptr;
  QLabel *heightFilterTitle_ = nullptr, *heightFilterLabel_ = nullptr,
         *heightRangeTop_ = nullptr, *heightRangeBottom_ = nullptr;
  QSlider *heightSlider_ = nullptr;
  QToolButton *heightFilterButton_ = nullptr;
  QToolButton *objectsButton_ = nullptr;
  QToolButton *objectsFold_ = nullptr;
  QWidget *objectsResizeGrip_ = nullptr;
  QSize objectsExpandedSize_{190, 360};
  QSize objectsResizeStart_;
  QPoint objectsResizePress_;
  bool objectsResizing_ = false;
  std::vector<std::unique_ptr<ccHObject>> objectClipboard_;
  QCheckBox *heightFilterEnabled_ = nullptr;
  double heightFilterMin_ = 0, heightFilterMax_ = 0,
         heightFilterCutoff_ = 0;
  bool heightFilterHasRange_ = false;
  QPoint heightFilterDrag_;
  bool heightFilterDragging_ = false, heightFilterPlaced_ = false;
  QWidget *toolbarContents_ = nullptr;
  BagReplayToolbar *bagReplayToolbar_ = nullptr;
  QPushButton *toolbarFold_ = nullptr;
  QWidget *routeDetails_ = nullptr, *routeStrip_ = nullptr;
  QDialog *routeDialog_ = nullptr;
  QPointer<QDialog> odinStartDialog_;
  QPointer<FlightReplayDialog> replayDialog_;
  bool bagReplayActive_ = false;
  mission::ReplayStatus::State bagReplayState_ = mission::ReplayStatus::LIVE;
  quint64 bagReplayEpoch_ = 0;
  QPointer<QButtonGroup> odinModeGroup_;
  QPointer<QListWidget> odinMapList_;
  QPointer<QLabel> odinMapHint_;
  QPointer<QPushButton> odinStartButton_;
  QString odinMapsRequest_;
  bool odinModesSupported_ = false;
  bool taskModeSupported_ = false;
  mission::TaskMode taskMode_ = mission::MODE_UNKNOWN;
  QString modeSwitchRequest_;
  QAction *taskModeStatus_ = nullptr, *realModeAction_ = nullptr,
          *simModeAction_ = nullptr;
  QLabel *mode_ = nullptr,
         *logCount_ = nullptr, *mapInfo_ = nullptr,
         *mapBadge_ = nullptr, *protocolHint_ = nullptr,
         *bitError_ = nullptr, *protocolError_ = nullptr,
         *networkDown_ = nullptr, *networkUp_ = nullptr,
         *routeStats_ = nullptr,
         *remoteRouteInfo_ = nullptr;
  QVector<QLabel *> statusCells_;
  QPushButton *latest_ = nullptr;
  QPushButton *connectButton_ = nullptr;
  QAction *demoAction_ = nullptr, *settingsMenuAction_ = nullptr;
  QHash<QObject *, QAction *> cardVisibilityActions_;
  QPushButton *logRecordButton_ = nullptr;
  QLabel *logRecordState_ = nullptr;
  QToolButton *mapDetailsButton_ = nullptr;
  QToolButton *gridButton_ = nullptr;
  QToolButton *ordinaryMapButton_ = nullptr, *colorMapButton_ = nullptr;
  QPushButton *remoteEditButton_ = nullptr, *remoteSaveButton_ = nullptr,
              *remoteDeleteButton_ = nullptr, *remoteRefreshButton_ = nullptr;
  QListWidget *logList_ = nullptr;
  QTableWidget *waypoints_ = nullptr;
  QDoubleSpinBox *altitude_ = nullptr;
  QCheckBox *accumulate_ = nullptr, *follow_ = nullptr, *picking_ = nullptr;
  QCheckBox *cameraFovToggle_ = nullptr, *lidarFovToggle_ = nullptr,
            *projectionToggle_ = nullptr, *inflationToggle_ = nullptr;
  QLabel *sensorOverlayStatus_ = nullptr;
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
  QHash<QString, MissionPoint> voxels_, colorVoxels_;
  QVector<MissionPoint> mapPoints_, livePoints_, colorMapPoints_;
  QVector<QVector3D> history_, egoPoints_;
  QVector<QVector3D> gridCenters_, remotePoints_;
  using GridCell = std::tuple<double, double, double>;
  std::set<GridCell> gridCells_;
  quint64 gridVersion_ = 0;
  bool gridInflated_ = false;
  QJsonArray remoteEntries_;
  QHash<int, QJsonObject> remoteDrafts_;
  QSet<int> remoteDeleted_;
  QString remoteRouteRequest_, remoteRoutePatchRequest_, remoteRouteSource_,
          remoteRouteRevision_;
  bool remoteEditMode_ = false;
  bool routeSyncedToOnboard_ = false;
  bool importRemoteOnReceive_ = false;
  bool importRemoteAsUploaded_ = false;
  quint32 routeImportMissionRevision_ = 0;
  double gridResolution_ = .1;
  QColor gridColor_ = QColor(235, 169, 86);
  double gridOpacity_ = 1.0;
  int gridSizePercent_ = 100;
  bool gridDirty_ = false;
  mission::MissionPlan mission_;
  mission::CompanionStatus lastStatus_;
  mission::VehicleState lastVehicle_;
  QElapsedTimer flightSampleAge_;
  QVector3D flightSamplePosition_;
  QString flightSampleFrame_, flightSampleMap_;
  double flightSeconds_ = 0, flightDistanceM_ = 0;
  QString baseUrl_ = "ws://192.168.144.50:8765", videoUrl_, frame_ = "odom",
          mapId_ = "local-map";
  QString connectionState_ = "未连接";
  QString videoUrlBeforeAgent_;
  bool videoUrlFromAgent_ = false;
  double voxel_ = 0.15, clearance_ = 1.0, cloudHz_ = 10;
  bool demo_ = false, dirty_ = false, ready_ = false, mapDirty_ = false;
  bool accumulationDirty_ = false, colorAccumulationDirty_ = false;
  bool autosavePathLogged_ = false, colorAutosavePathLogged_ = false;
  int colorDrawStep_ = 0;
  bool liveDirty_ = false, colorMapDirty_ = false,
       trailDirty_ = false, egoDirty_ = false;
  bool toolbarCollapsed_ = false;
  int demoStep_ = 0, unread_ = 0, drawStep_ = 0;
  quint64 demoSequence_ = 0;
  QPoint videoDrag_;
  QPoint videoResizeMouse_;
  QPoint videoResizeBottomLeft_;
  QSize videoResizeStart_;
  QSize videoFrameSize_ = QSize(16, 9);
  bool dragging_ = false;
  bool resizingVideo_ = false;
  bool videoPlaced_ = false;
  QFile logFile_;
  std::unique_ptr<QSaveFile> download_;
  std::unique_ptr<QCryptographicHash> downloadHash_;
  QString downloadRequest_, downloadId_;
  quint64 downloadOffset_ = 0, downloadSize_ = 0;
};
