#include "ProtocolClient.h"

#include <QApplication>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>

namespace {
QString fromWire(const std::string &value) {
  return QString::fromStdString(value);
}

class AndroidWindow final : public QWidget {
public:
  AndroidWindow() : client_(this, false) {
    setWindowTitle(QStringLiteral("3DG 任务监视"));
    auto *layout = new QVBoxLayout(this);
    layout->setSpacing(12);

    auto *title = new QLabel(QStringLiteral("3DG 任务监视"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(titleFont.pointSize() + 4);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *connection = new QHBoxLayout;
    endpoint_ = new QLineEdit(this);
    endpoint_->setPlaceholderText(QStringLiteral("ws://任务机地址:8765"));
    QSettings settings(QStringLiteral("3DG"), QStringLiteral("Android"));
    endpoint_->setText(settings.value(QStringLiteral("endpoint"),
                                      QStringLiteral("ws://192.168.144.50:8765"))
                           .toString());
    connection->addWidget(endpoint_, 1);
    button_ = new QPushButton(QStringLiteral("连接"), this);
    connection->addWidget(button_);
    layout->addLayout(connection);

    state_ = addLine(layout, QStringLiteral("连接：未连接"));
    vehicle_ = addLine(layout, QStringLiteral("飞行器：等待数据"));
    companion_ = addLine(layout, QStringLiteral("任务机：等待数据"));
    map_ = addLine(layout, QStringLiteral("地图：等待数据"));
    trajectory_ = addLine(layout, QStringLiteral("轨迹：等待数据"));
    video_ = addLine(layout, QStringLiteral("视频：等待数据"));
    link_ = addLine(layout, QStringLiteral("链路：等待数据"));

    auto *logTitle = new QLabel(QStringLiteral("诊断"), this);
    layout->addWidget(logTitle);
    diagnostics_ = new QPlainTextEdit(this);
    diagnostics_->setReadOnly(true);
    diagnostics_->document()->setMaximumBlockCount(100);
    layout->addWidget(diagnostics_, 1);

    connect(button_, &QPushButton::clicked, this, [this] {
      if (connecting_) {
        client_.stop();
        connecting_ = false;
        button_->setText(QStringLiteral("连接"));
        return;
      }
      const QUrl url(endpoint_->text().trimmed());
      if (!url.isValid() || (url.scheme() != QStringLiteral("ws") &&
                             url.scheme() != QStringLiteral("wss")) ||
          url.host().isEmpty() || url.port() < 1) {
        appendDiagnostic(QStringLiteral("请输入有效的 ws:// 或 wss:// 地址和端口"));
        return;
      }
      QSettings(QStringLiteral("3DG"), QStringLiteral("Android"))
          .setValue(QStringLiteral("endpoint"), url.toString());
      client_.open(url);
      connecting_ = true;
      button_->setText(QStringLiteral("断开"));
    });
    connect(&client_, &ProtocolClient::stateChanged, this,
            [this](const QString &state, bool ready) {
              state_->setText(QStringLiteral("连接：") + state);
              if (!ready) {
                vehicle_->setText(QStringLiteral("飞行器：等待数据"));
                companion_->setText(QStringLiteral("任务机：等待数据"));
                map_->setText(QStringLiteral("地图：等待数据"));
                trajectory_->setText(QStringLiteral("轨迹：等待数据"));
                video_->setText(QStringLiteral("视频：等待数据"));
              }
            });
    connect(&client_, &ProtocolClient::diagnostic, this,
            [this](const QString &text) { appendDiagnostic(text); });
    connect(&client_, &ProtocolClient::linkStatsChanged, this, [this] {
      link_->setText(QStringLiteral("链路：接收 %1 Mbps，发送 %2 Mbps")
                         .arg(client_.receiveMbps(), 0, 'f', 2)
                         .arg(client_.transmitMbps(), 0, 'f', 2));
    });
    connect(&client_, &ProtocolClient::message, this,
            [this](const mission::Envelope &message) { receive(message); });
  }

private:
  static QLabel *addLine(QVBoxLayout *layout, const QString &text) {
    auto *label = new QLabel(text);
    label->setWordWrap(true);
    layout->addWidget(label);
    return label;
  }

  void appendDiagnostic(const QString &text) {
    diagnostics_->appendPlainText(text);
  }

  void receive(const mission::Envelope &message) {
    if (message.has_vehicle()) {
      const auto &v = message.vehicle();
      QString pose = QStringLiteral("位置无效");
      if (v.pose_valid())
        pose = QStringLiteral("%1, %2, %3 m")
                   .arg(v.position().x(), 0, 'f', 1)
                   .arg(v.position().y(), 0, 'f', 1)
                   .arg(v.position().z(), 0, 'f', 1);
      QString flight = v.fcu_state_valid()
                           ? QStringLiteral("%1 / %2")
                                 .arg(v.armed() ? QStringLiteral("已解锁")
                                                : QStringLiteral("未解锁"),
                                      fromWire(v.flight_mode()))
                           : QStringLiteral("飞控状态不可用");
      vehicle_->setText(QStringLiteral("飞行器：%1；%2；电量 %3")
                            .arg(pose, flight,
                                 v.battery_valid()
                                     ? QStringLiteral("%1%").arg(v.battery_percent(), 0, 'f', 0)
                                     : QStringLiteral("未知")));
    } else if (message.has_status()) {
      const auto &s = message.status();
      companion_->setText(QStringLiteral("任务机：Odin %1；EGO %2；定位 %3")
                              .arg(fromWire(s.odin()), fromWire(s.ego()),
                                   fromWire(s.localization())));
      video_->setText(QStringLiteral("视频：%1；录制：%2")
                          .arg(s.video_running() ? QStringLiteral("运行中")
                                                 : QStringLiteral("未运行"),
                               s.recording() ? QStringLiteral("运行中")
                                             : QStringLiteral("未运行")));
    } else if (message.has_cloud()) {
      map_->setText(QStringLiteral("点云：%1 点；地图 %2")
                        .arg(message.cloud().point_count())
                        .arg(fromWire(message.cloud().map_id())));
    } else if (message.has_grid()) {
      const auto &grid = message.grid();
      map_->setText(QStringLiteral("栅格：版本 %1；完整点 %2；增量 +%3 / -%4")
                        .arg(grid.version())
                        .arg(grid.centers_size())
                        .arg(grid.added_size())
                        .arg(grid.removed_size()));
    } else if (message.has_trajectory()) {
      trajectory_->setText(QStringLiteral("轨迹：%1；%2 点")
                               .arg(fromWire(message.trajectory().trajectory_id()))
                               .arg(message.trajectory().points_size()));
    } else if (message.has_video()) {
      video_->setText(QStringLiteral("视频：%1；%2")
                          .arg(message.video().running() ? QStringLiteral("运行中")
                                                         : QStringLiteral("未运行"),
                               fromWire(message.video().url())));
    } else if (message.has_log()) {
      appendDiagnostic(fromWire(message.log().source()) + QStringLiteral(": ") +
                       fromWire(message.log().text()));
    }
  }

  ProtocolClient client_;
  QLineEdit *endpoint_ = nullptr;
  QPushButton *button_ = nullptr;
  QLabel *state_ = nullptr;
  QLabel *vehicle_ = nullptr;
  QLabel *companion_ = nullptr;
  QLabel *map_ = nullptr;
  QLabel *trajectory_ = nullptr;
  QLabel *video_ = nullptr;
  QLabel *link_ = nullptr;
  QPlainTextEdit *diagnostics_ = nullptr;
  bool connecting_ = false;
};
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  AndroidWindow window;
  window.show();
  return app.exec();
}
