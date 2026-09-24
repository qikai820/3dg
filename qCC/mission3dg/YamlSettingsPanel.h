#pragma once

#include "ProtocolClient.h"
#include <QHash>
#include <QJsonObject>
#include <QJsonValue>
#include <QMap>
#include <QVector>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

class YamlSettingsPanel final : public QWidget {
public:
  explicit YamlSettingsPanel(ProtocolClient *client, QWidget *parent = nullptr);
  void refresh();

private:
  struct Field {
    QString path;
    QString label;
    QJsonValue value;
    QJsonObject metadata;
    bool editable = true;
  };
  void receive(const mission::Envelope &);
  void openFile(const QString &id);
  void setDocument(const mission::YamlDocument &);
  void showGroup(const QString &group);
  void preview();
  void save();
  QJsonValue valueAt(const QString &pointer) const;
  QString shown(const QJsonValue &value) const;
  QJsonValue parsed(const QString &text, bool *ok) const;

  ProtocolClient *client_;
  QTreeWidget *tree_;
  QTableWidget *table_;
  QLabel *status_;
  QLineEdit *search_;
  QPushButton *reload_, *preview_, *save_;
  QHash<QString, int> requests_;
  QMap<QString, QVector<Field>> groups_;
  QMap<QString, QJsonValue> drafts_;
  QJsonObject content_, metadata_;
  QString documentId_, revision_, currentGroup_;
  bool loading_ = false;
};
