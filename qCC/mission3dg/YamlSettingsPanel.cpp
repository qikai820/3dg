#include "YamlSettingsPanel.h"
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <cmath>

namespace {
QString unescape(QString segment) {
  return segment.replace("~1", "/").replace("~0", "~");
}
QString pointerHead(const QString &pointer) {
  return unescape(pointer.section('/', 1, 1));
}
QString jsonScalar(const QJsonValue &value) {
  QJsonArray wrapper;
  wrapper.append(value);
  QByteArray data = QJsonDocument(wrapper).toJson(QJsonDocument::Compact);
  return QString::fromUtf8(data.mid(1, data.size() - 2));
}
} // namespace

YamlSettingsPanel::YamlSettingsPanel(ProtocolClient *client, QWidget *parent)
    : QWidget(parent), client_(client) {
  auto *layout = new QVBoxLayout(this);
  auto *hint = new QLabel(
      "参数来自任务机 YAML。保存后重启对应进程生效；机载航点只读显示，本地航点可在编辑器编排后上传。", this);
  hint->setWordWrap(true);
  layout->addWidget(hint);
  search_ = new QLineEdit(this);
  search_->setPlaceholderText("搜索参数名称或完整路径");
  layout->addWidget(search_);
  auto *split = new QSplitter(this);
  tree_ = new QTreeWidget(split);
  tree_->setHeaderHidden(true);
  tree_->setMinimumWidth(180);
  table_ = new QTableWidget(split);
  table_->setColumnCount(4);
  table_->setHorizontalHeaderLabels({"参数", "当前值 / 修改值", "单位", "说明"});
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  table_->verticalHeader()->hide();
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  split->addWidget(tree_);
  split->addWidget(table_);
  split->setStretchFactor(1, 1);
  layout->addWidget(split, 1);
  status_ = new QLabel("连接任务机后读取 YAML 配置", this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  auto *actions = new QHBoxLayout;
  reload_ = new QPushButton("重新读取", this);
  preview_ = new QPushButton("预览修改", this);
  save_ = new QPushButton("保存到任务机", this);
  actions->addWidget(reload_);
  actions->addStretch();
  actions->addWidget(preview_);
  actions->addWidget(save_);
  layout->addLayout(actions);
  connect(reload_, &QPushButton::clicked, this, [this] {
    if (!drafts_.isEmpty() &&
        QMessageBox::question(this, "放弃修改", "放弃未保存的参数修改并重新读取？") !=
            QMessageBox::Yes)
      return;
    drafts_.clear();
    if (documentId_.isEmpty())
      refresh();
    else
      openFile(documentId_);
  });
  connect(preview_, &QPushButton::clicked, this, [this] { preview(); });
  connect(save_, &QPushButton::clicked, this, [this] { save(); });
  connect(search_, &QLineEdit::textChanged, this,
          [this] { showGroup(currentGroup_); });
  connect(tree_, &QTreeWidget::itemClicked, this,
          [this](QTreeWidgetItem *item) {
            const QString file = item->data(0, Qt::UserRole).toString();
            const QString group = item->data(0, Qt::UserRole + 1).toString();
            if (file != documentId_) {
              if (!drafts_.isEmpty() &&
                  QMessageBox::question(
                      this, "放弃修改", "切换文件会放弃未保存的参数修改，继续？") !=
                      QMessageBox::Yes)
                return;
              drafts_.clear();
              openFile(file);
            } else
              showGroup(group);
          });
  connect(client_, &ProtocolClient::message, this,
          [this](const mission::Envelope &e) { receive(e); });
  connect(client_, &ProtocolClient::stateChanged, this,
          [this](const QString &, bool ready) {
            if (!ready || !client_->connected()) {
              status_->setText("任务机连接中断，当前参数不可保存");
              save_->setEnabled(false);
            } else
              save_->setEnabled(true);
          });
  save_->setEnabled(client_->connected());
}

void YamlSettingsPanel::refresh() {
  if (!client_->connected()) {
    status_->setText("任务机未连接；模拟模式不读取端侧 YAML");
    return;
  }
  mission::Command command;
  command.set_kind(mission::Command::GET_YAML_CATALOG);
  auto id = client_->sendCommand(command);
  if (!id.isEmpty()) {
    requests_[id] = mission::Command::GET_YAML_CATALOG;
    status_->setText("正在读取配置文件列表…");
  }
}

void YamlSettingsPanel::openFile(const QString &id) {
  if (id.isEmpty() || !client_->connected())
    return;
  mission::Command command;
  command.set_kind(mission::Command::GET_YAML_DOCUMENT);
  command.set_yaml_document_id(id.toStdString());
  const auto request = client_->sendCommand(command);
  if (!request.isEmpty()) {
    requests_[request] = mission::Command::GET_YAML_DOCUMENT;
    status_->setText("正在读取 " + id + "…");
  }
}

void YamlSettingsPanel::receive(const mission::Envelope &e) {
  const QString request = QString::fromStdString(e.request_id());
  if (!requests_.contains(request))
    return;
  const int kind = requests_.value(request);
  if (e.has_result()) {
    if (e.result().state() == mission::CommandResult::ACCEPTED)
      return;
    if (e.result().state() == mission::CommandResult::FAILED) {
      status_->setText(QString::fromStdString(e.result().detail()));
      requests_.remove(request);
    } else if (kind == mission::Command::PATCH_YAML_DOCUMENT) {
      status_->setText("参数已保存；重启对应进程后生效");
      requests_.remove(request);
    } else
      requests_.remove(request);
    return;
  }
  if (e.has_yaml_catalog() && kind == mission::Command::GET_YAML_CATALOG) {
    QSignalBlocker blocked(tree_);
    tree_->clear();
    for (const auto &file : e.yaml_catalog().files()) {
      auto *item = new QTreeWidgetItem(tree_);
      item->setText(0, QString::fromStdString(file.label()));
      item->setToolTip(0, QString::fromStdString(file.usage()));
      item->setData(0, Qt::UserRole, QString::fromStdString(file.id()));
    }
    if (tree_->topLevelItemCount())
      openFile(tree_->topLevelItem(0)->data(0, Qt::UserRole).toString());
  } else if (e.has_yaml_document() &&
             (kind == mission::Command::GET_YAML_DOCUMENT ||
              kind == mission::Command::PATCH_YAML_DOCUMENT)) {
    setDocument(e.yaml_document());
  }
}

QJsonValue YamlSettingsPanel::valueAt(const QString &pointer) const {
  QJsonValue value(content_);
  const auto segments = pointer.split('/', Qt::SkipEmptyParts);
  for (const auto &raw : segments) {
    const QString key = unescape(raw);
    if (value.isObject())
      value = value.toObject().value(key);
    else if (value.isArray())
      value = value.toArray().at(key.toInt());
    else
      return {};
  }
  return value;
}

void YamlSettingsPanel::setDocument(const mission::YamlDocument &document) {
  QJsonParseError error;
  const auto content = QJsonDocument::fromJson(
      QByteArray::fromStdString(document.content_json()), &error);
  if (error.error != QJsonParseError::NoError || !content.isObject()) {
    status_->setText("任务机返回的 YAML 参数树无效");
    return;
  }
  const auto metadata = QJsonDocument::fromJson(
      QByteArray::fromStdString(document.metadata_json()), &error);
  if (error.error != QJsonParseError::NoError || !metadata.isObject()) {
    status_->setText("任务机返回的参数说明无效");
    return;
  }
  content_ = content.object();
  metadata_ = metadata.object();
  documentId_ = QString::fromStdString(document.id());
  revision_ = QString::fromStdString(document.revision());
  drafts_.clear();
  groups_.clear();
  const auto definitions = metadata_.value("fields").toObject();
  const auto order = metadata_.value("order").toArray();
  for (const auto &pathValue : order) {
    const QString path = pathValue.toString();
    if (path.startsWith("/waypoints/"))
      continue;
    const QString head = pointerHead(path);
    const QString group = path.count('/') == 1 ? QString() : head;
    const auto definition = definitions.value(path).toObject();
    Field field{path, definition.value("label").toString(
                          unescape(path.section('/', -1))),
                valueAt(path), definition, head != "ego_cloud_filter"};
    if (!field.value.isUndefined())
      groups_[group].append(field);
  }
  if (content_.value("waypoints").isArray()) {
    Field waypoints{"/waypoints", "航点",
                    QString("%1 项，使用航点编辑器修改")
                        .arg(content_.value("waypoints").toArray().size()),
                    {}, false};
    groups_["waypoints"].append(waypoints);
  }
  const auto labels = metadata_.value("groups").toObject();
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    auto *file = tree_->topLevelItem(i);
    const bool active = file->data(0, Qt::UserRole).toString() == documentId_;
    qDeleteAll(file->takeChildren());
    if (!active)
      continue;
    for (auto it = groups_.cbegin(); it != groups_.cend(); ++it) {
      auto *group = new QTreeWidgetItem(file);
      group->setText(0, it.key().isEmpty() ? "基础参数"
                                          : labels.value(it.key()).toString(it.key()));
      group->setData(0, Qt::UserRole, documentId_);
      group->setData(0, Qt::UserRole + 1, it.key());
    }
    file->setExpanded(true);
  }
  currentGroup_ = groups_.isEmpty() ? QString() : groups_.cbegin().key();
  showGroup(currentGroup_);
  status_->setText(QString::fromStdString(document.usage()) +
                   " · 已读取，保存后重启对应进程生效");
}

QString YamlSettingsPanel::shown(const QJsonValue &value) const {
  if (value.isString())
    return value.toString();
  if (value.isBool())
    return value.toBool() ? "true" : "false";
  return jsonScalar(value);
}

QJsonValue YamlSettingsPanel::parsed(const QString &text, bool *ok) const {
  QJsonParseError error;
  const auto json = QJsonDocument::fromJson(
      QByteArray("[") + text.toUtf8() + "]", &error);
  *ok = error.error == QJsonParseError::NoError && json.isArray() &&
        json.array().size() == 1;
  return *ok ? json.array().at(0) : QJsonValue();
}

void YamlSettingsPanel::showGroup(const QString &group) {
  currentGroup_ = group;
  table_->setRowCount(0);
  const auto query = search_->text().trimmed();
  for (const auto &field : groups_.value(group)) {
    if (!query.isEmpty() && !field.label.contains(query, Qt::CaseInsensitive) &&
        !field.path.contains(query, Qt::CaseInsensitive))
      continue;
    const int row = table_->rowCount();
    table_->insertRow(row);
    auto *name = new QTableWidgetItem(field.label);
    name->setToolTip(field.path);
    table_->setItem(row, 0, name);
    table_->setItem(row, 2,
                    new QTableWidgetItem(field.metadata.value("unit").toString()));
    const QString note = field.editable
                             ? field.metadata.value("description").toString(
                                   "重启对应进程后生效")
                             : "只读；使用专用流程修改或已停用";
    table_->setItem(row, 3, new QTableWidgetItem(note));
    const auto current = drafts_.value(field.path, field.value);
    if (!field.editable || current.isNull() ||
        (!current.isString() && !current.isBool() && !current.isDouble())) {
      table_->setItem(row, 1, new QTableWidgetItem(shown(current)));
      continue;
    }
    const auto choices = field.metadata.value("choices").toArray();
    if (!choices.isEmpty()) {
      auto *combo = new QComboBox(table_);
      for (const auto &option : choices) {
        const QString text = option.toString();
        bool ok = false;
        const auto value = parsed(text.section('|', 0, 0), &ok);
        if (ok)
          combo->addItem(text.section('|', 1), jsonScalar(value));
      }
      combo->setCurrentIndex(combo->findData(jsonScalar(current)));
      connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
              this, [this, combo, field](int) {
                bool ok = false;
                const auto value = parsed(combo->currentData().toString(), &ok);
                if (ok)
                  drafts_[field.path] = value;
              });
      table_->setCellWidget(row, 1, combo);
    } else if (current.isBool()) {
      auto *check = new QCheckBox(table_);
      check->setChecked(current.toBool());
      connect(check, &QCheckBox::toggled, this,
              [this, field](bool checked) { drafts_[field.path] = checked; });
      table_->setCellWidget(row, 1, check);
    } else {
      auto *edit = new QLineEdit(shown(current), table_);
      connect(edit, &QLineEdit::textEdited, this,
              [this, field](const QString &text) {
                if (field.value.isString())
                  drafts_[field.path] = text;
                else {
                  bool ok = false;
                  const auto value = parsed(text, &ok);
                  if (ok && value.isDouble() && std::isfinite(value.toDouble()))
                    drafts_[field.path] = value;
                  else
                    drafts_[field.path] = QJsonValue();
                }
              });
      table_->setCellWidget(row, 1, edit);
    }
  }
}

void YamlSettingsPanel::preview() {
  if (drafts_.isEmpty()) {
    status_->setText("没有待保存的参数修改");
    return;
  }
  QStringList changes;
  for (auto it = drafts_.cbegin(); it != drafts_.cend(); ++it)
    changes << it.key() + ": " + shown(valueAt(it.key())) + " → " +
                   shown(it.value());
  QMessageBox::information(this, "参数修改预览", changes.join("\n"));
}

void YamlSettingsPanel::save() {
  if (drafts_.isEmpty() || !client_->connected())
    return;
  QJsonObject changes;
  for (auto it = drafts_.cbegin(); it != drafts_.cend(); ++it) {
    if (it.value().isNull() || it.value().isUndefined()) {
      status_->setText("存在无效数值：" + it.key());
      return;
    }
    if (it.value() != valueAt(it.key()))
      changes.insert(it.key(), it.value());
  }
  if (changes.isEmpty()) {
    drafts_.clear();
    status_->setText("参数没有变化");
    return;
  }
  QStringList lines;
  for (auto it = changes.begin(); it != changes.end(); ++it)
    lines << it.key() + ": " + shown(valueAt(it.key())) + " → " +
                 shown(it.value());
  if (QMessageBox::question(
          this, "保存 YAML 参数",
          "以下参数将写入任务机，重启对应进程后生效：\n\n" +
              lines.join("\n")) != QMessageBox::Yes)
    return;
  mission::Command command;
  command.set_kind(mission::Command::PATCH_YAML_DOCUMENT);
  command.set_yaml_document_id(documentId_.toStdString());
  command.set_yaml_revision(revision_.toStdString());
  command.set_yaml_patch_json(
      QJsonDocument(changes).toJson(QJsonDocument::Compact).toStdString());
  const auto request = client_->sendCommand(command);
  if (!request.isEmpty()) {
    requests_[request] = mission::Command::PATCH_YAML_DOCUMENT;
    status_->setText("正在保存并校验 YAML…");
  }
}
