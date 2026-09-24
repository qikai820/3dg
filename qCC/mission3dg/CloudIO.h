#pragma once
#include <QByteArray>
#include <QString>
#include <QVector>
#include <QtGlobal>
struct MissionPoint {
  float x = 0, y = 0, z = 0;
  unsigned char r = 190, g = 210, b = 220;
};
namespace CloudIO {
// Large source clouds are sampled for display; sourcePointCount reports the
// number declared by the PCD header before sampling.
bool readPCD(const QString &, QVector<MissionPoint> &, QString &,
             quint64 *sourcePointCount = nullptr);
bool writePCD(const QString &, const QVector<MissionPoint> &, QString &);
QVector<MissionPoint> decode(const QByteArray &, QString &);
QByteArray encode(const QVector<MissionPoint> &);
} // namespace CloudIO
