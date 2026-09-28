#include "QuadrotorModel.h"
#include <QDataStream>
#include <QFile>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <cmath>
#include <memory>

ccMesh *createQuadrotorModel() {
  QFile file(":/mission3dg/x500/quadrotor.mesh");
  if (!file.open(QIODevice::ReadOnly) || file.read(8) != "3DGQ0001")
    return nullptr;
  QDataStream input(&file);
  input.setByteOrder(QDataStream::LittleEndian);
  input.setFloatingPointPrecision(QDataStream::SinglePrecision);
  quint32 pointCount = 0, faceCount = 0;
  input >> pointCount >> faceCount;
  if (!pointCount || !faceCount || pointCount > 200000 || faceCount > 200000 ||
      file.size() != 16 + qint64(pointCount) * 15 + qint64(faceCount) * 12)
    return nullptr;
  auto vertices = std::make_unique<ccPointCloud>("X500 vertices");
  if (!vertices->reserve(pointCount) || !vertices->reserveTheRGBTable())
    return nullptr;
  for (quint32 i = 0; i < pointCount; ++i) {
    float x, y, z;
    quint8 r, g, b;
    input >> x >> y >> z >> r >> g >> b;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      return nullptr;
    vertices->addPoint(CCVector3(x, y, z));
    vertices->addColor(r, g, b);
  }
  auto model = std::make_unique<ccMesh>(vertices.get());
  model->addChild(vertices.release());
  if (!model->reserve(faceCount))
    return nullptr;
  for (quint32 i = 0; i < faceCount; ++i) {
    quint32 a, b, c;
    input >> a >> b >> c;
    if (a >= pointCount || b >= pointCount || c >= pointCount)
      return nullptr;
    model->addTriangle(a, b, c);
  }
  if (input.status() != QDataStream::Ok)
    return nullptr;
  model->setName("PX4 X500 quadrotor");
  model->getChild(0)->setEnabled(false);
  model->showColors(true);
  model->showNormals(model->computePerTriangleNormals());
  return model.release();
}
