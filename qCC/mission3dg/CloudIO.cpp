#include "CloudIO.h"
#include <QFile>
#include <QRegularExpression>
#include <QSaveFile>
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr quint64 kMaxSourcePoints = 10000000;
constexpr quint64 kMaxDisplayPoints = 2000000;
float f32(const char *p) {
  quint32 u = qFromLittleEndian<quint32>(p);
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}
void putf(char *p, float f) {
  quint32 u;
  std::memcpy(&u, &f, 4);
  qToLittleEndian(u, p);
}
bool valid(const MissionPoint &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::abs(p.x) < 1e7 && std::abs(p.y) < 1e7 && std::abs(p.z) < 1e7;
}
bool unlzf(const QByteArray &in, QByteArray &out) {
  int i = 0, o = 0;
  while (i < in.size()) {
    unsigned c = (unsigned char)in[i++];
    if (c < 32) {
      int n = int(c) + 1;
      if (i + n > in.size() || o + n > out.size())
        return false;
      std::memcpy(out.data() + o, in.constData() + i, n);
      i += n;
      o += n;
    } else {
      int n = c >> 5, back = (c & 31) << 8;
      if (n == 7) {
        if (i >= in.size())
          return false;
        n += (unsigned char)in[i++];
      }
      if (i >= in.size())
        return false;
      back += (unsigned char)in[i++];
      int ref = o - back - 1;
      n += 2;
      if (ref < 0 || o + n > out.size())
        return false;
      while (n--)
        out[o++] = out[ref++];
    }
  }
  return o == out.size();
}
} // namespace
QVector<MissionPoint> CloudIO::decode(const QByteArray &b, QString &error) {
  QVector<MissionPoint> pts;
  if (b.size() % 16) {
    error = "XYZRGB 数据长度不是 16 的倍数";
    return pts;
  }
  pts.reserve(b.size() / 16);
  for (int i = 0; i < b.size(); i += 16) {
    MissionPoint p;
    p.x = f32(b.constData() + i);
    p.y = f32(b.constData() + i + 4);
    p.z = f32(b.constData() + i + 8);
    p.r = (unsigned char)b[i + 12];
    p.g = (unsigned char)b[i + 13];
    p.b = (unsigned char)b[i + 14];
    if (valid(p))
      pts.push_back(p);
  }
  return pts;
}
QByteArray CloudIO::encode(const QVector<MissionPoint> &pts) {
  QByteArray b(pts.size() * 16, 0);
  for (int i = 0; i < pts.size(); ++i) {
    const auto &p = pts[i];
    char *q = b.data() + 16 * i;
    putf(q, p.x);
    putf(q + 4, p.y);
    putf(q + 8, p.z);
    q[12] = p.r;
    q[13] = p.g;
    q[14] = p.b;
  }
  return b;
}
bool CloudIO::writePCD(const QString &path, const QVector<MissionPoint> &pts,
                       QString &error) {
  QSaveFile f(path);
  if (!f.open(QIODevice::WriteOnly)) {
    error = f.errorString();
    return false;
  }
  const QByteArray header =
      QString("# 3DG XYZRGB map\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 "
              "4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %1\nHEIGHT 1\nVIEWPOINT 0 "
              "0 0 1 0 0 0\nPOINTS %1\nDATA binary\n")
          .arg(pts.size())
          .toUtf8();
  if (f.write(header) != header.size()) {
    error = f.errorString();
    return false;
  }
  QByteArray b(pts.size() * 16, 0);
  for (int i = 0; i < pts.size(); ++i) {
    const auto &p = pts[i];
    char *q = b.data() + 16 * i;
    putf(q, p.x);
    putf(q + 4, p.y);
    putf(q + 8, p.z);
    qToLittleEndian<quint32>((quint32(p.r) << 16) | (quint32(p.g) << 8) | p.b,
                             q + 12);
  }
  if (f.write(b) != b.size() || !f.commit()) {
    error = f.errorString();
    return false;
  }
  return true;
}
bool CloudIO::readPCD(const QString &path, QVector<MissionPoint> &pts,
                      QString &error, quint64 *sourcePointCount) {
  error.clear();
  if (sourcePointCount)
    *sourcePointCount = 0;
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    error = f.errorString();
    return false;
  }
  if (f.size() > 256 * 1024 * 1024) {
    error = "PCD 超过 256 MiB 限制";
    return false;
  }
  QStringList fields, types;
  QVector<int> sizes, counts;
  quint64 n = 0, w = 0, h = 1;
  QString data;
  for (int line = 0; line < 1000 && !f.atEnd(); ++line) {
    auto s = QString::fromLatin1(f.readLine(8192)).trimmed();
    if (s.startsWith('#') || s.isEmpty())
      continue;
    auto a = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    auto key = a.takeFirst().toUpper();
    if (key == "FIELDS")
      fields = a;
    else if (key == "TYPE")
      types = a;
    else if (key == "SIZE" || key == "COUNT") {
      QVector<int> v;
      for (auto x : a)
        v << x.toInt();
      if (key == "SIZE")
        sizes = v;
      else
        counts = v;
    } else if (key == "POINTS")
      n = a.value(0).toULongLong();
    else if (key == "WIDTH")
      w = a.value(0).toULongLong();
    else if (key == "HEIGHT")
      h = a.value(0).toULongLong();
    else if (key == "DATA") {
      data = a.value(0);
      break;
    }
  }
  if (!n) {
    if (w > kMaxSourcePoints || h > kMaxSourcePoints) {
      error = "PCD 点数超限";
      return false;
    }
    n = w * h;
  }
  if (counts.isEmpty())
    counts = QVector<int>(fields.size(), 1);
  if (!n || n > kMaxSourcePoints) {
    error = n ? QString("PCD 原始点数 %1 超过 1000 万上限").arg(n)
              : "PCD 头缺少有效点数";
    return false;
  }
  if (fields.size() != sizes.size() || fields.size() != types.size() ||
      fields.size() != counts.size() || fields.size() > 64) {
    error = "PCD 头字段无效";
    return false;
  }
  QVector<int> offsets, columns;
  int stride = 0, col = 0;
  for (int i = 0; i < fields.size(); ++i) {
    if (sizes[i] < 1 || sizes[i] > 8 || counts[i] < 1 || counts[i] > 16) {
      error = "不支持的 PCD 字段尺寸";
      return false;
    }
    offsets << stride;
    columns << col;
    stride += sizes[i] * counts[i];
    col += counts[i];
  }
  const int ix = fields.indexOf("x"), iy = fields.indexOf("y"),
            iz = fields.indexOf("z");
  for (int k : {ix, iy, iz})
    if (k < 0 || types[k] != "F" || (sizes[k] != 4 && sizes[k] != 8) ||
        counts[k] != 1) {
      error = "PCD 需要 float32/64 的 x y z";
      return false;
    }
  if (quint64(stride) * n > 256 * 1024 * 1024) {
    error = "PCD 解压尺寸超限";
    return false;
  }
  int rgb = fields.indexOf("rgb");
  if (rgb < 0)
    rgb = fields.indexOf("rgba");
  if (rgb >= 0 && (sizes[rgb] != 4 || counts[rgb] != 1)) {
    error = "rgb 字段必须为 4 字节";
    return false;
  }
  QByteArray b;
  bool soa = false;
  if (data == "binary")
    b = f.read(qint64(stride * n));
  else if (data == "binary_compressed") {
    auto hdr = f.read(8);
    if (hdr.size() != 8) {
      error = "压缩头不完整";
      return false;
    }
    quint32 compressed = qFromLittleEndian<quint32>(hdr.constData()),
            raw = qFromLittleEndian<quint32>(hdr.constData() + 4);
    if (raw != stride * n || compressed > 256 * 1024 * 1024) {
      error = "压缩长度无效";
      return false;
    }
    b.resize(raw);
    auto input = f.read(compressed);
    if (input.size() != int(compressed) || !unlzf(input, b)) {
      error = "PCD LZF 解压失败";
      return false;
    }
    soa = true;
  } else if (data != "ascii") {
    error = "不支持的 PCD DATA 类型";
    return false;
  }
  if (data != "ascii" && b.size() != int(stride * n)) {
    error = "PCD 数据被截断";
    return false;
  }
  const quint64 sampleStep =
      (n + kMaxDisplayPoints - 1) / kMaxDisplayPoints;
  QVector<MissionPoint> result;
  result.reserve(int((n + sampleStep - 1) / sampleStep));
  for (quint64 i = 0; i < n; ++i) {
    if (data != "ascii" && i % sampleStep != 0)
      continue;
    MissionPoint p;
    quint32 color = 0xbed2dc;
    if (data == "ascii") {
      const QByteArray line = f.readLine(65536);
      if (line.isEmpty()) {
        error = "PCD ASCII 行被截断";
        return false;
      }
      auto a = QString::fromLatin1(line).simplified().split(' ');
      if (a.size() < col) {
        error = "PCD ASCII 行被截断";
        return false;
      }
      bool ok = true;
      auto number = [&](int k) {
        bool one = false;
        double x = a[columns[k]].toDouble(&one);
        ok = ok && one;
        return x;
      };
      p.x = number(ix);
      p.y = number(iy);
      p.z = number(iz);
      if (rgb >= 0) {
        if (types[rgb] == "F") {
          float c = number(rgb);
          std::memcpy(&color, &c, 4);
        } else {
          bool one = false;
          color = a[columns[rgb]].toUInt(&one);
          ok = ok && one;
        }
      }
      if (!ok) {
        error = "PCD 数字无效";
        return false;
      }
    } else {
      auto ptr = [&](int k) {
        return b.constData() +
               (soa ? quint64(offsets[k]) * n + i * sizes[k] * counts[k]
                    : i * stride + offsets[k]);
      };
      auto number = [&](int k) {
        if (sizes[k] == 4)
          return double(f32(ptr(k)));
        quint64 bits = qFromLittleEndian<quint64>(ptr(k));
        double d;
        std::memcpy(&d, &bits, 8);
        return d;
      };
      p.x = number(ix);
      p.y = number(iy);
      p.z = number(iz);
      if (rgb >= 0)
        color = qFromLittleEndian<quint32>(ptr(rgb));
    }
    p.r = color >> 16;
    p.g = color >> 8;
    p.b = color;
    if (i % sampleStep == 0 && valid(p))
      result.push_back(p);
  }
  if (sourcePointCount)
    *sourcePointCount = n;
  pts = std::move(result);
  return true;
}
