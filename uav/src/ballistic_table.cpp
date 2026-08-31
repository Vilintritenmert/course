#include "uav_ai/ballistic_table.hpp"

#include <algorithm>
#include <fstream>

namespace uav {

namespace {

// Індекс нижнього вузла і коефіцієнт [0..1] для одного виміру. Значення за
// межами осі - clamp до крайнього інтервалу (frac 0 або 1).
struct Interp {
  int lo = 0;
  float frac = 0.F;
};

auto findInterp(float val, const std::vector<float> &axis) -> Interp {
  if (val <= axis.front()) {
    return {0, 0.F};
  }
  if (val >= axis.back()) {
    return {static_cast<int>(axis.size()) - 2, 1.F};
  }

  auto it = std::lower_bound(axis.begin(), axis.end(), val);
  int i = static_cast<int>(it - axis.begin()) - 1;
  if (i < 0) {
    i = 0;
  }
  const float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
  return {i, frac};
}

auto lerp(const BallisticTable::Result &a, const BallisticTable::Result &b,
          float t) -> BallisticTable::Result {
  return {a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t};
}

} // namespace

auto BallisticTable::load(const std::string &path) -> bool {
  std::ifstream f(path);
  if (!f.is_open()) {
    return false;
  }

  int nZ = 0;
  int nV = 0;
  int nM = 0;
  int nD = 0;
  int nL = 0;
  f >> nZ >> nV >> nM >> nD >> nL;

  axisZ0.resize(nZ);
  for (auto &v : axisZ0) {
    f >> v;
  }
  axisV0.resize(nV);
  for (auto &v : axisV0) {
    f >> v;
  }
  axisM.resize(nM);
  for (auto &v : axisM) {
    f >> v;
  }
  axisD.resize(nD);
  for (auto &v : axisD) {
    f >> v;
  }
  axisL.resize(nL);
  for (auto &v : axisL) {
    f >> v;
  }

  const size_t total = static_cast<size_t>(nZ) * nV * nM * nD * nL;
  data.resize(total);

  // Порядок: Z0 -> V0 -> m -> d -> l (зовнішній -> внутрішній).
  for (size_t i = 0; i < total; ++i) {
    f >> data[i].t >> data[i].hDist;
  }

  return f.good() || f.eof();
}

auto BallisticTable::lookup(float z0, float v0, float m, float d,
                            float l) const -> Result {
  const Interp iz = findInterp(z0, axisZ0);
  const Interp iv = findInterp(v0, axisV0);
  const Interp im = findInterp(m, axisM);
  const Interp id = findInterp(d, axisD);
  const Interp il = findInterp(l, axisL);

  // l: 32 -> 16
  Result v[16];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        for (int e = 0; e < 2; e++) {
          const auto &lo =
              at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const auto &hi =
              at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[(a * 8) + (b * 4) + (c * 2) + e] = lerp(lo, hi, il.frac);
        }
      }
    }
  }

  // d: 16 -> 8
  Result w[8];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        w[(a * 4) + (b * 2) + c] = lerp(v[(a * 8) + (b * 4) + (c * 2)],
                                        v[(a * 8) + (b * 4) + (c * 2) + 1],
                                        id.frac);
      }
    }
  }

  // m: 8 -> 4
  Result u[4];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      u[(a * 2) + b] = lerp(w[(a * 4) + (b * 2)], w[(a * 4) + (b * 2) + 1],
                            im.frac);
    }
  }

  // V0: 4 -> 2
  Result s[2];
  for (int a = 0; a < 2; a++) {
    s[a] = lerp(u[a * 2], u[(a * 2) + 1], iv.frac);
  }

  // Z0: 2 -> 1
  return lerp(s[0], s[1], iz.frac);
}

} // namespace uav
