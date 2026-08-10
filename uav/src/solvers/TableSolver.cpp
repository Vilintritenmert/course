#include <algorithm>
#include <memory>

#include "DroneContext.hpp"
#include "Position.hpp"
#include "TableSolver.hpp"

namespace {

using Result = BallisticTable::Result;

struct Interp {
  int lo;     // нижній індекс в осі
  float frac; // коефіцієнт [0..1]
};

Result lerp(const Result &a, const Result &b, float t) {
  return {a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t};
}

Interp findInterp(float val, const std::vector<float> &axis) {
  if (val <= axis.front())
    return {0, 0.0f};
  if (val >= axis.back())
    return {(int)axis.size() - 2, 1.0f};

  auto it = std::lower_bound(axis.begin(), axis.end(), val);
  int i = (int)(it - axis.begin()) - 1;
  if (i < 0)
    i = 0;

  float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
  return {i, frac};
}

Result lookup(const BallisticTable &table, float Z0, float V0, float m, float d,
              float l) {
  Interp iz = findInterp(Z0, table.axisZ0);
  Interp iv = findInterp(V0, table.axisV0);
  Interp im = findInterp(m, table.axisM);
  Interp id = findInterp(d, table.axisD);
  Interp il = findInterp(l, table.axisL);

  // l: 32 -> 16
  Result v[16];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        for (int e = 0; e < 2; e++) {
          const Result &lo =
              table.at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const Result &hi =
              table.at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }

  // d: 16 -> 8
  Result w[8];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2],
                                    v[a * 8 + b * 4 + c * 2 + 1], id.frac);

  // m: 8 -> 4
  Result u[4];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);

  // V0: 4 -> 2
  Result s[2];
  for (int a = 0; a < 2; a++)
    s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);

  // Z0: 2 -> 1
  return lerp(s[0], s[1], iz.frac);
}

} // namespace

TableSolver::TableSolver(const std::string &tablePath) {
  static const std::string kDefaultTablePath = "data/ballistic_table.txt";
  _table.load(tablePath.empty() ? kDefaultTablePath.c_str()
                                : tablePath.c_str());
}

float TableSolver::computeFlightTime(
    std::shared_ptr<DroneContext> droneDetails) {
  float z0 = droneDetails->getConfig()->getAltitude();
  float v0 = droneDetails->getConfig()->getAttackSpeed();
  float m = droneDetails->getAmmoParams().getMass();
  float d = droneDetails->getAmmoParams().getDrag();
  float l = droneDetails->getAmmoParams().getLift();

  return lookup(_table, z0, v0, m, d, l).t;
};

float TableSolver::computeHorizDist(
    float t, std::shared_ptr<DroneContext> droneDetails) {
  float z0 = droneDetails->getConfig()->getAltitude();
  float v0 = droneDetails->getConfig()->getAttackSpeed();
  float m = droneDetails->getAmmoParams().getMass();
  float d = droneDetails->getAmmoParams().getDrag();
  float l = droneDetails->getAmmoParams().getLift();

  return lookup(_table, z0, v0, m, d, l).hDist;
};

bool TableSolver::computeDropPoint(Position tgt, Position drone, float h,
                                   Position &drop) {
  Position diff = tgt - drone;
  float dist = Position::distance(diff);
  if (dist < 1e-3f)
    return false;
  drop = drone + diff * ((dist - h) / dist);
  return true;
};
