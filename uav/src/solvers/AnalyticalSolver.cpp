#include <cmath>

#include "AnalyticalSolver.hpp"

float AnalyticalSolver::computeFlightTime(DroneDetails *droneDetails) {
  float zd = droneDetails->getAltitude();
  float m = droneDetails->getAmmoParams().getMass();
  float d = droneDetails->getAmmoParams().getDrag();
  float l = droneDetails->getAmmoParams().getLift();
  float v = droneDetails->getAttackSpeed();

  float a = d * G * m - 2.0f * d * d * l * v;
  float b = -3.0f * G * m * m + 3.0f * d * l * m * v;
  float c = 6.0f * m * m * zd;

  if (fabsf(a) < 1e-9f)
    return sqrtf(2.0f * zd / G);

  float p = -b * b / (3.0f * a * a);
  float q = 2.0f * b * b * b / (27.0f * a * a * a) + c / a;

  if (p >= 0.0f)
    return sqrtf(2.0f * zd / G);

  float inner = 3.0f * q / (2.0f * p) * sqrtf(-3.0f / p);
  inner = fmaxf(-1.0f, fminf(1.0f, inner));
  float phi = acosf(inner);
  float u = 2.0f * sqrtf(-p / 3.0f) * cosf((phi + 4.0f * (float)M_PI) / 3.0f);
  float t = u - b / (3.0f * a);

  return (t > 0.0f) ? t : sqrtf(2.0f * zd / G);
};

float AnalyticalSolver::computeHorizDist(float t, DroneDetails *droneDetails) {
  float ad = droneDetails->getAmmoParams().getDrag();
  float as = droneDetails->getAttackSpeed();
  float am = droneDetails->getAmmoParams().getMass();
  float l = droneDetails->getAmmoParams().getLift();
  float l2 = l * l;
  float l4 = l2 * l2;
  float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

  float r1 = as * t - t2 * ad * as / (2.0f * am);
  float r2 = t3 * (6.0f * ad * G * l * am - 6.0f * ad * ad * (l2 - 1.0f) * as) /
             (36.0f * am * am);
  float r3 = 0.0f, r4 = 0.0f;

  if (l > 1e-6f) {
    float dn3 = 36.0f * (1.0f + l2) * (1.0f + l2) * am * am * am;
    r3 = t4 *
         (-6.0f * ad * ad * G * l * (1.0f + l2 + l4) * am +
          3.0f * ad * ad * ad * l2 * (1.0f + l2) * as +
          6.0f * ad * ad * ad * l4 * (1.0f + l2) * as) /
         dn3;
    float dn4 = 36.0f * (1.0f + l2) * am * am * am * am;
    r4 = t5 *
         (3.0f * ad * ad * ad * G * l * l2 * am -
          3.0f * ad * ad * ad * ad * l2 * (1.0f + l2) * as) /
         dn4;
  }

  return r1 + r2 + r3 + r4;
};

bool AnalyticalSolver::computeDropPoint(Position tgt, Position drone, float h,
                                        Position &drop) {
  Position d = tgt - drone;
  float dist = Position::distance(d);
  if (dist < 1e-3f)
    return false;
  drop = drone + d * ((dist - h) / dist);
  return true;
};