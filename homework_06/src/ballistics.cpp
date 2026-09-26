#include "ballistics.hpp"
#include <stdexcept>
#include <cstring>
#include <cmath>

using namespace std;

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
// We use magic numbers here because they are physical parameters of the ammunition and are not expected to change.

const string VOG_17 = "VOG-17";
const string M67 = "M67";
const string RKG_3 = "RKG-3";
const string GLIDING_VOG = "GLIDING-VOG";
const string GLIDING_RKG = "GLIDING-RKG";

auto getAmmoParameters(const string& ammo_name) -> AmmoParameters
{
  AmmoParameters params;
  if (ammo_name == VOG_17) {
    params.m = 0.35;
    params.d = 0.07;
    params.l = 0.;
  }
  else if (ammo_name == M67) {
    params.m = 0.6;
    params.d = 0.1;
    params.l = 0.;
  }
  else if (ammo_name == RKG_3) {
    params.m = 1.2;
    params.d = 0.1;
    params.l = 0.;
  }
  else if (ammo_name == GLIDING_VOG) {
    params.m = 0.45;
    params.d = 0.1;
    params.l = 1.;
  }
  else if (ammo_name == GLIDING_RKG) {
    params.m = 1.4;
    params.d = 0.1;
    params.l = 1.;
  }
  else {
    throw invalid_argument("ammo_name is not acceptable");
  }

  return params;
}

auto computeDropSolution(const BallisticsInput& input) -> DropSolution
{
  AmmoParameters ammoParams = getAmmoParameters(input.ammo_name);

  Coords dron = input.initial_dron;
  const double GRAVITY_ACCELERATION_ = 9.81;

  double v0 = input.attack_speed;
  double z0 = dron.z;

  double m = ammoParams.m;
  double d = ammoParams.d;
  double l = ammoParams.l;

  double a = d * GRAVITY_ACCELERATION_ * m - 2 * d * d * l * v0;
  double b = -3 * GRAVITY_ACCELERATION_ * m * m + 3 * d * l * m * v0;
  double c = 6 * m * m * z0;

  double p = -b * b / (3 * a * a);
  double q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  if ((p >= 0) || (fabs(3 * q / (2 * p) * sqrt(-3 / p)) > 1)) {
    throw invalid_argument("Has no solution");
  }

  double phi = acos(3 * q / (2 * p) * sqrt(-3 / p));

  double t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

  double hight_distance =
    (input.attack_speed * t) - (pow(t, 2) * d * input.attack_speed) / (2.0 * m) +
    (pow(t, 3) * (6.0 * d * GRAVITY_ACCELERATION_ * l * m - 6.0 * pow(d, 2) * (pow(l, 2) - 1.0) * input.attack_speed)) / (36.0 * pow(m, 2)) +
    (pow(t, 4) *
     (-6.0 * pow(d, 2) * GRAVITY_ACCELERATION_ * l * (1.0 + pow(l, 2) + pow(l, 4)) * m + 3.0 * pow(d, 3) * pow(l, 2) * (1.0 + pow(l, 2)) * input.attack_speed +
      6.0 * pow(d, 3) * pow(l, 4) * (1.0 + pow(l, 2)) * input.attack_speed)) /
      (36.0 * pow(1.0 + pow(l, 2), 2) * pow(m, 3)) +
    (pow(t, 5) * (3.0 * pow(d, 3) * GRAVITY_ACCELERATION_ * pow(l, 3) * m - 3.0 * pow(d, 4) * pow(l, 2) * (1.0 + pow(l, 2)) * input.attack_speed)) /
      (36.0 * (1.0 + pow(l, 2)) * pow(m, 4));

  double distance_to_target = sqrt(pow(input.target.x - dron.x, 2) + pow(input.target.y - dron.y, 2));

  bool is_close_distance = hight_distance + input.acceleration_path > distance_to_target;

  DropSolution drop_solution;

  const double EPSILON = 1e-6;
  if (is_close_distance) {
    if (fabs(distance_to_target) < EPSILON) {
      dron.x = input.target.x - (hight_distance + input.acceleration_path);
      dron.y = input.target.y;
      distance_to_target = hight_distance + input.acceleration_path;
    }
    else {
      dron.x = input.target.x - (input.target.x - dron.x) * (hight_distance + input.acceleration_path) / distance_to_target;
      dron.y = input.target.y - (input.target.y - dron.y) * (hight_distance + input.acceleration_path) / distance_to_target;
      distance_to_target = sqrt(pow(input.target.x - dron.x, 2) + pow(input.target.y - dron.y, 2));
    }

    drop_solution.tmp_x = dron.x;
    drop_solution.tmp_y = dron.y;
  }

  double ration = (distance_to_target - hight_distance) / distance_to_target;
  drop_solution.fire_x = dron.x + (input.target.x - dron.x) * ration;
  drop_solution.fire_y = dron.y + (input.target.y - dron.y) * ration;

  return drop_solution;
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)