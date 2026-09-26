#pragma once

#include <string>

using namespace std;

struct Coords {
  double x = 0., y = 0., z = 0.;
};

struct BallisticsInput {
  Coords initial_dron;
  Coords target;
  double attack_speed = 0;
  double acceleration_path = 0;
  string ammo_name = "";
};

struct DropSolution {
  double fire_x = 0, fire_y = 0;
  double tmp_x = 0, tmp_y = 0;
};

struct AmmoParameters {
  double m = 0, d = 0, l = 0;
  auto operator==(const AmmoParameters& other) const { return m == other.m && d == other.d && l == other.l; }
};

auto getAmmoParameters(const string& ammo_name) -> AmmoParameters;

auto computeDropSolution(const BallisticsInput& input) -> DropSolution;