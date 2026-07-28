#pragma once

#include <string>

class AmmoParams {
private:
  std::string _name;
  float _mass = 0, _drag = 0, _lift = 0;

public:
  AmmoParams(std::string name, float mass, float drag, float lift);
  std::string getName() const { return _name; };

  float getMass() const { return _mass; };
  float getDrag() const { return _drag; };
  float getLift() const { return _lift; };

  ~AmmoParams() = default;
};