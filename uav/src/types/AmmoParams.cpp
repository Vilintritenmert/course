#include "AmmoParams.hpp"

AmmoParams::AmmoParams(std::string name, float mass, float drag, float lift)
    : _name(name), _mass(mass), _drag(drag), _lift(lift) {}