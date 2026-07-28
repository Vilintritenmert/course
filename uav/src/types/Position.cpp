#include <cmath>

#include "Position.hpp"

Position Position::operator+(const Position &o) const {
  return Position(_x + o._x, _y + o._y);
};

Position Position::operator-(const Position &o) const {
  return Position(_x - o._x, _y - o._y);
};

Position Position::operator*(const float s) {
  return Position(_x * s, _y * s);
};
Position Position::operator/(const float s) { return Position(_x / s, _y / s); }
bool Position::operator==(const Position &o) {
  return _x == o._x && _y == o._y;
}

float Position::distance(const Position position) {
  return sqrtf(position._x * position._x + position._y * position._y);
}

Position Position::normalize(Position c) {
  float l = Position::distance(c);
  return l > 1e-9f ? c / l : Position(0.0f, 0.0f);
}
