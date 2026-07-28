#pragma once

class Position {
private:
  float _x, _y;

public:
  Position(const float x = 0, const float y = 0) : _x(x), _y(y){};

  float getX() const { return _x; };
  float getY() const { return _y; };

  void setX(const float x) { _x = x; }; 
  void setY(const float y) { _y = y; };

  Position operator+(const Position &o) const;

  Position operator-(const Position &o) const;

  Position operator*(const float s);
  Position operator/(const float s);
  bool operator==(const Position &o);

  static float distance(const Position position);

  static Position normalize(Position c);

  ~Position() = default;
};
