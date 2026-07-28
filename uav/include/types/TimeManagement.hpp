#pragma once

class TimeManagement {
private:
  float _simTimeStep, _arrayTimeStep;
  float _currentTime = 0.0f;

public:
  TimeManagement(float simTimeStep, float arrayTimeStep);

  float getCurrentTime() { return _currentTime; };

  float getArrayTimeStep() { return _arrayTimeStep; };

  void tick() { _currentTime += _simTimeStep; };

  void reset() { _currentTime = 0.0f; };
};