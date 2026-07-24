#pragma once

#include "json_helper.hpp"
#include "types.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::json;

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Position getTarget(int index) = 0;
  virtual ~ITargetProvider() = default;
};

class JSONTargetProvider : public ITargetProvider {
private:
  TimeManagement *_timeManagement;
  std::vector<std::vector<Position>> _targets;

  void initJsonTargets(const std::string jsonFilePath) {
    json rawData = loadJsonFile(jsonFilePath);

    _targets.clear();

    int targetCount = rawData["targetCount"];
    for (int i = 0; i < targetCount; ++i) {
      std::vector<Position> targetPositions;
      for (const auto &pos : rawData["targets"][i]["positions"]) {
        targetPositions.emplace_back(Position{pos["x"], pos["y"]});
      }

      _targets.push_back(targetPositions);
    }
  }

public:
  JSONTargetProvider(TimeManagement *timeManagement,
                     const std::string &jsonFilePath)
      : _timeManagement(timeManagement) {
    initJsonTargets(jsonFilePath);
  }

  int getTargetCount() override { return _targets.size(); }

  Position getTarget(int index) override {
    int rawIdx = (int)floorf(_timeManagement->getCurrentTime() /
                             _timeManagement->getArrayTimeStep());
    int idx = rawIdx % _timeManagement->getTimeStep();
    int next = (idx + 1) % _timeManagement->getTimeStep();
    float frac = (_timeManagement->getCurrentTime() -
                  rawIdx * _timeManagement->getArrayTimeStep()) /
                 _timeManagement->getArrayTimeStep();

    return _targets[index][idx] +
           (_targets[index][next] - _targets[index][idx]) * frac;

  } 

  ~JSONTargetProvider() = default;
};