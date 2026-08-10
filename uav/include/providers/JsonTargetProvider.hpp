#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ITargetProvider.hpp"
#include "TimeManagement.hpp"

using json = nlohmann::json;

class JSONTargetProvider : public ITargetProvider {
private:
  std::shared_ptr<TimeManagement> _timeManagement;
  std::vector<std::vector<Position>> _targets;

  void initJsonTargets(const std::string jsonFilePath);

public:
  JSONTargetProvider(std::shared_ptr<TimeManagement> timeManagement,
                     const std::string &jsonFilePath);

  Position getTarget(int index) override;

  int getTargetCount() override { return _targets.size(); }

  ~JSONTargetProvider() = default;
};