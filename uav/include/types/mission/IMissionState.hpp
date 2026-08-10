#pragma once

#include <memory>

#include "MissionEngagement.hpp"

class MissionProcessor;

class IMissionState {
public:
  virtual ~IMissionState() = default;

  virtual std::unique_ptr<IMissionState>
      execute(MissionProcessor &mp, const MissionEngagement &engagement) = 0;

  virtual const char *name() const = 0;
};
