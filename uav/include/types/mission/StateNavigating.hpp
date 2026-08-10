#pragma once

#include "IMissionState.hpp"

class StateNavigating : public IMissionState {
public:
  std::unique_ptr<IMissionState>
      execute(MissionProcessor &mp, const MissionEngagement &engagement) override;

  const char *name() const override { return "Navigating"; }
};
