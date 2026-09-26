#pragma once

#include "IDroneState.hpp"

class StateDecelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) override;

  const char *name() const override { return "Decelerating"; }

  DroneState id() const override { return DECELERATING; }
};
