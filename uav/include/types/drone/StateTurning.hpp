#pragma once

#include "IDroneState.hpp"

class StateTurning : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext &ctx) override;

  const char *name() const override { return "Turning"; }

  DroneState id() const override { return TURNING; }
};
