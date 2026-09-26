#pragma once

#include <memory>

#include "DroneContext.hpp"

class IDroneState {
public:
    virtual ~IDroneState() = default;

    virtual std::unique_ptr<IDroneState>
        execute(DroneContext& ctx) = 0;

    virtual const char* name() const = 0;

    virtual DroneState id() const = 0;
};
