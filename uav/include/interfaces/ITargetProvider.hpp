#pragma once

#include "Position.hpp"

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Position getTarget(int index) = 0;
  virtual ~ITargetProvider() = default;
};