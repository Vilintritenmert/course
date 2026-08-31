#pragma once

#include <string>

#include "uav_ai/types.hpp"

namespace uav {

// Завантажувач місії: читає конфіг дрона/симуляції та параметри обраного
// боєприпаса. Реалізації: FileConfigLoader.
class IConfigLoader {
public:
  virtual auto load(const std::string &dataDir) -> bool = 0;
  virtual auto getConfig() const -> const DroneConfig & = 0;
  virtual auto getAmmoParams() const -> const AmmoParams & = 0;
  virtual ~IConfigLoader() = default;
};

// Провайдер цілей: кількість цілей та їхній стан (позиція, швидкість) на
// поточний момент часу симуляції. Реалізації: JsonTargetProvider.
class ITargetProvider {
public:
  virtual auto load(const std::string &filePath) -> bool = 0;
  virtual auto getTargetCount() const -> int = 0;
  virtual auto getTarget(int index) const -> Target = 0;

  // Крок часу між точками траєкторії в джерелі даних (з конфігу місії,
  // не завжди зберігається разом із самими траєкторіями).
  virtual void setArrayTimeStep(float dt) = 0;

  // Просуває внутрішній час провайдера на dt (впливає на наступні getTarget).
  virtual void advance(float dt) = 0;
  // Повертає внутрішній час провайдера до нуля.
  virtual void reset() = 0;

  virtual ~ITargetProvider() = default;
};

// Калькулятор балістики: за поточною позицією дрона і станом цілі обчислює
// точку скиду (lead targeting). init() задає параметри, спільні для всієї
// симуляції (боєприпас, швидкість атаки, висота, шлях розгону).
// Реалізації: AnalyticalSolver.
class IBallisticSolver {
public:
  virtual auto init(const AmmoParams &ammo, float attackSpeed, float altitude,
                     float accelPath) -> bool = 0;
  virtual auto solve(const Coord &dronePos, const Target &target) const
      -> TargetCandidate = 0;

  // Горизонтальна дистанція, яку боєприпас пролітає під час падіння
  // (визначається лише ammo/attackSpeed/altitude, задається в init()).
  virtual auto getHorizontalDistance() const -> float = 0;

  virtual ~IBallisticSolver() = default;
};

} // namespace uav
