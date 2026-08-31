#pragma once

#include <string>

#include "uav_ai/ballistic_table.hpp"
#include "uav_ai/interfaces.hpp"

namespace uav {

// Табличний розв'язок балістики: замість аналітичного рівняння руху -
// попередньо обчислена 5-вимірна таблиця (Z0, V0, m, d, l) з лінійною
// інтерполяцією між вузлами. Логіка lead targeting (передбачення позиції
// цілі, планування розгону/гальмування) така сама, як у AnalyticalSolver -
// різниться лише джерело fallTime/horizontalDistance.
class TableSolver : public IBallisticSolver {
public:
  // Шлях до файлу таблиці за замовчуванням (відносно робочої директорії
  // запуску, поруч з іншими даними в data/).
  static constexpr const char *kDefaultTablePath = "data/ballistic_table.txt";

  explicit TableSolver(std::string tablePath = kDefaultTablePath);

  auto init(const AmmoParams &ammo, float attackSpeed, float altitude,
            float accelPath) -> bool override;
  auto solve(const Coord &dronePos, const Target &target) const
      -> TargetCandidate override;
  auto getHorizontalDistance() const -> float override {
    return horizontalDistance_;
  }

private:
  std::string tablePath_;
  BallisticTable table_;
  bool tableLoaded_ = false;

  float attackSpeed_ = 0.F;
  float accelPath_ = 0.F;
  float acceleration_ = 0.F;
  float fallTime_ = 0.F;
  float horizontalDistance_ = 0.F;
  bool ready_ = false;
};

} // namespace uav
