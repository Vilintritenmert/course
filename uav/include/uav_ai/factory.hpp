#pragma once

#include <memory>
#include <string>

#include "uav_ai/interfaces.hpp"

namespace uav {

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

// Кожна фабрична функція повертає новий об'єкт через unique_ptr на
// інтерфейс - викликаючий код не знає конкретний тип, володіння переходить
// разом з поверненим значенням. Коли з'явиться нова реалізація, достатньо
// додати один case тут - решта коду не зміниться.

auto createSolver(SolverType type) -> std::unique_ptr<IBallisticSolver>;
auto createProvider(ProviderType type, const std::string &path = {})
    -> std::unique_ptr<ITargetProvider>;
auto createLoader(LoaderType type) -> std::unique_ptr<IConfigLoader>;

} // namespace uav
