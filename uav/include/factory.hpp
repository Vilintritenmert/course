#pragma once

#include <memory>
#include <string>

#include "interfaces.hpp"

namespace uav {

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON, THREADED };
enum class LoaderType { FILE };

auto createSolver(SolverType type) -> std::unique_ptr<IBallisticSolver>;
auto createProvider(ProviderType type, const std::string &path = {})
    -> std::unique_ptr<ITargetProvider>;
auto createLoader(LoaderType type) -> std::unique_ptr<IConfigLoader>;

} // namespace uav
