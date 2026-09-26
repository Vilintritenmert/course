#pragma once

#include <memory>
#include <string>

#include "uav_v2/interfaces.hpp"

namespace uav {

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

auto createSolver(SolverType type) -> std::unique_ptr<IBallisticSolver>;
auto createProvider(ProviderType type, const std::string &path = {})
    -> std::unique_ptr<ITargetProvider>;
auto createLoader(LoaderType type) -> std::unique_ptr<IConfigLoader>;

}
