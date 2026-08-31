#include "uav_ai/factory.hpp"

#include "uav_ai/analytical_solver.hpp"
#include "uav_ai/file_config_loader.hpp"
#include "uav_ai/json_target_provider.hpp"
#include "uav_ai/table_solver.hpp"

namespace uav {

auto createSolver(SolverType type) -> std::unique_ptr<IBallisticSolver> {
  switch (type) {
  case SolverType::ANALYTICAL:
    return std::make_unique<AnalyticalSolver>();
  case SolverType::TABLE:
    return std::make_unique<TableSolver>();
  }
  return nullptr;
}

auto createProvider(ProviderType type, const std::string &path)
    -> std::unique_ptr<ITargetProvider> {
  std::unique_ptr<ITargetProvider> provider;
  switch (type) {
  case ProviderType::JSON:
    provider = std::make_unique<JsonTargetProvider>();
    break;
  }
  if (provider != nullptr && !path.empty()) {
    provider->load(path);
  }
  return provider;
}

auto createLoader(LoaderType type) -> std::unique_ptr<IConfigLoader> {
  switch (type) {
  case LoaderType::FILE:
    return std::make_unique<FileConfigLoader>();
  }
  return nullptr;
}

} // namespace uav
