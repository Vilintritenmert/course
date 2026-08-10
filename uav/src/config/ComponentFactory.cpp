#include "ComponentFactory.hpp"
#include "AnalyticalSolver.hpp"
#include "FileConfigLoader.hpp"
#include "JsonTargetProvider.hpp"
#include "TableSolver.hpp"
#include <memory>

ComponentFactory::ComponentFactory(std::shared_ptr<ConfigLoaderOptions> options)
    : _options(options) {
  initDefaultData();
};

void ComponentFactory::initDefaultData() {
  _configLoader = createLoader(LoaderType::FILE);
  _configLoader->loadConfig(_options);
  _timeManagement = std::make_unique<TimeManagement>(_configLoader->getConfig()->getSimTimeStep(),
                         _configLoader->getConfig()->getArrayTimeStep());
};

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type) {
  if (type == SolverType::ANALYTICAL) {
    return std::make_unique<AnalyticalSolver>();
  } 
  if (type == SolverType::TABLE) {
    return std::make_unique<TableSolver>(_options->getBallistikTable());
  }

  throw FactoryLoaderException("Unknown solver type");
};

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType type) {
  if (type == ProviderType::JSON) {
    return std::make_unique<JSONTargetProvider>(_timeManagement,
                                  _options->getTargetConfigFilePath());
  }

  throw FactoryLoaderException("Unknown provider type");
};

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(LoaderType type) {
  if (type == LoaderType::FILE) {
    return std::make_unique<JsonConfigLoader>();
  }

  throw FactoryLoaderException("Unknown loader type");
};

std::shared_ptr<TimeManagement> ComponentFactory::getTimeManagement() const {
  return _timeManagement;
};

std::shared_ptr<IConfigLoader> ComponentFactory::getConfigLoader() const {
  return _configLoader;
};