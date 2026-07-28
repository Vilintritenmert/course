#include "ComponentFactory.hpp"
#include "AnalyticalSolver.hpp"
#include "FileConfigLoader.hpp"
#include "JsonTargetProvider.hpp"

ComponentFactory::ComponentFactory(ConfigLoaderOptions options)
    : _options(options) {
  initDefaultData();
};

void ComponentFactory::initDefaultData() {
  _configLoader = createLoader(LoaderType::FILE);
  _configLoader->loadConfig(&_options);
  _timeManagement =
      new TimeManagement(_configLoader->getConfig()->getSimTimeStep(),
                         _configLoader->getConfig()->getArrayTimeStep());
};

IBallisticSolver *ComponentFactory::createSolver(SolverType type) {
  if (type == SolverType::ANALYTICAL) {
    return new AnalyticalSolver();
  }

  throw FactoryLoaderException("Unknown solver type");
};

ITargetProvider *ComponentFactory::createProvider(ProviderType type) {
  if (type == ProviderType::JSON) {
    return new JSONTargetProvider(_timeManagement,
                                  _options.getTargetConfigFilePath());
  }

  throw FactoryLoaderException("Unknown provider type");
};

IConfigLoader *ComponentFactory::createLoader(LoaderType type) {
  if (type == LoaderType::FILE) {
    return new JsonConfigLoader();
  }

  throw FactoryLoaderException("Unknown loader type");
};

TimeManagement *ComponentFactory::getTimeManagement() const {
  return _timeManagement;
};

IConfigLoader *ComponentFactory::getConfigLoader() const {
  return _configLoader;
};

ComponentFactory::~ComponentFactory() {
  delete _timeManagement;
  delete _configLoader;
};