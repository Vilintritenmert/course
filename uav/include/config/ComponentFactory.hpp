#pragma once

#include <string>

#include "TimeManagement.hpp"
#include "IBallisticSolver.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"

class FactoryLoaderException : public std::exception {
private:
  std::string _message;

public:
  FactoryLoaderException(const std::string &message) : _message(message) {}
  const char *what() const throw() { return _message.c_str(); }
};

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

class ComponentFactory {
  ConfigLoaderOptions _options;
  TimeManagement *_timeManagement;
  IConfigLoader *_configLoader;

public:
  ComponentFactory(ConfigLoaderOptions options);

  void initDefaultData();

  IBallisticSolver *createSolver(SolverType type);

  ITargetProvider *createProvider(ProviderType type);

  IConfigLoader *createLoader(LoaderType type);

  TimeManagement *getTimeManagement() const;

  IConfigLoader *getConfigLoader() const;

  ~ComponentFactory();
};