#pragma once

#include <memory>
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

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

class ComponentFactory {
  std::shared_ptr<ConfigLoaderOptions> _options;
  std::shared_ptr<TimeManagement> _timeManagement;
  std::shared_ptr<IConfigLoader> _configLoader;

public:
  ComponentFactory(std::shared_ptr<ConfigLoaderOptions> options);

  void initDefaultData();

  std::unique_ptr<IBallisticSolver> createSolver(SolverType type);

  std::unique_ptr<ITargetProvider> createProvider(ProviderType type);

  std::unique_ptr<IConfigLoader> createLoader(LoaderType type);

  std::shared_ptr<TimeManagement> getTimeManagement() const;

  std::shared_ptr<IConfigLoader> getConfigLoader() const;

  ~ComponentFactory() = default ;
};