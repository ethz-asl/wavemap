#include "wavemap/pipeline/pipeline.h"

#include "wavemap/core/integrator/integrator_factory.h"

namespace wavemap {
void Pipeline::clear() {
  clearIntegrators();
  clearOperations();
}

bool Pipeline::hasIntegrator(const std::string& integrator_name) const {
  return integrators_.count(integrator_name);
}

bool Pipeline::eraseIntegrator(const std::string& integrator_name) {
  return integrators_.erase(integrator_name);
}

IntegratorBase* Pipeline::getIntegrator(const std::string& integrator_name) {
  if (auto it = integrators_.find(integrator_name); it != integrators_.end()) {
    return it->second.get();
  }
  return nullptr;
}

IntegratorBase* Pipeline::addIntegrator(const std::string& integrator_name,
                                        const param::Value& integrator_params) {
  auto integrator = IntegratorFactory::create(integrator_params, occupancy_map_,
                                              thread_pool_);
  return addIntegrator(integrator_name, std::move(integrator));
}

IntegratorBase* Pipeline::addIntegrator(
    const std::string& integrator_name,
    std::unique_ptr<IntegratorBase> integrator) {
  if (integrator) {
    auto [it, success] =
        integrators_.try_emplace(integrator_name, std::move(integrator));
    if (success) {
      return it->second.get();
    } else {
      LOG(WARNING) << "Could not add integrator. Another integrator with the "
                      "same name already exists.";
    }
  }

  LOG(WARNING) << "Ignoring request to add integrator. "
                  "Integrator is null pointer.";
  return nullptr;
}

MapOperationBase* Pipeline::addOperation(const param::Value& operation_params) {
  auto operation_handler =
      MapOperationFactory::create(operation_params, occupancy_map_);
  return addOperation(std::move(operation_handler));
}

MapOperationBase* Pipeline::addOperation(
    std::unique_ptr<MapOperationBase> operation) {
  return operation ? operations_.emplace_back(std::move(operation)).get()
                   : nullptr;
}

void Pipeline::runOperations(bool force_run_all) {
  for (auto& operation : operations_) {
    operation->run(force_run_all);
  }
}
}  // namespace wavemap
