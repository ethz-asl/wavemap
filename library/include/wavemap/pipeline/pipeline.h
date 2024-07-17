#ifndef WAVEMAP_PIPELINE_PIPELINE_H_
#define WAVEMAP_PIPELINE_PIPELINE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "wavemap/core/integrator/integrator_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"
#include "wavemap/pipeline/map_operations/map_operation_base.h"
#include "wavemap/pipeline/map_operations/map_operation_factory.h"

namespace wavemap {
class Pipeline {
 public:
  using IntegratorMap =
      std::unordered_map<std::string, std::unique_ptr<IntegratorBase>>;
  using OperationsArray = std::vector<std::unique_ptr<MapOperationBase>>;

  explicit Pipeline(MapBase::Ptr occupancy_map,
                    std::shared_ptr<ThreadPool> thread_pool = nullptr)
      : occupancy_map_(std::move(occupancy_map)),
        thread_pool_(thread_pool ? std::move(thread_pool)
                                 : std::make_shared<ThreadPool>()) {}

  void clear();

  bool hasIntegrator(const std::string& integrator_name) const;
  bool eraseIntegrator(const std::string& integrator_name);
  IntegratorBase* addIntegrator(const std::string& integrator_name,
                                const param::Value& integrator_params);
  IntegratorBase* addIntegrator(const std::string& integrator_name,
                                std::unique_ptr<IntegratorBase> integrator);
  IntegratorBase* getIntegrator(const std::string& integrator_name);
  const IntegratorMap& getIntegrators() { return integrators_; }
  void clearIntegrators() { integrators_.clear(); }

  MapOperationBase* addOperation(const param::Value& operation_params);
  MapOperationBase* addOperation(std::unique_ptr<MapOperationBase> operation);
  const OperationsArray& getOperations() { return operations_; }
  void clearOperations() { operations_.clear(); }

  template <typename MeasurementT>
  bool runIntegrators(const std::vector<std::string>& integrator_names,
                      const MeasurementT& measurement);
  void runOperations(bool force_run_all = false);

  template <typename MeasurementT>
  bool runPipeline(const std::vector<std::string>& integrator_names,
                   const MeasurementT& measurement);

 private:
  // Map data structure
  const MapBase::Ptr occupancy_map_;

  // Threadpool shared among all input handlers and operations
  const std::shared_ptr<ThreadPool> thread_pool_;

  // Measurement integrators that update the map
  IntegratorMap integrators_;

  // Operations to perform after map updates
  OperationsArray operations_;
};
}  // namespace wavemap

#include "wavemap/pipeline/impl/pipeline_inl.h"

#endif  // WAVEMAP_PIPELINE_PIPELINE_H_
