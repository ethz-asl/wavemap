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
/*
 * A class to build pipelines of measurement integrators and map operations
 */
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

  // Copy construction is not supported
  Pipeline(const Pipeline&) = delete;

  //! Deregister all measurement integrators and map operations
  void clear();

  //! Returns true if an integrator with the given name has been registered
  bool hasIntegrator(const std::string& integrator_name) const;
  //! Deregister the integrator with the given name. Returns true if it existed.
  bool eraseIntegrator(const std::string& integrator_name);
  //! Create and register a new integrator
  IntegratorBase* addIntegrator(const std::string& integrator_name,
                                const param::Value& integrator_params);
  //! Register the given integrator, transferring ownership
  IntegratorBase* addIntegrator(const std::string& integrator_name,
                                std::unique_ptr<IntegratorBase> integrator);
  //! Get a pointer to the given integrator, returns nullptr if it does not
  //! exist
  IntegratorBase* getIntegrator(const std::string& integrator_name);
  //! Access all registered integrators (read-only)
  const IntegratorMap& getIntegrators() { return integrators_; }
  //! Deregister all integrators
  void clearIntegrators() { integrators_.clear(); }

  //! Create and register a new map operation
  MapOperationBase* addOperation(const param::Value& operation_params);
  //! Register the given map operation, transferring ownership
  MapOperationBase* addOperation(std::unique_ptr<MapOperationBase> operation);
  //! Access all registered map operations (read-only)
  const OperationsArray& getOperations() { return operations_; }
  //! Deregister all map operations
  void clearOperations() { operations_.clear(); }

  //! Integrate a given measurement
  template <typename MeasurementT>
  bool runIntegrators(const std::vector<std::string>& integrator_names,
                      const MeasurementT& measurement);
  //! Run the map operations
  void runOperations(bool force_run_all = false);

  //! Integrate a given measurement, then run the map operations
  template <typename MeasurementT>
  bool runPipeline(const std::vector<std::string>& integrator_names,
                   const MeasurementT& measurement);

 private:
  //! Map data structure
  const MapBase::Ptr occupancy_map_;

  //! Threadpool shared among all input handlers and operations
  const std::shared_ptr<ThreadPool> thread_pool_;

  //! Measurement integrators that update the map
  IntegratorMap integrators_;

  //! Operations to perform after map updates
  OperationsArray operations_;
};
}  // namespace wavemap

#include "wavemap/pipeline/impl/pipeline_inl.h"

#endif  // WAVEMAP_PIPELINE_PIPELINE_H_
