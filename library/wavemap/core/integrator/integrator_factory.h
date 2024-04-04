#ifndef WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
#define WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/core/integrator/integrator_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/thread_pool.h"

namespace wavemap {
class IntegratorFactory {
 public:
  static IntegratorBase::Ptr create(
      const param::Value& params, MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr,
      std::optional<IntegratorType> default_integrator_type = std::nullopt);

  static IntegratorBase::Ptr create(
      IntegratorType integrator_type, const param::Value& params,
      MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool = nullptr);
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
