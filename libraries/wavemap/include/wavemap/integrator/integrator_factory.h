#ifndef WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
#define WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/integrator_base.h"
#include "wavemap/utils/thread_pool.h"

namespace wavemap {
class IntegratorFactory {
 public:
  static IntegratorBase::Ptr create(
      const param::Value& params,
      VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool,
      std::optional<IntegratorType> default_integrator_type = std::nullopt);

  static IntegratorBase::Ptr create(
      IntegratorType integrator_type, const param::Value& params,
      VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool);
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
