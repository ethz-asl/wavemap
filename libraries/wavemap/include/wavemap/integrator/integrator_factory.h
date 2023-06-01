#ifndef WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
#define WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_

#include <string>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/integrator/integrator_base.h"

namespace wavemap {
class IntegratorFactory {
 public:
  static IntegratorBase::Ptr create(
      const param::Map& params, VolumetricDataStructureBase::Ptr occupancy_map,
      std::optional<IntegratorType> default_integrator_type = std::nullopt);

  static IntegratorBase::Ptr create(
      IntegratorType integrator_type, const param::Map& params,
      VolumetricDataStructureBase::Ptr occupancy_map);
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_INTEGRATOR_FACTORY_H_
