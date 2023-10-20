#ifndef WAVEMAP_MAP_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_
#define WAVEMAP_MAP_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_

#include <string>

#include "wavemap/map/volumetric_data_structure_base.h"

namespace wavemap {
class VolumetricDataStructureFactory {
 public:
  static VolumetricDataStructureBase::Ptr create(
      const param::Value& params,
      std::optional<VolumetricDataStructureType> default_data_structure_type =
          std::nullopt);

  static VolumetricDataStructureBase::Ptr create(
      VolumetricDataStructureType data_structure_type,
      const param::Value& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_MAP_VOLUMETRIC_DATA_STRUCTURE_FACTORY_H_
