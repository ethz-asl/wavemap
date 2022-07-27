#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_H_

#include <memory>
#include <string>

#include <wavemap_common/data_structure/volumetric/volumetric_data_structure_base.h>

namespace wavemap {
class VolumetricDataStructure3D
    : public virtual VolumetricDataStructureBase<3> {
 public:
  using Ptr = std::shared_ptr<VolumetricDataStructure3D>;

  using VolumetricDataStructureBase<3>::VolumetricDataStructureBase;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_3D_H_
