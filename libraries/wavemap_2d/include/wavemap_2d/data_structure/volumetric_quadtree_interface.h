#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric/volumetric_ndtree_interface.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class VolumetricQuadtreeInterface : public virtual VolumetricNdtreeInterface<2>,
                                    public VolumetricDataStructure2D {
 public:
  using Ptr = std::shared_ptr<VolumetricQuadtreeInterface>;
  using VolumetricNdtreeInterface<2>::VolumetricNdtreeInterface;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
