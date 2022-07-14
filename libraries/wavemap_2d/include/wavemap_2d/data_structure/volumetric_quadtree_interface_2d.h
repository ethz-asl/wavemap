#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_2D_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric_quadtree_interface.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class VolumetricQuadtreeInterface2D : public VolumetricQuadtreeInterface<2>,
                                      public VolumetricDataStructure2D {
 public:
  using Ptr = std::shared_ptr<VolumetricQuadtreeInterface2D>;
  using VolumetricQuadtreeInterface<2>::VolumetricQuadtreeInterface;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_2D_H_
