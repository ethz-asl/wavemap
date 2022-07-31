#ifndef WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_INTERFACE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_INTERFACE_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric/wavelet_ndtree_interface.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class WaveletQuadtreeInterface : public virtual WaveletNdtreeInterface<2>,
                                 public VolumetricDataStructure2D {
 public:
  using Ptr = std::shared_ptr<WaveletQuadtreeInterface>;

  using WaveletNdtreeInterface<2>::WaveletNdtreeInterface;
  using VolumetricDataStructure2D::VolumetricDataStructure2D;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_INTERFACE_H_
