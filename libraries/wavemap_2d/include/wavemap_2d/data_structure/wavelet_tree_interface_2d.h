#ifndef WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_INTERFACE_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_INTERFACE_2D_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric/wavelet_tree_interface.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class WaveletTreeInterface2D : public WaveletTreeInterface<2>,
                               public VolumetricDataStructure2D {
 public:
  using Ptr = std::shared_ptr<WaveletTreeInterface2D>;

  using WaveletTreeInterface<2>::WaveletTreeInterface;
  using VolumetricDataStructure2D::VolumetricDataStructure2D;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_INTERFACE_2D_H_
