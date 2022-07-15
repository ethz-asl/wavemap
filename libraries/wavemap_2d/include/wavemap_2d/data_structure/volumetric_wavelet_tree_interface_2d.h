#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_2D_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric/volumetric_wavelet_tree_interface.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
class VolumetricWaveletTreeInterface2D
    : public VolumetricWaveletTreeInterface<2>,
      public VolumetricDataStructure2D {
 public:
  using Ptr = std::shared_ptr<VolumetricWaveletTreeInterface2D>;
  using VolumetricWaveletTreeInterface<2>::VolumetricWaveletTreeInterface;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_WAVELET_TREE_INTERFACE_2D_H_
