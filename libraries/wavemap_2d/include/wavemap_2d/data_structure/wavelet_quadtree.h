#ifndef WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_H_

#include <wavemap_common/data_structure/volumetric/wavelet_ndtree.h>

#include "wavemap_2d/data_structure/wavelet_quadtree_interface.h"

namespace wavemap {
template <typename CellT>
class WaveletQuadtree : public WaveletNdtree<CellT, 2>,
                        public WaveletQuadtreeInterface {
 public:
  using WaveletNdtree<CellT, 2>::WaveletNdtree;
  using WaveletQuadtreeInterface::WaveletQuadtreeInterface;

  cv::Mat getImage(bool /* use_color */) const override {
    // TODO(victorr): Implement this
    return {};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_WAVELET_QUADTREE_H_
