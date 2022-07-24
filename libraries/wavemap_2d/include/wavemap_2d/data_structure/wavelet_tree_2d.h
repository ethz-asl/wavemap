#ifndef WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_2D_H_

#include <wavemap_common/data_structure/volumetric/wavelet_tree.h>

#include "wavemap_2d/data_structure/wavelet_tree_interface_2d.h"

namespace wavemap {
template <typename CellT>
class WaveletTree2D : public WaveletTree<CellT, 2>,
                      public WaveletTreeInterface2D {
 public:
  using WaveletTree<CellT, 2>::WaveletTree;
  using WaveletTreeInterface2D::WaveletTreeInterface2D;

  cv::Mat getImage(bool /* use_color */) const override {
    // TODO(victorr): Implement this
    return {};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_2D_H_
