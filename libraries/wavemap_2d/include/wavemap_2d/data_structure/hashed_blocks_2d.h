#ifndef WAVEMAP_2D_DATA_STRUCTURE_HASHED_BLOCKS_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_HASHED_BLOCKS_2D_H_

#include <wavemap_common/data_structure/volumetric/hashed_blocks.h>

#include "wavemap_2d/data_structure/volumetric_data_structure_2d.h"

namespace wavemap {
template <typename CellT>
class HashedBlocks2D : public HashedBlocks<CellT, 2>,
                       public VolumetricDataStructure2D {
 public:
  using HashedBlocks<CellT, 2>::HashedBlocks;
  using VolumetricDataStructure2D::VolumetricDataStructure2D;

  cv::Mat getImage(bool /* use_color */) const override {
    // TODO(victorr): Implement this
    return {};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_HASHED_BLOCKS_2D_H_
