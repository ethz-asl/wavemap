#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_

#include <utility>

#include <wavemap_common/indexing/ndtree_index.h>

#include "wavemap_2d/data_structure/volumetric_data_structure.h"

namespace wavemap {
class VolumetricQuadtreeInterface : public VolumetricDataStructure {
 public:
  static constexpr QuadtreeIndex::Element kMaxHeight = 14;

  using VolumetricDataStructure::VolumetricDataStructure;
  ~VolumetricQuadtreeInterface() override = default;

  virtual QuadtreeIndex::ChildArray getFirstChildIndices() const = 0;

  virtual Index2D getMinPossibleIndex() const = 0;
  virtual Index2D getMaxPossibleIndex() const = 0;

  using VolumetricDataStructure::setCellValue;
  virtual void setCellValue(const QuadtreeIndex& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructure::addToCellValue;
  virtual void addToCellValue(const QuadtreeIndex& index,
                              FloatingPoint update) = 0;

  using VolumetricDataStructure::forEachLeaf;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
