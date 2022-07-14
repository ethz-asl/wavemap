#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_

#include <memory>
#include <utility>

#include "wavemap_common/data_structure/volumetric_data_structure_base.h"
#include "wavemap_common/indexing/ndtree_index.h"

namespace wavemap {
template <int dim>
class VolumetricQuadtreeInterface
    : public virtual VolumetricDataStructureBase<dim> {
 public:
  using Ptr = std::shared_ptr<VolumetricQuadtreeInterface>;

  static constexpr QuadtreeIndex::Element kMaxHeight = 14;

  using VolumetricDataStructureBase<dim>::VolumetricDataStructureBase;

  virtual QuadtreeIndex::ChildArray getFirstChildIndices() const = 0;

  virtual Index<dim> getMinPossibleIndex() const = 0;
  virtual Index<dim> getMaxPossibleIndex() const = 0;

  using VolumetricDataStructureBase<dim>::setCellValue;
  virtual void setCellValue(const QuadtreeIndex& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructureBase<dim>::addToCellValue;
  virtual void addToCellValue(const QuadtreeIndex& index,
                              FloatingPoint update) = 0;

  using VolumetricDataStructureBase<dim>::forEachLeaf;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_QUADTREE_INTERFACE_H_
