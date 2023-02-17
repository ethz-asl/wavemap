#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_INTERFACE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_INTERFACE_H_

#include <memory>
#include <utility>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <int dim>
class VolumetricNdtreeInterface
    : public virtual VolumetricDataStructureBase<dim> {
 public:
  using Ptr = std::shared_ptr<VolumetricNdtreeInterface>;

  // TODO(victorr): Make this configurable
  static constexpr NdtreeIndexElement kMaxHeight = 14;

  using VolumetricDataStructureBase<dim>::VolumetricDataStructureBase;

  virtual typename NdtreeIndex<dim>::ChildArray getFirstChildIndices()
      const = 0;

  virtual Index<dim> getMinPossibleIndex() const = 0;
  virtual Index<dim> getMaxPossibleIndex() const = 0;

  using VolumetricDataStructureBase<dim>::setCellValue;
  virtual void setCellValue(const NdtreeIndex<dim>& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructureBase<dim>::addToCellValue;
  virtual void addToCellValue(const NdtreeIndex<dim>& index,
                              FloatingPoint update) = 0;

  using VolumetricDataStructureBase<dim>::forEachLeaf;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_INTERFACE_H_
