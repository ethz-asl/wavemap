#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_INTERFACE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_INTERFACE_H_

#include <memory>
#include <utility>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
class VolumetricOctreeInterface : public virtual VolumetricDataStructureBase {
 public:
  using Ptr = std::shared_ptr<VolumetricOctreeInterface>;

  // TODO(victorr): Make this configurable
  static constexpr NdtreeIndexElement kMaxHeight = 14;

  using VolumetricDataStructureBase::VolumetricDataStructureBase;

  virtual typename OctreeIndex::ChildArray getFirstChildIndices() const = 0;

  virtual Index3D getMinPossibleIndex() const = 0;
  virtual Index3D getMaxPossibleIndex() const = 0;

  using VolumetricDataStructureBase::setCellValue;
  virtual void setCellValue(const OctreeIndex& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructureBase::addToCellValue;
  virtual void addToCellValue(const OctreeIndex& index,
                              FloatingPoint update) = 0;

  using VolumetricDataStructureBase::forEachLeaf;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_INTERFACE_H_
