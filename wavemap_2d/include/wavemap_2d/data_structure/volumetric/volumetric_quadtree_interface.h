#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_QUADTREE_INTERFACE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_QUADTREE_INTERFACE_H_

#include <utility>

#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
class VolumetricQuadtreeInterface : public VolumetricDataStructure {
 public:
  template <typename... ConstructorArgs>
  explicit VolumetricQuadtreeInterface(ConstructorArgs&&... args)
      : VolumetricDataStructure(std::forward<ConstructorArgs>(args)...) {}
  ~VolumetricQuadtreeInterface() override = default;

  virtual Index getMinPossibleIndex() const = 0;
  virtual Index getMaxPossibleIndex() const = 0;
  virtual QuadtreeIndex::Element getMaxDepth() const = 0;
  virtual FloatingPoint getRootNodeWidth() const = 0;

  using VolumetricDataStructure::setCellValue;
  virtual void setCellValue(const QuadtreeIndex& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructure::addToCellValue;
  virtual void addToCellValue(const QuadtreeIndex& index,
                              FloatingPoint update) = 0;

  using VolumetricDataStructure::forEachLeaf;

  virtual FloatingPoint computeNodeWidthAtDepth(
      QuadtreeIndex::Element depth) = 0;
  virtual Vector computeNodeHalvedDiagonalAtDepth(
      QuadtreeIndex::Element depth) = 0;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_QUADTREE_INTERFACE_H_
