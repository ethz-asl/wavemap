#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_

#include <string>

#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/volumetric/volumetric_octree_interface.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <typename CellT>
class VolumetricOctree : public virtual VolumetricOctreeInterface {
 public:
  using CellType = CellT;
  using NodeType = NdtreeNode<typename CellT::Specialized, kDim>;
  static constexpr bool kRequiresPruningForThresholding = true;

  // Use the base class' constructor
  using VolumetricOctreeInterface::VolumetricOctreeInterface;

  bool empty() const override { return ndtree_.empty(); }
  size_t size() const override { return ndtree_.size(); }
  void prune() override;
  void clear() override { return ndtree_.clear(); }

  typename OctreeIndex::ChildArray getFirstChildIndices() const override;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinPossibleIndex() const override;
  Index3D getMaxPossibleIndex() const override;

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value) override;
  void addToCellValue(const Index3D& index, FloatingPoint update) override;
  void addToCellValue(const OctreeIndex& index, FloatingPoint update) override;

  void forEachLeaf(
      typename VolumetricDataStructureBase::IndexedLeafVisitorFunction
          visitor_fn) const override;

  NodeType& getRootNode() { return ndtree_.getRootNode(); }
  const NodeType& getRootNode() const { return ndtree_.getRootNode(); }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() {
    return ndtree_.template getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() const {
    return ndtree_.template getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const override { return ndtree_.getMemoryUsage(); }

  bool save(const std::string& file_path_prefix) const override;
  bool load(const std::string& file_path_prefix) override;

 private:
  struct StackElement {
    const OctreeIndex node_index;
    const NodeType& node;
    const typename CellT::Specialized parent_value{};
  };

  Ndtree<typename CellT::Specialized, kDim,
         VolumetricOctreeInterface::kMaxHeight>
      ndtree_;

  static OctreeIndex getInternalRootNodeIndex() {
    return OctreeIndex{VolumetricOctreeInterface::kMaxHeight,
                       OctreeIndex::Position::Zero()};
  }
  const OctreeIndex root_node_index_offset_{
      VolumetricOctreeInterface::kMaxHeight - 1, OctreeIndex::Position::Ones()};
  const Index3D root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  OctreeIndex toInternal(const Index3D& index) const {
    return convert::indexAndHeightToNodeIndex<kDim>(index + root_index_offset_,
                                                    0);
  }
  OctreeIndex toInternal(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index3D toExternalIndex(const Index3D& index) const {
    return index - root_index_offset_;
  }
  OctreeIndex toExternalNodeIndex(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }

  const NodeType* getDeepestNodeAtIndex(const Index3D& index) const;

  template <typename T>
  friend class VolumetricOctreeTest_IndexConversions_Test;
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/volumetric_octree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_OCTREE_H_
