#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_H_

#include <string>

#include "wavemap_common/data_structure/ndtree/ndtree.h"
#include "wavemap_common/data_structure/volumetric/volumetric_ndtree_interface.h"
#include "wavemap_common/indexing/index_conversions.h"
#include "wavemap_common/indexing/ndtree_index.h"

namespace wavemap {
template <typename CellT, int dim>
class VolumetricNdtree : public virtual VolumetricNdtreeInterface<dim> {
 public:
  using CellType = CellT;
  using NodeType = NdtreeNode<typename CellT::Specialized, dim>;
  static constexpr bool kRequiresPruningForThresholding = true;

  // Use the base class' constructor
  using VolumetricNdtreeInterface<dim>::VolumetricNdtreeInterface;

  bool empty() const override { return ndtree_.empty(); }
  size_t size() const override { return ndtree_.size(); }
  void prune() override;
  void clear() override { return ndtree_.clear(); }

  typename NdtreeIndex<dim>::ChildArray getFirstChildIndices() const override;

  Index<dim> getMinIndex() const override;
  Index<dim> getMaxIndex() const override;
  Index<dim> getMinPossibleIndex() const override;
  Index<dim> getMaxPossibleIndex() const override;

  FloatingPoint getCellValue(const Index<dim>& index) const override;
  void setCellValue(const Index<dim>& index, FloatingPoint new_value) override;
  void setCellValue(const NdtreeIndex<dim>& index,
                    FloatingPoint new_value) override;
  void addToCellValue(const Index<dim>& index, FloatingPoint update) override;
  void addToCellValue(const NdtreeIndex<dim>& index,
                      FloatingPoint update) override;

  void forEachLeaf(
      typename VolumetricDataStructureBase<dim>::IndexedLeafVisitorFunction
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

  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 private:
  struct StackElement {
    const NdtreeIndex<dim> node_index;
    const NodeType& node;
    const typename CellT::Specialized parent_value{};
  };

  Ndtree<typename CellT::Specialized, dim,
         VolumetricNdtreeInterface<dim>::kMaxHeight>
      ndtree_;

  static NdtreeIndex<dim> getInternalRootNodeIndex() {
    return NdtreeIndex<dim>{VolumetricNdtreeInterface<dim>::kMaxHeight,
                            NdtreeIndex<dim>::Position::Zero()};
  }
  const NdtreeIndex<dim> root_node_index_offset_{
      VolumetricNdtreeInterface<dim>::kMaxHeight - 1,
      NdtreeIndex<dim>::Position::Ones()};
  const Index<dim> root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  NdtreeIndex<dim> toInternal(const Index<dim>& index) const {
    return convert::indexAndHeightToNodeIndex<dim>(index + root_index_offset_,
                                                   0);
  }
  NdtreeIndex<dim> toInternal(const NdtreeIndex<dim>& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index<dim> height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index<dim> toExternalIndex(const Index<dim>& index) const {
    return index - root_index_offset_;
  }
  NdtreeIndex<dim> toExternalNodeIndex(
      const NdtreeIndex<dim>& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index<dim> height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }

  const NodeType* getDeepestNodeAtIndex(const Index<dim>& index) const;

  template <typename T>
  friend class VolumetricNdtreeTest_IndexConversions_Test;
};
}  // namespace wavemap

#include "wavemap_common/data_structure/volumetric/impl/volumetric_ndtree_inl.h"

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_NDTREE_H_
