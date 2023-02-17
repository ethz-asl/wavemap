#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_H_

#include <string>

#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/data_structure/volumetric/wavelet_ndtree_interface.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <typename CellT, int dim>
class WaveletNdtree : public virtual WaveletNdtreeInterface<dim> {
 public:
  static_assert(std::is_same_v<typename CellT::Specialized, FloatingPoint>);

  using CellType = CellT;
  static constexpr bool kRequiresPruningForThresholding = true;

  // Use the base class' constructor
  using WaveletNdtreeInterface<dim>::WaveletNdtreeInterface;

  bool empty() const override { return ndtree_.empty(); }
  size_t size() const override { return ndtree_.size(); }
  void prune() override;
  void clear() override;

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

  typename WaveletNdtreeInterface<dim>::Coefficients::Scale& getRootScale()
      override {
    return root_scale_coefficient_;
  }
  const typename WaveletNdtreeInterface<dim>::Coefficients::Scale&
  getRootScale() const override {
    return root_scale_coefficient_;
  }
  typename WaveletNdtreeInterface<dim>::NodeType& getRootNode() override {
    return ndtree_.getRootNode();
  }
  const typename WaveletNdtreeInterface<dim>::NodeType& getRootNode()
      const override {
    return ndtree_.getRootNode();
  }
  typename WaveletNdtreeInterface<dim>::NodeType* getNode(
      const NdtreeIndex<dim>& node_index) override;
  const typename WaveletNdtreeInterface<dim>::NodeType* getNode(
      const NdtreeIndex<dim>& node_index) const override;

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
    const typename WaveletNdtreeInterface<dim>::NodeType& node;
    const typename WaveletNdtreeInterface<dim>::Coefficients::Scale
        scale_coefficient{};
  };

  typename WaveletNdtreeInterface<dim>::Coefficients::Scale
      root_scale_coefficient_{};
  Ndtree<typename WaveletNdtreeInterface<dim>::Coefficients::Details, dim,
         VolumetricNdtreeInterface<dim>::kMaxHeight - 1>
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
};
}  // namespace wavemap

#include "wavemap/data_structure/volumetric/impl/wavelet_ndtree_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_WAVELET_NDTREE_H_
