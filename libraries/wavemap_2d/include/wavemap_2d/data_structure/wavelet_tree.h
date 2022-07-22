#ifndef WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_H_

#include <string>

#include <wavemap_common/data_structure/ndtree/ndtree.h>
#include <wavemap_common/data_structure/volumetric/cell_types/haar_transform.h>
#include <wavemap_common/indexing/ndtree_index.h>

#include "wavemap_2d/data_structure/wavelet_tree_interface_2d.h"

namespace wavemap {
template <typename CellT>
class WaveletTree : public WaveletTreeInterface2D {
 public:
  static_assert(std::is_same_v<typename CellT::Specialized, FloatingPoint>);

  using CellType = CellT;
  static constexpr bool kRequiresPruningForThresholding = true;

  // Use the base class' constructor
  using WaveletTreeInterface2D::WaveletTreeInterface2D;

  bool empty() const override { return quadtree_.empty(); }
  size_t size() const override { return quadtree_.size(); }
  void prune() override;
  void clear() override;

  QuadtreeIndex::ChildArray getFirstChildIndices() const override;

  Index2D getMinIndex() const override;
  Index2D getMaxIndex() const override;
  Index2D getMinPossibleIndex() const override;
  Index2D getMaxPossibleIndex() const override;

  FloatingPoint getCellValue(const Index2D& index) const override;
  void setCellValue(const Index2D& index, FloatingPoint new_value) override;
  void setCellValue(const QuadtreeIndex& index,
                    FloatingPoint new_value) override;
  void addToCellValue(const Index2D& index, FloatingPoint update) override;
  void addToCellValue(const QuadtreeIndex& index,
                      FloatingPoint update) override;

  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const override;

  ScaleCoefficient& getRootScale() override { return root_scale_coefficient_; }
  const ScaleCoefficient& getRootScale() const override {
    return root_scale_coefficient_;
  }
  NodeType& getRootNode() override { return quadtree_.getRootNode(); }
  const NodeType& getRootNode() const override {
    return quadtree_.getRootNode();
  }
  NodeType* getNode(const QuadtreeIndex& node_index) override;
  const NodeType* getNode(const QuadtreeIndex& node_index) const override;

  template <TraversalOrder traversal_order>
  auto getNodeIterator() {
    return quadtree_.template getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() const {
    return quadtree_.template getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const override { return quadtree_.getMemoryUsage(); }

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

 private:
  struct StackElement {
    const QuadtreeIndex node_index;
    const NodeType& node;
    const ScaleCoefficient scale_coefficient{};
  };

  ScaleCoefficient root_scale_coefficient_{};
  Ndtree<DetailCoefficients, 2, kMaxHeight - 1> quadtree_;

  static QuadtreeIndex getInternalRootNodeIndex() {
    return QuadtreeIndex{kMaxHeight, QuadtreeIndex::Position::Zero()};
  }
  const QuadtreeIndex root_node_index_offset_{kMaxHeight - 1,
                                              QuadtreeIndex::Position::Ones()};
  const Index2D root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  QuadtreeIndex toInternal(const Index2D& index) const {
    return convert::indexAndHeightToNodeIndex<2>(index + root_index_offset_, 0);
  }
  QuadtreeIndex toInternal(const QuadtreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index2D height_adjusted_offset =
        int_math::div_exp2(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index2D toExternalIndex(const Index2D& index) const {
    return index - root_index_offset_;
  }
  QuadtreeIndex toExternalNodeIndex(const QuadtreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index2D height_adjusted_offset =
        int_math::div_exp2(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }
};
}  // namespace wavemap

#include "wavemap_2d/data_structure/impl/wavelet_tree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_WAVELET_TREE_H_
