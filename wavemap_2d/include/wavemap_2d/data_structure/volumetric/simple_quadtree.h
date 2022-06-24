#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_

#include <string>

#include "wavemap_2d/data_structure/generic/quadtree/quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_quadtree_interface.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
template <typename CellT>
class SimpleQuadtree : public VolumetricQuadtreeInterface {
 public:
  using CellType = CellT;
  static constexpr bool kRequiresPruningForThresholding = true;

  using VolumetricQuadtreeInterface::VolumetricQuadtreeInterface;
  ~SimpleQuadtree() override = default;

  bool empty() const override { return quadtree_.empty(); }
  size_t size() const override { return quadtree_.size(); }
  void prune() override;
  void clear() override { return quadtree_.clear(); }

  QuadtreeIndex::ChildArray getFirstChildIndices() const override;

  Index getMinIndex() const override;
  Index getMaxIndex() const override;
  Index getMinPossibleIndex() const override;
  Index getMaxPossibleIndex() const override;

  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void setCellValue(const QuadtreeIndex& index,
                    FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;
  void addToCellValue(const QuadtreeIndex& index,
                      FloatingPoint update) override;

  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const override;

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
  using NodeType = Node<typename CellT::Specialized>;
  Quadtree<typename CellT::Specialized, kMaxHeight> quadtree_;

  static QuadtreeIndex getInternalRootNodeIndex() {
    return QuadtreeIndex{kMaxHeight, QuadtreeIndex::Position::Zero()};
  }
  const QuadtreeIndex root_node_index_offset_{kMaxHeight - 1,
                                              QuadtreeIndex::Position::Ones()};
  const Index root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  QuadtreeIndex toInternal(const Index& index) const {
    return convert::indexAndHeightToNodeIndex(index + root_index_offset_, 0);
  }
  QuadtreeIndex toInternal(const QuadtreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index height_adjusted_offset =
        int_math::div_exp2(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index toExternalIndex(const Index& index) const {
    return index - root_index_offset_;
  }
  QuadtreeIndex toExternalNodeIndex(const QuadtreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index height_adjusted_offset =
        int_math::div_exp2(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }

  const NodeType* getDeepestNodeAtIndex(const Index& index) const;

  template <typename T>
  friend class SimpleQuadtreeTest_IndexConversions_Test;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/volumetric/impl/simple_quadtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_
