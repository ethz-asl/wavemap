#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_

#include <string>

#include "wavemap_2d/data_structure/generic/quadtree/quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/quadtree_index.h"

namespace wavemap_2d {
template <typename CellT>
class ScalarQuadtree : public VolumetricDataStructure {
 public:
  using CellType = CellT;
  using NodeType = Node<typename CellT::Specialized>;

  explicit ScalarQuadtree(FloatingPoint resolution)
      : VolumetricDataStructure(resolution),
        max_depth_(14),
        root_node_width_(std::exp2(max_depth_) * resolution),
        root_node_offset_(Index::Constant(std::exp2(max_depth_ - 1))) {}
  ~ScalarQuadtree() override = default;

  bool empty() const override { return quadtree_.empty(); }
  size_t size() const override { return quadtree_.size(); }
  void clear() override { return quadtree_.clear(); }
  void prune() { return quadtree_.prune(); }
  void averageAndPrune();

  Index getMinPossibleIndex() const;
  Index getMaxPossibleIndex() const;
  Index getMinIndex() const override;
  Index getMaxIndex() const override;
  NodeIndexElement getMaxDepth() const { return max_depth_; }

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;

  template <TraversalOrder traversal_order>
  auto getIterator() {
    return quadtree_.template getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getIterator() const {
    return quadtree_.template getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const override { return quadtree_.getMemoryUsage(); }

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

  QuadtreeIndex indexToNodeIndex(const Index& index) const {
    return computeNodeIndexFromIndexAndDepth(index + root_node_offset_,
                                             max_depth_, max_depth_);
  }
  Index nodeIndexToIndex(const QuadtreeIndex& node_index) const {
    return computeIndexFromNodeIndex(node_index, max_depth_) -
           root_node_offset_;
  }

  FloatingPoint computeNodeWidthAtDepth(NodeIndexElement depth);
  Vector computeNodeHalvedDiagonalAtDepth(NodeIndexElement depth);

 protected:
  using CellDataSpecialized = typename CellT::Specialized;

  Quadtree<CellDataSpecialized> quadtree_;
  NodeIndexElement max_depth_;
  FloatingPoint root_node_width_;
  Index root_node_offset_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/volumetric/impl/scalar_quadtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_
