#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_

#include <string>
#include <utility>

#include "wavemap_2d/data_structure/generic/quadtree/quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_quadtree_interface.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
template <typename CellT>
class SimpleQuadtree : public VolumetricQuadtreeInterface {
 public:
  using CellType = CellT;
  using NodeType = Node<typename CellT::Specialized>;

  explicit SimpleQuadtree(FloatingPoint resolution)
      : VolumetricQuadtreeInterface(resolution),
        max_depth_(14),
        root_node_width_(std::exp2f(max_depth_) * resolution),
        root_node_offset_(Index::Constant(int_math::exp2(max_depth_ - 1))) {}
  ~SimpleQuadtree() override = default;

  bool empty() const override { return quadtree_.empty(); }
  size_t size() const override { return quadtree_.size(); }
  void prune() override;
  void clear() override { return quadtree_.clear(); }

  Index getMinIndex() const override;
  Index getMaxIndex() const override;
  Index getMinPossibleIndex() const override;
  Index getMaxPossibleIndex() const override;
  QuadtreeIndex::Element getMaxDepth() const override { return max_depth_; }
  FloatingPoint getRootNodeWidth() const override { return root_node_width_; }

  bool hasCell(const Index& index) const override;
  QuadtreeIndex::Element getDepthAtIndex(const Index& index);
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

  QuadtreeIndex indexToNodeIndex(const Index& index) const {
    return convert::indexAndDepthToNodeIndex(index + root_node_offset_,
                                             max_depth_, max_depth_);
  }
  Index nodeIndexToIndex(const QuadtreeIndex& node_index) const {
    return convert::nodeIndexToIndex(node_index, max_depth_) -
           root_node_offset_;
  }

  FloatingPoint computeNodeWidthAtDepth(QuadtreeIndex::Element depth) override;
  Vector computeNodeHalvedDiagonalAtDepth(
      QuadtreeIndex::Element depth) override;

 private:
  using CellDataSpecialized = typename CellT::Specialized;

  Quadtree<CellDataSpecialized> quadtree_;
  QuadtreeIndex::Element max_depth_;
  FloatingPoint root_node_width_;
  Index root_node_offset_;

  const Node<CellDataSpecialized>* getDeepestNodeAtIndex(
      const Index& index) const;
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/volumetric/impl/simple_quadtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SIMPLE_QUADTREE_H_
