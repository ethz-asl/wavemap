#ifndef WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_QUADTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/generic/pointcloud.h"
#include "wavemap_2d/data_structure/generic/quadtree/node.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/indexing/quadtree_index.h"
#include "wavemap_2d/iterator/subtree_iterator.h"

namespace wavemap_2d {
template <typename CellT>
class Quadtree : public VolumetricDataStructure {
 public:
  using CellType = CellT;
  using NodeType = Node<typename CellT::Specialized>;

  explicit Quadtree(FloatingPoint resolution)
      : VolumetricDataStructure(resolution),
        max_depth_(14),
        root_node_width_(std::exp2(max_depth_) * resolution),
        root_node_offset_(Index::Constant(std::exp2(max_depth_ - 1))) {}
  ~Quadtree() override = default;

  bool empty() const override { return root_node_.empty(); }
  size_t size() const override;
  void clear() override { root_node_.deleteChildrenArray(); }
  void prune();

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
    return Subtree<NodeType, traversal_order>(&root_node_);
  }
  template <TraversalOrder traversal_order>
  auto getIterator() const {
    return Subtree<const NodeType, traversal_order>(&root_node_);
  }

  size_t getMemoryUsage() const override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

  NodeIndex indexToNodeIndex(const Index& index) const {
    return computeNodeIndexFromIndexAndDepth(index + root_node_offset_,
                                             max_depth_, max_depth_);
  }
  Index nodeIndexToIndex(const NodeIndex& node_index) const {
    return computeIndexFromNodeIndex(node_index, max_depth_) -
           root_node_offset_;
  }

 protected:
  using CellDataSpecialized = typename CellT::Specialized;

  NodeIndexElement max_depth_;
  FloatingPoint root_node_width_;
  Index root_node_offset_;
  NodeType root_node_;

  bool hasNode(const NodeIndex& index) { return getNode(index); }
  void allocateNode(const NodeIndex& index) {
    constexpr bool kAutoAllocate = true;
    getNode(index, kAutoAllocate);
  }
  bool removeNode(const NodeIndex& index);
  NodeType* getNode(const NodeIndex& index, bool auto_allocate = false);
  const NodeType* getNode(const NodeIndex& index) const;

  FloatingPoint computeNodeWidthAtDepth(NodeIndexElement depth);
  Vector computeNodeHalvedDiagonalAtDepth(NodeIndexElement depth);
};
}  // namespace wavemap_2d

#include "wavemap_2d/data_structure/generic/quadtree/impl/quadtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_GENERIC_QUADTREE_QUADTREE_H_
