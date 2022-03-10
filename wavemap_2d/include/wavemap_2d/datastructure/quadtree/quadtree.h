#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/datastructure/pointcloud.h"
#include "wavemap_2d/datastructure/quadtree/node.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"
#include "wavemap_2d/indexing/index.h"

namespace wavemap_2d {
template <typename CellT>
class Quadtree : public DataStructureBase {
 public:
  using CellType = CellT;
  using NodeType = Node<typename CellT::Specialized>;

  explicit Quadtree(FloatingPoint resolution)
      : DataStructureBase(resolution),
        max_depth_(14),
        root_node_width_(std::exp2(max_depth_) * resolution),
        root_node_offset_(Index::Constant(std::exp2(max_depth_ - 1))) {
    updateLookupTables();
  }
  ~Quadtree() override = default;

  bool empty() const override { return root_node_.empty(); }
  size_t size() const override;
  void clear() override { root_node_.deleteChildrenArray(); }
  void prune() { root_node_.pruneChildren(); }

  Index getMinPossibleIndex() const;
  Index getMaxPossibleIndex() const;
  Index getMinIndex() const override;
  Index getMaxIndex() const override;
  NodeIndexElement getMaxDepth() const { return max_depth_; }

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;

  template <typename Functor>
  void applyBottomUp(Functor&& fn) {
    root_node_.applyBottomUp(fn);
  }
  template <typename Functor>
  void applyBottomUp(Functor&& fn) const {
    root_node_.applyBottomUp(fn);
  }
  template <typename Functor>
  void applyTopDown(Functor&& fn) {
    root_node_.applyTopDown(fn);
  }
  template <typename Functor>
  void applyTopDown(Functor&& fn) const {
    root_node_.applyTopDown(fn);
  }

  size_t getMemoryUsage() const override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

  NodeIndex computeNodeIndexFromIndexAndDepth(const Index& index,
                                              NodeIndexElement depth) const;
  NodeIndex computeNodeIndexFromCenter(const Point& center,
                                       NodeIndexElement depth) const;
  Index computeIndexFromNodeIndex(const NodeIndex& node_index) const;
  Point computeNodeCenterFromNodeIndex(const NodeIndex& node_index) const;

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

  FloatingPoint getNodeWidthAtDepth(NodeIndexElement depth) const {
    DCHECK_LE(depth, max_depth_);
    return luts_.node_widths_at_depth_[depth];
  }
  Vector getNodeHalvedDiagonalAtDepth(NodeIndexElement depth) const {
    DCHECK_LE(depth, max_depth_);
    return luts_.node_halved_diagonals_at_depth_[depth];
  }

  FloatingPoint computeNodeWidthAtDepth(NodeIndexElement depth);
  Vector computeNodeHalvedDiagonalAtDepth(NodeIndexElement depth);
  struct {
    std::vector<FloatingPoint> node_widths_at_depth_;
    std::vector<Vector> node_halved_diagonals_at_depth_;
  } luts_;
  void updateLookupTables();
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/quadtree_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_
