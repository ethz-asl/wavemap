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

namespace wavemap_2d {
template <typename CellTypeT>
class Quadtree : public DataStructureBase {
 public:
  using CellType = CellTypeT;

  explicit Quadtree(FloatingPoint resolution)
      : DataStructureBase(resolution),
        max_depth_(14),
        root_node_width_(std::exp2(max_depth_) * resolution) {
    updateLookupTables();
  }
  ~Quadtree() { root_node_.pruneChildren(); }

  bool empty() const override { return !root_node_.hasAtLeastOneChild(); }
  size_t size() const override;
  void clear() override { root_node_.pruneChildren(); }

  size_t getMemoryUsage() const override;

  NodeIndexElement getMaxDepth() const { return max_depth_; }
  Index getMinPossibleIndex() const {
    return (-getNodeHalvedDiagonalAtDepth(0u)).template cast<IndexElement>();
  }
  Index getMaxPossibleIndex() const {
    return getNodeHalvedDiagonalAtDepth(0u).template cast<IndexElement>();
  }

  // TODO(victorr): Replace this with a more fine grained method
  Index getMinIndex() const override { return getMinPossibleIndex(); }
  Index getMaxIndex() const override { return getMaxPossibleIndex(); }

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

  // TODO(victorr): Add indexing unit tests
  // TODO(victorr): Check if this this indexing convention is the best option
  //                for wavelet trees
  // TODO(victorr): Move these computations to an index handler that can be
  //                shared among future hierarchical datastructures
  NodeIndex computeNodeIndexFromIndexAndDepth(const Index& index,
                                              NodeIndexElement depth) const;
  NodeIndex computeNodeIndexFromCenter(const Point& center,
                                       NodeIndexElement depth) const;
  Point computeNodeCenterFromIndex(const NodeIndex& index) const;
  Point computeNodeCornerFromIndex(const NodeIndex& index) const;

  FloatingPoint getNodeWidthAtDepth(NodeIndexElement depth) const {
    CHECK_LE(depth, max_depth_);
    return luts_.node_widths_at_depth_[depth];
  }
  Vector getNodeHalvedDiagonalAtDepth(NodeIndexElement depth) const {
    CHECK_LE(depth, max_depth_);
    return luts_.node_halved_diagonals_at_depth_[depth];
  }

 protected:
  using CellDataSpecialized = typename CellTypeT::Specialized;

  NodeIndexElement max_depth_;
  FloatingPoint root_node_width_;
  Node<CellDataSpecialized> root_node_;

  bool hasNode(const NodeIndex& index) { return getNode(index); }
  void allocateNode(const NodeIndex& index) {
    constexpr bool kAutoAllocate = true;
    getNode(index, kAutoAllocate);
  }
  bool removeNode(const NodeIndex& index);
  Node<CellDataSpecialized>* getNode(const NodeIndex& index,
                                     bool auto_allocate = false);
  const Node<CellDataSpecialized>* getNode(const NodeIndex& index) const;

  FloatingPoint computeNodeWidthAtDepth(NodeIndexElement depth);
  Vector computeNodeHalvedDiagonalAtDepth(NodeIndexElement depth);

  void updateLookupTables();
  struct {
    std::vector<FloatingPoint> node_widths_at_depth_;
    std::vector<Vector> node_halved_diagonals_at_depth_;
  } luts_;
};
}  // namespace wavemap_2d

#include "wavemap_2d/datastructure/quadtree/quadtree_inl.h"

#endif  // WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_
