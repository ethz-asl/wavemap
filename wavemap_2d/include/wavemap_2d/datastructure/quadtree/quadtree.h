#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_

#include <utility>
#include <vector>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/datastructure_base.h"
#include "wavemap_2d/datastructure/quadtree/node.h"
#include "wavemap_2d/datastructure/quadtree/node_index.h"
#include "wavemap_2d/datastructure/quadtree/quadtree_iterators.h"
#include "wavemap_2d/pointcloud.h"

namespace wavemap_2d {
template <typename NodeDataType>
class Quadtree : public DataStructureBase {
 public:
  explicit Quadtree(FloatingPoint resolution)
      : DataStructureBase(resolution),
        max_depth_(14),
        root_node_width_(std::exp2(max_depth_) * resolution) {
    updateLookupTables();
  }
  ~Quadtree() { root_node_.pruneChildren(); }

  NodeIndex computeNodeIndexFromCenter(const Point& center,
                                       NodeIndexElement depth) const;
  Point computeNodeCenterFromIndex(const NodeIndex& index) const;
  Point computeNodeCornerFromIndex(const NodeIndex& index) const;

  NodeIndexElement getTreeDepthAtPosition(const Point& position) const {
    CHECK(false) << "Not yet implemented";
    return 0;
  }

  FloatingPoint getNodeWidthAtDepth(NodeIndexElement depth) const {
    CHECK_LE(depth, max_depth_);
    return luts_.node_widths_at_depth_[depth];
  }
  Vector getNodeHalvedDiagonalAtDepth(NodeIndexElement depth) const {
    CHECK_LE(depth, max_depth_);
    return luts_.node_halved_diagonals_at_depth_[depth];
  }

  bool hasNodeWithIndex(const NodeIndex& index) {
    return getNodeByIndex(index);
  }
  void allocateNodeWithIndex(const NodeIndex& index) {
    getNodeByIndex(index, /* auto_allocate */ true);
  }
  bool removeNodeWithIndex(const NodeIndex& index);

  // NOTE: Pointers to the nodes themselves are not exposed (except for a const
  //       pointer to the root node). One reason being that they could otherwise
  //       be deleted by users without calling Quadtree::removeNode(...) method
  //       which would lead to inconsistencies in the Quadtree and lead to
  //       segfaults.
  const Node<NodeDataType>* getRootNodeConstPtr() const { return &root_node_; }
  NodeDataType* getNodeDataByIndex(const NodeIndex& index,
                                   const bool auto_allocate = false);

  Pointcloud getLeaveCenters(NodeIndexElement max_depth) const;
  std::vector<PointWithValue> getLeaveValues(NodeIndexElement max_depth) const;

  QuadtreeIterator<NodeDataType, TraversalOrder::kBreadthFirst>
  getBreadthFirstIterator() const {
    return QuadtreeIterator<NodeDataType, TraversalOrder::kBreadthFirst>(this);
  }
  QuadtreeIterator<NodeDataType, TraversalOrder::kDepthFirst>
  getDepthFirstIterator() const {
    return QuadtreeIterator<NodeDataType, TraversalOrder::kDepthFirst>(this);
  }

 protected:
  NodeIndexElement max_depth_;
  FloatingPoint root_node_width_;

  Node<NodeDataType> root_node_;

  Node<NodeDataType>* getNodeByIndex(const NodeIndex& index,
                                     bool auto_allocate = false);

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
