#ifndef WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_
#define WAVEMAP_2D_DATASTRUCTURE_QUADTREE_QUADTREE_H_

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

  bool empty() const override { return !root_node_.hasNotNullChildren(); }
  size_t size() const override {
    // TODO(victorr): Compute the size (e.g. with DFS)
    return 0u;
  }
  void clear() override { root_node_.pruneChildren(); }

  bool hasCell(const Index& index) const override;
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;

  cv::Mat getImage(bool use_color) const override;
  bool save(const std::string& file_path_prefix,
            bool use_floating_precision) const override;
  bool load(const std::string& file_path_prefix,
            bool used_floating_precision) override;

  NodeIndex computeNodeIndexFromIndex(const Index& index,
                                      NodeIndexElement depth) const {
    return computeNodeIndexFromCenter(index.template cast<FloatingPoint>(),
                                      depth);
  }
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

  bool hasNode(const NodeIndex& index) { return getNode(index); }
  void allocateNode(const NodeIndex& index) {
    getNode(index, /* auto_allocate */ true);
  }
  bool removeNode(const NodeIndex& index);

  // NOTE: Pointers to the nodes themselves are not exposed (except for a const
  //       pointer to the root node). One reason being that they could otherwise
  //       be deleted by users without calling Quadtree::removeNode(...) method
  //       which would lead to inconsistencies in the Quadtree and lead to
  //       segfaults.
  const Node<CellDataSpecialized>* getRootNodePtr() const {
    return &root_node_;
  }

  NodeIndexElement max_depth_;
  FloatingPoint root_node_width_;
  Node<CellDataSpecialized> root_node_;

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
