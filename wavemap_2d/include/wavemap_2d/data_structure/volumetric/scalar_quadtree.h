#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_

#include <string>
#include <utility>

#include "wavemap_2d/data_structure/generic/quadtree/quadtree.h"
#include "wavemap_2d/data_structure/volumetric/volumetric_data_structure.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
class VolumetricQuadtree : public VolumetricDataStructure {
 public:
  template <typename... ConstructorArgs>
  explicit VolumetricQuadtree(ConstructorArgs&&... args)
      : VolumetricDataStructure(std::forward<ConstructorArgs>(args)...) {}

  virtual Index getMinPossibleIndex() const = 0;
  virtual Index getMaxPossibleIndex() const = 0;
  virtual QuadtreeIndex::Element getMaxDepth() const = 0;
  virtual FloatingPoint getRootNodeWidth() const = 0;

  using VolumetricDataStructure::setCellValue;
  virtual void setCellValue(const QuadtreeIndex& index,
                            FloatingPoint new_value) = 0;
  using VolumetricDataStructure::addToCellValue;
  virtual void addToCellValue(const QuadtreeIndex& index,
                              FloatingPoint update) = 0;

  virtual FloatingPoint computeNodeWidthAtDepth(
      QuadtreeIndex::Element depth) = 0;
  virtual Vector computeNodeHalvedDiagonalAtDepth(
      QuadtreeIndex::Element depth) = 0;
};

template <typename CellT>
class ScalarQuadtree : public VolumetricQuadtree {
 public:
  using CellType = CellT;
  using NodeType = Node<typename CellT::Specialized>;
  struct NodeIndexPtrPair {
    QuadtreeIndex idx;
    Node<SaturatingOccupancyCell::Specialized>* ptr = nullptr;
  };

  explicit ScalarQuadtree(FloatingPoint resolution)
      : VolumetricQuadtree(resolution),
        max_depth_(14),
        root_node_width_(std::exp2f(max_depth_) * resolution),
        root_node_offset_(Index::Constant(std::exp2(max_depth_ - 1))) {}
  ~ScalarQuadtree() override = default;

  bool empty() const override { return quadtree_.empty(); }
  size_t size() const override { return quadtree_.size(); }
  void prune() override { return quadtree_.prune(); }
  void clear() override { return quadtree_.clear(); }
  void averageAndPrune();

  Index getMinIndex() const override;
  Index getMaxIndex() const override;
  Index getMinPossibleIndex() const override;
  Index getMaxPossibleIndex() const override;
  QuadtreeIndex::Element getMaxDepth() const override { return max_depth_; }
  FloatingPoint getRootNodeWidth() const override { return root_node_width_; }
  NodeIndexPtrPair getRootNodeIndexPtrPair() {
    return {QuadtreeIndex{}, &quadtree_.getRootNode()};
  }

  bool hasCell(const Index& index) const override;
  QuadtreeIndex::Element getDepthAtIndex(const Index& index);
  FloatingPoint getCellValue(const Index& index) const override;
  void setCellValue(const Index& index, FloatingPoint new_value) override;
  void setCellValue(const QuadtreeIndex& index,
                    FloatingPoint new_value) override;
  void addToCellValue(const Index& index, FloatingPoint update) override;
  void addToCellValue(const QuadtreeIndex& index,
                      FloatingPoint update) override;

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

#include "wavemap_2d/data_structure/volumetric/impl/scalar_quadtree_inl.h"

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_SCALAR_QUADTREE_H_
