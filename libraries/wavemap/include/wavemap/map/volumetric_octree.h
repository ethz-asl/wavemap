#ifndef WAVEMAP_MAP_VOLUMETRIC_OCTREE_H_
#define WAVEMAP_MAP_VOLUMETRIC_OCTREE_H_

#include <memory>
#include <string>

#include "wavemap/config/config_base.h"
#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/map/map_base.h"

namespace wavemap {
/**
 * Config struct for the octree volumetric data structure.
 */
struct VolumetricOctreeConfig : ConfigBase<VolumetricOctreeConfig, 4> {
  Meters<FloatingPoint> min_cell_width = 0.1f;

  FloatingPoint min_log_odds = -2.f;
  FloatingPoint max_log_odds = 4.f;

  IndexElement tree_height = 14;

  static MemberMap memberMap;

  // Constructors
  VolumetricOctreeConfig() = default;
  VolumetricOctreeConfig(FloatingPoint min_cell_width,
                         FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                         IndexElement tree_height)
      : min_cell_width(min_cell_width),
        min_log_odds(min_log_odds),
        max_log_odds(max_log_odds),
        tree_height(tree_height) {}

  // Conversion to DataStructureBase config
  operator MapBaseConfig() const {  // NOLINT
    return {min_cell_width, min_log_odds, max_log_odds};
  }

  bool isValid(bool verbose) const override;
};

class VolumetricOctree : public MapBase {
 public:
  using Ptr = std::shared_ptr<VolumetricOctree>;
  using ConstPtr = std::shared_ptr<const VolumetricOctree>;
  using Config = VolumetricOctreeConfig;
  using OctreeType = Octree<FloatingPoint>;
  using NodeType = OctreeType::NodeType;

  static constexpr bool kRequiresExplicitThresholding = true;

  explicit VolumetricOctree(const VolumetricOctreeConfig& config)
      : MapBase(config), config_(config.checkValid()) {}

  bool empty() const override { return ndtree_.empty(); }
  size_t size() const override { return ndtree_.size(); }
  void threshold() override;
  void prune() override;
  void clear() override { return ndtree_.clear(); }

  typename OctreeIndex::ChildArray getFirstChildIndices() const;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinPossibleIndex() const;
  Index3D getMaxPossibleIndex() const;
  IndexElement getTreeHeight() const override { return config_.tree_height; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
  void addToCellValue(const Index3D& index, FloatingPoint update) override;
  void addToCellValue(const OctreeIndex& index, FloatingPoint update);

  void forEachLeaf(
      typename MapBase::IndexedLeafVisitorFunction visitor_fn) const override;

  NodeType& getRootNode() { return ndtree_.getRootNode(); }
  const NodeType& getRootNode() const { return ndtree_.getRootNode(); }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() {
    return ndtree_.getIterator<traversal_order>();
  }
  template <TraversalOrder traversal_order>
  auto getNodeIterator() const {
    return ndtree_.getIterator<traversal_order>();
  }

  size_t getMemoryUsage() const override { return ndtree_.getMemoryUsage(); }

 private:
  const VolumetricOctreeConfig config_;

  OctreeType ndtree_{config_.tree_height};

  OctreeIndex getInternalRootNodeIndex() const {
    return OctreeIndex{config_.tree_height, OctreeIndex::Position::Zero()};
  }
  const OctreeIndex root_node_index_offset_{config_.tree_height - 1,
                                            OctreeIndex::Position::Ones()};
  const Index3D root_index_offset_ =
      convert::nodeIndexToMinCornerIndex(root_node_index_offset_);

  OctreeIndex toInternal(const Index3D& index) const {
    return convert::indexAndHeightToNodeIndex<kDim>(index + root_index_offset_,
                                                    0);
  }
  OctreeIndex toInternal(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position + height_adjusted_offset};
  }
  Index3D toExternalIndex(const Index3D& index) const {
    return index - root_index_offset_;
  }
  OctreeIndex toExternalNodeIndex(const OctreeIndex& node_index) const {
    DCHECK_LE(node_index.height, root_node_index_offset_.height);
    const Index3D height_adjusted_offset =
        int_math::div_exp2_floor(root_index_offset_, node_index.height);
    return {node_index.height, node_index.position - height_adjusted_offset};
  }

  const NodeType* getDeepestNodeAtIndex(const Index3D& index) const;

  friend class VolumetricOctreeTest_IndexConversions_Test;
};
}  // namespace wavemap

#include "wavemap/map/impl/volumetric_octree_inl.h"

#endif  // WAVEMAP_MAP_VOLUMETRIC_OCTREE_H_
