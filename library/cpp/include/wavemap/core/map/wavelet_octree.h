#ifndef WAVEMAP_CORE_MAP_WAVELET_OCTREE_H_
#define WAVEMAP_CORE_MAP_WAVELET_OCTREE_H_

#include <memory>
#include <string>

#include "wavemap/core/config/config_base.h"
#include "wavemap/core/data_structure/ndtree/ndtree.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/cell_types/haar_transform.h"
#include "wavemap/core/map/map_base.h"

namespace wavemap {
/**
 * Config struct for the wavelet octree volumetric data structure.
 */
struct WaveletOctreeConfig : ConfigBase<WaveletOctreeConfig, 4> {
  //! Maximum resolution of the map, set as the width of the smallest cell that
  //! it can represent.
  Meters<FloatingPoint> min_cell_width = 0.1f;

  //! Lower threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint min_log_odds = -2.f;
  //! Upper threshold for the occupancy values stored in the map, in log-odds.
  FloatingPoint max_log_odds = 4.f;

  //! Height of the octree used to store the map.
  IndexElement tree_height = 14;

  static MemberMap memberMap;

  // Constructors
  WaveletOctreeConfig() = default;
  WaveletOctreeConfig(FloatingPoint min_cell_width, FloatingPoint min_log_odds,
                      FloatingPoint max_log_odds, IndexElement tree_height)
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

class WaveletOctree : public MapBase {
 public:
  using Ptr = std::shared_ptr<WaveletOctree>;
  using ConstPtr = std::shared_ptr<const WaveletOctree>;
  using Config = WaveletOctreeConfig;
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using NodeType = NdtreeNode<typename Coefficients::Details, kDim>;

  static constexpr bool kRequiresExplicitThresholding = true;

  explicit WaveletOctree(const WaveletOctreeConfig& config)
      : MapBase(config), config_(config.checkValid()) {}

  bool empty() const override;
  size_t size() const override { return ndtree_.size(); }
  void threshold() override;
  void prune() override;
  void clear() override;

  typename OctreeIndex::ChildArray getFirstChildIndices() const;

  Index3D getMinIndex() const override;
  Index3D getMaxIndex() const override;
  Index3D getMinPossibleIndex() const;
  Index3D getMaxPossibleIndex() const;
  IndexElement getTreeHeight() const override { return config_.tree_height; }
  const WaveletOctreeConfig& getConfig() const { return config_; }

  FloatingPoint getCellValue(const Index3D& index) const override;
  FloatingPoint getCellValue(const OctreeIndex& index) const;
  void setCellValue(const Index3D& index, FloatingPoint new_value) override;
  void setCellValue(const OctreeIndex& index, FloatingPoint new_value);
  void addToCellValue(const Index3D& index, FloatingPoint update) override;
  void addToCellValue(const OctreeIndex& index, FloatingPoint update);

  void forEachLeaf(
      typename MapBase::IndexedLeafVisitorFunction visitor_fn) const override;

  Coefficients::Scale& getRootScale() { return root_scale_coefficient_; }
  const Coefficients::Scale& getRootScale() const {
    return root_scale_coefficient_;
  }
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
  const WaveletOctreeConfig config_;

  Octree<Coefficients::Details> ndtree_{config_.tree_height - 1};
  Coefficients::Scale root_scale_coefficient_{};

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

  Coefficients::Scale recursiveThreshold(NodeType& node,
                                         Coefficients::Scale scale_coefficient);
  Coefficients::Scale recursivePrune(NodeType& node,
                                     Coefficients::Scale scale_coefficient);
};
}  // namespace wavemap

#include "wavemap/core/map/impl/wavelet_octree_inl.h"

#endif  // WAVEMAP_CORE_MAP_WAVELET_OCTREE_H_
