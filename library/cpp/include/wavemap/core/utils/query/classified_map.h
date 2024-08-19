#ifndef WAVEMAP_CORE_UTILS_QUERY_CLASSIFIED_MAP_H_
#define WAVEMAP_CORE_UTILS_QUERY_CLASSIFIED_MAP_H_

#include <limits>
#include <memory>
#include <utility>

#include "wavemap/core/data_structure/ndtree/ndtree.h"
#include "wavemap/core/data_structure/ndtree_block_hash.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/utils/query/occupancy_classifier.h"
#include "wavemap/core/utils/query/query_accelerator.h"

namespace wavemap {
struct ChildBitset {
  uint8_t bitset{};

  void set(NdtreeIndexRelativeChild child_idx, bool value = true);
  bool operator[](NdtreeIndexRelativeChild child_idx) const;
  bool any() const;
};

class ClassifiedMap {
 public:
  struct NodeData {
    ChildBitset has_free;
    ChildBitset has_occupied;
    ChildBitset has_unobserved;

    Occupancy::Mask occupancyMask() const;
    Occupancy::Mask childOccupancyMask(
        NdtreeIndexRelativeChild child_idx) const;
  };
  using HeightType = IndexElement;
  using BlockHashMap = OctreeBlockHash<NodeData>;
  using Block = BlockHashMap::Block;
  using Node = BlockHashMap::Node;
  static constexpr int kDim = 3;

  using Ptr = std::shared_ptr<ClassifiedMap>;
  using ConstPtr = std::shared_ptr<const ClassifiedMap>;

  ClassifiedMap(FloatingPoint min_cell_width, IndexElement tree_height,
                const OccupancyClassifier& classifier);

  ClassifiedMap(const HashedWaveletOctree& occupancy_map,
                const OccupancyClassifier& classifier);

  ClassifiedMap(const HashedWaveletOctree& occupancy_map,
                const OccupancyClassifier& classifier,
                const HashedBlocks& esdf_map, FloatingPoint robot_radius);

  bool empty() const { return block_map_.empty(); }

  FloatingPoint getMinCellWidth() const { return min_cell_width_; }
  IndexElement getTreeHeight() const { return block_map_.getMaxHeight(); }

  Index3D getMinIndex() const;
  Index3D getMaxIndex() const;
  Index3D getMinBlockIndex() const { return block_map_.getMinBlockIndex(); }
  Index3D getMaxBlockIndex() const { return block_map_.getMaxBlockIndex(); }
  IndexElement getLastResultHeight() const { return query_cache_.height; }

  void update(const HashedWaveletOctree& occupancy_map);
  void update(const HashedWaveletOctree& occupancy_map,
              const HashedBlocks& esdf_map, FloatingPoint robot_radius);

  bool has(const Index3D& index, Occupancy::Id occupancy_type) const;
  bool has(const OctreeIndex& index, Occupancy::Id occupancy_type) const;
  bool has(const Index3D& index, Occupancy::Mask occupancy_mask) const;
  bool has(const OctreeIndex& index, Occupancy::Mask occupancy_mask) const;

  bool isFully(const Index3D& index, Occupancy::Id occupancy_type) const;
  bool isFully(const OctreeIndex& index, Occupancy::Id occupancy_type) const;
  bool isFully(const Index3D& index, Occupancy::Mask occupancy_mask) const;
  bool isFully(const OctreeIndex& index, Occupancy::Mask occupancy_mask) const;

  bool hasBlock(const Index3D& block_index) const;
  const Block* getBlock(const Index3D& block_index) const;
  const BlockHashMap& getBlockMap() const { return block_map_; }

  bool hasNode(const OctreeIndex& index) const { return getNode(index); }
  const Node* getNode(const OctreeIndex& index) const;
  std::pair<const Node*, HeightType> getNodeOrAncestor(
      const OctreeIndex& index) const;

  bool hasValue(const OctreeIndex& index) const;
  std::optional<Occupancy::Mask> getValue(const OctreeIndex& index) const;
  std::pair<std::optional<Occupancy::Mask>, HeightType> getValueOrAncestor(
      const OctreeIndex& index) const;

  auto& getHashMap() { return block_map_.getHashMap(); }
  const auto& getHashMap() const { return block_map_.getHashMap(); }

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const;

  using IndexedLeafVisitorFunction =
      std::function<void(const OctreeIndex& index, Occupancy::Mask occupancy)>;
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn,
                   IndexElement termination_height = 0) const;
  void forEachLeafMatching(Occupancy::Id occupancy_type,
                           IndexedLeafVisitorFunction visitor_fn,
                           IndexElement termination_height = 0) const;
  void forEachLeafMatching(Occupancy::Mask occupancy_mask,
                           IndexedLeafVisitorFunction visitor_fn,
                           IndexElement termination_height = 0) const;

 private:
  using HaarTransform = HashedWaveletOctree::Block::Transform;
  using ChildAverages =
      HashedWaveletOctree::Block::Coefficients::CoefficientsArray;

  const IndexElement tree_height_;
  const FloatingPoint min_cell_width_;
  const IndexElement cells_per_block_side_ = int_math::exp2(tree_height_);
  const OccupancyClassifier classifier_;
  BlockHashMap block_map_{tree_height_};

  // Cache previous queries
  struct QueryCache {
    explicit QueryCache(IndexElement tree_height) : tree_height(tree_height) {}

    const IndexElement tree_height;

    Index3D block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::max());
    IndexElement height = tree_height;
    MortonIndex morton_code = std::numeric_limits<MortonIndex>::max();

    const Block* block = nullptr;
    std::array<const Node*, morton::kMaxTreeHeight<3>> node_stack{};

    const Block* getBlock(const Index3D& block_index,
                          const BlockHashMap& block_map);
    std::pair<const Node*, HeightType> getNodeOrAncestor(
        const OctreeIndex& index, const BlockHashMap& block_map);
    // TODO(victorr): Write more unit tests for these accelerated accessors

    bool has(const OctreeIndex& index, Occupancy::Mask occupancy_mask,
             const BlockHashMap& block_map);
    bool isFully(const OctreeIndex& index, Occupancy::Mask occupancy_mask,
                 const BlockHashMap& block_map);

    void reset();
  };
  mutable QueryCache query_cache_{tree_height_};

  void recursiveClassifier(
      const HashedWaveletOctreeBlock::NodeType& occupancy_node,
      FloatingPoint average_occupancy, Node& classified_node);

  void recursiveClassifier(
      const OctreeIndex& node_index,
      const HashedWaveletOctreeBlock::NodeType* occupancy_node,
      FloatingPoint occupancy_average,
      QueryAccelerator<HashedBlocks::DenseBlockHash>& esdf_map,
      FloatingPoint robot_radius, Node& classified_node);
};
}  // namespace wavemap

#include "wavemap/core/utils/query/impl/classified_map_inl.h"

#endif  // WAVEMAP_CORE_UTILS_QUERY_CLASSIFIED_MAP_H_
