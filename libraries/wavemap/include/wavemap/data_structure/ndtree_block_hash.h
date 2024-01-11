#ifndef WAVEMAP_DATA_STRUCTURE_NDTREE_BLOCK_HASH_H_
#define WAVEMAP_DATA_STRUCTURE_NDTREE_BLOCK_HASH_H_

#include <utility>

#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
template <typename CellDataT, int dim>
class NdtreeBlockHash {
 public:
  static constexpr IndexElement kDim = dim;

  using Block = Ndtree<CellDataT, dim>;
  using Node = typename Block::NodeType;
  using BlockHashMap = SpatialHash<Block, dim>;

  using IndexType = NdtreeIndex<dim>;
  using HeightType = IndexElement;
  static constexpr HeightType kChunkHeight = 1;

  explicit NdtreeBlockHash(HeightType max_tree_height,
                           CellDataT default_value = CellDataT{})
      : max_height_(max_tree_height), default_value_(default_value) {}

  bool empty() const { return block_map_.empty(); }
  size_t size() const;
  void clear() { block_map_.clear(); }

  HeightType getMaxHeight() const { return max_height_; }

  bool hasBlock(const Index<dim>& block_index) const;
  bool eraseBlock(const Index<dim>& block_index);
  Block* getBlock(const Index<dim>& block_index);
  const Block* getBlock(const Index<dim>& block_index) const;
  Block& getOrAllocateBlock(const Index<dim>& block_index);

  bool hasNode(const IndexType& index) const;
  bool eraseNode(const IndexType& index);
  Node* getNode(const IndexType& index);
  const Node* getNode(const IndexType& index) const;
  Node& getOrAllocateNode(const IndexType& index);

  std::pair<Node*, HeightType> getNodeOrAncestor(const IndexType& index);
  std::pair<const Node*, HeightType> getNodeOrAncestor(
      const IndexType& index) const;

  // NOTE: The values are stored in trees. Therefore, allocating a new node also
  //       allocates all the parents of the node that do not yet exist. Their
  //       data fields will be initialized to default_value_.
  //       If you want to make sure a value has been explicitly set by you,
  //       check that it differs from the default_value_. For example, using the
  //       equalsDefaultValue(value) method below.
  bool hasValue(const IndexType& index) const;
  CellDataT* getValue(const IndexType& index);
  const CellDataT* getValue(const IndexType& index) const;
  CellDataT& getOrAllocateValue(const IndexType& index);

  std::pair<CellDataT*, HeightType> getValueOrAncestor(const IndexType& index);
  std::pair<const CellDataT*, HeightType> getValueOrAncestor(
      const IndexType& index) const;

  const CellDataT& getValueOrDefault(const IndexType& index) const;
  const CellDataT& getDefaultValue() const { return default_value_; }
  bool equalsDefaultValue(const CellDataT& value) const;

  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn);
  template <typename IndexedBlockVisitor>
  void forEachBlock(IndexedBlockVisitor visitor_fn) const;

  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn);
  template <typename IndexedLeafVisitorFunction>
  void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const;

  Index<dim> indexToBlockIndex(const IndexType& index) const;
  IndexType indexToCellIndex(const IndexType& index) const;
  IndexType cellAndBlockIndexToIndex(const Index<dim>& block_index,
                                     const IndexType& cell_index) const;

 protected:
  const HeightType max_height_;
  const CellDataT default_value_;
  BlockHashMap block_map_;
};

template <typename CellDataT>
using BinaryTreeBlockHash = NdtreeBlockHash<CellDataT, 1>;
template <typename CellDataT>
using QuadtreeBlockHash = NdtreeBlockHash<CellDataT, 2>;
template <typename CellDataT>
using OctreeBlockHash = NdtreeBlockHash<CellDataT, 3>;
}  // namespace wavemap

#include "wavemap/data_structure/impl/ndtree_block_hash_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_NDTREE_BLOCK_HASH_H_
