#ifndef WAVEMAP_IO_IMPL_STREAM_CONVERSIONS_INL_H_
#define WAVEMAP_IO_IMPL_STREAM_CONVERSIONS_INL_H_

#include <stack>

#include "wavemap/core/utils/bits/bit_operations.h"

namespace wavemap::io {
template <typename CellDataT, typename CellDataSerializerT>
bool mapToStream(const HashedWaveletOctreeT<CellDataT>& map,
                 std::ostream& ostream) {
  if (!ostream.good()) {
    return false;
  }

  using Block = typename HashedWaveletOctreeT<CellDataT>::Block;
  using NodeConstRefType = typename Block::OctreeType::NodeConstRefType;

  struct StackElement {
    const CellDataT scale;
    NodeConstRefType node;
  };

  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kLayeredHashedWaveletOctree;
  storage_format.write(ostream);

  streamable::HashedWaveletOctreeHeader header;
  header.min_cell_width = map.getMinCellWidth();
  header.min_log_odds = map.getMinLogOdds();
  header.max_log_odds = map.getMaxLogOdds();
  header.tree_height = map.getTreeHeight();
  header.num_blocks = map.getHashMap().size();
  header.write(ostream);

  map.forEachBlock([&ostream, min_log_odds, max_log_odds](
                       const Index3D& block_index, const auto& block) {
    if (!ostream.good()) {
      return;
    }

    streamable::Index3D streamable_block_index{block_index.x(),
                                               block_index.y(),
                                               block_index.z()};
    streamable_block_index.write(ostream);
    CellDataSerializerT::write(ostream, block.getRootScale());

    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
    while (!stack.empty()) {
      const CellDataT scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      streamable::UInt8 allocated_children_bitset{};
      for (const CellDataT& coefficient : node.data()) {
        CellDataSerializerT::write(ostream, coefficient);
      }

      const auto child_scales = Block::Transform::backward({scale, node.data()});
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const auto child_scale = child_scales[relative_child_idx];
        const FloatingPoint child_occupancy =
            static_cast<FloatingPoint>(child_scale);
        if (child_occupancy < min_log_odds || max_log_odds < child_occupancy) {
          continue;
        }

        const auto* child = node.getChild(relative_child_idx);
        if (child) {
          stack.emplace(StackElement{child_scale, *child});
          allocated_children_bitset += (1 << relative_child_idx);
        }
      }

      ostream.write(reinterpret_cast<const char*>(&allocated_children_bitset),
                    sizeof(allocated_children_bitset));
    }
  });

  return ostream.good();
}

template <typename CellDataT, typename CellDataSerializerT>
bool streamToMap(std::istream& istream,
                 typename HashedWaveletOctreeT<CellDataT>::Ptr& map) {
  if (!istream.good()) {
    return false;
  }

  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kLayeredHashedWaveletOctree) {
    return false;
  }

  const auto header = streamable::HashedWaveletOctreeHeader::read(istream);
  HashedWaveletOctreeConfig config;
  config.min_cell_width = header.min_cell_width;
  config.min_log_odds = header.min_log_odds;
  config.max_log_odds = header.max_log_odds;
  config.tree_height = header.tree_height;
  map = std::make_shared<HashedWaveletOctreeT<CellDataT>>(config);

  using Block = typename HashedWaveletOctreeT<CellDataT>::Block;
  using NodeType = typename Block::OctreeType::NodeType;

  for (size_t block_idx = 0; block_idx < header.num_blocks; ++block_idx) {
    if (!istream.good()) {
      return false;
    }

    const auto streamable_block_index = streamable::Index3D::read(istream);
    const Index3D block_index{streamable_block_index.x,
                              streamable_block_index.y,
                              streamable_block_index.z};
    auto& block = map->getOrAllocateBlock(block_index);
    block.getRootScale() = CellDataSerializerT::read(istream);

    std::stack<NodeType*> stack;
    stack.emplace(&block.getRootNode());
    while (!stack.empty()) {
      NodeType* node = stack.top();
      stack.pop();

      for (CellDataT& coefficient : node->data()) {
        coefficient = CellDataSerializerT::read(istream);
      }

      streamable::UInt8 allocated_children_bitset{};
      istream.read(reinterpret_cast<char*>(&allocated_children_bitset),
                   sizeof(allocated_children_bitset));

      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists = bit_ops::is_bit_set(
            allocated_children_bitset, relative_child_idx);
        if (child_exists) {
          stack.emplace(&node->getOrAllocateChild(relative_child_idx));
        }
      }
    }
  }

  return istream.good();
}

template <typename CellDataT, typename CellDataSerializerT>
bool mapToStream(const HashedChunkedWaveletOctreeT<CellDataT>& map,
                 std::ostream& ostream) {
  if (!ostream.good()) {
    return false;
  }

  using Block = typename HashedChunkedWaveletOctreeT<CellDataT>::Block;
  using NodeConstRefType = typename Block::OctreeType::NodeConstRefType;
  struct StackElement {
    const CellDataT scale;
    NodeConstRefType node;
  };

  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kLayeredHashedWaveletOctree;
  storage_format.write(ostream);

  streamable::HashedWaveletOctreeHeader header;
  header.min_cell_width = map.getMinCellWidth();
  header.min_log_odds = map.getMinLogOdds();
  header.max_log_odds = map.getMaxLogOdds();
  header.tree_height = map.getTreeHeight();
  header.num_blocks = map.getHashMap().size();
  header.write(ostream);

  map.forEachBlock([&ostream, min_log_odds, max_log_odds](
                       const Index3D& block_index, const auto& block) {
    if (!ostream.good()) {
      return;
    }

    streamable::Index3D streamable_block_index{
        block_index.x(), block_index.y(), block_index.z()};
    streamable_block_index.write(ostream);
    CellDataSerializerT::write(ostream, block.getRootScale());

    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
    while (!stack.empty()) {
      const CellDataT scale = stack.top().scale;
      const auto node = stack.top().node;
      stack.pop();

      streamable::UInt8 allocated_children_bitset{};
      for (const CellDataT& coefficient : node.data()) {
        CellDataSerializerT::write(ostream, coefficient);
      }

      const auto child_scales = Block::Transform::backward({scale, node.data()});
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const auto child_scale = child_scales[relative_child_idx];
        const FloatingPoint child_occupancy =
            static_cast<FloatingPoint>(child_scale);
        if (child_occupancy < min_log_odds || max_log_odds < child_occupancy) {
          continue;
        }
        if (auto child = node.getChild(relative_child_idx); child) {
          stack.emplace(StackElement{child_scale, *child});
          allocated_children_bitset += (1 << relative_child_idx);
        }
      }

      ostream.write(reinterpret_cast<const char*>(&allocated_children_bitset),
                    sizeof(allocated_children_bitset));
    }
  });
  return ostream.good();
}

template <typename CellDataT, typename CellDataSerializerT>
bool streamToMap(
    std::istream& istream,
    typename HashedChunkedWaveletOctreeT<CellDataT>::Ptr& map) {
  typename HashedWaveletOctreeT<CellDataT>::Ptr logical_map;
  if (!streamToMap<CellDataT, CellDataSerializerT>(istream, logical_map) ||
      !logical_map) {
    return false;
  }

  HashedChunkedWaveletOctreeConfig config;
  config.min_cell_width = logical_map->getMinCellWidth();
  config.min_log_odds = logical_map->getMinLogOdds();
  config.max_log_odds = logical_map->getMaxLogOdds();
  config.tree_height = logical_map->getTreeHeight();
  map = std::make_shared<HashedChunkedWaveletOctreeT<CellDataT>>(config);
  logical_map->forEachVoxelLeaf(
      [&map](const OctreeIndex& node_index, const CellDataT& value) {
        const auto block_index = map->indexToBlockIndex(node_index);
        auto& block = map->getOrAllocateBlock(block_index);
        block.setVoxelValue(map->indexToCellIndex(node_index), value);
      });
  return true;
}
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_IMPL_STREAM_CONVERSIONS_INL_H_
