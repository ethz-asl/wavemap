#include "wavemap_io/stream_conversions.h"

namespace wavemap::io {
bool mapToStream(const VolumetricDataStructureBase& map,
                 std::ostream& ostream) {
  if (const auto* wavelet_octree = dynamic_cast<const WaveletOctree*>(&map);
      wavelet_octree) {
    io::mapToStream(*wavelet_octree, ostream);
    return true;
  }

  if (const auto* hashed_wavelet_octree =
          dynamic_cast<const HashedWaveletOctree*>(&map);
      hashed_wavelet_octree) {
    io::mapToStream(*hashed_wavelet_octree, ostream);
    return true;
  }

  if (const auto* hashed_chunked_wavelet_octree =
          dynamic_cast<const HashedChunkedWaveletOctree*>(&map);
      hashed_chunked_wavelet_octree) {
    io::mapToStream(*hashed_chunked_wavelet_octree, ostream);
    return true;
  }

  LOG(WARNING) << "Could not serialize requested map to stream. "
                  "Map type not yet supported.";
  return false;
}

bool streamToMap(std::istream& istream, VolumetricDataStructureBase::Ptr& map) {
  const auto storage_format = streamable::StorageFormat::peek(istream);
  switch (storage_format.toTypeId()) {
    case streamable::StorageFormat::kWaveletOctree: {
      auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map);
      streamToMap(istream, wavelet_octree);
      map = wavelet_octree;
      return true;
    }
    case streamable::StorageFormat::kHashedWaveletOctree: {
      auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map);
      streamToMap(istream, hashed_wavelet_octree);
      map = hashed_wavelet_octree;
      return true;
    }
    default:
      LOG(WARNING) << "Could not deserialize map stream to a wavemap map. "
                      "Unsupported map type.";
      map = nullptr;
      return false;
  }
}

void mapToStream(const WaveletOctree& map, std::ostream& ostream) {
  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kWaveletOctree;
  storage_format.write(ostream);

  streamable::WaveletOctreeHeader wavelet_octree_header;
  wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  wavelet_octree_header.tree_height = map.getTreeHeight();
  wavelet_octree_header.root_node_scale_coefficient = map.getRootScale();
  wavelet_octree_header.write(ostream);

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    streamable::WaveletOctreeNode streamable_node;
    std::copy(node.data().begin(), node.data().end(),
              streamable_node.detail_coefficients.begin());
    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        streamable_node.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    streamable_node.write(ostream);
  }
}

void streamToMap(std::istream& istream, WaveletOctree::Ptr& map) {
  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kWaveletOctree) {
    return;
  }

  const auto wavelet_octree_header =
      streamable::WaveletOctreeHeader::read(istream);

  WaveletOctreeConfig config;
  config.min_cell_width = wavelet_octree_header.min_cell_width;
  config.min_log_odds = wavelet_octree_header.min_log_odds;
  config.max_log_odds = wavelet_octree_header.max_log_odds;
  config.tree_height = wavelet_octree_header.tree_height;
  map = std::make_shared<WaveletOctree>(config);

  map->getRootScale() = wavelet_octree_header.root_node_scale_coefficient;

  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  while (!stack.empty()) {
    WaveletOctree::NodeType* node = stack.top();
    stack.pop();

    const auto read_node = streamable::WaveletOctreeNode::read(istream);
    std::copy(read_node.detail_coefficients.begin(),
              read_node.detail_coefficients.end(), node->data().begin());
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          read_node.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(node->allocateChild(relative_child_idx));
      }
    }
  }
}

void mapToStream(const HashedWaveletOctree& map, std::ostream& ostream) {
  struct StackElement {
    const FloatingPoint scale;
    const HashedWaveletOctreeBlock::NodeType& node;
  };

  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kHashedWaveletOctree;
  storage_format.write(ostream);

  streamable::HashedWaveletOctreeHeader hashed_wavelet_octree_header;
  hashed_wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  hashed_wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  hashed_wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  hashed_wavelet_octree_header.tree_height = map.getTreeHeight();
  hashed_wavelet_octree_header.num_blocks = map.getBlocks().size();
  hashed_wavelet_octree_header.write(ostream);

  for (const auto& [block_index, block] : map.getBlocks()) {
    streamable::HashedWaveletOctreeBlockHeader block_header;
    block_header.root_node_offset = {block_index.x(), block_index.y(),
                                     block_index.z()};
    block_header.root_node_scale_coefficient = block.getRootScale();
    block_header.write(ostream);

    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      streamable::WaveletOctreeNode streamable_node;
      std::copy(node.data().begin(), node.data().end(),
                streamable_node.detail_coefficients.begin());
      const auto child_scales =
          HashedWaveletOctreeBlock::Transform::backward({scale, node.data()});
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const auto child_scale = child_scales[relative_child_idx];
        if (child_scale < min_log_odds || max_log_odds < child_scale) {
          continue;
        }
        const auto* child = node.getChild(relative_child_idx);
        if (child) {
          stack.emplace(StackElement{child_scale, *child});
          streamable_node.allocated_children_bitset +=
              (1 << relative_child_idx);
        }
      }
      streamable_node.write(ostream);
    }
  }
}

void streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map) {
  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kHashedWaveletOctree) {
    return;
  }

  const auto hashed_wavelet_octree_header =
      streamable::HashedWaveletOctreeHeader::read(istream);

  HashedWaveletOctreeConfig config;
  config.min_cell_width = hashed_wavelet_octree_header.min_cell_width;
  config.min_log_odds = hashed_wavelet_octree_header.min_log_odds;
  config.max_log_odds = hashed_wavelet_octree_header.max_log_odds;
  config.tree_height = hashed_wavelet_octree_header.tree_height;
  map = std::make_shared<HashedWaveletOctree>(config);

  for (size_t block_idx = 0;
       block_idx < hashed_wavelet_octree_header.num_blocks; ++block_idx) {
    const auto block_header =
        streamable::HashedWaveletOctreeBlockHeader::read(istream);
    auto& block = map->getOrAllocateBlock({block_header.root_node_offset.x,
                                           block_header.root_node_offset.y,
                                           block_header.root_node_offset.z});
    block.getRootScale() = block_header.root_node_scale_coefficient;

    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    while (!stack.empty()) {
      WaveletOctree::NodeType* node = stack.top();
      stack.pop();

      const auto read_node = streamable::WaveletOctreeNode::read(istream);
      std::copy(read_node.detail_coefficients.begin(),
                read_node.detail_coefficients.end(), node->data().begin());
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists =
            read_node.allocated_children_bitset & (1 << relative_child_idx);
        if (child_exists) {
          stack.emplace(node->allocateChild(relative_child_idx));
        }
      }
    }
  }
}

void mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream) {
  struct StackElement {
    const OctreeIndex node_index;
    const HashedChunkedWaveletOctreeBlock::NodeChunkType& chunk;
    const FloatingPoint scale_coefficient;
  };

  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;
  const auto tree_height = map.getTreeHeight();
  const auto chunk_height = map.getChunkHeight();

  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kHashedWaveletOctree;
  storage_format.write(ostream);

  streamable::HashedWaveletOctreeHeader hashed_wavelet_octree_header;
  hashed_wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  hashed_wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  hashed_wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  hashed_wavelet_octree_header.tree_height = map.getTreeHeight();
  hashed_wavelet_octree_header.num_blocks = map.getBlocks().size();
  hashed_wavelet_octree_header.write(ostream);

  for (const auto& [block_index, block] : map.getBlocks()) {
    streamable::HashedWaveletOctreeBlockHeader block_header;
    block_header.root_node_offset = {block_index.x(), block_index.y(),
                                     block_index.z()};
    block_header.root_node_scale_coefficient = block.getRootScale();
    block_header.write(ostream);

    std::stack<StackElement> stack;
    stack.emplace(StackElement{{tree_height, block_index},
                               block.getRootChunk(),
                               block.getRootScale()});
    while (!stack.empty()) {
      const OctreeIndex index = stack.top().node_index;
      const FloatingPoint scale = stack.top().scale_coefficient;
      const auto& chunk = stack.top().chunk;
      stack.pop();

      const MortonIndex morton_code = convert::nodeIndexToMorton(index);
      const int chunk_top_height =
          chunk_height * int_math::div_round_up(index.height, chunk_height);
      const LinearIndex relative_node_index =
          OctreeIndex::computeTreeTraversalDistance(
              morton_code, chunk_top_height, index.height);

      streamable::WaveletOctreeNode streamable_node;
      const auto& node_data = chunk.nodeData(relative_node_index);
      std::copy(node_data.begin(), node_data.end(),
                streamable_node.detail_coefficients.begin());

      if (!chunk.nodeHasAtLeastOneChild(relative_node_index)) {
        continue;
      }

      const auto child_scales =
          HashedWaveletOctreeBlock::Transform::backward({scale, node_data});
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const FloatingPoint child_scale = child_scales[relative_child_idx];
        if (child_scale < min_log_odds || max_log_odds < child_scale) {
          continue;
        }

        const OctreeIndex child_index =
            index.computeChildIndex(relative_child_idx);
        if (child_index.height % chunk_height == 0) {
          const MortonIndex child_morton =
              convert::nodeIndexToMorton(child_index);
          const LinearIndex linear_child_index =
              OctreeIndex::computeLevelTraversalDistance(
                  child_morton, chunk_top_height, child_index.height);
          if (chunk.hasChild(linear_child_index)) {
            const auto& child_chunk = *chunk.getChild(linear_child_index);
            stack.emplace(StackElement{child_index, child_chunk, child_scale});
            streamable_node.allocated_children_bitset +=
                (1 << relative_child_idx);
          }
        } else {
          stack.emplace(StackElement{child_index, chunk, child_scale});
          streamable_node.allocated_children_bitset +=
              (1 << relative_child_idx);
        }
      }

      streamable_node.write(ostream);
    }
  }
}
}  // namespace wavemap::io
