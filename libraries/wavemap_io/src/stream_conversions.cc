#include "wavemap_io/stream_conversions.h"

namespace wavemap::io {
bool mapToStream(const MapBase& map, std::ostream& ostream) {
  // Call the appropriate mapToStream converter based on the map's derived type
  if (const auto* hashed_blocks = dynamic_cast<const HashedBlocks*>(&map);
      hashed_blocks) {
    io::mapToStream(*hashed_blocks, ostream);
    return true;
  }
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

bool streamToMap(std::istream& istream, MapBase::Ptr& map) {
  // Call the appropriate streamToMap converter based on the received map's type
  const auto storage_format = streamable::StorageFormat::peek(istream);
  switch (storage_format) {
    case streamable::StorageFormat::kHashedBlocks: {
      auto hashed_blocks = std::dynamic_pointer_cast<HashedBlocks>(map);
      if (!streamToMap(istream, hashed_blocks)) {
        return false;
      }
      map = hashed_blocks;
      return true;
    }
    case streamable::StorageFormat::kWaveletOctree: {
      auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map);
      if (!streamToMap(istream, wavelet_octree)) {
        return false;
      }
      map = wavelet_octree;
      return true;
    }
    case streamable::StorageFormat::kHashedWaveletOctree: {
      auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map);
      if (!streamToMap(istream, hashed_wavelet_octree)) {
        return false;
      }
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

void mapToStream(const HashedBlocks& map, std::ostream& ostream) {
  // Serialize the map's data structure type
  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kHashedBlocks;
  storage_format.write(ostream);

  // Serialize the map and data structure's metadata
  streamable::HashedBlocksHeader hashed_blocks_header;
  hashed_blocks_header.min_cell_width = map.getMinCellWidth();
  hashed_blocks_header.min_log_odds = map.getMinLogOdds();
  hashed_blocks_header.max_log_odds = map.getMaxLogOdds();
  hashed_blocks_header.num_blocks = map.getHashMap().size();
  hashed_blocks_header.write(ostream);

  // Iterate over all the map's blocks
  map.forEachBlock(
      [&ostream](const Index3D& block_index, const HashedBlocks::Block& block) {
        // Serialize the block's metadata
        streamable::HashedBlockHeader block_header;
        block_header.block_offset = {block_index.x(), block_index.y(),
                                     block_index.z()};
        block_header.write(ostream);

        // Serialize the block's data (its cells)
        for (FloatingPoint value : block.data()) {
          streamable::Float value_serialized = value;
          ostream.write(reinterpret_cast<const char*>(&value_serialized),
                        sizeof(value_serialized));
        }
      });
}

bool streamToMap(std::istream& istream, HashedBlocks::Ptr& map) {
  // Make sure the map in the input stream is of the correct type
  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kHashedBlocks) {
    return false;
  }

  // Deserialize the map's config and initialize the data structure
  const auto hashed_blocks_header =
      streamable::HashedBlocksHeader::read(istream);
  MapBaseConfig config;
  config.min_cell_width = hashed_blocks_header.min_cell_width;
  config.min_log_odds = hashed_blocks_header.min_log_odds;
  config.max_log_odds = hashed_blocks_header.max_log_odds;
  map = std::make_shared<HashedBlocks>(config);

  // Deserialize all the blocks
  for (size_t block_idx = 0; block_idx < hashed_blocks_header.num_blocks;
       ++block_idx) {
    // Deserialize the block header, containing its position
    const auto block_header = streamable::HashedBlockHeader::read(istream);
    const Index3D block_index{block_header.block_offset.x,
                              block_header.block_offset.y,
                              block_header.block_offset.z};
    auto& block = map->getOrAllocateBlock(block_index);

    // Deserialize the block's remaining data (its cells)
    for (LinearIndex linear_index = 0;
         linear_index < HashedBlocks::Block::kCellsPerBlock; ++linear_index) {
      streamable::Float value_deserialized{};
      istream.read(reinterpret_cast<char*>(&value_deserialized),
                   sizeof(value_deserialized));
      block[linear_index] = value_deserialized;
    }
  }

  return true;
}

void mapToStream(const WaveletOctree& map, std::ostream& ostream) {
  // Serialize the map's data structure type
  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kWaveletOctree;
  storage_format.write(ostream);

  // Serialize the map and data structure's metadata
  streamable::WaveletOctreeHeader wavelet_octree_header;
  wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  wavelet_octree_header.tree_height = map.getTreeHeight();
  // Wavelet scale coefficient of the root node
  wavelet_octree_header.root_node_scale_coefficient = map.getRootScale();
  wavelet_octree_header.write(ostream);

  // Serialize the map's data (all nodes of the octree)
  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    // Serialize the node's data
    streamable::WaveletOctreeNode streamable_node;
    std::copy(node.data().begin(), node.data().end(),
              streamable_node.detail_coefficients.begin());
    // Indicate which of its children will be serialized next
    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        streamable_node.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    streamable_node.write(ostream);
  }
}

bool streamToMap(std::istream& istream, WaveletOctree::Ptr& map) {
  // Make sure the map in the input stream is of the correct type
  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kWaveletOctree) {
    return false;
  }

  // Deserialize the map's config and initialize the data structure
  const auto wavelet_octree_header =
      streamable::WaveletOctreeHeader::read(istream);
  WaveletOctreeConfig config;
  config.min_cell_width = wavelet_octree_header.min_cell_width;
  config.min_log_odds = wavelet_octree_header.min_log_odds;
  config.max_log_odds = wavelet_octree_header.max_log_odds;
  config.tree_height = wavelet_octree_header.tree_height;
  map = std::make_shared<WaveletOctree>(config);

  // Deserialize the map's data
  // We start with the scale coefficient stored in the map's root node,
  // which corresponds to the average value of the entire map
  map->getRootScale() = wavelet_octree_header.root_node_scale_coefficient;

  // Followed by the remaining data stored in octree nodes
  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  while (!stack.empty()) {
    WaveletOctree::NodeType* node = stack.top();
    stack.pop();

    // Deserialize the node's (wavelet) detail coefficients
    const auto read_node = streamable::WaveletOctreeNode::read(istream);
    std::copy(read_node.detail_coefficients.begin(),
              read_node.detail_coefficients.end(), node->data().begin());

    // Evaluate which of the node's children are coming next
    // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
    //       the nodes are popped from the stack in increasing order.
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists = bit_ops::is_bit_set(
          read_node.allocated_children_bitset, relative_child_idx);
      if (child_exists) {
        stack.emplace(&node->getOrAllocateChild(relative_child_idx));
      }
    }
  }

  return true;
}

void mapToStream(const HashedWaveletOctree& map, std::ostream& ostream) {
  // Define convenience types and constants
  struct StackElement {
    const FloatingPoint scale;
    const HashedWaveletOctreeBlock::NodeType& node;
  };
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  // Indicate the map's data structure type
  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kHashedWaveletOctree;
  storage_format.write(ostream);

  // Serialize the map and data structure's metadata
  streamable::HashedWaveletOctreeHeader hashed_wavelet_octree_header;
  hashed_wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  hashed_wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  hashed_wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  hashed_wavelet_octree_header.tree_height = map.getTreeHeight();
  hashed_wavelet_octree_header.num_blocks = map.getHashMap().size();
  hashed_wavelet_octree_header.write(ostream);

  // Iterate over all the map's blocks
  map.forEachBlock([&ostream, min_log_odds, max_log_odds](
                       const Index3D& block_index, const auto& block) {
    // Serialize the block's metadata
    streamable::HashedWaveletOctreeBlockHeader block_header;
    block_header.root_node_offset = {block_index.x(), block_index.y(),
                                     block_index.z()};
    // Wavelet scale coefficient of the block's root node
    block_header.root_node_scale_coefficient = block.getRootScale();
    block_header.write(ostream);

    // Serialize the block's data (all nodes of its octree)
    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      // Serialize the node's data
      streamable::WaveletOctreeNode streamable_node;
      std::copy(node.data().begin(), node.data().end(),
                streamable_node.detail_coefficients.begin());

      // Evaluate which of its children should be serialized
      const auto child_scales =
          HashedWaveletOctreeBlock::Transform::backward({scale, node.data()});
      // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
      //       the nodes are popped from the stack in increasing order.
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        // If the child is saturated, we don't need to store its descendants
        const auto child_scale = child_scales[relative_child_idx];
        if (child_scale < min_log_odds || max_log_odds < child_scale) {
          continue;
        }
        // Otherwise, indicate that the child will be serialized
        // and add it to the stack
        const auto* child = node.getChild(relative_child_idx);
        if (child) {
          stack.emplace(StackElement{child_scale, *child});
          streamable_node.allocated_children_bitset +=
              (1 << relative_child_idx);
        }
      }
      streamable_node.write(ostream);
    }
  });
}

bool streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map) {
  // Make sure the map in the input stream is of the correct type
  if (streamable::StorageFormat::read(istream) !=
      streamable::StorageFormat::kHashedWaveletOctree) {
    return false;
  }

  // Deserialize the map's config and initialize the data structure
  const auto hashed_wavelet_octree_header =
      streamable::HashedWaveletOctreeHeader::read(istream);
  HashedWaveletOctreeConfig config;
  config.min_cell_width = hashed_wavelet_octree_header.min_cell_width;
  config.min_log_odds = hashed_wavelet_octree_header.min_log_odds;
  config.max_log_odds = hashed_wavelet_octree_header.max_log_odds;
  config.tree_height = hashed_wavelet_octree_header.tree_height;
  map = std::make_shared<HashedWaveletOctree>(config);

  // Deserialize all the blocks
  for (size_t block_idx = 0;
       block_idx < hashed_wavelet_octree_header.num_blocks; ++block_idx) {
    // Deserialize the block header, containing its position and scale coeff.
    const auto block_header =
        streamable::HashedWaveletOctreeBlockHeader::read(istream);
    const Index3D block_index{block_header.root_node_offset.x,
                              block_header.root_node_offset.y,
                              block_header.root_node_offset.z};
    auto& block = map->getOrAllocateBlock(block_index);
    // Wavelet scale coefficient of the block's root node
    block.getRootScale() = block_header.root_node_scale_coefficient;

    // Deserialize the block's remaining data into octree nodes
    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    while (!stack.empty()) {
      WaveletOctree::NodeType* node = stack.top();
      stack.pop();

      // Deserialize the node's (wavelet) detail coefficients
      const auto read_node = streamable::WaveletOctreeNode::read(istream);
      std::copy(read_node.detail_coefficients.begin(),
                read_node.detail_coefficients.end(), node->data().begin());

      // Evaluate which of the node's children are coming next
      // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
      //       the nodes are popped from the stack in increasing order.
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists = bit_ops::is_bit_set(
            read_node.allocated_children_bitset, relative_child_idx);
        if (child_exists) {
          stack.emplace(&node->getOrAllocateChild(relative_child_idx));
        }
      }
    }
  }

  return true;
}

void mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream) {
  // Define convenience types and constants
  struct StackElement {
    const FloatingPoint scale;
    HashedChunkedWaveletOctreeBlock::ChunkedOctreeType::NodeConstRefType node;
  };
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  // Indicate the map's data structure type
  streamable::StorageFormat storage_format =
      streamable::StorageFormat::kHashedWaveletOctree;
  storage_format.write(ostream);

  // Serialize the map and data structure's metadata
  streamable::HashedWaveletOctreeHeader hashed_wavelet_octree_header;
  hashed_wavelet_octree_header.min_cell_width = map.getMinCellWidth();
  hashed_wavelet_octree_header.min_log_odds = map.getMinLogOdds();
  hashed_wavelet_octree_header.max_log_odds = map.getMaxLogOdds();
  hashed_wavelet_octree_header.tree_height = map.getTreeHeight();
  hashed_wavelet_octree_header.num_blocks = map.getHashMap().size();
  hashed_wavelet_octree_header.write(ostream);

  // Iterate over all the map's blocks
  map.forEachBlock([&ostream, min_log_odds, max_log_odds](
                       const Index3D& block_index, const auto& block) {
    // Serialize the block's metadata
    streamable::HashedWaveletOctreeBlockHeader block_header;
    block_header.root_node_offset = {block_index.x(), block_index.y(),
                                     block_index.z()};
    // Wavelet scale coefficient of the block's root node
    block_header.root_node_scale_coefficient = block.getRootScale();
    block_header.write(ostream);

    // Serialize the block's data (all nodes of its octree)
    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      auto node = stack.top().node;
      stack.pop();

      // Serialize the node's data
      streamable::WaveletOctreeNode streamable_node;
      std::copy(node.data().begin(), node.data().end(),
                streamable_node.detail_coefficients.begin());

      // Evaluate which of its children should be serialized
      const auto child_scales =
          HashedWaveletOctreeBlock::Transform::backward({scale, node.data()});
      // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
      //       the nodes are popped from the stack in increasing order.
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        // If the child is saturated, we don't need to store its descendants
        const auto child_scale = child_scales[relative_child_idx];
        if (child_scale < min_log_odds || max_log_odds < child_scale) {
          continue;
        }
        // Otherwise, indicate that the child will be serialized
        // and add it to the stack
        if (auto child = node.getChild(relative_child_idx); child) {
          stack.emplace(StackElement{child_scale, *child});
          streamable_node.allocated_children_bitset +=
              (1 << relative_child_idx);
        }
      }
      streamable_node.write(ostream);
    }
  });
}
}  // namespace wavemap::io
