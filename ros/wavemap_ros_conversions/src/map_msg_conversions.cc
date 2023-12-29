#include "wavemap_ros_conversions/map_msg_conversions.h"

#include <ros/console.h>
#include <tracy/Tracy.hpp>

namespace wavemap::convert {
bool mapToRosMsg(const MapBase& map, const std::string& frame_id,
                 const ros::Time& stamp, wavemap_msgs::Map& msg) {
  // Write the msg header
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  // Write the map data
  if (const auto* hashed_blocks = dynamic_cast<const HashedBlocks*>(&map);
      hashed_blocks) {
    convert::mapToRosMsg(*hashed_blocks, msg.hashed_blocks.emplace_back());
    return true;
  }
  if (const auto* wavelet_octree = dynamic_cast<const WaveletOctree*>(&map);
      wavelet_octree) {
    convert::mapToRosMsg(*wavelet_octree, msg.wavelet_octree.emplace_back());
    return true;
  }
  if (const auto* hashed_wavelet_octree =
          dynamic_cast<const HashedWaveletOctree*>(&map);
      hashed_wavelet_octree) {
    convert::mapToRosMsg(*hashed_wavelet_octree,
                         msg.hashed_wavelet_octree.emplace_back());
    return true;
  }
  if (const auto* hashed_chunked_wavelet_octree =
          dynamic_cast<const HashedChunkedWaveletOctree*>(&map);
      hashed_chunked_wavelet_octree) {
    convert::mapToRosMsg(*hashed_chunked_wavelet_octree,
                         msg.hashed_wavelet_octree.emplace_back());
    return true;
  }

  ROS_WARN(
      "Could not serialize requested map to ROS msg. "
      "Map type not yet supported.");
  return false;
}

bool rosMsgToMap(const wavemap_msgs::Map& msg, MapBase::Ptr& map) {
  ZoneScoped;
  // Check validity
  bool is_valid = true;
  std::string error_msg =
      "Maps must be serialized as either one hashed block data structure, "
      "wavelet octree, or hashed wavelet octree. ";
  if (1 < msg.hashed_blocks.size()) {
    error_msg += "Message contains multiple hashed block data structures. ";
    is_valid = false;
  }
  if (1 < msg.wavelet_octree.size()) {
    error_msg += "Message contains multiple wavelet octrees. ";
    is_valid = false;
  }
  if (1 < msg.hashed_wavelet_octree.size()) {
    error_msg += "Message contains multiple hashed wavelet octrees. ";
    is_valid = false;
  }
  if (msg.hashed_blocks.empty() && msg.wavelet_octree.empty() &&
      msg.hashed_wavelet_octree.empty()) {
    error_msg += "Message contains neither. ";
    is_valid = false;
  }
  if (!is_valid) {
    ROS_WARN_STREAM(error_msg + "Ignoring.");
    map = nullptr;
    return false;
  }

  // Read the data
  if (!msg.hashed_blocks.empty()) {
    auto hashed_blocks = std::dynamic_pointer_cast<HashedBlocks>(map);
    rosMsgToMap(msg.hashed_blocks.front(), hashed_blocks);
    map = hashed_blocks;
    return true;
  }
  if (!msg.wavelet_octree.empty()) {
    auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map);
    rosMsgToMap(msg.wavelet_octree.front(), wavelet_octree);
    map = wavelet_octree;
    return true;
  }
  if (!msg.hashed_wavelet_octree.empty()) {
    auto hashed_wavelet_octree =
        std::dynamic_pointer_cast<HashedWaveletOctree>(map);
    rosMsgToMap(msg.hashed_wavelet_octree.front(), hashed_wavelet_octree);
    map = hashed_wavelet_octree;
    return true;
  }

  ROS_WARN(
      "Conversion of the requested map ROS msg to a wavemap map is "
      "not yet supported.");
  map = nullptr;
  return false;
}

void mapToRosMsg(const HashedBlocks& map, wavemap_msgs::HashedBlocks& msg) {
  ZoneScoped;
  // Serialize the map and data structure's metadata
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();

  // Indicate which blocks are allocated in the map
  // NOTE: This is done such that subscribers know when blocks should be removed
  //       during incremental map transmission.
  msg.allocated_block_indices.reserve(map.getHashMap().size());
  map.forEachBlock([&msg](const Index3D& block_index, const auto& /*block*/) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  });

  // Serialize all blocks
  map.forEachBlock(
      [&msg](const Index3D& block_index, const HashedBlocks::Block& block) {
        auto& block_msg = msg.blocks.emplace_back();
        // Serialize the block's metadata
        block_msg.block_offset = {block_index.x(), block_index.y(),
                                  block_index.z()};
        // Serialize the block's data (dense grid)
        block_msg.values.reserve(HashedBlocks::Block::kCellsPerBlock);
        for (FloatingPoint value : block.data()) {
          block_msg.values.emplace_back(value);
        }
      });
}

void rosMsgToMap(const wavemap_msgs::HashedBlocks& msg,
                 HashedBlocks::Ptr& map) {
  ZoneScoped;
  // Deserialize the map's config
  MapBaseConfig config;
  config.min_cell_width = msg.min_cell_width;
  config.min_log_odds = msg.min_log_odds;
  config.max_log_odds = msg.max_log_odds;

  // Check if the map already exists and has compatible settings
  if (map && map->getConfig() == config) {
    // Load allocated block list into a hash table for quick membership lookups
    std::unordered_set<Index3D, Index3DHash> allocated_blocks;
    for (const auto& block_index : msg.allocated_block_indices) {
      allocated_blocks.emplace(block_index.x, block_index.y, block_index.z);
    }
    // Remove local blocks that should no longer exist according to the map msg
    map->getHashMap().eraseBlockIf(
        [&allocated_blocks](const Index3D& block_index, const auto& /*block*/) {
          return !allocated_blocks.count(block_index);
        });
  } else {
    // Otherwise create a new map
    map = std::make_shared<HashedBlocks>(config);
  }

  // Deserialize all blocks
  for (const auto& block_msg : msg.blocks) {
    // Get the block
    const Index3D block_index{block_msg.block_offset[0],
                              block_msg.block_offset[1],
                              block_msg.block_offset[2]};
    auto& block = map->getOrAllocateBlock(block_index);

    // Deserialize the block's data (dense grid)
    for (LinearIndex linear_index = 0; linear_index < block_msg.values.size();
         ++linear_index) {
      block[linear_index] = block_msg.values[linear_index];
    }
  }
}

void mapToRosMsg(const WaveletOctree& map, wavemap_msgs::WaveletOctree& msg) {
  ZoneScoped;
  // Serialize the map and data structure's metadata
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();
  // Wavelet scale coefficient of the root node
  msg.root_node_scale_coefficient = map.getRootScale();

  // Serialize the map's data (all nodes of the octree)
  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    // Serialize the node's data
    auto& node_msg = msg.nodes.emplace_back();
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());
    // Indicate which of its children will be serialized next
    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}

void rosMsgToMap(const wavemap_msgs::WaveletOctree& msg,
                 WaveletOctree::Ptr& map) {
  ZoneScoped;
  // Deserialize the map's config
  WaveletOctreeConfig config;
  config.min_cell_width = msg.min_cell_width;
  config.min_log_odds = msg.min_log_odds;
  config.max_log_odds = msg.max_log_odds;
  config.tree_height = msg.tree_height;

  // Check if the map already exists and has compatible settings
  if (!map || map->getConfig() != config) {
    // Otherwise create a new one
    map = std::make_shared<WaveletOctree>(config);
  }

  // Deserialize the map's data
  // We start with the scale coefficient stored in the map's root node,
  // which corresponds to the average value of the entire map
  map->getRootScale() = msg.root_node_scale_coefficient;

  // Followed by the remaining data stored in octree nodes
  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  for (const auto& node_msg : msg.nodes) {
    DCHECK(!stack.empty());
    WaveletOctree::NodeType* node = stack.top();
    stack.pop();

    // Deserialize the node's (wavelet) detail coefficients
    std::copy(node_msg.detail_coefficients.cbegin(),
              node_msg.detail_coefficients.cend(), node->data().begin());

    // Evaluate which of the node's children are coming next
    // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
    //       the nodes are popped from the stack in increasing order.
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(&node->getOrAllocateChild(relative_child_idx));
      }
    }
  }
}

void mapToRosMsg(
    const HashedWaveletOctree& map, wavemap_msgs::HashedWaveletOctree& msg,
    std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks,
    std::shared_ptr<ThreadPool> thread_pool) {
  ZoneScoped;
  // Constants
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  // Serialize the map and data structure's metadata
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();

  // Indicate which blocks are allocated in the map
  // NOTE: This is done such that subscribers know when blocks should be removed
  //       during incremental map transmission.
  msg.allocated_block_indices.reserve(map.getHashMap().size());
  map.forEachBlock([&msg](const Index3D& block_index, const auto& /*block*/) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  });

  // If blocks to include were specified, check that they exist
  // and remove the ones that do not
  if (include_blocks) {
    for (auto include_block_it = include_blocks->begin();
         include_block_it != include_blocks->end();) {
      if (map.hasBlock(*include_block_it)) {
        ++include_block_it;
      } else {
        include_block_it = include_blocks->erase(include_block_it);
      }
    }
  } else {  // Otherwise, include all blocks
    include_blocks.emplace();
    map.forEachBlock(
        [&include_blocks](const Index3D& block_index, const auto& /*block*/) {
          include_blocks->emplace(block_index);
        });
  }

  // Serialize the specified blocks
  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    if (const auto* block = map.getBlock(block_index); block) {
      auto& block_msg = msg.blocks[block_idx++];
      // If a thread pool was provided, use it
      if (thread_pool) {
        thread_pool->add_task(
            [block_index, block, min_log_odds, max_log_odds, &block_msg]() {
              blockToRosMsg(block_index, *block, min_log_odds, max_log_odds,
                            block_msg);
            });
      } else {  // Otherwise, use the current thread
        blockToRosMsg(block_index, *block, min_log_odds, max_log_odds,
                      block_msg);
      }
    }
  }

  // If a thread pool was used, wait for all jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}

void blockToRosMsg(const HashedWaveletOctree::BlockIndex& block_index,
                   const HashedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap_msgs::HashedWaveletOctreeBlock& msg) {
  ZoneScoped;
  // Convenience type for elements on the stack used to iterate over the map
  struct StackElement {
    const FloatingPoint scale;
    const HashedWaveletOctreeBlock::NodeType& node;
  };

  // Serialize the block's metadata
  msg.root_node_offset.x = block_index.x();
  msg.root_node_offset.y = block_index.y();
  msg.root_node_offset.z = block_index.z();
  // Wavelet scale coefficient of the block's root node
  msg.root_node_scale_coefficient = block.getRootScale();

  // Serialize the block's data (all nodes of its octree)
  std::stack<StackElement> stack;
  stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
  while (!stack.empty()) {
    const FloatingPoint scale = stack.top().scale;
    const auto& node = stack.top().node;
    stack.pop();

    // Serialize the node's data
    auto& node_msg = msg.nodes.emplace_back();
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());
    node_msg.allocated_children_bitset = 0;

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
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}

void rosMsgToMap(const wavemap_msgs::HashedWaveletOctree& msg,
                 HashedWaveletOctree::Ptr& map) {
  ZoneScoped;
  // Deserialize the map's config and initialize the data structure
  HashedWaveletOctreeConfig config;
  config.min_cell_width = msg.min_cell_width;
  config.min_log_odds = msg.min_log_odds;
  config.max_log_odds = msg.max_log_odds;
  config.tree_height = msg.tree_height;

  // Check if the map already exists and has compatible settings
  if (map && map->getConfig() == config) {
    // Load allocated block list into a hash table for quick membership lookups
    std::unordered_set<Index3D, Index3DHash> allocated_blocks;
    for (const auto& block_index : msg.allocated_block_indices) {
      allocated_blocks.emplace(block_index.x, block_index.y, block_index.z);
    }
    // Remove local blocks that should no longer exist according to the map msg
    map->getHashMap().eraseBlockIf(
        [&allocated_blocks](const Index3D& block_index, const auto& /*block*/) {
          return !allocated_blocks.count(block_index);
        });
  } else {
    // Otherwise create a new map
    map = std::make_shared<HashedWaveletOctree>(config);
  }

  // Deserialize all the transferred blocks
  for (const auto& block_msg : msg.blocks) {
    const Index3D block_index{block_msg.root_node_offset.x,
                              block_msg.root_node_offset.y,
                              block_msg.root_node_offset.z};

    // Reset the block if it already existed
    const bool block_existed = map->hasBlock(block_index);
    auto& block = map->getOrAllocateBlock(block_index);
    if (block_existed) {
      block.clear();
    }

    // Deserialize the wavelet scale coefficient of the block's root node
    block.getRootScale() = block_msg.root_node_scale_coefficient;

    // Deserialize the block's remaining data into octree nodes
    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    for (const auto& node_msg : block_msg.nodes) {
      DCHECK(!stack.empty());
      WaveletOctree::NodeType* node = stack.top();
      stack.pop();

      // Deserialize the node's (wavelet) detail coefficients
      std::copy(node_msg.detail_coefficients.cbegin(),
                node_msg.detail_coefficients.cend(), node->data().begin());

      // Evaluate which of the node's children are coming next
      // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
      //       the nodes are popped from the stack in increasing order.
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists =
            node_msg.allocated_children_bitset & (1 << relative_child_idx);
        if (child_exists) {
          stack.emplace(&node->getOrAllocateChild(relative_child_idx));
        }
      }
    }
  }
}

void mapToRosMsg(
    const HashedChunkedWaveletOctree& map,
    wavemap_msgs::HashedWaveletOctree& msg,
    std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks,
    std::shared_ptr<ThreadPool> thread_pool) {
  ZoneScoped;
  // Constants
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;
  const auto tree_height = map.getTreeHeight();

  // Serialize the map and data structure's metadata
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();

  // Indicate which blocks are allocated in the map
  // NOTE: This is done such that subscribers know when blocks should be removed
  //       during incremental map transmission.
  msg.allocated_block_indices.reserve(map.getHashMap().size());
  map.forEachBlock([&msg](const Index3D& block_index, const auto& /*block*/) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  });

  // If blocks to include were specified, check that they exist
  // and remove the ones that don't
  if (include_blocks) {
    for (auto include_block_it = include_blocks->begin();
         include_block_it != include_blocks->end();) {
      if (map.hasBlock(*include_block_it)) {
        ++include_block_it;
      } else {
        include_block_it = include_blocks->erase(include_block_it);
      }
    }
  } else {  // Otherwise, include all blocks
    include_blocks.emplace();
    map.forEachBlock(
        [&include_blocks](const Index3D& block_index, const auto& /*block*/) {
          include_blocks->emplace(block_index);
        });
  }

  // Serialize the specified blocks
  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    if (const auto* block = map.getBlock(block_index); block) {
      auto& block_msg = msg.blocks[block_idx++];
      // If a thread pool was provided, use it
      if (thread_pool) {
        thread_pool->add_task([block_index, block, min_log_odds, max_log_odds,
                               tree_height, &block_msg]() {
          blockToRosMsg(block_index, *block, min_log_odds, max_log_odds,
                        block_msg);
        });
      } else {  // Otherwise, use the current thread
        blockToRosMsg(block_index, *block, min_log_odds, max_log_odds,
                      block_msg);
      }
    }
  }

  // If a thread pool was used, wait for all jobs to finish
  if (thread_pool) {
    thread_pool->wait_all();
  }
}

void blockToRosMsg(const HashedChunkedWaveletOctree::BlockIndex& block_index,
                   const HashedChunkedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap_msgs::HashedWaveletOctreeBlock& msg) {
  ZoneScoped;
  // Define convenience types and constants
  struct StackElement {
    const FloatingPoint scale;
    HashedChunkedWaveletOctreeBlock::ChunkedOctreeType::NodeConstRefType node;
  };

  // Serialize the block's metadata
  msg.root_node_offset.x = block_index.x();
  msg.root_node_offset.y = block_index.y();
  msg.root_node_offset.z = block_index.z();
  // Wavelet scale coefficient of the block's root node
  msg.root_node_scale_coefficient = block.getRootScale();

  // Serialize the block's data (all nodes of its octree)
  std::stack<StackElement> stack;
  stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
  while (!stack.empty()) {
    const FloatingPoint scale = stack.top().scale;
    auto node = stack.top().node;
    stack.pop();

    // Serialize the node's data
    auto& node_msg = msg.nodes.emplace_back();
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());
    node_msg.allocated_children_bitset = 0;

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
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}
}  // namespace wavemap::convert
