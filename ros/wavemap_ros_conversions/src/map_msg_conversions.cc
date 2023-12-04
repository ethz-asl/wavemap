#include "wavemap_ros_conversions/map_msg_conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <tracy/Tracy.hpp>

namespace wavemap::convert {
bool mapToRosMsg(const VolumetricDataStructureBase& map,
                 const std::string& frame_id, const rclcpp::Time& stamp,
                 wavemap_msgs::msg::Map& msg) {
  // Write the msg header
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  // Write the map data
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

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "Could not serialize requested map to ROS msg. "
      "Map type not yet supported.");
  return false;
}

bool rosMsgToMap(const wavemap_msgs::msg::Map& msg,
                 VolumetricDataStructureBase::Ptr& map) {
  ZoneScoped;
  // Check validity
  if ((msg.wavelet_octree.size() == 1) !=
      (msg.hashed_wavelet_octree.size() != 1)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
        "Maps must be serialized either as wavelet octrees or hashed "
        "wavelet octrees. Encountered message contains both. Ignoring.");
    map = nullptr;
    return false;
  }

  // Read the data
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

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "Conversion of the requested map ROS msg to a wavemap map is "
      "not yet supported.");
  map = nullptr;
  return false;
}

void mapToRosMsg(const WaveletOctree& map, wavemap_msgs::msg::WaveletOctree& msg) {
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

void rosMsgToMap(const wavemap_msgs::msg::WaveletOctree& msg,
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
        stack.emplace(node->allocateChild(relative_child_idx));
      }
    }
  }
}

void mapToRosMsg(
    const HashedWaveletOctree& map, wavemap_msgs::msg::HashedWaveletOctree& msg,
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
  msg.allocated_block_indices.reserve(map.getBlocks().size());
  for (const auto& [block_index, _] : map.getBlocks()) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  }

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
    for (const auto& [block_index, block] : map.getBlocks()) {
      include_blocks->emplace(block_index);
    }
  }

  // Serialize the specified blocks
  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    const auto& block = map.getBlock(block_index);
    auto& block_msg = msg.blocks[block_idx++];
    // If a thread pool was provided, use it
    if (thread_pool) {
      thread_pool->add_task([&]() {
        blockToRosMsg(block_index, block, min_log_odds, max_log_odds,
                      block_msg);
      });
    } else {  // Otherwise, use the current thread
      blockToRosMsg(block_index, block, min_log_odds, max_log_odds, block_msg);
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
                   wavemap_msgs::msg::HashedWaveletOctreeBlock& msg) {
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

void rosMsgToMap(const wavemap_msgs::msg::HashedWaveletOctree& msg,
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
    for (auto it = map->getBlocks().begin(); it != map->getBlocks().end();) {
      const auto block_index = it->first;
      if (!allocated_blocks.count(block_index)) {
        it = map->getBlocks().erase(it);
      } else {
        ++it;
      }
    }
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
          stack.emplace(node->allocateChild(relative_child_idx));
        }
      }
    }
  }
}

void mapToRosMsg(
    const HashedChunkedWaveletOctree& map,
    wavemap_msgs::msg::HashedWaveletOctree& msg,
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
  msg.allocated_block_indices.reserve(map.getBlocks().size());
  for (const auto& [block_index, _] : map.getBlocks()) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  }

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
    for (const auto& [block_index, block] : map.getBlocks()) {
      include_blocks->emplace(block_index);
    }
  }

  // Serialize the specified blocks
  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    const auto& block = map.getBlock(block_index);
    auto& block_msg = msg.blocks[block_idx++];
    // If a thread pool was provided, use it
    if (thread_pool) {
      thread_pool->add_task([&]() {
        blockToRosMsg(block_index, block, min_log_odds, max_log_odds,
                      tree_height, block_msg);
      });
    } else {  // Otherwise, use the current thread
      blockToRosMsg(block_index, block, min_log_odds, max_log_odds, tree_height,
                    block_msg);
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
                   IndexElement tree_height,
                   wavemap_msgs::msg::HashedWaveletOctreeBlock& msg) {
  ZoneScoped;
  // Define convenience types and constants
  struct StackElement {
    const OctreeIndex node_index;
    const HashedChunkedWaveletOctreeBlock::NodeChunkType& chunk;
    const FloatingPoint scale_coefficient;
  };
  constexpr IndexElement chunk_height =
      HashedChunkedWaveletOctreeBlock::kChunkHeight;

  // Serialize the block's metadata
  msg.root_node_offset.x = block_index.x();
  msg.root_node_offset.y = block_index.y();
  msg.root_node_offset.z = block_index.z();
  // Wavelet scale coefficient of the block's root node
  msg.root_node_scale_coefficient = block.getRootScale();

  // Serialize the block's data (all nodes of its octree)
  std::stack<StackElement> stack;
  stack.emplace(StackElement{
      {tree_height, block_index}, block.getRootChunk(), block.getRootScale()});
  while (!stack.empty()) {
    const OctreeIndex index = stack.top().node_index;
    const FloatingPoint scale = stack.top().scale_coefficient;
    const auto& chunk = stack.top().chunk;
    stack.pop();

    // Compute the node's index w.r.t. the data chunk that contains it
    const MortonIndex morton_code = convert::nodeIndexToMorton(index);
    const int chunk_top_height =
        chunk_height * int_math::div_round_up(index.height, chunk_height);
    const LinearIndex relative_node_index =
        OctreeIndex::computeTreeTraversalDistance(morton_code, chunk_top_height,
                                                  index.height);

    // Serialize the node's data
    auto& node_msg = msg.nodes.emplace_back();
    const auto& node_data = chunk.nodeData(relative_node_index);
    std::copy(node_data.cbegin(), node_data.cend(),
              node_msg.detail_coefficients.begin());
    node_msg.allocated_children_bitset = 0;

    // If the node has no children, continue
    if (!chunk.nodeHasAtLeastOneChild(relative_node_index)) {
      continue;
    }

    // Otherwise, evaluate which of its children should be serialized
    const auto child_scales =
        HashedWaveletOctreeBlock::Transform::backward({scale, node_data});
    // NOTE: We iterate and add nodes to the stack in decreasing order s.t.
    //       the nodes are popped from the stack in increasing order.
    for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      // If the child is saturated, we don't need to store its descendants
      const FloatingPoint child_scale = child_scales[relative_child_idx];
      if (child_scale < min_log_odds || max_log_odds < child_scale) {
        continue;
      }

      // Check if the child is no longer in the current chunk
      const OctreeIndex child_index =
          index.computeChildIndex(relative_child_idx);
      if (child_index.height % chunk_height == 0) {
        // If so, check if the chunk exists
        const MortonIndex child_morton =
            convert::nodeIndexToMorton(child_index);
        const LinearIndex linear_child_index =
            OctreeIndex::computeLevelTraversalDistance(
                child_morton, chunk_top_height, child_index.height);
        if (chunk.hasChild(linear_child_index)) {
          const auto& child_chunk = *chunk.getChild(linear_child_index);
          // Indicate that the child will be serialized
          // and add it to the stack
          stack.emplace(StackElement{child_index, child_chunk, child_scale});
          node_msg.allocated_children_bitset += (1 << relative_child_idx);
        }
      } else {
        // Indicate that the child will be serialized and add it to the stack
        stack.emplace(StackElement{child_index, chunk, child_scale});
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}
}  // namespace wavemap::convert
