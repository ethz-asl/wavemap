#include "wavemap_ros_conversions/map_msg_conversions.h"

namespace wavemap::convert {
bool mapToRosMsg(const VolumetricDataStructureBase& map,
                 const std::string& frame_id, const ros::Time& stamp,
                 wavemap_msgs::Map& msg) {
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

  LOG(WARNING) << "Could not serialize requested map to ROS msg. "
                  "Map type not yet supported.";
  return false;
}

bool rosMsgToMap(const wavemap_msgs::Map& msg,
                 VolumetricDataStructureBase::Ptr& map) {
  // Check validity
  if ((msg.wavelet_octree.size() == 1) !=
      (msg.hashed_wavelet_octree.size() != 1)) {
    LOG(WARNING)
        << "Maps must be serialized either as wavelet octrees or hashed "
           "wavelet octrees. Encountered message contains both. Ignoring.";
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

  LOG(WARNING) << "Conversion of the requested map ROS msg to a wavemap map is "
                  "not yet supported.";
  map = nullptr;
  return false;
}

void mapToRosMsg(const WaveletOctree& map, wavemap_msgs::WaveletOctree& msg) {
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();
  msg.root_node_scale_coefficient = map.getRootScale();

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    auto& node_msg = msg.nodes.emplace_back();
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());
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

  map->getRootScale() = msg.root_node_scale_coefficient;

  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  for (const auto& node_msg : msg.nodes) {
    CHECK(!stack.empty());
    WaveletOctree::NodeType* node = stack.top();
    stack.pop();

    std::copy(node_msg.detail_coefficients.cbegin(),
              node_msg.detail_coefficients.cend(), node->data().begin());
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

void mapToRosMsg(const HashedWaveletOctree& map,
                 wavemap_msgs::HashedWaveletOctree& msg,
                 const std::optional<std::unordered_set<Index3D, Index3DHash>>&
                     include_blocks) {
  struct StackElement {
    const FloatingPoint scale;
    const HashedWaveletOctreeBlock::NodeType& node;
  };

  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();

  for (const auto& [block_index, block] : map.getBlocks()) {
    if (include_blocks.has_value() && !include_blocks->count(block_index)) {
      continue;
    }

    auto& block_msg = msg.blocks.emplace_back();
    block_msg.root_node_offset = {block_index.x(), block_index.y(),
                                  block_index.z()};
    block_msg.root_node_scale_coefficient = block.getRootScale();

    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});

    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      auto& node_msg = block_msg.nodes.emplace_back();
      std::copy(node.data().cbegin(), node.data().cend(),
                node_msg.detail_coefficients.begin());
      node_msg.allocated_children_bitset = 0;

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
          node_msg.allocated_children_bitset += (1 << relative_child_idx);
        }
      }
    }
  }
}

void rosMsgToMap(const wavemap_msgs::HashedWaveletOctree& msg,
                 HashedWaveletOctree::Ptr& map) {
  HashedWaveletOctreeConfig config;
  config.min_cell_width = msg.min_cell_width;
  config.min_log_odds = msg.min_log_odds;
  config.max_log_odds = msg.max_log_odds;
  config.tree_height = msg.tree_height;
  // Check if the map already exists and has compatible settings
  if (!map || map->getConfig() != config) {
    // Otherwise create a new one
    map = std::make_shared<HashedWaveletOctree>(config);
  }

  for (const auto& block_msg : msg.blocks) {
    const Index3D block_index{block_msg.root_node_offset[0],
                              block_msg.root_node_offset[1],
                              block_msg.root_node_offset[2]};
    auto& block = map->getOrAllocateBlock(block_index);

    block.getRootScale() = block_msg.root_node_scale_coefficient;

    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    for (const auto& node_msg : block_msg.nodes) {
      CHECK(!stack.empty());
      WaveletOctree::NodeType* node = stack.top();
      stack.pop();

      std::copy(node_msg.detail_coefficients.cbegin(),
                node_msg.detail_coefficients.cend(), node->data().begin());
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

void mapToRosMsg(const HashedChunkedWaveletOctree& map,
                 wavemap_msgs::HashedWaveletOctree& msg,
                 const std::optional<std::unordered_set<Index3D, Index3DHash>>&
                     include_blocks) {
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

  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();

  for (const auto& [block_index, block] : map.getBlocks()) {
    if (include_blocks.has_value() && !include_blocks->count(block_index)) {
      continue;
    }

    auto& block_msg = msg.blocks.emplace_back();
    block_msg.root_node_offset = {block_index.x(), block_index.y(),
                                  block_index.z()};
    block_msg.root_node_scale_coefficient = block.getRootScale();

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

      auto& node_msg = block_msg.nodes.emplace_back();
      const auto& node_data = chunk.nodeData(relative_node_index);
      std::copy(node_data.cbegin(), node_data.cend(),
                node_msg.detail_coefficients.begin());
      node_msg.allocated_children_bitset = 0;

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
            node_msg.allocated_children_bitset += (1 << relative_child_idx);
          }
        } else {
          stack.emplace(StackElement{child_index, chunk, child_scale});
          node_msg.allocated_children_bitset += (1 << relative_child_idx);
        }
      }
    }
  }
}
}  // namespace wavemap::convert
