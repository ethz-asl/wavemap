#include "wavemap_ros_conversions/map_msg_conversions.h"

namespace wavemap::convert {
wavemap_msgs::Map mapToRosMsg(const VolumetricDataStructureBase::ConstPtr& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than) {
  if (const auto octree =
          std::dynamic_pointer_cast<const VolumetricOctree>(map);
      octree) {
    return convert::mapToRosMsg(*octree, frame_id, stamp);
  }

  if (const auto wavelet_octree =
          std::dynamic_pointer_cast<const WaveletOctree>(map);
      wavelet_octree) {
    return convert::mapToRosMsg(*wavelet_octree, frame_id, stamp);
  }

  if (const auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<const HashedWaveletOctree>(map);
      hashed_wavelet_octree) {
    return convert::mapToRosMsg(*hashed_wavelet_octree, frame_id, stamp,
                                ignore_blocks_older_than);
  }

  if (const auto hashed_chunked_wavelet_octree =
          std::dynamic_pointer_cast<const HashedChunkedWaveletOctree>(map);
      hashed_chunked_wavelet_octree) {
    return convert::mapToRosMsg(*hashed_chunked_wavelet_octree, frame_id, stamp,
                                ignore_blocks_older_than);
  }

  LOG(WARNING) << "Conversion of the requested map type to ROS msgs is not yet "
                  "supported.";
  return {};
}

wavemap_msgs::Map mapToRosMsg(const VolumetricOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = frame_id;

  auto& octree_msg = map_msg.octree.emplace_back();
  octree_msg.min_cell_width = map.getMinCellWidth();

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    auto& node_msg = octree_msg.nodes.emplace_back();
    node_msg.node_value = node.data();

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const WaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = frame_id;

  auto& wavelet_octree_msg = map_msg.wavelet_octree.emplace_back();
  wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
  wavelet_octree_msg.root_node_scale_coefficient = map.getRootScale();

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    auto& node_msg = wavelet_octree_msg.nodes.emplace_back();
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const HashedWaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = frame_id;

  for (const auto& [block_index, block] : map.getBlocks()) {
    if (0.f < ignore_blocks_older_than &&
        ignore_blocks_older_than < block.getTimeSinceLastUpdated()) {
      continue;
    }

    auto& wavelet_octree_msg = map_msg.wavelet_octree.emplace_back();
    wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.x());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.y());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.z());
    wavelet_octree_msg.root_node_scale_coefficient = block.getRootScale();

    constexpr FloatingPoint kNumericalNoise = 1e-3f;
    const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
    const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

    struct StackElement {
      const FloatingPoint scale;
      const HashedWaveletOctreeBlock::NodeType& node;
    };
    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});

    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      auto& node_msg = wavelet_octree_msg.nodes.emplace_back();
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

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const HashedChunkedWaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = frame_id;

  for (const auto& [block_index, block] : map.getBlocks()) {
    if (0.f < ignore_blocks_older_than &&
        ignore_blocks_older_than < block.getTimeSinceLastUpdated()) {
      continue;
    }

    auto& wavelet_octree_msg = map_msg.wavelet_octree.emplace_back();
    wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.x());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.y());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.z());
    wavelet_octree_msg.root_node_scale_coefficient = block.getRootScale();

    constexpr FloatingPoint kNumericalNoise = 1e-3f;
    const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
    const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;
    const auto tree_height = map.getTreeHeight();
    const auto chunk_height = map.getChunkHeight();

    struct StackElement {
      const OctreeIndex node_index;
      const HashedChunkedWaveletOctreeBlock::NodeChunkType& chunk;
      const FloatingPoint scale_coefficient;
    };
    std::stack<StackElement> stack;
    stack.emplace(StackElement{{tree_height, block_index},
                               block.getRootChunk(),
                               block.getRootScale()});

    while (!stack.empty()) {
      const OctreeIndex index = stack.top().node_index;
      const FloatingPoint scale = stack.top().scale_coefficient;
      const auto& chunk = stack.top().chunk;
      stack.pop();

      const MortonCode morton_code = convert::nodeIndexToMorton(index);
      const int chunk_top_height =
          chunk_height * int_math::div_round_up(index.height, chunk_height);
      const LinearIndex relative_node_index =
          OctreeIndex::computeTreeTraversalDistance(
              morton_code, chunk_top_height, index.height);

      auto& node_msg = wavelet_octree_msg.nodes.emplace_back();
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
          const MortonCode child_morton =
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

  return map_msg;
}

void rosMsgToMap(const wavemap_msgs::Map& map_msg,
                 VolumetricDataStructureBase::Ptr& map) {
  if (!map_msg.wavelet_octree.empty()) {
    if (map_msg.wavelet_octree.size() == 1) {
      auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map);
      rosMsgToMap(map_msg, wavelet_octree);
      map = wavelet_octree;
      return;
    } else {
      auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map);
      rosMsgToMap(map_msg, hashed_wavelet_octree);
      map = hashed_wavelet_octree;
      return;
    }
  }

  if (!map_msg.octree.empty()) {
    auto octree = std::dynamic_pointer_cast<VolumetricOctree>(map);
    rosMsgToMap(map_msg, octree);
    map = octree;
    return;
  }

  LOG(WARNING) << "Conversion of the requested map ROS msg to a wavemap map is "
                  "not yet supported.";
  map = nullptr;
}

void rosMsgToMap(const wavemap_msgs::Map& map_msg, VolumetricOctree::Ptr& map) {
  if (map) {
    map->clear();
  } else {
    VolumetricOctreeConfig config;
    config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
    map = std::make_shared<VolumetricOctree>(config);
  }

  const auto& octree_msg = map_msg.octree.front();
  std::stack<VolumetricOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  for (const auto& node_msg : octree_msg.nodes) {
    CHECK(!stack.empty());
    VolumetricOctree::NodeType* current_node = stack.top();
    CHECK_NOTNULL(current_node);
    stack.pop();

    current_node->data() = node_msg.node_value;
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }
}

void rosMsgToMap(const wavemap_msgs::Map& map_msg, WaveletOctree::Ptr& map) {
  if (map) {
    map->clear();
  } else {
    WaveletOctreeConfig config;
    config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
    map = std::make_shared<WaveletOctree>(config);
  }

  const auto& wavelet_octree_msg = map_msg.wavelet_octree.front();
  map->getRootScale() = wavelet_octree_msg.root_node_scale_coefficient;

  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  for (const auto& node_msg : wavelet_octree_msg.nodes) {
    CHECK(!stack.empty());
    WaveletOctree::NodeType* current_node = stack.top();
    CHECK_NOTNULL(current_node);
    stack.pop();

    std::copy(node_msg.detail_coefficients.cbegin(),
              node_msg.detail_coefficients.cend(),
              current_node->data().begin());
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }
}

void rosMsgToMap(const wavemap_msgs::Map& map_msg,
                 HashedWaveletOctree::Ptr& map) {
  if (!map) {
    HashedWaveletOctreeConfig config;
    config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
    map = std::make_shared<HashedWaveletOctree>(config);
  }

  for (const auto& block_msg : map_msg.wavelet_octree) {
    const Index3D block_index{block_msg.root_node_offset[0],
                              block_msg.root_node_offset[1],
                              block_msg.root_node_offset[2]};
    CHECK(!block_index.hasNaN()) << block_index;
    auto& block = map->getOrAllocateBlock(block_index);

    block.getRootScale() = block_msg.root_node_scale_coefficient;

    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    for (const auto& node_msg : block_msg.nodes) {
      CHECK(!stack.empty());
      WaveletOctree::NodeType* current_node = stack.top();
      CHECK_NOTNULL(current_node);
      stack.pop();

      std::copy(node_msg.detail_coefficients.cbegin(),
                node_msg.detail_coefficients.cend(),
                current_node->data().begin());
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists =
            node_msg.allocated_children_bitset & (1 << relative_child_idx);
        if (child_exists) {
          stack.emplace(current_node->allocateChild(relative_child_idx));
        }
      }
    }
  }
}
}  // namespace wavemap::convert
