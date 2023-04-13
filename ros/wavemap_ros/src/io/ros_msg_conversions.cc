#include "wavemap_ros/io/ros_msg_conversions.h"

namespace wavemap {
wavemap_msgs::Map mapToRosMsg(const VolumetricOctree& map,
                              const std::string& frame_id) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = frame_id;

  auto& octree_msg = map_msg.octree.emplace_back();
  octree_msg.min_cell_width = map.getMinCellWidth();

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    wavemap_msgs::OctreeNode node_msg;
    node_msg.node_value = node.data();

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    octree_msg.nodes.emplace_back(node_msg);
  }

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const WaveletOctree& map,
                              const std::string& frame_id) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = frame_id;

  auto& wavelet_octree_msg = map_msg.wavelet_octree.emplace_back();
  wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
  wavelet_octree_msg.root_node_scale_coefficient = map.getRootScale();

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    wavemap_msgs::WaveletOctreeNode node_msg;
    std::copy(node.data().cbegin(), node.data().cend(),
              node_msg.detail_coefficients.begin());

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    wavelet_octree_msg.nodes.emplace_back(node_msg);
  }

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const HashedWaveletOctree& map,
                              const std::string& frame_id,
                              FloatingPoint ignore_blocks_older_than) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
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
    const auto min_log_odds = map.getConfig().min_log_odds + kNumericalNoise;
    const auto max_log_odds = map.getConfig().max_log_odds - kNumericalNoise;

    struct StackElement {
      const FloatingPoint scale;
      const HashedWaveletOctree::NodeType& node;
    };
    std::stack<StackElement> stack;
    stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});

    while (!stack.empty()) {
      const FloatingPoint scale = stack.top().scale;
      const auto& node = stack.top().node;
      stack.pop();

      wavemap_msgs::WaveletOctreeNode node_msg;
      std::copy(node.data().cbegin(), node.data().cend(),
                node_msg.detail_coefficients.begin());
      node_msg.allocated_children_bitset = 0;

      const auto child_scales =
          HashedWaveletOctree::Transform::backward({scale, node.data()});
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
      wavelet_octree_msg.nodes.emplace_back(node_msg);
    }
  }

  return map_msg;
}

wavemap_msgs::Map mapToRosMsg(const HashedChunkedWaveletOctree& map,
                              const std::string& frame_id,
                              FloatingPoint ignore_blocks_older_than) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
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
    const auto min_log_odds = map.getConfig().min_log_odds + kNumericalNoise;
    const auto max_log_odds = map.getConfig().max_log_odds - kNumericalNoise;
    const auto tree_height = map.getTreeHeight();
    const auto chunk_height = map.getChunkHeight();

    struct StackElement {
      const OctreeIndex node_index;
      const HashedChunkedWaveletOctree::NodeChunkType& chunk;
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
      const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
          morton_code, chunk_top_height, index.height);

      wavemap_msgs::WaveletOctreeNode node_msg;
      std::copy(chunk.data(value_index).cbegin(),
                chunk.data(value_index).cend(),
                node_msg.detail_coefficients.begin());
      node_msg.allocated_children_bitset = 0;

      const HashedWaveletOctree::Coefficients::CoefficientsArray child_scales =
          HashedWaveletOctree::Transform::backward(
              {scale, {chunk.data(value_index)}});

      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const OctreeIndex child_index =
            index.computeChildIndex(relative_child_idx);
        const FloatingPoint child_scale = child_scales[relative_child_idx];
        if (child_scale < min_log_odds || max_log_odds < child_scale) {
          continue;
        }

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
      wavelet_octree_msg.nodes.emplace_back(node_msg);
    }
  }

  return map_msg;
}
}  // namespace wavemap
