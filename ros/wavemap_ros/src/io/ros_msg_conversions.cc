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
                              const std::string& frame_id) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = frame_id;

  for (const auto& [block_index, block] : map.getBlocks()) {
    auto& wavelet_octree_msg = map_msg.wavelet_octree.emplace_back();
    wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.x());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.y());
    wavelet_octree_msg.root_node_offset.emplace_back(block_index.z());
    wavelet_octree_msg.root_node_scale_coefficient = block.getRootScale();

    for (const auto& node :
         block.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
      wavemap_msgs::WaveletOctreeNode node_msg;
      std::copy(node.data().cbegin(), node.data().cend(),
                node_msg.detail_coefficients.begin());

      for (int relative_child_idx = 0;
           relative_child_idx < OctreeIndex::kNumChildren;
           ++relative_child_idx) {
        if (node.hasChild(relative_child_idx)) {
          node_msg.allocated_children_bitset += (1 << relative_child_idx);
        }
      }
      wavelet_octree_msg.nodes.emplace_back(node_msg);
    }
  }

  return map_msg;
}

void octreeFromRosMsg(const wavemap_msgs::Octree& octree_msg,
                      VolumetricOctree& octree) {
  octree.clear();

  std::stack<typename VolumetricOctree::NodeType*> stack;
  stack.emplace(&octree.getRootNode());
  for (const auto& node_msg : octree_msg.nodes) {
    CHECK(!stack.empty());
    const auto current_node = stack.top();
    stack.pop();

    current_node->data() = node_msg.node_value;
    for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }
}
}  // namespace wavemap
