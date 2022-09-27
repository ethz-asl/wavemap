#ifndef WAVEMAP_3D_ROS_IO_ROS_MSG_CONVERSIONS_H_
#define WAVEMAP_3D_ROS_IO_ROS_MSG_CONVERSIONS_H_

#include <algorithm>
#include <stack>
#include <utility>

#include <wavemap_3d/data_structure/volumetric_octree.h>
#include <wavemap_msgs/Map.h>

namespace wavemap {
template <typename CellT>
wavemap_msgs::Map mapToRosMsg(const VolumetricOctree<CellT>& map) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = "odom";

  auto& octree_msg = map_msg.octree.template emplace_back();
  octree_msg.min_cell_width = map.getMinCellWidth();

  for (const auto& node :
       map.template getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    wavemap_msgs::OctreeNode node_msg;
    node_msg.node_value = node.data();

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    octree_msg.nodes.template emplace_back(node_msg);
  }

  return map_msg;
}

template <typename CellT>
wavemap_msgs::Map mapToRosMsg(const WaveletOctree<CellT>& map) {
  wavemap_msgs::Map map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = "odom";

  auto& wavelet_octree_msg = map_msg.wavelet_octree.template emplace_back();
  wavelet_octree_msg.min_cell_width = map.getMinCellWidth();
  wavelet_octree_msg.root_node_scale_coefficient = map.getRootScale();

  for (const auto& node :
       map.template getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    wavemap_msgs::WaveletOctreeNode node_msg;
    std::copy(node.data().begin(), node.data().end(),
              node_msg.detail_coefficients.begin());

    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    wavelet_octree_msg.nodes.template emplace_back(node_msg);
  }

  return map_msg;
}

template <typename CellT>
void octreeFromRosMsg(const wavemap_msgs::Octree& octree_msg,
                      VolumetricOctree<CellT>& octree) {
  octree.clear();

  std::stack<typename VolumetricOctree<CellT>::NodeType*> stack;
  stack.template emplace(&octree.getRootNode());
  for (const auto& node_msg : octree_msg.nodes) {
    CHECK(!stack.empty());
    const auto current_node = std::move(stack.top());
    stack.pop();

    current_node->data() = node_msg.node_value;
    for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.template emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_ROS_IO_ROS_MSG_CONVERSIONS_H_
