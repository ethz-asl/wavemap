#ifndef WAVEMAP_ROS_CONVERSIONS_IMPL_LAYERED_MAP_MSG_CONVERSIONS_INL_H_
#define WAVEMAP_ROS_CONVERSIONS_IMPL_LAYERED_MAP_MSG_CONVERSIONS_INL_H_

#include <stack>

#include <ros/console.h>
#include <wavemap/core/utils/bits/bit_operations.h>
#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap::convert {
template <typename CellDataT, typename CellDataRosConverterT>
bool mapToRosMsg(const HashedWaveletOctreeT<CellDataT>& map, const std::string& frame_id, const ros::Time& stamp, wavemap_msgs::Map& msg, std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks, std::shared_ptr<ThreadPool> thread_pool) {
  ProfilerZoneScoped;
  // Layered equivalent of the existing MapBase wrapper
  // Fills the common ROS header and stores the typed map inside wavemap_msgs::Map.
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  // The custom voxel type needs the user-provided converter
  auto& layered_map_msg = msg.layered_hashed_wavelet_octree.emplace_back();
  convert::mapToRosMsg<CellDataT, CellDataRosConverterT>(map, layered_map_msg, std::move(include_blocks), std::move(thread_pool));
  return true;
}

template <typename CellDataT, typename CellDataRosConverterT>
void mapToRosMsg(const HashedWaveletOctreeT<CellDataT>& map, wavemap_msgs::LayeredHashedWaveletOctree& msg, std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks, std::shared_ptr<ThreadPool> thread_pool) {
  ProfilerZoneScoped;
  // Use a small margin when deciding whether child nodes are saturated
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  // Copy the map metadata first
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();
  msg.layer_names = CellDataRosConverterT::layerNames();
  msg.layer_types = CellDataRosConverterT::layerTypes();
  msg.layer_min_values =
      CellDataRosConverterT::makeLayerMinimums(map.getThresholdConfig());
  msg.layer_max_values =
      CellDataRosConverterT::makeLayerMaximums(map.getThresholdConfig());
  msg.layer_visualizations =
      CellDataRosConverterT::layerVisualizations();

  // Always publish the full list of allocated block indices
  msg.allocated_block_indices.reserve(map.getHashMap().size());
  map.forEachBlock([&msg](const Index3D& block_index, const auto& /*block*/) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  });

  // If the caller requested a subset of blocks, discard indices that are not currently allocated
  if (include_blocks) {
    for (auto include_block_it = include_blocks->begin();
         include_block_it != include_blocks->end();) {
      if (map.hasBlock(*include_block_it)) {
        ++include_block_it;
      } else {
        include_block_it = include_blocks->erase(include_block_it);
      }
    }
  } else {
    include_blocks.emplace();
    map.forEachBlock(
        [&include_blocks](const Index3D& block_index, const auto& /*block*/) {
          include_blocks->emplace(block_index);
        });
  }

  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    if (const auto* block = map.getBlock(block_index); block) {
      auto& block_msg = msg.blocks[block_idx++];
      // Block conversion is independent per block,
      // so it can safely run in the optional thread pool when the caller has one available.
      if (thread_pool) {
        thread_pool->add_task([block_index, block, min_log_odds, max_log_odds,
                               &block_msg]() {
          blockToRosMsg<CellDataT, CellDataRosConverterT>(
              block_index, *block, min_log_odds, max_log_odds, block_msg);
        });
      } else {
        blockToRosMsg<CellDataT, CellDataRosConverterT>(
            block_index, *block, min_log_odds, max_log_odds, block_msg);
      }
    } else {
      ROS_ERROR("Block index not found. This should never happen.");
    }
  }
  if (thread_pool) {
    thread_pool->wait_all();
  }
}

template <typename CellDataT, typename CellDataRosConverterT>
void blockToRosMsg(
    const typename HashedWaveletOctreeT<CellDataT>::BlockIndex& block_index,
    const typename HashedWaveletOctreeT<CellDataT>::Block& block,
    FloatingPoint min_log_odds, FloatingPoint max_log_odds,
    wavemap_msgs::LayeredHashedWaveletOctreeBlock& msg) {
  using Block = typename HashedWaveletOctreeT<CellDataT>::Block;
  using NodeConstRefType = typename Block::OctreeType::NodeConstRefType;

  struct StackElement {
    const CellDataT scale;
    NodeConstRefType node;
  };

  // Store the block position and root scale separately from the internal wavelet nodes
  // Occupancy is kept in the explicit occupancy field, while extra layers are encoded through the user-provided converter
  msg.root_node_offset.x = block_index.x();
  msg.root_node_offset.y = block_index.y();
  msg.root_node_offset.z = block_index.z();
  msg.root_node_occupancy_scale_coefficient = static_cast<FloatingPoint>(block.getRootScale());
  msg.root_node_layers = CellDataRosConverterT::makeLayers();
  CellDataRosConverterT::appendLayerValues(block.getRootScale(), msg.root_node_layers);

  // Traverse the block's wavelet tree without recursion
  // Each stack entry carries the node and its reconstructed scale coefficient, to decide whether the children are saturated and should be serialized
  std::stack<StackElement> stack;
  stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
  while (!stack.empty()) {
    const CellDataT scale = stack.top().scale;
    const auto& node = stack.top().node;
    stack.pop();

    auto& node_msg = msg.nodes.emplace_back();
    node_msg.detail_layers = CellDataRosConverterT::makeLayers();
    size_t coefficient_idx = 0u;
    // Each wavelet node stores eight detail coefficients.
    // Occupancy is copied into the fixed array; all custom layer values are appended layer-major into detail_layers by CellDataRosConverterT
    for (const CellDataT& coefficient : node.data()) {
      node_msg.occupancy_detail_coefficients[coefficient_idx++] =
          static_cast<FloatingPoint>(coefficient);
      CellDataRosConverterT::appendLayerValues(coefficient,
                                               node_msg.detail_layers);
    }

    // Reconstruct child scales from the parent scale and detail coefficients
    // Only children whose occupancy is not saturated need to be sent further
    const auto child_scales = Block::Transform::backward({scale, node.data()});
    for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const auto child_scale = child_scales[relative_child_idx];
      const FloatingPoint child_occupancy =
          static_cast<FloatingPoint>(child_scale);
      if (child_occupancy < min_log_odds || max_log_odds < child_occupancy) {
        continue;
      }

      if (const auto* child = node.getChild(relative_child_idx); child) {
        stack.emplace(StackElement{child_scale, *child});
        // The bitset records which children have serialized node data.
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}

template <typename CellDataT, typename CellDataRosConverterT>
bool mapToRosMsg(const HashedChunkedWaveletOctreeT<CellDataT>& map, const std::string& frame_id, const ros::Time& stamp, wavemap_msgs::Map& msg, std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks, std::shared_ptr<ThreadPool> thread_pool) {
  ProfilerZoneScoped;
  // Layered equivalent of the existing MapBase wrapper
  // Fills the common ROS header and stores the typed map inside wavemap_msgs::Map.
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  // The custom voxel type needs the user-provided converter
  auto& layered_map_msg = msg.layered_hashed_wavelet_octree.emplace_back();
  convert::mapToRosMsg<CellDataT, CellDataRosConverterT>(map, layered_map_msg, std::move(include_blocks), std::move(thread_pool));
  return true;
}

template <typename CellDataT, typename CellDataRosConverterT>
void mapToRosMsg(const HashedChunkedWaveletOctreeT<CellDataT>& map, wavemap_msgs::LayeredHashedWaveletOctree& msg, std::optional<std::unordered_set<Index3D, Index3DHash>> include_blocks, std::shared_ptr<ThreadPool> thread_pool) {
  ProfilerZoneScoped;
  // Use a small margin when deciding whether child nodes are saturated
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  const auto min_log_odds = map.getMinLogOdds() + kNumericalNoise;
  const auto max_log_odds = map.getMaxLogOdds() - kNumericalNoise;

  // Copy the map metadata first
  msg.min_cell_width = map.getMinCellWidth();
  msg.min_log_odds = map.getMinLogOdds();
  msg.max_log_odds = map.getMaxLogOdds();
  msg.tree_height = map.getTreeHeight();
  msg.layer_names = CellDataRosConverterT::layerNames();
  msg.layer_types = CellDataRosConverterT::layerTypes();
  msg.layer_min_values =
      CellDataRosConverterT::makeLayerMinimums(map.getThresholdConfig());
  msg.layer_max_values =
      CellDataRosConverterT::makeLayerMaximums(map.getThresholdConfig());
  msg.layer_visualizations =
      CellDataRosConverterT::layerVisualizations();

  // Always publish the full list of allocated block indices
  msg.allocated_block_indices.reserve(map.getHashMap().size());
  map.forEachBlock([&msg](const Index3D& block_index, const auto& /*block*/) {
    auto& block_index_msg = msg.allocated_block_indices.emplace_back();
    block_index_msg.x = block_index.x();
    block_index_msg.y = block_index.y();
    block_index_msg.z = block_index.z();
  });

  // If the caller requested a subset of blocks, discard indices that are not currently allocated
  if (include_blocks) {
    for (auto include_block_it = include_blocks->begin();
         include_block_it != include_blocks->end();) {
      if (map.hasBlock(*include_block_it)) {
        ++include_block_it;
      } else {
        include_block_it = include_blocks->erase(include_block_it);
      }
    }
  } else {
    include_blocks.emplace();
    map.forEachBlock(
        [&include_blocks](const Index3D& block_index, const auto& /*block*/) {
          include_blocks->emplace(block_index);
        });
  }

  int block_idx = 0;
  msg.blocks.resize(include_blocks->size());
  for (const auto& block_index : include_blocks.value()) {
    if (const auto* block = map.getBlock(block_index); block) {
      auto& block_msg = msg.blocks[block_idx++];
      // Block conversion is independent per block,
      // so it can safely run in the optional thread pool when the caller has one available.
      if (thread_pool) {
        thread_pool->add_task([block_index, block, min_log_odds, max_log_odds,
                               &block_msg]() {
          blockToRosMsg<CellDataT, CellDataRosConverterT>(
              block_index, *block, min_log_odds, max_log_odds, block_msg);
        });
      } else {
        blockToRosMsg<CellDataT, CellDataRosConverterT>(
            block_index, *block, min_log_odds, max_log_odds, block_msg);
      }
    } else {
      ROS_ERROR("Block index not found. This should never happen.");
    }
  }
  if (thread_pool) {
    thread_pool->wait_all();
  }
}

template <typename CellDataT, typename CellDataRosConverterT>
void blockToRosMsg(
    const typename HashedChunkedWaveletOctreeT<CellDataT>::BlockIndex& block_index,
    const typename HashedChunkedWaveletOctreeT<CellDataT>::Block& block,
    FloatingPoint min_log_odds, FloatingPoint max_log_odds,
    wavemap_msgs::LayeredHashedWaveletOctreeBlock& msg) {
  using Block = typename HashedChunkedWaveletOctreeT<CellDataT>::Block;
  using NodeConstRefType = typename Block::OctreeType::NodeConstRefType;

  struct StackElement {
    const CellDataT scale;
    NodeConstRefType node;
  };

  // Store the block position and root scale separately from the internal wavelet nodes
  // Occupancy is kept in the explicit occupancy field, while extra layers are encoded through the user-provided converter
  msg.root_node_offset.x = block_index.x();
  msg.root_node_offset.y = block_index.y();
  msg.root_node_offset.z = block_index.z();
  msg.root_node_occupancy_scale_coefficient = static_cast<FloatingPoint>(block.getRootScale());
  msg.root_node_layers = CellDataRosConverterT::makeLayers();
  CellDataRosConverterT::appendLayerValues(block.getRootScale(), msg.root_node_layers);

  // Traverse the block's wavelet tree without recursion
  // Each stack entry carries the node and its reconstructed scale coefficient, to decide whether the children are saturated and should be serialized
  std::stack<StackElement> stack;
  stack.emplace(StackElement{block.getRootScale(), block.getRootNode()});
  while (!stack.empty()) {
    const CellDataT scale = stack.top().scale;
    const auto node = stack.top().node;
    stack.pop();

    auto& node_msg = msg.nodes.emplace_back();
    node_msg.detail_layers = CellDataRosConverterT::makeLayers();
    size_t coefficient_idx = 0u;
    // Each wavelet node stores eight detail coefficients.
    // Occupancy is copied into the fixed array; all custom layer values are appended layer-major into detail_layers by CellDataRosConverterT
    for (const CellDataT& coefficient : node.data()) {
      node_msg.occupancy_detail_coefficients[coefficient_idx++] =
          static_cast<FloatingPoint>(coefficient);
      CellDataRosConverterT::appendLayerValues(coefficient,
                                               node_msg.detail_layers);
    }

    // Reconstruct child scales from the parent scale and detail coefficients
    // Only children whose occupancy is not saturated need to be sent further
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
        // The bitset records which children have serialized node data.
        node_msg.allocated_children_bitset += (1 << relative_child_idx);
      }
    }
  }
}

template <typename CellDataT, typename CellDataRosConverterT>
void rosMsgToMap(const wavemap_msgs::LayeredHashedWaveletOctree& msg,
                 typename HashedWaveletOctreeT<CellDataT>::Ptr& map) {
  ProfilerZoneScoped;
  // Rebuild the wavemap config from the message metadata
  // If the destination map is null or incompatible, allocate a fresh layered map instance.
  HashedWaveletOctreeConfig config;
  config.min_cell_width = msg.min_cell_width;
  config.min_log_odds = msg.min_log_odds;
  config.max_log_odds = msg.max_log_odds;
  config.tree_height = msg.tree_height;

  if (!map || map->getConfig() != config) {
    map = std::make_shared<HashedWaveletOctreeT<CellDataT>>(config);
  }

  // Synchronize allocated blocks with the message
  // Blocks not listed in allocated_block_indices are removed from the destination map.
  std::unordered_set<Index3D, Index3DHash> allocated_blocks;
  for (const auto& block_index : msg.allocated_block_indices) {
    allocated_blocks.emplace(block_index.x, block_index.y, block_index.z);
  }
  map->eraseBlockIf(
      [&allocated_blocks](const Index3D& block_index, const auto& /*block*/) {
        return !allocated_blocks.count(block_index);
      });

  using Block = typename HashedWaveletOctreeT<CellDataT>::Block;
  using NodeType = typename Block::OctreeType::NodeType;

  for (const auto& block_msg : msg.blocks) {
    // Allocate or fetch the target block and reconstruct its root voxel data from occupancy plus the serialized custom layers
    const Index3D block_index{block_msg.root_node_offset.x,
                              block_msg.root_node_offset.y,
                              block_msg.root_node_offset.z};
    auto& block = map->getOrAllocateBlock(block_index);
    block.getRootScale() = CellDataRosConverterT::readCellData(
        block_msg.root_node_occupancy_scale_coefficient,
        block_msg.root_node_layers, 0u);

    std::stack<NodeType*> stack;
    stack.emplace(&block.getRootNode());
    for (const auto& node_msg : block_msg.nodes) {
      DCHECK(!stack.empty());
      NodeType* node = stack.top();
      stack.pop();

      // Reconstruct each detail coefficient
      // The converter receives the fixed occupancy coefficient and the layer-major custom data for the same coefficient index.
      for (size_t coefficient_idx = 0u; coefficient_idx < node->data().size();
           ++coefficient_idx) {
        node->data()[coefficient_idx] = CellDataRosConverterT::readCellData(
            node_msg.occupancy_detail_coefficients[coefficient_idx],
            node_msg.detail_layers, coefficient_idx);
      }

      // Recreate only the children marked in the transmitted bitset
      // Their node contents will be filled by subsequent node messages in stack traversal order.
      for (int relative_child_idx = OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists = bit_ops::is_bit_set(
            node_msg.allocated_children_bitset, relative_child_idx);
        if (child_exists) {
          stack.emplace(&node->getOrAllocateChild(relative_child_idx));
        }
      }
    }
  }
}

template <typename CellDataT, typename CellDataRosConverterT>
bool rosMsgToMap(const wavemap_msgs::Map& msg, typename HashedWaveletOctreeT<CellDataT>::Ptr& map) {
  ProfilerZoneScoped;
  // The typed layered wrapper accepts exactly one layered map and no legacy map payloads. This keeps the old generic MapBase path unchanged while avoiding ambiguous custom conversions.
  const bool has_one_layered_map = msg.layered_hashed_wavelet_octree.size() == 1u;
  const bool has_legacy_payload = !msg.hashed_blocks.empty() || !msg.wavelet_octree.empty() || !msg.hashed_wavelet_octree.empty();
  if (!has_one_layered_map || has_legacy_payload) {
    ROS_WARN(
        "Layered map ROS msg must contain exactly one layered hashed wavelet "
        "octree and no legacy map payloads. Ignoring.");
    map = nullptr;
    return false;
  }

  convert::rosMsgToMap<CellDataT, CellDataRosConverterT>(msg.layered_hashed_wavelet_octree.front(), map);
  return static_cast<bool>(map);
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_IMPL_LAYERED_MAP_MSG_CONVERSIONS_INL_H_
