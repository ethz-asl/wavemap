#include "wavemap_file_conversions/proto_conversions.h"

namespace wavemap::convert {
void indexToProto(const Index3D& index, proto::Index* index_proto) {
  CHECK_NOTNULL(index_proto);
  index_proto->set_x(index.x());
  index_proto->set_x(index.y());
  index_proto->set_x(index.z());
}

void protoToIndex(const proto::Index& index_proto, Index3D& index) {
  index.x() = index_proto.x();
  index.y() = index_proto.y();
  index.z() = index_proto.z();
}

void detailsToProto(
    const HaarCoefficients<FloatingPoint, 3>::Details& coefficients,
    google::protobuf::RepeatedField<float>* coefficients_proto) {
  CHECK_NOTNULL(coefficients_proto);
  for (int idx = 0;
       idx < HaarCoefficients<FloatingPoint, 3>::kNumDetailCoefficients;
       ++idx) {
    coefficients_proto->Add(coefficients[idx]);
  }
}

void protoToDetails(
    const google::protobuf::RepeatedField<float>& coefficients_proto,
    HaarCoefficients<FloatingPoint, 3>::Details& coefficients) {
  for (int idx = 0;
       idx < HaarCoefficients<FloatingPoint, 3>::kNumDetailCoefficients;
       ++idx) {
    coefficients[idx] = coefficients_proto.Get(idx);
  }
}

bool mapToProto(const VolumetricDataStructureBase& map, proto::Map* map_proto) {
  if (const auto* wavelet_octree = dynamic_cast<const WaveletOctree*>(&map);
      wavelet_octree) {
    convert::mapToProto(*wavelet_octree, map_proto);
    return true;
  }

  if (const auto* hashed_wavelet_octree =
          dynamic_cast<const HashedWaveletOctree*>(&map);
      hashed_wavelet_octree) {
    convert::mapToProto(*hashed_wavelet_octree, map_proto);
    return true;
  }

  if (const auto* hashed_chunked_wavelet_octree =
          dynamic_cast<const HashedChunkedWaveletOctree*>(&map);
      hashed_chunked_wavelet_octree) {
    convert::mapToProto(*hashed_chunked_wavelet_octree, map_proto);
    return true;
  }

  LOG(WARNING) << "Could not serialize requested map to proto. "
                  "Map type not yet supported.";
  return false;
}

void protoToMap(const proto::Map& map_proto,
                VolumetricDataStructureBase::Ptr& map) {
  switch (map_proto.data_case()) {
    case proto::Map::kWaveletOctree: {
      auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map);
      protoToMap(map_proto, wavelet_octree);
      map = wavelet_octree;
      return;
    }
    case proto::Map::kHashedWaveletOctree: {
      auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map);
      protoToMap(map_proto, hashed_wavelet_octree);
      map = hashed_wavelet_octree;
      return;
    }
    case proto::Map::DATA_NOT_SET:
      LOG(WARNING) << "Could not deserialize map proto to a wavemap map. "
                      "Empty map.";
      map = nullptr;
      return;

    default:
      LOG(WARNING) << "Could not deserialize map proto to a wavemap map. "
                      "Unsupported map type.";
      map = nullptr;
      return;
  }
}

void mapToProto(const WaveletOctree& map, proto::Map* map_proto) {
  CHECK_NOTNULL(map_proto);
  map_proto->set_min_cell_width(map.getMinCellWidth());
  map_proto->set_min_log_odds(map.getMinLogOdds());
  map_proto->set_max_log_odds(map.getMaxLogOdds());

  auto* wavelet_octree_proto =
      CHECK_NOTNULL(map_proto->mutable_wavelet_octree());
  wavelet_octree_proto->set_tree_height(map.getTreeHeight());
  wavelet_octree_proto->set_root_node_scale_coefficient(map.getRootScale());

  for (const auto& node :
       map.getNodeIterator<TraversalOrder::kDepthFirstPreorder>()) {
    auto* node_proto = CHECK_NOTNULL(wavelet_octree_proto->add_nodes());
    detailsToProto(node.data(), node_proto->mutable_detail_coefficients());

    uint8_t allocated_children_bitset = 0;
    for (int relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      if (node.hasChild(relative_child_idx)) {
        allocated_children_bitset += (1 << relative_child_idx);
      }
    }
    node_proto->set_allocated_children_bitset(allocated_children_bitset);
  }
}

void protoToMap(const proto::Map& map_proto, WaveletOctree::Ptr& map) {
  if (map_proto.data_case() != proto::Map::kWaveletOctree) {
    return;
  }
  const auto& wavelet_octree_proto = map_proto.wavelet_octree();

  WaveletOctreeConfig config;
  config.min_cell_width = map_proto.min_cell_width();
  config.min_log_odds = map_proto.min_log_odds();
  config.max_log_odds = map_proto.max_log_odds();
  config.tree_height = wavelet_octree_proto.tree_height();
  map = std::make_shared<WaveletOctree>(config);

  map->getRootScale() = wavelet_octree_proto.root_node_scale_coefficient();

  std::stack<WaveletOctree::NodeType*> stack;
  stack.emplace(&map->getRootNode());
  for (const auto& node_proto : wavelet_octree_proto.nodes()) {
    CHECK(!stack.empty());
    WaveletOctree::NodeType* current_node = stack.top();
    CHECK_NOTNULL(current_node);
    stack.pop();

    protoToDetails(node_proto.detail_coefficients(), current_node->data());
    const auto allocated_children_bitset =
        node_proto.allocated_children_bitset();
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }
}

void mapToProto(const HashedWaveletOctree& map, proto::Map* map_proto) {
  CHECK_NOTNULL(map_proto);
  map_proto->set_min_cell_width(map.getMinCellWidth());
  map_proto->set_min_log_odds(map.getMinLogOdds());
  map_proto->set_max_log_odds(map.getMaxLogOdds());

  auto* hashed_wavelet_octree_proto =
      CHECK_NOTNULL(map_proto->mutable_hashed_wavelet_octree());
  hashed_wavelet_octree_proto->set_tree_height(map.getTreeHeight());

  for (const auto& [block_index, block] : map.getBlocks()) {
    auto* block_proto =
        CHECK_NOTNULL(hashed_wavelet_octree_proto->add_blocks());
    indexToProto(block_index, block_proto->mutable_root_node_offset());
    block_proto->set_root_node_scale_coefficient(block.getRootScale());

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

      auto* node_proto = CHECK_NOTNULL(block_proto->add_nodes());
      detailsToProto(node.data(), node_proto->mutable_detail_coefficients());

      uint8_t allocated_children_bitset = 0;
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
          allocated_children_bitset += (1 << relative_child_idx);
        }
      }
      node_proto->set_allocated_children_bitset(allocated_children_bitset);
    }
  }
}

void protoToMap(const proto::Map& map_proto, HashedWaveletOctree::Ptr& map) {
  if (map_proto.data_case() != proto::Map::kHashedWaveletOctree) {
    return;
  }
  const auto& hashed_wavelet_octree_proto = map_proto.hashed_wavelet_octree();

  HashedWaveletOctreeConfig config;
  config.min_cell_width = map_proto.min_cell_width();
  config.min_log_odds = map_proto.min_log_odds();
  config.max_log_odds = map_proto.max_log_odds();
  config.tree_height = hashed_wavelet_octree_proto.tree_height();
  map = std::make_shared<HashedWaveletOctree>(config);

  for (const auto& block_proto : hashed_wavelet_octree_proto.blocks()) {
    Index3D block_index;
    protoToIndex(block_proto.root_node_offset(), block_index);
    CHECK(!block_index.hasNaN()) << block_index;
    auto& block = map->getOrAllocateBlock(block_index);

    block.getRootScale() = block_proto.root_node_scale_coefficient();

    std::stack<WaveletOctree::NodeType*> stack;
    stack.emplace(&block.getRootNode());
    for (const auto& node_proto : block_proto.nodes()) {
      CHECK(!stack.empty());
      WaveletOctree::NodeType* current_node = stack.top();
      CHECK_NOTNULL(current_node);
      stack.pop();

      protoToDetails(node_proto.detail_coefficients(), current_node->data());
      const auto allocated_children_bitset =
          node_proto.allocated_children_bitset();
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists =
            allocated_children_bitset & (1 << relative_child_idx);
        if (child_exists) {
          stack.emplace(current_node->allocateChild(relative_child_idx));
        }
      }
    }
  }
}

void mapToProto(const HashedChunkedWaveletOctree& map, proto::Map* map_proto) {
  CHECK_NOTNULL(map_proto);
  map_proto->set_min_cell_width(map.getMinCellWidth());
  map_proto->set_min_log_odds(map.getMinLogOdds());
  map_proto->set_max_log_odds(map.getMaxLogOdds());

  auto* hashed_wavelet_octree_proto =
      CHECK_NOTNULL(map_proto->mutable_hashed_wavelet_octree());
  hashed_wavelet_octree_proto->set_tree_height(map.getTreeHeight());

  for (const auto& [block_index, block] : map.getBlocks()) {
    auto* block_proto =
        CHECK_NOTNULL(hashed_wavelet_octree_proto->add_blocks());
    indexToProto(block_index, block_proto->mutable_root_node_offset());
    block_proto->set_root_node_scale_coefficient(block.getRootScale());

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

      const MortonIndex morton_code = convert::nodeIndexToMorton(index);
      const int chunk_top_height =
          chunk_height * int_math::div_round_up(index.height, chunk_height);
      const LinearIndex relative_node_index =
          OctreeIndex::computeTreeTraversalDistance(
              morton_code, chunk_top_height, index.height);

      auto* node_proto = CHECK_NOTNULL(block_proto->add_nodes());
      const auto& node_data = chunk.nodeData(relative_node_index);
      detailsToProto(node_data, node_proto->mutable_detail_coefficients());

      if (!chunk.nodeHasAtLeastOneChild(relative_node_index)) {
        continue;
      }

      uint8_t allocated_children_bitset = 0;
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
            allocated_children_bitset += (1 << relative_child_idx);
          }
        } else {
          stack.emplace(StackElement{child_index, chunk, child_scale});
          allocated_children_bitset += (1 << relative_child_idx);
        }
      }
      node_proto->set_allocated_children_bitset(allocated_children_bitset);
    }
  }
}
}  // namespace wavemap::convert
