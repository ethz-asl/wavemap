#include "wavemap/utils/query/classified_map.h"

#include <tracy/Tracy.hpp>

namespace wavemap {
void ClassifiedMap::update(const HashedWaveletOctree& occupancy_map) {
  ZoneScoped;
  query_cache_.reset();
  occupancy_map.forEachBlock(
      [this](const Index3D& block_index, const auto& occupancy_block) {
        auto& classified_block = block_map_.getOrAllocateBlock(block_index);
        recursiveClassifier(occupancy_block.getRootNode(),
                            occupancy_block.getRootScale(),
                            classified_block.getRootNode());
      });
}

std::pair<std::optional<Occupancy::Mask>, ClassifiedMap::HeightType>
ClassifiedMap::getValueOrAncestor(const OctreeIndex& index) const {
  const OctreeIndex parent_index = index.computeParentIndex();
  const auto [ancestor, ancestor_height] =
      query_cache_.getNodeOrAncestor(parent_index, block_map_);
  if (ancestor) {
    const MortonIndex morton = convert::nodeIndexToMorton(index);
    const NdtreeIndexRelativeChild relative_child_idx =
        OctreeIndex::computeRelativeChildIndex(morton, ancestor_height);
    return {ancestor->data().childOccupancyMask(relative_child_idx),
            ancestor_height - 1};
  }
  return {std::nullopt, getTreeHeight()};
}

void ClassifiedMap::forEachLeafMatching(
    Occupancy::Mask occupancy_mask,
    std::function<void(const OctreeIndex&)> visitor_fn,
    IndexElement termination_height) const {
  block_map_.forEachBlock([occupancy_mask, termination_height, &visitor_fn](
                              const Index3D& block_index, const Block& block) {
    struct StackElement {
      const OctreeIndex node_index;
      const Node& node;
    };
    std::stack<StackElement> stack;
    stack.emplace(StackElement{OctreeIndex{block.getMaxHeight(), block_index},
                               block.getRootNode()});
    while (!stack.empty()) {
      const OctreeIndex node_index = stack.top().node_index;
      const Node& node = stack.top().node;
      stack.pop();

      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < OctreeIndex::kNumChildren; ++child_idx) {
        const auto region_occupancy = node.data().childOccupancyMask(child_idx);
        if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
          continue;
        }
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask) ||
            child_node_index.height <= termination_height) {
          visitor_fn(child_node_index);
        } else if (const Node* child_node = node.getChild(child_idx);
                   child_node) {
          stack.emplace(StackElement{child_node_index, *child_node});
        }
      }
    }
  });
}

std::pair<const ClassifiedMap::Node*, ClassifiedMap::HeightType>
ClassifiedMap::QueryCache::getNodeOrAncestor(
    const OctreeIndex& index, const ClassifiedMap::BlockHashMap& block_map) {
  // Remember previous query indices and compute new ones
  const IndexElement previous_height = height;
  const MortonIndex previous_morton_code = morton_code;
  morton_code = convert::nodeIndexToMorton(index);

  // Fetch the block if needed and return null if it doesn't exist
  if (!getBlock(block_map.indexToBlockIndex(index), block_map)) {
    return {nullptr, tree_height};
  }

  // Compute the last ancestor the current and previous query had in common
  if (height != tree_height) {
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code, index.height, previous_morton_code, previous_height);
    height = last_common_ancestor;
  }
  DCHECK_LE(height, tree_height);

  if (height == index.height) {
    DCHECK_NOTNULL(node_stack[height]);
    return {node_stack[height], height};
  }

  // Walk down the tree from height to index.height
  for (; index.height < height;) {
    DCHECK_NOTNULL(node_stack[height]);
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, height);
    // Check if the child is allocated
    const Node* child = node_stack[height]->getChild(child_index);
    if (!child) {
      return {node_stack[height], height};
    }
    node_stack[--height] = child;
  }

  return {node_stack[height], height};
}

bool ClassifiedMap::QueryCache::has(const OctreeIndex& index,
                                    Occupancy::Mask occupancy_mask,
                                    const BlockHashMap& block_map) {
  // Remember previous query indices and compute new ones
  const IndexElement previous_height = height;
  const MortonIndex previous_morton_code = morton_code;
  morton_code = convert::nodeIndexToMorton(index);

  // Fetch the block if needed and return false if it doesn't exist
  if (!getBlock(block_map.indexToBlockIndex(index), block_map)) {
    return false;
  }

  // Compute the last ancestor the current and previous query had in common
  if (height != tree_height) {
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code, index.height, previous_morton_code, previous_height);
    height = last_common_ancestor;
  }
  DCHECK_LE(height, tree_height);

  if (height == index.height) {
    DCHECK_NOTNULL(node_stack[height]);
    const auto region_occupancy = node_stack[height]->data().occupancyMask();
    return OccupancyClassifier::has(region_occupancy, occupancy_mask);
  }

  // Walk down the tree from height to index.height
  while (true) {
    DCHECK_NOTNULL(node_stack[height]);
    const Node* parent_node = node_stack[height];
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, height);
    const auto region_occupancy =
        parent_node->data().childOccupancyMask(child_index);
    if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask)) {
      return true;
    } else if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
      return false;
    }
    // Check if the child is allocated
    if (height - 1 == index.height || !parent_node->hasChild(child_index)) {
      // Return the result of OccupancyClassifier::has(region_occupancy,
      // occupancy_mask), which is always true if this branch is reached.
      return true;
    }
    node_stack[--height] = parent_node->getChild(child_index);
  }
}

bool ClassifiedMap::QueryCache::isFully(const OctreeIndex& index,
                                        Occupancy::Mask occupancy_mask,
                                        const BlockHashMap& block_map) {
  // Remember previous query indices and compute new ones
  const IndexElement previous_height = height;
  const MortonIndex previous_morton_code = morton_code;
  morton_code = convert::nodeIndexToMorton(index);

  // Fetch the block if needed and return false if it doesn't exist
  if (!getBlock(block_map.indexToBlockIndex(index), block_map)) {
    return false;
  }

  // Compute the last ancestor the current and previous query had in common
  if (height != tree_height) {
    auto last_common_ancestor = OctreeIndex::computeLastCommonAncestorHeight(
        morton_code, index.height, previous_morton_code, previous_height);
    height = last_common_ancestor;
  }
  DCHECK_LE(height, tree_height);

  if (height == index.height) {
    DCHECK_NOTNULL(node_stack[height]);
    const auto region_occupancy = node_stack[height]->data().occupancyMask();
    return OccupancyClassifier::isFully(region_occupancy, occupancy_mask);
  }

  // Walk down the tree from height to index.height
  while (true) {
    DCHECK_NOTNULL(node_stack[height]);
    const Node* parent_node = node_stack[height];
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, height);
    const auto region_occupancy =
        parent_node->data().childOccupancyMask(child_index);
    if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask)) {
      return true;
    } else if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
      return false;
    }
    // Check if the child is allocated
    if (height - 1 == index.height || !parent_node->hasChild(child_index)) {
      // Return the result of OccupancyClassifier::isFully(region_occupancy,
      // occupancy_mask), which is always false if this branch is reached.
      return false;
    }
    node_stack[--height] = parent_node->getChild(child_index);
  }
}

void ClassifiedMap::QueryCache::reset() {
  block_index = Index3D::Constant(std::numeric_limits<IndexElement>::max());
  height = tree_height;
  morton_code = std::numeric_limits<MortonIndex>::max();

  block = nullptr;
  node_stack = std::array<const Node*, morton::kMaxTreeHeight<3>>{};
}

void ClassifiedMap::recursiveClassifier(  // NOLINT
    const HashedWaveletOctreeBlock::NodeType& occupancy_node,
    FloatingPoint average_occupancy, ClassifiedMap::Node& classified_node) {
  const auto child_occupancies =
      HashedWaveletOctree::Block::Transform::backward(
          {average_occupancy, occupancy_node.data()});
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const FloatingPoint child_occupancy = child_occupancies[child_idx];
    // If the node has children, recurse
    if (occupancy_node.hasChild(child_idx)) {
      const auto* occupancy_child_node = occupancy_node.getChild(child_idx);
      auto& classified_child_node =
          classified_node.getOrAllocateChild(child_idx);
      recursiveClassifier(*occupancy_child_node, child_occupancy,
                          classified_child_node);
      const bool child_has_free = classified_child_node.data().has_free.any();
      const bool child_has_occupied =
          classified_child_node.data().has_occupied.any();
      const bool child_has_unobserved =
          classified_child_node.data().has_unobserved.any();
      classified_node.data().has_free.set(child_idx, child_has_free);
      classified_node.data().has_occupied.set(child_idx, child_has_occupied);
      classified_node.data().has_unobserved.set(child_idx,
                                                child_has_unobserved);
      if (child_has_free + child_has_occupied + child_has_unobserved == 1) {
        classified_node.eraseChild(child_idx);
      }
    } else {  // Otherwise, the node is a leaf
      classified_node.data().has_free.set(
          child_idx, classifier_.is(child_occupancy, Occupancy::kFree));
      classified_node.data().has_occupied.set(
          child_idx, classifier_.is(child_occupancy, Occupancy::kOccupied));
      classified_node.data().has_unobserved.set(
          child_idx, classifier_.is(child_occupancy, Occupancy::kUnobserved));
    }
  }
}
}  // namespace wavemap
