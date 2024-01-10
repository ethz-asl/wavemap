#include "wavemap/utils/query/classified_map.h"

namespace wavemap {
void ClassifiedMap::update(const HashedWaveletOctree& occupancy_map) {
  occupancy_map.forEachBlock(
      [this](const Index3D& block_index, const auto& occupancy_block) {
        auto& classified_block = block_map_.getOrAllocateBlock(block_index);
        recursiveClassifier(occupancy_block.getRootNode(),
                            occupancy_block.getRootScale(),
                            classified_block.getRootNode());
      });
}

bool ClassifiedMap::has(const OctreeIndex& index,
                        Occupancy::Mask occupancy_mask) const {
  // Cache the last block index
  static Index3D block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  static const Block* block = nullptr;

  // Fetch the current block if not already cached
  if (const Index3D new_block_index = block_map_.indexToBlockIndex(index);
      new_block_index != block_index) {
    block_index = new_block_index;
    block = block_map_.getBlock(block_index);
  }

  // If the block doesn't exist, we're done
  if (!block) {
    return false;
  }

  // Otherwise, descend the tree
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const Node* node = &block->getRootNode();
  for (int parent_height = block_map_.getMaxHeight();; --parent_height) {
    if (!node || parent_height <= index.height) {
      // Return the result of OccupancyClassifier::has(region_occupancy,
      // occupancy_mask), which is always true if this branch is reached.
      return true;
    }
    const NdtreeIndexRelativeChild child_idx =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    const auto region_occupancy = childOccupancyMask(*node, child_idx);
    if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask)) {
      return true;
    } else if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
      return false;
    }
    node = node->getChild(child_idx);
  }

  return false;
}

bool ClassifiedMap::isFully(const OctreeIndex& index,
                            Occupancy::Mask occupancy_mask) const {
  // Cache the last block index
  static Index3D block_index =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  static const Block* block = nullptr;

  // Fetch the current block if not already cached
  if (const Index3D new_block_index = block_map_.indexToBlockIndex(index);
      new_block_index != block_index) {
    block_index = new_block_index;
    block = block_map_.getBlock(block_index);
  }

  // If the block doesn't exist, we're done
  if (!block) {
    return false;
  }

  // Otherwise, descend the tree
  const MortonIndex morton_code = convert::nodeIndexToMorton(index);
  const Node* node = &block->getRootNode();
  for (int parent_height = block_map_.getMaxHeight();; --parent_height) {
    if (!node || parent_height <= index.height) {
      // Return the result of OccupancyClassifier::isFully(region_occupancy,
      // occupancy_mask), which is always false if this branch is reached.
      return false;
    }
    const NdtreeIndexRelativeChild child_idx =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    const auto region_occupancy = childOccupancyMask(*node, child_idx);
    if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask)) {
      return true;
    } else if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
      return false;
    }
    node = node->getChild(child_idx);
  }

  return false;
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
        const auto region_occupancy = childOccupancyMask(node, child_idx);
        if (!OccupancyClassifier::has(region_occupancy, occupancy_mask)) {
          continue;
        }
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (OccupancyClassifier::isFully(region_occupancy, occupancy_mask)) {
          visitor_fn(child_node_index);
        } else if (const Node* child_node = node.getChild(child_idx);
                   child_node && termination_height < child_node_index.height) {
          stack.emplace(StackElement{child_node_index, *child_node});
        }
      }
    }
  });
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
