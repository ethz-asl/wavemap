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
