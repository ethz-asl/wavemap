#include "wavemap/core/utils/query/classified_map.h"

#include <limits>
#include <stack>
#include <utility>
#include <vector>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
ClassifiedMap::ClassifiedMap(FloatingPoint min_cell_width,
                             IndexElement tree_height,
                             const OccupancyClassifier& classifier)
    : tree_height_(tree_height),
      min_cell_width_(min_cell_width),
      classifier_(classifier) {}

ClassifiedMap::ClassifiedMap(const HashedWaveletOctree& occupancy_map,
                             const OccupancyClassifier& classifier)
    : ClassifiedMap(occupancy_map.getMinCellWidth(),
                    occupancy_map.getTreeHeight(), classifier) {
  update(occupancy_map);
}

ClassifiedMap::ClassifiedMap(const HashedWaveletOctree& occupancy_map,
                             const OccupancyClassifier& classifier,
                             const HashedBlocks& esdf_map,
                             FloatingPoint robot_radius)
    : ClassifiedMap(occupancy_map.getMinCellWidth(),
                    occupancy_map.getTreeHeight(), classifier) {
  update(occupancy_map, esdf_map, robot_radius);
}

Index3D ClassifiedMap::getMinIndex() const {
  return cells_per_block_side_ * getMinBlockIndex();
}

Index3D ClassifiedMap::getMaxIndex() const {
  if (empty()) {
    return Index3D::Zero();
  }
  return cells_per_block_side_ * (getMaxBlockIndex().array() + 1) - 1;
}

void ClassifiedMap::update(const HashedWaveletOctree& occupancy_map) {
  ProfilerZoneScoped;
  // Reset the query cache
  query_cache_.reset();

  // Erase blocks that no longer exist
  block_map_.eraseBlockIf(
      [&occupancy_map](const Index3D& block_index, const auto& /*block*/) {
        return !occupancy_map.hasBlock(block_index);
      });

  // Update all existing blocks
  occupancy_map.forEachBlock(
      [this](const Index3D& block_index, const auto& occupancy_block) {
        auto& classified_block = block_map_.getOrAllocateBlock(block_index);
        recursiveClassifier(occupancy_block.getRootNode(),
                            occupancy_block.getRootScale(),
                            classified_block.getRootNode());
      });
}

void ClassifiedMap::update(const HashedWaveletOctree& occupancy_map,
                           const HashedBlocks& esdf_map,
                           FloatingPoint robot_radius) {
  ProfilerZoneScoped;
  // Reset the query cache
  query_cache_.reset();

  // Check that the ESDF is valid for our query and compatible with the occ map
  CHECK_GE(esdf_map.getDefaultValue(), robot_radius);
  CHECK_GE(esdf_map.getMaxLogOdds(), robot_radius);
  CHECK_NEAR(esdf_map.getMinCellWidth(), occupancy_map.getMinCellWidth(),
             kEpsilon);

  // Erase blocks that no longer exist
  block_map_.eraseBlockIf(
      [&occupancy_map](const Index3D& block_index, const auto& /*block*/) {
        return !occupancy_map.hasBlock(block_index);
      });

  // Update all existing blocks
  QueryAccelerator esdf_accelerator{
      dynamic_cast<const HashedBlocks::DenseBlockHash&>(esdf_map)};
  occupancy_map.forEachBlock(
      [this, &esdf_accelerator, block_height = occupancy_map.getTreeHeight(),
       robot_radius](const Index3D& block_index, const auto& occupancy_block) {
        const OctreeIndex block_node_index{block_height, block_index};
        auto& classified_block = block_map_.getOrAllocateBlock(block_index);
        recursiveClassifier(block_node_index, &occupancy_block.getRootNode(),
                            occupancy_block.getRootScale(), esdf_accelerator,
                            robot_radius, classified_block.getRootNode());
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

void ClassifiedMap::forEachLeaf(IndexedLeafVisitorFunction visitor_fn,
                                IndexElement termination_height) const {
  forEachBlock([&visitor_fn, termination_height](const Index3D& block_index,
                                                 const Block& block) {
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
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (node.hasChild(child_idx) &&
            termination_height < child_node_index.height) {
          const Node& child_node = *node.getChild(child_idx);
          stack.emplace(StackElement{child_node_index, child_node});
        } else {
          const Occupancy::Mask child_occupancy =
              node.data().childOccupancyMask(child_idx);
          std::invoke(visitor_fn, child_node_index, child_occupancy);
        }
      }
    }
  });
}

void ClassifiedMap::forEachLeafMatching(Occupancy::Mask occupancy_mask,
                                        IndexedLeafVisitorFunction visitor_fn,
                                        IndexElement termination_height) const {
  block_map_.forEachBlock([occupancy_mask, termination_height, &visitor_fn](
                              const Index3D& block_index, const Block& block) {
    struct StackElement {
      const OctreeIndex node_index;
      const Node& node;
    };
    std::stack<StackElement, std::vector<StackElement>> stack;
    stack.emplace(StackElement{OctreeIndex{block.getMaxHeight(), block_index},
                               block.getRootNode()});
    while (!stack.empty()) {
      const OctreeIndex node_index = stack.top().node_index;
      const Node& node = stack.top().node;
      stack.pop();

      for (NdtreeIndexRelativeChild child_idx = 0;
           child_idx < OctreeIndex::kNumChildren; ++child_idx) {
        const auto child_occupancy = node.data().childOccupancyMask(child_idx);
        if (!OccupancyClassifier::has(child_occupancy, occupancy_mask)) {
          continue;
        }
        const OctreeIndex child_node_index =
            node_index.computeChildIndex(child_idx);
        if (OccupancyClassifier::isFully(child_occupancy, occupancy_mask) ||
            child_node_index.height <= termination_height) {
          std::invoke(visitor_fn, child_node_index, child_occupancy);
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
      HaarTransform::backward({average_occupancy, occupancy_node.data()});
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
      const bool is_free = classifier_.is(child_occupancy, Occupancy::kFree);
      const bool is_occupied =
          classifier_.is(child_occupancy, Occupancy::kOccupied);
      const bool is_unobserved =
          classifier_.is(child_occupancy, Occupancy::kUnobserved);
      classified_node.data().has_free.set(child_idx, is_free);
      classified_node.data().has_occupied.set(child_idx, is_occupied);
      classified_node.data().has_unobserved.set(child_idx, is_unobserved);
    }
  }
}

void ClassifiedMap::recursiveClassifier(  // NOLINT
    const OctreeIndex& node_index,
    const HashedWaveletOctreeBlock::NodeType* occupancy_node,
    FloatingPoint occupancy_average,
    QueryAccelerator<HashedBlocks::DenseBlockHash>& esdf_map,
    FloatingPoint robot_radius, ClassifiedMap::Node& classified_node) {
  // Compute the child occupancies, if appropriate
  ChildAverages child_occupancies{};
  if (occupancy_node) {
    child_occupancies =
        HaarTransform::backward({occupancy_average, occupancy_node->data()});
  }

  // Iterate over all children
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    // Get the child's index, occupancy node pointer and occupancy
    const OctreeIndex child_index = node_index.computeChildIndex(child_idx);
    const auto* child_occupancy_node =
        occupancy_node ? occupancy_node->getChild(child_idx) : nullptr;
    const FloatingPoint child_occupancy =
        occupancy_node ? child_occupancies[child_idx] : occupancy_average;

    // If the ESDF resolution has been reached,
    // or we're in a region that's fully occupied or unobserved, classify
    const bool esdf_resolution_reached = child_index.height == 0;
    const bool is_free = classifier_.is(child_occupancy, Occupancy::kFree);
    const bool is_non_free_leaf = !is_free && !child_occupancy_node;
    if (esdf_resolution_reached || is_non_free_leaf) {
      // If the child's occupancy is free, check if it's also free in the ESDF
      if (is_free) {
        DCHECK_EQ(child_index.height, 0);
        if (auto* distance = esdf_map.getValue(child_index.position);
            distance && *distance < robot_radius) {
          // The child is not free in the ESDF (robot would be in collision)
          classified_node.data().has_free.set(child_idx, false);
          classified_node.data().has_occupied.set(child_idx, true);
          classified_node.data().has_unobserved.set(child_idx, false);
          continue;
        }
        // NOTE: The ESDF is defined up to ESDF.max_distance which we asserted
        //       to be larger than the robot_radius. So if a cell's ESDF
        //       distance is undefined, it must be safe to traverse.
      }
      // Otherwise, the classification result only depends on the occupancy
      const bool is_occupied =
          classifier_.is(child_occupancy, Occupancy::kOccupied);
      const bool is_unobserved =
          classifier_.is(child_occupancy, Occupancy::kUnobserved);
      classified_node.data().has_free.set(child_idx, is_free);
      classified_node.data().has_occupied.set(child_idx, is_occupied);
      classified_node.data().has_unobserved.set(child_idx, is_unobserved);
      continue;
    }

    // Otherwise, recursively evaluate the child's descendants
    auto& classified_child_node = classified_node.getOrAllocateChild(child_idx);
    recursiveClassifier(child_index, child_occupancy_node, child_occupancy,
                        esdf_map, robot_radius, classified_child_node);
    // Collect the results
    const bool child_has_free = classified_child_node.data().has_free.any();
    const bool child_has_occupied =
        classified_child_node.data().has_occupied.any();
    const bool child_has_unobserved =
        classified_child_node.data().has_unobserved.any();
    // Store the results
    classified_node.data().has_free.set(child_idx, child_has_free);
    classified_node.data().has_occupied.set(child_idx, child_has_occupied);
    classified_node.data().has_unobserved.set(child_idx, child_has_unobserved);
    // Prune away homogeneous children
    if (child_has_free + child_has_occupied + child_has_unobserved == 1) {
      classified_node.eraseChild(child_idx);
    }
  }
}
}  // namespace wavemap
