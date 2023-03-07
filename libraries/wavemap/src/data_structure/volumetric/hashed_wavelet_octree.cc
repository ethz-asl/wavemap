#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"

#include <unordered_set>

namespace wavemap {
size_t HashedWaveletOctree::size() const {
  size_t size = 0u;
  for (const auto& [block_index, block] : blocks_) {
    size += block.size();
  }
  return size;
}

void HashedWaveletOctree::prune() {
  std::unordered_set<BlockIndex, VoxbloxIndexHash<3>> blocks_to_remove;
  for (auto& [block_index, block] : blocks_) {
    block.prune();
    if (block.empty()) {
      blocks_to_remove.emplace(block_index);
    }
  }
  for (const auto& index : blocks_to_remove) {
    blocks_.erase(index);
  }
}

size_t HashedWaveletOctree::getMemoryUsage() const {
  // TODO(victorr): Also include the memory usage of the unordered map itself
  size_t memory_usage = 0u;
  for (const auto& [block_index, block] : blocks_) {
    memory_usage += block.getMemoryUsage();
  }
  return memory_usage;
}

Index3D HashedWaveletOctree::getMinIndex() const {
  if (!empty()) {
    Index3D min_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::max());
    for (const auto& [block_index, block] : blocks_) {
      min_block_index = min_block_index.cwiseMin(block_index);
    }
    return kCellsPerBlockSide * min_block_index;
  }
  return Index3D::Zero();
}

Index3D HashedWaveletOctree::getMaxIndex() const {
  if (!empty()) {
    Index3D max_block_index =
        Index3D::Constant(std::numeric_limits<IndexElement>::lowest());
    for (const auto& [block_index, block] : blocks_) {
      max_block_index = max_block_index.cwiseMax(block_index);
    }
    return kCellsPerBlockSide * (max_block_index + Index3D::Ones());
  }
  return Index3D::Zero();
}

FloatingPoint HashedWaveletOctree::getCellValue(const Index3D& index) const {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (blocks_.count(block_index)) {
    const auto& block = blocks_.at(block_index);
    const CellIndex cell_index =
        computeCellIndexFromBlockIndexAndIndex(block_index, index);
    return block.getCellValue(cell_index);
  }
  return 0.f;
}

void HashedWaveletOctree::setCellValue(const Index3D& index,
                                       FloatingPoint new_value) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (!blocks_.count(block_index)) {
    blocks_.try_emplace(block_index, this);
  }
  auto& block = blocks_.at(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  block.setCellValue(cell_index, new_value);
}

void HashedWaveletOctree::addToCellValue(const Index3D& index,
                                         FloatingPoint update) {
  const BlockIndex block_index = computeBlockIndexFromIndex(index);
  if (!blocks_.count(block_index)) {
    blocks_.try_emplace(block_index, this);
  }
  auto& block = blocks_.at(block_index);
  const CellIndex cell_index =
      computeCellIndexFromBlockIndexAndIndex(block_index, index);
  block.addToCellValue(cell_index, update);
}

void HashedWaveletOctree::forEachLeaf(
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  for (const auto& [block_index, block] : blocks_) {
    block.forEachLeaf(block_index, visitor_fn);
  }
}

HashedWaveletOctree::CellIndex
HashedWaveletOctree::computeCellIndexFromBlockIndexAndIndex(
    const HashedWaveletOctree::BlockIndex& block_index, const Index3D& index) {
  const Index3D origin = kCellsPerBlockSide * block_index;
  const Index3D cell_index = index - origin;
  CHECK((0 <= cell_index.array()).all());
  return convert::indexAndHeightToNodeIndex(cell_index, 0);
}

void HashedWaveletOctree::Block::prune() {
  std::function<Coefficients::Scale(NodeType&, Coefficients::Scale)>
      recursive_fn = [&recursive_fn, this](
                         NodeType& node,
                         Coefficients::Scale scale_coefficient) {
        Coefficients::CoefficientsArray child_scale_coefficients =
            Transform::backward({scale_coefficient, node.data()});

        bool has_at_least_one_child = false;
        for (NdtreeIndexRelativeChild child_idx = 0;
             child_idx < OctreeIndex::kNumChildren; ++child_idx) {
          if (node.hasChild(child_idx)) {
            NodeType& child_node = *node.getChild(child_idx);
            child_scale_coefficients[child_idx] =
                recursive_fn(child_node, child_scale_coefficients[child_idx]);
            if (!child_node.hasChildrenArray() &&
                std::all_of(child_node.data().cbegin(),
                            child_node.data().cend(), [](auto coefficient) {
                              return std::abs(coefficient) < 1e-3f;
                            })) {
              node.deleteChild(child_idx);
            } else {
              has_at_least_one_child = true;
            }
          } else {
            child_scale_coefficients[child_idx] -=
                parent_->clamp(child_scale_coefficients[child_idx]);
          }
        }
        if (!has_at_least_one_child) {
          node.deleteChildrenArray();
        }

        const auto [scale_update, detail_updates] =
            Transform::forward(child_scale_coefficients);
        node.data() -= detail_updates;

        return scale_update;
      };

  root_scale_coefficient_ -=
      recursive_fn(ndtree_.getRootNode(), root_scale_coefficient_);
}

FloatingPoint HashedWaveletOctree::Block::getCellValue(
    const OctreeIndex& index) const {
  const MortonCode morton_code = index.computeMortonCode();
  const NodeType* node = &ndtree_.getRootNode();
  FloatingPoint value = root_scale_coefficient_;
  for (int parent_height = kTreeHeight; index.height < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    value = Transform::backwardSingleChild({value, node->data()}, child_index);
    if (!node->hasChild(child_index)) {
      break;
    }
    node = node->getChild(child_index);
  }
  return value;
}

void HashedWaveletOctree::Block::setCellValue(const OctreeIndex& index,
                                              FloatingPoint new_value) {
  const MortonCode morton_code = index.computeMortonCode();
  std::vector<NodeType*> node_ptrs;
  const int height_difference = kTreeHeight - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  FloatingPoint current_value = root_scale_coefficient_;
  for (int parent_height = kTreeHeight; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    current_value = Transform::backwardSingleChild(
        {current_value, current_parent->data()}, child_index);
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{new_value - current_value, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_node = node_ptrs.back();
    node_ptrs.pop_back();
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }

  root_scale_coefficient_ += coefficients.scale;
}

void HashedWaveletOctree::Block::addToCellValue(const OctreeIndex& index,
                                                FloatingPoint update) {
  const MortonCode morton_code = index.computeMortonCode();

  std::vector<NodeType*> node_ptrs;
  const int height_difference = kTreeHeight - index.height;
  node_ptrs.reserve(height_difference);
  node_ptrs.emplace_back(&ndtree_.getRootNode());
  for (int parent_height = kTreeHeight; index.height + 1 < parent_height;
       --parent_height) {
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    NodeType* current_parent = node_ptrs.back();
    if (!current_parent->hasChild(child_index)) {
      current_parent->allocateChild(child_index);
    }
    node_ptrs.emplace_back(current_parent->getChild(child_index));
  }
  DCHECK_EQ(node_ptrs.size(), height_difference);

  Coefficients::Parent coefficients{update, {}};
  for (int parent_height = index.height + 1; parent_height <= kTreeHeight;
       ++parent_height) {
    NodeType* current_node = node_ptrs.back();
    node_ptrs.pop_back();
    const NdtreeIndexRelativeChild child_index =
        OctreeIndex::computeRelativeChildIndex(morton_code, parent_height);
    coefficients =
        Transform::forwardSingleChild(coefficients.scale, child_index);
    current_node->data() += coefficients.details;
  }
  root_scale_coefficient_ += coefficients.scale;
}

void HashedWaveletOctree::Block::forEachLeaf(
    const BlockIndex& block_index,
    VolumetricDataStructureBase::IndexedLeafVisitorFunction visitor_fn) const {
  if (empty()) {
    return;
  }

  std::stack<StackElement> stack;
  stack.emplace(StackElement{OctreeIndex{kTreeHeight, block_index},
                             ndtree_.getRootNode(), root_scale_coefficient_});
  while (!stack.empty()) {
    const OctreeIndex node_index = stack.top().node_index;
    const NodeType& node = stack.top().node;
    const FloatingPoint node_scale_coefficient = stack.top().scale_coefficient;
    stack.pop();

    const Coefficients::CoefficientsArray child_scale_coefficients =
        Transform::backward({node_scale_coefficient, {node.data()}});
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];
      if (node.hasChild(child_idx)) {
        const NodeType& child_node = *node.getChild(child_idx);
        stack.emplace(StackElement{child_node_index, child_node,
                                   child_scale_coefficient});
      } else {
        visitor_fn(child_node_index, child_scale_coefficient);
      }
    }
  }
}
}  // namespace wavemap
