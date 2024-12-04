#ifndef WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_

#include <array>
#include <memory>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"

namespace wavemap {
template <typename DataT, int dim>
class NdtreeNode {
 public:
  using DataType = DataT;
  static constexpr int kNumChildren = NdtreeIndex<dim>::kNumChildren;

  NdtreeNode() = default;
  ~NdtreeNode() = default;

  template <typename... Args>
  explicit NdtreeNode(Args&&... args) : data_(std::forward<Args>(args)...) {}

  // Delete copy constructor and assignment operator to avoid accidental copies
  NdtreeNode(const NdtreeNode& other_tree) = delete;
  NdtreeNode& operator=(const NdtreeNode&) = delete;

  // Allow move construction and assignments
  NdtreeNode(NdtreeNode&&) = default;
  NdtreeNode& operator=(NdtreeNode&&) = default;

  bool empty() const;
  void clear();

  size_t getMemoryUsage() const;

  bool hasNonzeroData() const;
  bool hasNonzeroData(FloatingPoint threshold) const;
  DataT& data() { return data_; }
  const DataT& data() const { return data_; }

  bool hasChildrenArray() const { return static_cast<bool>(children_); }
  bool hasAtLeastOneChild() const;
  void deleteChildrenArray() { children_.reset(); }

  bool hasChild(NdtreeIndexRelativeChild child_index) const;
  bool eraseChild(NdtreeIndexRelativeChild child_index);

  NdtreeNode* getChild(NdtreeIndexRelativeChild child_index);
  const NdtreeNode* getChild(NdtreeIndexRelativeChild child_index) const;
  template <typename... DefaultArgs>
  NdtreeNode& getOrAllocateChild(NdtreeIndexRelativeChild child_index,
                                 DefaultArgs&&... args);

  friend bool operator==(const NdtreeNode& lhs, const NdtreeNode& rhs) {
    return &rhs == &lhs;
  }

 private:
  using ChildPtr = std::unique_ptr<NdtreeNode>;
  using ChildrenArray = std::array<ChildPtr, kNumChildren>;

  DataT data_{};
  std::unique_ptr<ChildrenArray> children_;
};
}  // namespace wavemap

#include "wavemap/core/data_structure/ndtree/impl/ndtree_node_inl.h"

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_NDTREE_NDTREE_NODE_H_
