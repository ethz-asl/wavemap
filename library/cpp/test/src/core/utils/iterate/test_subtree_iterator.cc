#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/data_structure/ndtree/ndtree_node.h"
#include "wavemap/core/utils/iterate/subtree_iterator.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
class SubtreeIteratorTest : public FixtureBase {
 protected:
  using NodeType = NdtreeNode<int, 2>;

  static std::unique_ptr<NodeType> demoTree() {
    auto root_node = std::make_unique<NodeType>(1);
    auto& child_0 = root_node->getOrAllocateChild(0, 2);
    auto& child_1 = root_node->getOrAllocateChild(1, 3);

    child_0.getOrAllocateChild(0, 4);
    auto& child_01 = child_0.getOrAllocateChild(1, 5);
    child_0.getOrAllocateChild(2, 6);
    auto& child_11 = child_1.getOrAllocateChild(1, 7);
    auto& child_13 = child_1.getOrAllocateChild(3, 8);

    child_01.getOrAllocateChild(2, 9);
    child_01.getOrAllocateChild(3, 10);
    child_11.getOrAllocateChild(0, 11);
    child_11.getOrAllocateChild(1, 12);
    child_11.getOrAllocateChild(2, 13);
    child_11.getOrAllocateChild(3, 14);
    child_13.getOrAllocateChild(2, 15);

    return root_node;
  }
};

TEST_F(SubtreeIteratorTest, DepthFirstPreorderTraversal) {
  std::unique_ptr<const NodeType> root_node = demoTree();
  const std::vector<int> expected_nodes{1, 2,  4,  5,  9,  10, 6, 3,
                                        7, 11, 12, 13, 14, 8,  15};
  auto subtree_iterator =
      Subtree<const NodeType, TraversalOrder::kDepthFirstPreorder>(
          root_node.get());

  // Test iteration order and completeness
  int idx = 0;
  for (const NodeType& node : subtree_iterator) {
    EXPECT_EQ(node.data(), expected_nodes[idx++]);
  }
  EXPECT_EQ(idx, expected_nodes.size());

  // Test compatibility with std::distance (e.g. to count the number of nodes)
  EXPECT_EQ(std::distance(subtree_iterator.begin(), subtree_iterator.end()),
            expected_nodes.size());
}

TEST_F(SubtreeIteratorTest, DepthFirstPostorderTraversal) {
  std::unique_ptr<const NodeType> root_node = demoTree();
  const std::vector<int> expected_nodes{4,  9,  10, 5,  6, 2, 11, 12,
                                        13, 14, 7,  15, 8, 3, 1};
  auto subtree_iterator =
      Subtree<const NodeType, TraversalOrder::kDepthFirstPostorder>(
          root_node.get());

  // Test iteration order and completeness
  int idx = 0;
  for (const NodeType& node : subtree_iterator) {
    EXPECT_EQ(node.data(), expected_nodes[idx++]);
  }
  EXPECT_EQ(idx, expected_nodes.size());

  // Test compatibility with std::distance (e.g. to count the number of nodes)
  EXPECT_EQ(std::distance(subtree_iterator.begin(), subtree_iterator.end()),
            expected_nodes.size());
}
TEST_F(SubtreeIteratorTest, BreadthFirstTraversal) {
  std::unique_ptr<const NodeType> root_node = demoTree();
  const std::vector<int> expected_nodes{1, 2,  3,  4,  5,  6,  7, 8,
                                        9, 10, 11, 12, 13, 14, 15};
  auto subtree_iterator =
      Subtree<const NodeType, TraversalOrder::kBreadthFirst>(root_node.get());

  // Test iteration order and completeness
  int idx = 0;
  for (const NodeType& node : subtree_iterator) {
    EXPECT_EQ(node.data(), expected_nodes[idx++]);
  }
  EXPECT_EQ(idx, expected_nodes.size());

  // Test compatibility with std::distance (e.g. to count the number of nodes)
  EXPECT_EQ(std::distance(subtree_iterator.begin(), subtree_iterator.end()),
            expected_nodes.size());
}
}  // namespace wavemap
