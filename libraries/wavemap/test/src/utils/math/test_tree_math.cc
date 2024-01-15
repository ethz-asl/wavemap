#include <gtest/gtest.h>

#include "wavemap/utils/math/tree_math.h"

namespace wavemap {
TEST(TreeMathTest, PerfectTreeLeafCounts) {
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<1>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<1>(2), 2);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<1>(3), 4);

  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<2>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<2>(2), 4);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<2>(3), 16);

  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<3>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<3>(2), 8);
  EXPECT_EQ(tree_math::perfect_tree::num_leaf_nodes<3>(3), 64);
}

TEST(TreeMathTest, PerfectTreeTotalNodeCounts) {
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<1>(0), 0);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<1>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<1>(2), 3);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<1>(3), 7);

  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<2>(0), 0);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<2>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<2>(2), 5);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<2>(3), 21);

  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<3>(0), 0);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<3>(1), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<3>(2), 9);
  EXPECT_EQ(tree_math::perfect_tree::num_total_nodes<3>(3), 73);
}

TEST(TreeMathTest, PerfectTreeInnerNodeCounts) {
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<1>(2), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<1>(3), 3);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<1>(4), 7);

  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<2>(2), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<2>(3), 5);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<2>(4), 21);

  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<3>(2), 1);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<3>(3), 9);
  EXPECT_EQ(tree_math::perfect_tree::num_inner_nodes<3>(4), 73);
}

TEST(TreeMathTest, PerfectTreeTotalNodeFastCounts) {
  constexpr int kMaxHeight = 20;

  for (int i = 0; i <= kMaxHeight; ++i) {
    EXPECT_EQ(tree_math::perfect_tree::num_total_nodes_fast<1>(i),
              tree_math::perfect_tree::num_total_nodes<1>(i));
  }

  for (int i = 0; i <= kMaxHeight; ++i) {
    EXPECT_EQ(tree_math::perfect_tree::num_total_nodes_fast<2>(i),
              tree_math::perfect_tree::num_total_nodes<2>(i));
  }
  for (int i = 0; i <= kMaxHeight; ++i) {
    EXPECT_EQ(tree_math::perfect_tree::num_total_nodes_fast<3>(i),
              tree_math::perfect_tree::num_total_nodes<3>(i));
  }
}
}  // namespace wavemap
