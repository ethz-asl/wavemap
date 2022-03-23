#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/cell_types/scalar_occupancy_cell.h"
#include "wavemap_2d/datastructure/quadtree/quadtree.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename CellType>
class QuadtreeTest : public FixtureBase {
 protected:
  Quadtree<CellType> getRandomMap() {
    Quadtree<CellType> random_map(getRandomResolution());
    const Index min_index = -getRandomIndex().cwiseAbs();
    const Index max_index = getRandomIndex().cwiseAbs();
    random_map.addToCellValue(min_index, 0.f);
    random_map.addToCellValue(max_index, 0.f);
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        random_map.addToCellValue(index, getRandomUpdate());
      }
    }
    return random_map;
  }

  NodeIndexElement getRandomDepth() const {
    constexpr NodeIndexElement kMinDepth = 0;
    constexpr NodeIndexElement kMaxDepth = 14;
    return random_number_generator_->getRandomInteger(kMinDepth, kMaxDepth);
  }

  std::vector<Index> getRandomIndexVectorWithinBounds(
      const Index& min_index, const Index& max_index) const {
    CHECK((min_index.array() < max_index.array()).all());
    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);
    std::vector<Index> random_indices;
    random_indices.reserve(num_indices);
    while (random_indices.size() < num_indices) {
      const Index random_index = getRandomIndex();
      if ((min_index.array() < random_index.array() &&
           random_index.array() < max_index.array())
              .all()) {
        random_indices.emplace_back(random_index);
      }
    }
    return random_indices;
  }

  std::vector<NodeIndex> getRandomNodeIndexVectorWithinBounds(
      const Index& min_index, const Index& max_index,
      NodeIndexElement min_depth, NodeIndexElement max_depth) const {
    CHECK((min_index.array() < max_index.array()).all());
    CHECK_LE(min_depth, max_depth);
    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);
    std::vector<NodeIndex> random_indices;
    random_indices.reserve(num_indices);
    while (random_indices.size() < num_indices) {
      const NodeIndex random_index = {.depth = getRandomDepth(),
                                      .position = getRandomIndex()};
      if (min_depth <= random_index.depth && random_index.depth <= max_depth &&
          (min_index.array() < random_index.position.array() &&
           random_index.position.array() < max_index.array())
              .all()) {
        random_indices.emplace_back(random_index);
      }
    }
    return random_indices;
  }
};

using CellTypes =
    ::testing::Types<UnboundedOccupancyCell, SaturatingOccupancyCell>;
TYPED_TEST_SUITE(QuadtreeTest, CellTypes);

TYPED_TEST(QuadtreeTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  Quadtree<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
}

TYPED_TEST(QuadtreeTest, Resizing) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Quadtree<TypeParam> map(TestFixture::getRandomResolution());
    ASSERT_TRUE(map.empty());
    ASSERT_EQ(map.size(), 0u);

    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVectorWithinBounds(
            map.getMinPossibleIndex(), map.getMaxPossibleIndex());

    const Index& first_random_index = random_indices[0];
    map.addToCellValue(first_random_index, 0.f);
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), map.getMaxDepth());

    Index min_index = first_random_index;
    Index max_index = first_random_index;
    for (auto index_it = ++random_indices.begin();
         index_it != random_indices.end(); ++index_it) {
      min_index = min_index.cwiseMin(*index_it);
      max_index = max_index.cwiseMax(*index_it);
      map.addToCellValue(*index_it, 0.f);
    }
    EXPECT_GE(map.size(), map.getMaxDepth());
    size_t max_unique_nodes = 0u;
    const size_t num_inserted_nodes = random_indices.size();
    for (NodeIndexElement depth = 0u; depth <= map.getMaxDepth(); ++depth) {
      const size_t max_unique_nodes_at_depth = std::exp2(kMapDimension * depth);
      if (max_unique_nodes_at_depth < num_inserted_nodes) {
        max_unique_nodes += max_unique_nodes_at_depth;
      } else {
        max_unique_nodes += num_inserted_nodes;
      }
    }
    EXPECT_LE(map.size(), max_unique_nodes);

    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
      }
    }

    map.clear();
    EXPECT_TRUE(map.empty());
    EXPECT_EQ(map.size(), 0u);
  }
}

TYPED_TEST(QuadtreeTest, Insertion) {
  // TODO(victorr): Test whether out of bounds accesses/insertions are handled
  //                correctly (e.g. throw error or do nothing and print error).
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Quadtree<TypeParam> map(TestFixture::getRandomResolution());
    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVectorWithinBounds(
            map.getMinPossibleIndex(), map.getMaxPossibleIndex());
    for (const Index& random_index : random_indices) {
      FloatingPoint expected_value = 0.f;
      map.setCellValue(random_index, 0.f);
      for (const FloatingPoint random_update :
           TestFixture::getRandomUpdateVector()) {
        map.addToCellValue(random_index, random_update);
        expected_value = std::max(
            TypeParam::kLowerBound,
            std::min(expected_value + random_update, TypeParam::kUpperBound));
      }
      EXPECT_NEAR(map.getCellValue(random_index), expected_value,
                  expected_value * 1e-6);
    }
  }
}

TYPED_TEST(QuadtreeTest, NodeIndexConversions) {
  const Quadtree<TypeParam> map(TestFixture::getRandomResolution());

  // Generate a combination of random and handpicked node indices for testing
  const NodeIndexElement max_depth = map.getMaxDepth();
  std::vector<NodeIndex> random_indices =
      TestFixture::getRandomNodeIndexVectorWithinBounds(
          map.getMinPossibleIndex(), map.getMaxPossibleIndex(), 1, max_depth);
  random_indices.emplace_back(NodeIndex{.depth = 0, .position = {0, 0}});
  for (NodeIndexElement index_depth = 1; index_depth < max_depth;
       ++index_depth) {
    for (NodeIndexElement index_x = -1; index_x <= 1; ++index_x) {
      for (NodeIndexElement index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NodeIndex{.depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const NodeIndex& node_index : random_indices) {
    // Compare to coordinate convention
    {
      const Index index_from_quadtree =
          map.computeIndexFromNodeIndex(node_index);
      const Index index_from_convention = computeNearestIndexForScaledPoint(
          node_index.position.template cast<FloatingPoint>() *
              std::exp2(max_depth - node_index.depth) -
          Vector::Constant(std::exp2(max_depth - 1)));
      EXPECT_EQ(index_from_quadtree, index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(index_from_quadtree)
          << " does not match convention "
          << EigenFormat::oneLine(index_from_convention);
    }

    // Roundtrip through regular indices (integer coordinates)
    {
      const Index index = map.computeIndexFromNodeIndex(node_index);
      const NodeIndex roundtrip_node_index =
          map.computeNodeIndexFromIndexAndDepth(index, node_index.depth);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }

    // Roundtrip through real valued coordinates
    {
      const Point node_center = map.computeNodeCenterFromNodeIndex(node_index);
      const NodeIndex roundtrip_node_index =
          map.computeNodeIndexFromCenter(node_center, node_index.depth);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << EigenFormat::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
  }
}

TYPED_TEST(QuadtreeTest, ChildParentIndexing) {
  const Quadtree<TypeParam> map(TestFixture::getRandomResolution());

  // Generate a combination of random and handpicked node indices for testing
  const NodeIndexElement max_depth = map.getMaxDepth();
  std::vector<NodeIndex> random_indices =
      TestFixture::getRandomNodeIndexVectorWithinBounds(
          map.getMinPossibleIndex(), map.getMaxPossibleIndex(), max_depth,
          max_depth);
  random_indices.emplace_back(NodeIndex{.depth = 0, .position = {0, 0}});
  for (NodeIndexElement index_depth = 1; index_depth < max_depth;
       ++index_depth) {
    for (NodeIndexElement index_x = 0; index_x <= 1; ++index_x) {
      for (NodeIndexElement index_y = 0; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NodeIndex{.depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test round trips between children and parents
  const NodeIndex root_index{.depth = 0, .position = {0, 0}};
  for (const NodeIndex& node_index : random_indices) {
    const NodeIndex top_parent_index = node_index.computeParentIndex(0);
    EXPECT_EQ(top_parent_index, root_index)
        << "The index of the highest parent of node " << node_index.toString()
        << " is " << top_parent_index.toString()
        << " while it should equal the root node index "
        << root_index.toString() << ".";
    for (NodeRelativeChildIndex relative_child_idx = 0;
         relative_child_idx < NodeIndex::kNumChildren; ++relative_child_idx) {
      const NodeIndex child_index =
          node_index.computeChildIndex(relative_child_idx);
      EXPECT_EQ(child_index.computeRelativeChildIndex(), relative_child_idx);
      EXPECT_EQ(child_index.computeParentIndex(), node_index);
    }
  }
}

TYPED_TEST(QuadtreeTest, Pruning) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    Quadtree<TypeParam> map(TestFixture::getRandomResolution());
    const std::vector<Index> random_indices =
        TestFixture::getRandomIndexVectorWithinBounds(
            map.getMinPossibleIndex(), map.getMaxPossibleIndex());

    // Check that zero values are inserted but removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index& random_index : random_indices) {
      map.setCellValue(random_index, 0.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_TRUE(map.empty());

    // Check that non-zero values are inserted but not removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index& random_index : random_indices) {
      map.setCellValue(random_index, 1.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_FALSE(map.empty());
  }
}
}  // namespace wavemap_2d
