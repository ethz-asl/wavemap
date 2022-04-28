#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_scan_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
class CoarseToFineIntegratorTest : public FixtureBase {
 protected:
  PosedPointcloud<> getRandomPointcloud(FloatingPoint min_angle,
                                        FloatingPoint max_angle, int n_beams,
                                        FloatingPoint min_distance,
                                        FloatingPoint max_distance) const {
    CHECK_LT(min_angle, max_angle);
    CHECK_LT(min_distance, max_distance);

    Pointcloud<> pointcloud;
    pointcloud.resize(n_beams);

    const FloatingPoint angle_increment =
        (max_angle - min_angle) / static_cast<FloatingPoint>(n_beams - 1);
    for (int index = 0; index < n_beams; ++index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      const FloatingPoint angle =
          min_angle + static_cast<FloatingPoint>(index) * angle_increment;

      pointcloud[index] = range * RangeImage::angleToBearing(angle);
    }

    return {Transformation(), pointcloud};
  }
};

TEST_F(CoarseToFineIntegratorTest, HierarchicalRangeImage) {
  for (int repetition = 0; repetition < 10; ++repetition) {
    // Config
    constexpr FloatingPoint kMinAngle = -M_PI_2f32;
    constexpr FloatingPoint kMaxAngle = M_PI_2f32;
    const int n_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;

    // Generate the hierarchical range image
    const PosedPointcloud<> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, n_beams, kMinDistance, kMaxDistance);
    RangeImage range_image(kMinAngle, kMaxAngle, n_beams);
    CoarseToFineScanIntegrator::computeRangeImage(random_pointcloud,
                                                  range_image);
    HierarchicalRangeImage hierarchical_range_image(range_image);

    // Test all the bounds from top to bottom
    const auto max_depth = static_cast<BinaryTreeIndex::Element>(
        hierarchical_range_image.getMaxDepth());
    BinaryTreeIndex index;
    for (index.depth = 0; index.depth <= max_depth; ++index.depth) {
      const BinaryTreeIndex::Element num_elements_at_level = 1 << index.depth;
      for (index.position[0] = 0; index.position[0] < num_elements_at_level;
           ++index.position[0]) {
        EXPECT_LE(hierarchical_range_image.getLowerBound(index),
                  hierarchical_range_image.getUpperBound(index));
        if (index.depth == max_depth) {
          const BinaryTreeIndex::Element first_child_idx =
              index.computeChildIndex(0).position[0];
          const BinaryTreeIndex::Element second_child_idx =
              index.computeChildIndex(1).position[0];
          if (second_child_idx < range_image.getNBeams()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            std::min(range_image[first_child_idx],
                                     range_image[second_child_idx]));
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            std::max(range_image[first_child_idx],
                                     range_image[second_child_idx]));
          } else if (first_child_idx < range_image.getNBeams()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            range_image[first_child_idx]);
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            range_image[first_child_idx]);
          }
        } else {
          const BinaryTreeIndex first_child_idx = index.computeChildIndex(0);
          const BinaryTreeIndex second_child_idx = index.computeChildIndex(1);
          EXPECT_FLOAT_EQ(
              hierarchical_range_image.getLowerBound(index),
              std::min(
                  hierarchical_range_image.getLowerBound(first_child_idx),
                  hierarchical_range_image.getLowerBound(second_child_idx)));
          EXPECT_FLOAT_EQ(
              hierarchical_range_image.getUpperBound(index),
              std::max(
                  hierarchical_range_image.getUpperBound(first_child_idx),
                  hierarchical_range_image.getUpperBound(second_child_idx)));
        }
      }
    }
  }
}

TEST_F(CoarseToFineIntegratorTest, RangeImageIntersector) {}
}  // namespace wavemap_2d
