#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"
#include "wavemap_2d/iterator/grid_iterator.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
class CoarseToFineIntegratorTest : public FixtureBase {
 protected:
  Transformation getRandomTransformation() const {
    return {Transformation::Rotation(getRandomAngle()), getRandomTranslation()};
  }

  PosedPointcloud<> getRandomPointcloud(FloatingPoint min_angle,
                                        FloatingPoint max_angle, int num_beams,
                                        FloatingPoint min_distance,
                                        FloatingPoint max_distance) const {
    CHECK_LT(min_angle, max_angle);
    CHECK_LT(min_distance, max_distance);

    Pointcloud<> pointcloud;
    pointcloud.resize(num_beams);

    const FloatingPoint angle_increment =
        (max_angle - min_angle) / static_cast<FloatingPoint>(num_beams - 1);
    for (int index = 0; index < num_beams; ++index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      const FloatingPoint angle =
          min_angle + static_cast<FloatingPoint>(index) * angle_increment;

      pointcloud[index] = range * RangeImage::angleToBearing(angle);
    }

    return {getRandomTransformation(), pointcloud};
  }
};

TEST_F(CoarseToFineIntegratorTest, HierarchicalRangeImage) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    constexpr FloatingPoint kMinAngle = -M_PI_2f32;
    constexpr FloatingPoint kMaxAngle = M_PI_2f32;
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, num_beams, kMinDistance, kMaxDistance);

    // Create the hierarchical range image
    const RangeImage range_image = CoarseToFineIntegrator::computeRangeImage(
        random_pointcloud, kMinAngle, kMaxAngle, num_beams);
    HierarchicalRangeImage hierarchical_range_image(range_image);

    // Test all the bounds from top to bottom
    const auto max_depth = static_cast<BinaryTreeIndex::Element>(
        hierarchical_range_image.getMaxDepth());
    const auto pyramid_max_depth = static_cast<BinaryTreeIndex::Element>(
        hierarchical_range_image.getNumBoundLevels());
    for (BinaryTreeIndex index{0, BinaryTreeIndex::Position::Zero()};
         index.depth <= max_depth; ++index.depth) {
      const BinaryTreeIndex::Element num_elements_at_level = 1 << index.depth;
      for (index.position.x() = 0; index.position.x() < num_elements_at_level;
           ++index.position.x()) {
        // Avoid out-of-bounds range image access when we're at the leaf level
        if (index.depth == max_depth) {
          if (range_image.getNumBeams() <= index.position.x()) {
            continue;
          }
        }

        // Check if the different accessors return the same values
        EXPECT_LE(hierarchical_range_image.getLowerBound(index),
                  hierarchical_range_image.getUpperBound(index));
        EXPECT_EQ(hierarchical_range_image.getBounds(index).lower,
                  hierarchical_range_image.getLowerBound(index));
        EXPECT_EQ(hierarchical_range_image.getBounds(index).upper,
                  hierarchical_range_image.getUpperBound(index));

        // Check if the values returned by the accessors are correct
        if (index.depth == max_depth) {
          // At the leaf level the bounds should match range image itself
          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          range_image[index.position.x()]);
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          range_image[index.position.x()]);
        } else if (index.depth == pyramid_max_depth) {
          // At the first pyramid level, the bounds should correspond to min/max
          // pooling the range image with a downsampling factor of 2
          const BinaryTreeIndex::Element first_child_idx =
              index.computeChildIndex(0).position.x();
          const BinaryTreeIndex::Element second_child_idx =
              index.computeChildIndex(1).position.x();
          if (second_child_idx < range_image.getNumBeams()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            std::min(range_image[first_child_idx],
                                     range_image[second_child_idx]));
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            std::max(range_image[first_child_idx],
                                     range_image[second_child_idx]));
          } else if (first_child_idx < range_image.getNumBeams()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            range_image[first_child_idx]);
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            range_image[first_child_idx]);
          } else {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index), 0.f);
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index), 0.f);
          }
        } else {
          // At all other levels, the bounds correspond to min/max the bounds of
          // the previous level
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

    // Test range bounds on all subintervals and compare to brute force
    for (int start_idx = 0; start_idx < range_image.getNumBeams();
         ++start_idx) {
      for (int end_idx = start_idx; end_idx < range_image.getNumBeams();
           ++end_idx) {
        // Check if the different accessors return the same values
        const Bounds bounds =
            hierarchical_range_image.getRangeBounds(start_idx, end_idx);
        const FloatingPoint lower_bound =
            hierarchical_range_image.getRangeLowerBound(start_idx, end_idx);
        const FloatingPoint upper_bound =
            hierarchical_range_image.getRangeUpperBound(start_idx, end_idx);
        EXPECT_LE(lower_bound, upper_bound);
        EXPECT_EQ(bounds.lower, lower_bound);
        EXPECT_EQ(bounds.upper, upper_bound);

        // Compare against brute force
        const int range_length = end_idx - start_idx + 1;
        CHECK_GE(range_length, 1);
        const RangeImage::RangeImageData::ConstBlockXpr range =
            range_image.getData().block(0, start_idx, 1, range_length);
        const FloatingPoint lower_bound_brute_force = range.minCoeff();
        const FloatingPoint upper_bound_brute_force = range.maxCoeff();
        EXPECT_LE(lower_bound, lower_bound_brute_force);
        EXPECT_GE(upper_bound, upper_bound_brute_force);
      }
    }
  }
}

TEST_F(CoarseToFineIntegratorTest, RangeImageIntersector) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint resolution = getRandomResolution();
    constexpr FloatingPoint kMinAngle = -M_PI_2f32;
    constexpr FloatingPoint kMaxAngle = M_PI_2f32;
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 10.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, num_beams, kMinDistance, kMaxDistance);

    // Create the hierarchical range image
    const RangeImage range_image = CoarseToFineIntegrator::computeRangeImage(
        random_pointcloud, kMinAngle, kMaxAngle, num_beams);
    RangeImageIntersector range_image_intersector(range_image);

    const FloatingPoint resolution_inv = 1.f / resolution;
    constexpr QuadtreeIndex::Element kMaxDepth = 10;
    const Index min_index = convert::pointToCeilIndex(
        random_pointcloud.getOrigin() - Vector::Constant(kMaxDistance),
        resolution_inv);
    const Index max_index = convert::pointToCeilIndex(
        random_pointcloud.getOrigin() + Vector::Constant(kMaxDistance),
        resolution_inv);
    for (const Index& index :
         getRandomIndexVector(min_index, max_index, 50, 100)) {
      const QuadtreeIndex::Element depth =
          getRandomNdtreeIndexDepth(kMaxDepth - 2, kMaxDepth);
      const QuadtreeIndex query_index =
          convert::indexAndDepthToNodeIndex(index, depth, kMaxDepth);
      const QuadtreeIndex::Element depth_diff = kMaxDepth - query_index.depth;
      const Index min_reference_index =
          int_math::exp2(depth_diff) * query_index.position;
      const QuadtreeIndex::Element max_child_offset =
          int_math::exp2(depth_diff) - 1;
      const Index max_reference_index{
          min_reference_index.x() + max_child_offset,
          min_reference_index.y() + max_child_offset};
      bool has_free = false;
      bool has_occupied = false;
      bool has_unknown = false;
      const Transformation T_C_W = random_pointcloud.getPose().inverse();
      for (const Index& reference_index :
           Grid(min_reference_index, max_reference_index)) {
        const Point W_cell_center =
            convert::indexToCenterPoint(reference_index, resolution);
        const Point C_cell_center = T_C_W * W_cell_center;
        const FloatingPoint d_C_cell = C_cell_center.norm();
        if (BeamModel::kRangeMax < d_C_cell) {
          has_unknown = true;
          continue;
        }

        const RangeImageIndex range_image_index =
            range_image.bearingToNearestIndex(C_cell_center);
        if (range_image_index < 0 ||
            range_image.getNumBeams() <= range_image_index) {
          has_unknown = true;
          continue;
        }

        const FloatingPoint range_image_distance =
            range_image[range_image_index];
        if (d_C_cell < range_image_distance) {
          has_free = true;
        } else if (d_C_cell <=
                   range_image_distance + BeamModel::kRangeDeltaThresh) {
          has_occupied = true;
        } else {
          has_unknown = true;
        }
      }
      ASSERT_TRUE(has_free || has_occupied || has_unknown);
      RangeImageIntersector::IntersectionType reference_intersection_type;
      if (has_occupied) {
        reference_intersection_type =
            RangeImageIntersector::IntersectionType::kPossiblyOccupied;
      } else if (has_free) {
        reference_intersection_type =
            RangeImageIntersector::IntersectionType::kFreeOrUnknown;
      } else {
        reference_intersection_type =
            RangeImageIntersector::IntersectionType::kFullyUnknown;
      }

      const FloatingPoint node_width =
          resolution *
          std::exp2f(static_cast<FloatingPoint>(kMaxDepth - query_index.depth));
      const Point W_node_center =
          convert::indexToCenterPoint(query_index.position, node_width);

      const Point W_node_bottom_left =
          W_node_center - Vector::Constant(node_width / 2.f);
      const Point& t_W_C = random_pointcloud.getOrigin();
      Eigen::Matrix<FloatingPoint, 2, 4> W_cell_corners =
          W_node_bottom_left.replicate<1, 4>();
      for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
        for (int dim_idx = 0; dim_idx < 2; ++dim_idx) {
          if (corner_idx & (0b1 << dim_idx)) {
            W_cell_corners(dim_idx, corner_idx) += node_width;
          }
        }
      }
      const AABB<Point> W_cell_aabb{W_cell_corners.col(0),
                                    W_cell_corners.col(3)};
      const Eigen::Matrix<FloatingPoint, 2, 4> C_cell_corners =
          T_C_W.transformVectorized(W_cell_corners);
      const RangeImageIntersector::IntersectionType returned_intersection_type =
          range_image_intersector.determineIntersectionType(
              range_image, t_W_C, W_cell_aabb, C_cell_corners);
      EXPECT_TRUE(reference_intersection_type <= returned_intersection_type)
          << "Expected "
          << RangeImageIntersector::getIntersectionTypeStr(
                 reference_intersection_type)
          << " but got "
          << RangeImageIntersector::getIntersectionTypeStr(
                 returned_intersection_type);
    }
  }
}
}  // namespace wavemap_2d
