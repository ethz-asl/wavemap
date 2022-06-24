#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/coarse_to_fine_integrator.h"
#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/hierarchical_range_image.h"
#include "wavemap_2d/iterator/grid_iterator.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/angle_utils.h"
#include "wavemap_2d/utils/container_print_utils.h"

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
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, num_beams, kMinDistance, kMaxDistance);

    // Create the hierarchical range image
    const auto range_image =
        std::make_shared<RangeImage>(CoarseToFineIntegrator::computeRangeImage(
            random_pointcloud, kMinAngle, kMaxAngle, num_beams));
    HierarchicalRangeImage hierarchical_range_image(range_image);

    // Test all the bounds from top to bottom
    const BinaryTreeIndex::Element max_height =
        hierarchical_range_image.getMaxHeight();
    for (BinaryTreeIndex index{0, BinaryTreeIndex::Position::Zero()};
         index.height <= max_height; ++index.height) {
      const BinaryTreeIndex::Element num_elements_at_level =
          int_math::exp2(max_height - index.height);
      for (index.position.x() = 0; index.position.x() < num_elements_at_level;
           ++index.position.x()) {
        // Avoid out-of-bounds range image access when we're at the leaf level
        if (index.height == 0 &&
            range_image->getNumBeams() <= index.position.x()) {
          continue;
        }

        // Check if the different accessors return the same values
        EXPECT_LE(hierarchical_range_image.getLowerBound(index),
                  hierarchical_range_image.getUpperBound(index));
        EXPECT_EQ(hierarchical_range_image.getBounds(index).lower,
                  hierarchical_range_image.getLowerBound(index));
        EXPECT_EQ(hierarchical_range_image.getBounds(index).upper,
                  hierarchical_range_image.getUpperBound(index));

        // Check if the values returned by the accessors are correct
        if (index.height == 0) {
          // At the leaf level the bounds should match range image itself
          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          range_image->operator[](index.position.x()));
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          range_image->operator[](index.position.x()));
        } else if (index.height == 1) {
          // At the first pyramid level, the bounds should correspond to min/max
          // pooling the range image with a downsampling factor of 2
          const BinaryTreeIndex::Element first_child_idx =
              index.computeChildIndex(0).position.x();
          const BinaryTreeIndex::Element second_child_idx =
              index.computeChildIndex(1).position.x();
          if (second_child_idx < range_image->getNumBeams()) {
            EXPECT_FLOAT_EQ(
                hierarchical_range_image.getLowerBound(index),
                std::min(range_image->operator[](first_child_idx),
                         range_image->operator[](second_child_idx)));
            EXPECT_FLOAT_EQ(
                hierarchical_range_image.getUpperBound(index),
                std::max(range_image->operator[](first_child_idx),
                         range_image->operator[](second_child_idx)));
          } else if (first_child_idx < range_image->getNumBeams()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            range_image->operator[](first_child_idx));
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            range_image->operator[](first_child_idx));
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
    for (int start_idx = 0; start_idx < range_image->getNumBeams();
         ++start_idx) {
      for (int end_idx = start_idx; end_idx < range_image->getNumBeams();
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
            range_image->getData().block(0, start_idx, 1, range_length);
        const FloatingPoint lower_bound_brute_force = range.minCoeff();
        const FloatingPoint upper_bound_brute_force = range.maxCoeff();
        EXPECT_LE(lower_bound, lower_bound_brute_force);
        EXPECT_GE(upper_bound, upper_bound_brute_force);
      }
    }
  }
}

TEST_F(CoarseToFineIntegratorTest, ApproxAtan2) {
  constexpr int kNumAngles = 360 * 1000;
  for (int i = 0; i <= kNumAngles; ++i) {
    const FloatingPoint angle = kTwoPi * static_cast<FloatingPoint>(i) /
                                static_cast<FloatingPoint>(kNumAngles);
    const Vector bearing{std::cos(angle), std::sin(angle)};
    EXPECT_NEAR(RangeImageIntersector::atan2_approx(bearing.y(), bearing.x()),
                std::atan2(bearing.y(), bearing.x()),
                RangeImageIntersector::kWorstCaseAtan2ApproxError);
  }
}

TEST_F(CoarseToFineIntegratorTest, AabbMinMaxProjectedAngle) {
  struct QueryAndExpectedResults {
    AABB<Point> W_aabb;
    Transformation T_W_C;

    QueryAndExpectedResults(AABB<Point> W_aabb, const Transformation& T_W_C)
        : W_aabb(std::move(W_aabb)), T_W_C(T_W_C) {}
  };

  // Generate test set
  std::vector<QueryAndExpectedResults> tests;
  {
    // Manually define initial AABBs
    std::list<AABB<Point>> aabbs{{Point::Zero(), Point::Ones()},
                                 {Point::Zero(), Point{0.5f, 1.f}},
                                 {Point::Zero(), Point{1.f, 0.5f}}};
    // Insert copies of the initial AABBs flipped across the Y-axis
    std::generate_n(std::back_inserter(aabbs), aabbs.size(),
                    [aabbs_it = aabbs.cbegin()]() mutable {
                      AABB<Point> aabb_flipped{
                          {-aabbs_it->max.x(), aabbs_it->min.y()},
                          {-aabbs_it->min.x(), aabbs_it->max.y()}};
                      ++aabbs_it;
                      return aabb_flipped;
                    });
    // Insert copies of all above AABBs flipped across the X-axis
    std::generate_n(std::back_inserter(aabbs), aabbs.size(),
                    [aabbs_it = aabbs.cbegin()]() mutable {
                      AABB<Point> aabb_flipped{
                          {aabbs_it->min.x(), -aabbs_it->max.y()},
                          {aabbs_it->max.x(), -aabbs_it->min.y()}};
                      ++aabbs_it;
                      return aabb_flipped;
                    });
    // NOTE: The AABB set now consists of the initial AABBs and Y-flipped,
    //       X-flipped, and XY-flipped copies.

    // Create tests by combining the above AABBs (incl. random rescaling and
    // translations) with identity and random sensor poses
    for (const auto& aabb : aabbs) {
      for (int i = 0; i < 1000; ++i) {
        const FloatingPoint random_scale = 1.f / getRandomMinCellWidth();
        for (const Vector& t_random :
             {getRandomTranslation(), Vector{getRandomSignedDistance(), 0.f},
              Vector{0.f, getRandomSignedDistance()}}) {
          const AABB<Point> aabb_scaled{random_scale * aabb.min,
                                        random_scale * aabb.max};
          const AABB<Point> aabb_translated{aabb.min + t_random,
                                            aabb.max + t_random};
          const AABB<Point> aabb_scaled_translated{aabb_scaled.min + t_random,
                                                   aabb_scaled.max + t_random};
          const Transformation T_W_C_random = getRandomTransformation();
          tests.emplace_back(aabb_scaled, Transformation());
          tests.emplace_back(aabb_translated, Transformation());
          tests.emplace_back(aabb_scaled_translated, Transformation());
          tests.emplace_back(aabb, T_W_C_random);
          tests.emplace_back(aabb_scaled, T_W_C_random);
          tests.emplace_back(aabb_translated, T_W_C_random);
          tests.emplace_back(aabb_scaled_translated, T_W_C_random);
        }
      }
    }
  }

  // Run tests
  int error_count = 0;
  for (const auto& test : tests) {
    RangeImageIntersector::MinMaxAnglePair reference_angle_pair;
    const AABB<Point>::Corners C_cell_corners =
        test.T_W_C.inverse().transformVectorized(test.W_aabb.corners());
    std::array<FloatingPoint, AABB<Point>::kNumCorners> angles{};
    if (test.W_aabb.containsPoint(test.T_W_C.getPosition())) {
      reference_angle_pair.min_angle = -kPi;
      reference_angle_pair.max_angle = kPi;
    } else {
      for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
        angles[corner_idx] =
            RangeImage::bearingToAngle(C_cell_corners.col(corner_idx));
      }
      std::sort(angles.begin(), angles.end());
      const FloatingPoint min_angle = angles[0];
      const FloatingPoint max_angle = angles[3];
      const bool angle_range_wraps_around = kPi < max_angle - min_angle;
      if (angle_range_wraps_around) {
        const FloatingPoint smallest_angle_above_zero =
            *std::upper_bound(angles.begin(), angles.end(), 0.f);
        const FloatingPoint greatest_angle_below_zero =
            *std::prev(std::upper_bound(angles.begin(), angles.end(), 0.f));
        reference_angle_pair.min_angle = smallest_angle_above_zero;
        reference_angle_pair.max_angle = greatest_angle_below_zero;
      } else {
        reference_angle_pair.min_angle = min_angle;
        reference_angle_pair.max_angle = max_angle;
      }
    }
    const RangeImageIntersector::MinMaxAnglePair returned_angle_pair =
        RangeImageIntersector::getAabbMinMaxProjectedAngle(test.T_W_C,
                                                           test.W_aabb);
    constexpr FloatingPoint kOneAndAHalfDegree = 0.0261799f;
    const bool angle_range_wraps_around =
        kPi < reference_angle_pair.max_angle - reference_angle_pair.min_angle;
    bool check_failed = false;
    if (angle_range_wraps_around) {
      EXPECT_GE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                0.f)
          << (check_failed = true);
      EXPECT_LE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                kOneAndAHalfDegree);
      EXPECT_LE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                0.f)
          << (check_failed = true);
      EXPECT_GE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                -kOneAndAHalfDegree)
          << (check_failed = true);
    } else {
      EXPECT_LE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                0.f)
          << (check_failed = true);
      EXPECT_GE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                -kOneAndAHalfDegree)
          << (check_failed = true);
      EXPECT_GE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                0.f)
          << (check_failed = true);
      EXPECT_LE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                kOneAndAHalfDegree)
          << (check_failed = true);
    }
    if (check_failed) {
      std::cerr << "For\n-W_aabb: " << test.W_aabb.toString()
                << "\n-T_W_C: " << test.T_W_C << "\nWith C_cell_corners:\n"
                << C_cell_corners << "\nangles_C_cell_corners:\n"
                << ToString(angles) << "\nand reference min/max angles: "
                << reference_angle_pair.min_angle << ", "
                << reference_angle_pair.max_angle
                << "\nWe got min/max angles: " << returned_angle_pair.min_angle
                << ", " << returned_angle_pair.max_angle
                << "\nThis is error nr " << ++error_count << "\n"
                << std::endl;
    }
  }
}

TEST_F(CoarseToFineIntegratorTest, RangeImageIntersectionType) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 10.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, num_beams, kMinDistance, kMaxDistance);

    // Create the hierarchical range image
    const auto range_image =
        std::make_shared<RangeImage>(CoarseToFineIntegrator::computeRangeImage(
            random_pointcloud, kMinAngle, kMaxAngle, num_beams));
    RangeImageIntersector range_image_intersector(range_image);

    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    constexpr QuadtreeIndex::Element kMaxHeight = 10;
    const Index min_index = convert::pointToCeilIndex(
        random_pointcloud.getOrigin() - Vector::Constant(kMaxDistance),
        min_cell_width_inv);
    const Index max_index = convert::pointToCeilIndex(
        random_pointcloud.getOrigin() + Vector::Constant(kMaxDistance),
        min_cell_width_inv);
    for (const Index& index :
         getRandomIndexVector(min_index, max_index, 50, 100)) {
      const QuadtreeIndex::Element height =
          getRandomNdtreeIndexHeight(2, kMaxHeight);
      const QuadtreeIndex query_index =
          convert::indexAndHeightToNodeIndex(index, height);
      const Index min_reference_index =
          convert::nodeIndexToMinCornerIndex(query_index);
      const Index max_reference_index =
          convert::nodeIndexToMaxCornerIndex(query_index);
      bool has_free = false;
      bool has_occupied = false;
      bool has_unknown = false;
      const Transformation T_C_W = random_pointcloud.getPose().inverse();
      for (const Index& reference_index :
           Grid(min_reference_index, max_reference_index)) {
        const Point W_cell_center =
            convert::indexToCenterPoint(reference_index, min_cell_width);
        const Point C_cell_center = T_C_W * W_cell_center;
        const FloatingPoint d_C_cell = C_cell_center.norm();
        if (BeamModel::kRangeMax < d_C_cell) {
          has_unknown = true;
          continue;
        }

        const RangeImageIndex range_image_index =
            range_image->bearingToNearestIndex(C_cell_center);
        if (range_image_index < 0 ||
            range_image->getNumBeams() <= range_image_index) {
          has_unknown = true;
          continue;
        }

        const FloatingPoint range_image_distance =
            range_image->operator[](range_image_index);
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
          convert::heightToCellWidth(min_cell_width, query_index.height);
      const Point W_node_center =
          convert::indexToCenterPoint(query_index.position, node_width);

      const Point W_node_bottom_left =
          W_node_center - Vector::Constant(node_width / 2.f);
      const AABB<Point> W_cell_aabb{
          W_node_bottom_left,
          W_node_bottom_left + Vector::Constant(node_width)};
      const RangeImageIntersector::IntersectionType returned_intersection_type =
          range_image_intersector.determineIntersectionType(
              random_pointcloud.getPose(), W_cell_aabb);
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
