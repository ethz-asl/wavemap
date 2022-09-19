#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/container_print_utils.h>

#include "wavemap_2d/integrator/projective/coarse_to_fine/hierarchical_range_image_1d.h"
#include "wavemap_2d/integrator/projective/coarse_to_fine/range_image_1d_intersector.h"

namespace wavemap {
class RangeImage1DIntersectorTest : public FixtureBase {
 protected:
  PosedPointcloud<Point2D> getRandomPointcloud(
      FloatingPoint min_angle, FloatingPoint max_angle, int num_beams,
      FloatingPoint min_distance, FloatingPoint max_distance) const {
    CHECK_LT(min_angle, max_angle);
    CHECK_LT(min_distance, max_distance);

    Pointcloud<Point2D> pointcloud;
    pointcloud.resize(num_beams);

    const CircularProjector circular_projector(min_angle, max_angle, num_beams);
    for (int index = 0; index < num_beams; ++index) {
      const FloatingPoint range =
          getRandomSignedDistance(min_distance, max_distance);
      pointcloud[index] = range * circular_projector.indexToBearing(index);
    }

    return {getRandomTransformation<2>(), pointcloud};
  }
};

TEST_F(RangeImage1DIntersectorTest, AabbMinMaxProjectedAngle) {
  struct QueryAndExpectedResults {
    AABB<Point2D> W_aabb;
    Transformation2D T_W_C;

    QueryAndExpectedResults(AABB<Point2D> W_aabb, const Transformation2D& T_W_C)
        : W_aabb(std::move(W_aabb)), T_W_C(T_W_C) {}
  };

  // Generate test set
  std::vector<QueryAndExpectedResults> tests;
  {
    // Manually define initial AABBs
    std::list<AABB<Point2D>> aabbs{{Point2D::Zero(), Point2D::Ones()},
                                   {Point2D::Zero(), {0.5f, 1.f}},
                                   {Point2D::Zero(), {1.f, 0.5f}}};
    // Insert copies of the initial AABBs flipped across the Y-axis
    std::generate_n(std::back_inserter(aabbs), aabbs.size(),
                    [aabbs_it = aabbs.cbegin()]() mutable {
                      AABB<Point2D> aabb_flipped{
                          {-aabbs_it->max.x(), aabbs_it->min.y()},
                          {-aabbs_it->min.x(), aabbs_it->max.y()}};
                      ++aabbs_it;
                      return aabb_flipped;
                    });
    // Insert copies of all above AABBs flipped across the X-axis
    std::generate_n(std::back_inserter(aabbs), aabbs.size(),
                    [aabbs_it = aabbs.cbegin()]() mutable {
                      AABB<Point2D> aabb_flipped{
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
        for (const Vector2D& t_random :
             {getRandomTranslation<2>(),
              Vector2D{getRandomSignedDistance(), 0.f},
              Vector2D{0.f, getRandomSignedDistance()}}) {
          const AABB<Point2D> aabb_scaled{random_scale * aabb.min,
                                          random_scale * aabb.max};
          const AABB<Point2D> aabb_translated{aabb.min + t_random,
                                              aabb.max + t_random};
          const AABB<Point2D> aabb_scaled_translated{
              aabb_scaled.min + t_random, aabb_scaled.max + t_random};
          const Transformation2D T_W_C_random = getRandomTransformation<2>();
          tests.emplace_back(aabb_scaled, Transformation2D());
          tests.emplace_back(aabb_translated, Transformation2D());
          tests.emplace_back(aabb_scaled_translated, Transformation2D());
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
    RangeImage1DIntersector::MinMaxAnglePair reference_angle_pair;
    const AABB<Point2D>::Corners C_cell_corners =
        test.T_W_C.inverse().transformVectorized(test.W_aabb.corners());
    std::array<FloatingPoint, AABB<Point2D>::kNumCorners> angles{};
    if (test.W_aabb.containsPoint(test.T_W_C.getPosition())) {
      reference_angle_pair.min_angle = -kPi;
      reference_angle_pair.max_angle = kPi;
    } else {
      for (int corner_idx = 0; corner_idx < AABB<Point2D>::kNumCorners;
           ++corner_idx) {
        angles[corner_idx] =
            CircularProjector::bearingToAngle(C_cell_corners.col(corner_idx));
      }
      std::sort(angles.begin(), angles.end());
      const FloatingPoint min_angle = angles[0];
      const FloatingPoint max_angle = angles[3];
      const bool angle_range_wraps_around = kPi < max_angle - min_angle;
      if (angle_range_wraps_around) {
        const auto it = std::upper_bound(angles.cbegin(), angles.cend(), 0.f);
        const FloatingPoint smallest_angle_above_zero = *it;
        const FloatingPoint greatest_angle_below_zero = *std::prev(it);
        reference_angle_pair.min_angle = smallest_angle_above_zero;
        reference_angle_pair.max_angle = greatest_angle_below_zero;
      } else {
        reference_angle_pair.min_angle = min_angle;
        reference_angle_pair.max_angle = max_angle;
      }
    }

    const RangeImage1DIntersector::MinMaxAnglePair returned_angle_pair =
        RangeImage1DIntersector::getAabbMinMaxProjectedAngle(test.T_W_C,
                                                             test.W_aabb);
    constexpr FloatingPoint kOneAndAHalfDegree = 0.0261799f;
    const bool angle_range_wraps_around =
        kPi < reference_angle_pair.max_angle - reference_angle_pair.min_angle;

    bool check_failed = false;
    auto canary_token = [&check_failed]() {
      check_failed = true;
      return "";
    };
    if (angle_range_wraps_around) {
      EXPECT_GE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                0.f)
          << canary_token();
      EXPECT_LE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                kOneAndAHalfDegree);
      EXPECT_LE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                0.f)
          << canary_token();
      EXPECT_GE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                -kOneAndAHalfDegree)
          << canary_token();
    } else {
      EXPECT_LE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                0.f)
          << canary_token();
      EXPECT_GE(angle_math::normalize(returned_angle_pair.min_angle -
                                      reference_angle_pair.min_angle),
                -kOneAndAHalfDegree)
          << canary_token();
      EXPECT_GE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                0.f)
          << canary_token();
      EXPECT_LE(angle_math::normalize(returned_angle_pair.max_angle -
                                      reference_angle_pair.max_angle),
                kOneAndAHalfDegree)
          << canary_token();
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

TEST_F(RangeImage1DIntersectorTest, RangeImageIntersectionType) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random pointcloud
    const FloatingPoint min_cell_width = getRandomMinCellWidth();
    constexpr FloatingPoint kMinAngle = -kHalfPi;
    constexpr FloatingPoint kMaxAngle = kHalfPi;
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 10.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    const PosedPointcloud<Point2D> random_pointcloud = getRandomPointcloud(
        kMinAngle, kMaxAngle, num_beams, kMinDistance, kMaxDistance);

    // Create the hierarchical range image
    CircularProjector circular_projector(kMinAngle, kMaxAngle, num_beams);
    const auto range_image =
        std::make_shared<PosedRangeImage1D>(circular_projector);
    range_image->importPointcloud(random_pointcloud, circular_projector);
    RangeImage1DIntersector range_image_intersector(range_image);

    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    constexpr QuadtreeIndex::Element kMaxHeight = 10;
    const Index2D min_index = convert::pointToCeilIndex<2>(
        random_pointcloud.getOrigin() - Vector2D::Constant(kMaxDistance),
        min_cell_width_inv);
    const Index2D max_index = convert::pointToCeilIndex<2>(
        random_pointcloud.getOrigin() + Vector2D::Constant(kMaxDistance),
        min_cell_width_inv);
    for (const Index2D& index :
         getRandomIndexVector(min_index, max_index, 50, 100)) {
      const QuadtreeIndex::Element height =
          getRandomNdtreeIndexHeight(2, kMaxHeight);
      const QuadtreeIndex query_index =
          convert::indexAndHeightToNodeIndex(index, height);
      const Index2D min_reference_index =
          convert::nodeIndexToMinCornerIndex(query_index);
      const Index2D max_reference_index =
          convert::nodeIndexToMaxCornerIndex(query_index);
      bool has_free = false;
      bool has_occupied = false;
      bool has_unknown = false;
      const Transformation2D T_C_W = random_pointcloud.getPose().inverse();
      for (const Index2D& reference_index :
           Grid(min_reference_index, max_reference_index)) {
        const Point2D W_cell_center =
            convert::indexToCenterPoint(reference_index, min_cell_width);
        const Point2D C_cell_center = T_C_W * W_cell_center;
        const FloatingPoint d_C_cell = C_cell_center.norm();
        if (ContinuousVolumetricLogOdds<2>::kRangeMax < d_C_cell) {
          has_unknown = true;
          continue;
        }

        const IndexElement range_image_index =
            circular_projector.bearingToNearestIndex(C_cell_center);
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
                   range_image_distance +
                       ContinuousVolumetricLogOdds<2>::kRangeDeltaThresh) {
          has_occupied = true;
        } else {
          has_unknown = true;
        }
      }
      ASSERT_TRUE(has_free || has_occupied || has_unknown);
      IntersectionType reference_intersection_type;
      if (has_occupied) {
        reference_intersection_type = IntersectionType::kPossiblyOccupied;
      } else if (has_free) {
        reference_intersection_type = IntersectionType::kFreeOrUnknown;
      } else {
        reference_intersection_type = IntersectionType::kFullyUnknown;
      }

      const FloatingPoint node_width =
          convert::heightToCellWidth(min_cell_width, query_index.height);
      const Point2D W_node_center =
          convert::indexToCenterPoint(query_index.position, node_width);

      const Point2D W_node_bottom_left =
          W_node_center - Vector2D::Constant(node_width / 2.f);
      const AABB<Point2D> W_cell_aabb{
          W_node_bottom_left,
          W_node_bottom_left + Vector2D::Constant(node_width)};
      const IntersectionType returned_intersection_type =
          range_image_intersector.determineIntersectionType(
              random_pointcloud.getPose(), W_cell_aabb, circular_projector);
      EXPECT_TRUE(reference_intersection_type <= returned_intersection_type)
          << "Expected " << getIntersectionTypeStr(reference_intersection_type)
          << " but got " << getIntersectionTypeStr(returned_intersection_type);
    }
  }
}
}  // namespace wavemap
