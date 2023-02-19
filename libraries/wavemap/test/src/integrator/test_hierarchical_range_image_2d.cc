#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/integrator/projective/coarse_to_fine/hierarchical_range_bounds_2d.h"
#include "wavemap/iterator/grid_iterator.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
class HierarchicalRangeImage2DTest : public FixtureBase {
 protected:
  RangeImage2D getRandomRangeImage() {
    const int num_rows = getRandomIndexElement(100, 2048);
    const int num_cols = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    RangeImage2D range_image(num_rows, num_cols);
    for (const Index2D& index : Grid<2>(
             Index2D::Zero(), range_image.getDimensions() - Index2D::Ones())) {
      range_image.getRange(index) =
          getRandomSignedDistance(kMinDistance, kMaxDistance);
    }
    return range_image;
  }
};

TEST_F(HierarchicalRangeImage2DTest, PyramidConstruction) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random hierarchical range image
    constexpr bool kAzimuthMayWrap = false;
    constexpr FloatingPoint kPointcloudIntegratorMinRange = 0.5f;
    auto range_image = std::make_shared<RangeImage2D>(getRandomRangeImage());
    const Index2D range_image_dims = range_image->getDimensions();
    const Index2D range_image_dims_scaled = range_image_dims.cwiseProduct(
        HierarchicalRangeBounds2D::getImageToPyramidScaleFactor());
    HierarchicalRangeBounds2D hierarchical_range_image(
        range_image, kAzimuthMayWrap, kPointcloudIntegratorMinRange);

    // Test all the bounds from top to bottom
    const NdtreeIndexElement max_height =
        hierarchical_range_image.getMaxHeight();
    for (NdtreeIndexElement height = 0; height <= max_height; ++height) {
      const Index2D current_level_dims =
          int_math::div_exp2_ceil(range_image_dims_scaled, height);
      for (const Index2D& position :
           Grid<2>(Index2D::Zero(), current_level_dims - Index2D::Ones())) {
        QuadtreeIndex index{height, position};
        // Avoid out-of-bounds range image access when we're at the leaf level
        if (index.height == 0 &&
            (range_image_dims.array() <= index.position.array()).any()) {
          continue;
        }

        // Check if the different accessors return the same values
        EXPECT_EQ(hierarchical_range_image.getBounds(index).lower,
                  hierarchical_range_image.getLowerBound(index))
            << "For index " << index.toString() << ", range image size "
            << range_image_dims << " and scale factor "
            << HierarchicalRangeBounds2D::getImageToPyramidScaleFactor();
        EXPECT_EQ(hierarchical_range_image.getBounds(index).upper,
                  hierarchical_range_image.getUpperBound(index))
            << "For index " << index.toString() << ", range image width "
            << range_image_dims << " and scale factor "
            << HierarchicalRangeBounds2D::getImageToPyramidScaleFactor();

        // Check if the values returned by the accessors are correct
        if (index.height == 0) {
          // At the leaf level the bounds should match range image itself
          if ((index.position.array() < range_image_dims.array()).all()) {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            range_image->getRange(index.position));
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            range_image->getRange(index.position));
          }
        } else if (index.height == 1) {
          // At the first pyramid level, the bounds should correspond to min/max
          // pooling the range image with a downsampling factor of 2
          Bounds<FloatingPoint> child_bounds;
          for (NdtreeIndexRelativeChild relative_child_idx = 0;
               relative_child_idx < QuadtreeIndex::kNumChildren;
               ++relative_child_idx) {
            QuadtreeIndex child_idx =
                index.computeChildIndex(relative_child_idx);
            child_idx.position = child_idx.position.cwiseQuotient(
                HierarchicalRangeBounds2D::getImageToPyramidScaleFactor());
            if ((child_idx.position.array() < range_image_dims.array()).all()) {
              const FloatingPoint range_image_value =
                  range_image->getRange(child_idx.position);
              if (range_image_value < hierarchical_range_image.getMinRange()) {
                child_bounds.lower =
                    std::min(child_bounds.lower,
                             HierarchicalRangeBounds2D::
                                 getUnknownRangeImageValueLowerBound());
                child_bounds.upper =
                    std::max(child_bounds.upper,
                             HierarchicalRangeBounds2D::
                                 getUnknownRangeImageValueUpperBound());
              } else {
                child_bounds.lower =
                    std::min(child_bounds.lower, range_image_value);
                child_bounds.upper =
                    std::max(child_bounds.upper, range_image_value);
              }
            } else {
              child_bounds.lower =
                  std::min(child_bounds.lower,
                           HierarchicalRangeBounds2D::
                               getUnknownRangeImageValueLowerBound());
              child_bounds.upper =
                  std::max(child_bounds.upper,
                           HierarchicalRangeBounds2D::
                               getUnknownRangeImageValueUpperBound());
            }
          }

          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          child_bounds.lower);
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          child_bounds.upper);
        } else {
          // At all other levels, the bounds correspond to min/max the bounds of
          // the previous level
          Bounds<FloatingPoint> child_bounds;
          for (NdtreeIndexRelativeChild relative_child_idx = 0;
               relative_child_idx < QuadtreeIndex::kNumChildren;
               ++relative_child_idx) {
            const QuadtreeIndex child_idx =
                index.computeChildIndex(relative_child_idx);
            const bool child_exists =
                (child_idx.position.array() <
                 int_math::div_exp2_ceil(range_image_dims_scaled,
                                         index.height - 1)
                     .array())
                    .all();
            if (child_exists) {
              child_bounds.lower =
                  std::min(child_bounds.lower,
                           hierarchical_range_image.getLowerBound(child_idx));
              child_bounds.upper =
                  std::max(child_bounds.upper,
                           hierarchical_range_image.getUpperBound(child_idx));
            } else {
              child_bounds.lower =
                  std::min(child_bounds.lower,
                           HierarchicalRangeBounds2D::
                               getUnknownRangeImageValueLowerBound());
              child_bounds.upper =
                  std::max(child_bounds.upper,
                           HierarchicalRangeBounds2D::
                               getUnknownRangeImageValueUpperBound());
            }
          }

          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          child_bounds.lower);
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          child_bounds.upper);
        }
      }
    }
  }
}

TEST_F(HierarchicalRangeImage2DTest, RangeBoundQueries) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random hierarchical range image
    constexpr bool kAzimuthMayWrap = false;
    constexpr FloatingPoint kPointcloudIntegratorMinRange = 0.5f;
    auto range_image = std::make_shared<RangeImage2D>(getRandomRangeImage());
    const Index2D range_image_dims = range_image->getDimensions();
    HierarchicalRangeBounds2D hierarchical_range_image(
        range_image, kAzimuthMayWrap, kPointcloudIntegratorMinRange);

    // Test range bounds on all sub-intervals and compare to brute force
    for (int subrange_idx = 0; subrange_idx < 1000; ++subrange_idx) {
      // Get bottom left and upper right corners of random sub-interval
      const Index2D start_idx = FixtureBase::getRandomIndex<2>(
          Index2D::Zero(), range_image_dims - Index2D::Ones());
      const Index2D end_idx = FixtureBase::getRandomIndex<2>(
          start_idx, range_image_dims - Index2D::Ones());

      // Check if the different accessors return the same values
      const Bounds bounds =
          hierarchical_range_image.getBounds(start_idx, end_idx);
      const FloatingPoint lower_bound =
          hierarchical_range_image.getLowerBound(start_idx, end_idx);
      const FloatingPoint upper_bound =
          hierarchical_range_image.getUpperBound(start_idx, end_idx);
      EXPECT_LE(lower_bound, upper_bound);
      EXPECT_EQ(bounds.lower, lower_bound);
      EXPECT_EQ(bounds.upper, upper_bound);

      // Compare against brute force
      Bounds<FloatingPoint> bounds_brute_force;
      for (const Index2D& index : Grid(start_idx, end_idx)) {
        const FloatingPoint range_value = range_image->getRange(index);
        if (hierarchical_range_image.getMinRange() < range_value) {
          bounds_brute_force.lower =
              std::min(bounds_brute_force.lower, range_value);
          bounds_brute_force.upper =
              std::max(bounds_brute_force.upper, range_value);
        } else {
          bounds_brute_force.lower = std::min(
              bounds_brute_force.lower,
              HierarchicalRangeBounds2D::getUnknownRangeImageValueLowerBound());
          bounds_brute_force.upper = std::max(
              bounds_brute_force.upper,
              HierarchicalRangeBounds2D::getUnknownRangeImageValueUpperBound());
        }
      }
      EXPECT_LE(lower_bound, bounds_brute_force.lower);
      EXPECT_GE(upper_bound, bounds_brute_force.upper);
    }
  }
}
}  // namespace wavemap
