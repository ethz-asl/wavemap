#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/iterator/grid_iterator.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_3d/integrator/projective/coarse_to_fine/hierarchical_range_image_2d.h"

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
      range_image[index] = getRandomSignedDistance(kMinDistance, kMaxDistance);
    }
    return range_image;
  }
};

TEST_F(HierarchicalRangeImage2DTest, PyramidConstruction) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random hierarchical range image
    constexpr bool kAzimuthAllowedToWrapAround = false;
    auto range_image = std::make_shared<RangeImage2D>(getRandomRangeImage());
    const Index2D range_image_dims = range_image->getDimensions();
    HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>
        hierarchical_range_image(range_image);

    // Test all the bounds from top to bottom
    const NdtreeIndexElement max_height =
        hierarchical_range_image.getMaxHeight();
    for (NdtreeIndexElement height = 0; height <= max_height; ++height) {
      const Index2D current_level_dims =
          int_math::div_exp2_ceil(range_image_dims, height);
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
            << "For index " << index.toString() << " and range image width "
            << range_image_dims;
        EXPECT_EQ(hierarchical_range_image.getBounds(index).upper,
                  hierarchical_range_image.getUpperBound(index))
            << "For index " << index.toString() << " and range image width "
            << range_image_dims;

        // Check if the values returned by the accessors are correct
        if (index.height == 0) {
          // At the leaf level the bounds should match range image itself
          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          range_image->operator[](index.position));
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          range_image->operator[](index.position));
        } else if (index.height == 1) {
          // At the first pyramid level, the bounds should correspond to min/max
          // pooling the range image with a downsampling factor of 2
          Bounds<FloatingPoint> child_bounds;
          for (NdtreeIndexRelativeChild relative_child_idx = 0;
               relative_child_idx < QuadtreeIndex::kNumChildren;
               ++relative_child_idx) {
            const QuadtreeIndex child_idx =
                index.computeChildIndex(relative_child_idx);
            if ((child_idx.position.array() < range_image_dims.array()).all()) {
              const FloatingPoint range_image_value =
                  range_image->operator[](child_idx.position);
              child_bounds.lower =
                  std::min(child_bounds.lower, range_image_value);
              child_bounds.upper =
                  std::max(child_bounds.upper, range_image_value);
            } else {
              child_bounds.lower = std::min(
                  child_bounds.lower,
                  HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>::
                      getUnknownRangeImageValueLowerBound());
              child_bounds.upper = std::max(
                  child_bounds.upper,
                  HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>::
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
                 int_math::div_exp2_ceil(range_image_dims, index.height - 1)
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
              child_bounds.lower = std::min(
                  child_bounds.lower,
                  HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>::
                      getUnknownRangeImageValueLowerBound());
              child_bounds.upper = std::max(
                  child_bounds.upper,
                  HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>::
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
    constexpr bool kAzimuthAllowedToWrapAround = false;
    auto range_image = std::make_shared<RangeImage2D>(getRandomRangeImage());
    const Index2D range_image_dims = range_image->getDimensions();
    HierarchicalRangeImage2D<kAzimuthAllowedToWrapAround>
        hierarchical_range_image(range_image);

    // Test range bounds on all sub-intervals and compare to brute force
    for (const Index2D& start_idx :
         Grid<2>(Index2D::Zero(), range_image_dims - Index2D::Ones())) {
      for (Index2D end_idx = start_idx; end_idx.x() < range_image_dims.x();
           ++end_idx.x()) {
        for (end_idx.y() = range_image_dims.y();
             end_idx.y() < range_image_dims.y(); ++end_idx.y()) {
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
          const Index2D range_dims = end_idx - start_idx + Index2D::Ones();
          CHECK((1 <= range_dims.array()).all());
          const RangeImage2D::Data::ConstBlockXpr range =
              range_image->getData().block(start_idx.x(), start_idx.y(),
                                           range_dims.x(), range_dims.y());
          const FloatingPoint lower_bound_brute_force = range.minCoeff();
          const FloatingPoint upper_bound_brute_force = range.maxCoeff();
          EXPECT_LE(lower_bound, lower_bound_brute_force);
          EXPECT_GE(upper_bound, upper_bound_brute_force);
        }
      }
    }
  }
}
}  // namespace wavemap
