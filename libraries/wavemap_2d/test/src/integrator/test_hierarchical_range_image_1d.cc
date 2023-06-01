#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_2d/integrator/projective/coarse_to_fine/hierarchical_range_image_1d.h"

namespace wavemap {
class HierarchicalRangeImage1DTest : public FixtureBase {
 protected:
  RangeImage1D getRandomRangeImage() {
    const int num_beams = getRandomIndexElement(100, 2048);
    constexpr FloatingPoint kMinDistance = 0.f;
    constexpr FloatingPoint kMaxDistance = 30.f;
    RangeImage1D range_image(num_beams);
    for (IndexElement idx = 0; idx < range_image.getNumBeams(); ++idx) {
      range_image.getRange(idx) =
          getRandomSignedDistance(kMinDistance, kMaxDistance);
    }
    return range_image;
  }
};

TEST_F(HierarchicalRangeImage1DTest, PyramidConstruction) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random hierarchical range image
    auto range_image = std::make_shared<RangeImage1D>(getRandomRangeImage());
    const IndexElement num_beams = range_image->getNumBeams();
    HierarchicalRangeImage1D hierarchical_range_image(range_image);

    // Test all the bounds from top to bottom
    const BinaryTreeIndex::Element max_height =
        hierarchical_range_image.getMaxHeight();
    for (BinaryTreeIndex index{0, BinaryTreeIndex::Position::Zero()};
         index.height <= max_height; ++index.height) {
      const BinaryTreeIndex::Element num_elements_at_level =
          int_math::div_exp2_ceil(num_beams, index.height);
      for (index.position.x() = 0; index.position.x() < num_elements_at_level;
           ++index.position.x()) {
        // Avoid out-of-bounds range image access when we're at the leaf level
        if (index.height == 0 &&
            range_image->getNumBeams() <= index.position.x()) {
          continue;
        }

        // Check if the different accessors return the same values
        EXPECT_EQ(hierarchical_range_image.getBounds(index).lower,
                  hierarchical_range_image.getLowerBound(index))
            << "For index " << index.toString() << " and range image width "
            << range_image->getNumBeams();
        EXPECT_EQ(hierarchical_range_image.getBounds(index).upper,
                  hierarchical_range_image.getUpperBound(index))
            << "For index " << index.toString() << " and range image width "
            << range_image->getNumBeams();

        // Check if the values returned by the accessors are correct
        if (index.height == 0) {
          // At the leaf level the bounds should match range image itself
          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          range_image->getRange(index.position.x()));
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          range_image->getRange(index.position.x()));
        } else if (index.height == 1) {
          // At the first pyramid level, the bounds should correspond to min/max
          // pooling the range image with a downsampling factor of 2
          Bounds<FloatingPoint> child_bounds;
          const BinaryTreeIndex::Element first_child_idx =
              index.computeChildIndex(0).position.x();
          const BinaryTreeIndex::Element second_child_idx =
              index.computeChildIndex(1).position.x();
          if (second_child_idx < range_image->getNumBeams()) {
            child_bounds.lower =
                std::min(range_image->getRange(first_child_idx),
                         range_image->getRange(second_child_idx));
            child_bounds.upper =
                std::max(range_image->getRange(first_child_idx),
                         range_image->getRange(second_child_idx));
          } else if (first_child_idx < range_image->getNumBeams()) {
            child_bounds.lower =
                std::min(range_image->getRange(first_child_idx),
                         HierarchicalRangeImage1D::
                             getUnknownRangeImageValueLowerBound());
            child_bounds.upper =
                std::max(range_image->getRange(first_child_idx),
                         HierarchicalRangeImage1D::
                             getUnknownRangeImageValueUpperBound());
          } else {
            child_bounds.lower =
                HierarchicalRangeImage1D::getUnknownRangeImageValueLowerBound();
            child_bounds.upper =
                HierarchicalRangeImage1D::getUnknownRangeImageValueUpperBound();
          }

          EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                          child_bounds.lower);
          EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                          child_bounds.upper);
        } else {
          // At all other levels, the bounds correspond to min/max the bounds of
          // the previous level
          const BinaryTreeIndex first_child_idx = index.computeChildIndex(0);
          const BinaryTreeIndex second_child_idx = index.computeChildIndex(1);
          const bool second_child_exists =
              second_child_idx.position.x() <
              int_math::div_exp2_ceil(num_beams, index.height - 1);
          if (second_child_exists) {
            EXPECT_FLOAT_EQ(
                hierarchical_range_image.getLowerBound(index),
                std::min(
                    hierarchical_range_image.getLowerBound(first_child_idx),
                    hierarchical_range_image.getLowerBound(second_child_idx)))
                << "For index " << index.toString() << " and range image with "
                << range_image->getNumBeams() << " beams";
            EXPECT_FLOAT_EQ(
                hierarchical_range_image.getUpperBound(index),
                std::max(
                    hierarchical_range_image.getUpperBound(first_child_idx),
                    hierarchical_range_image.getUpperBound(second_child_idx)))
                << "For index " << index.toString() << " and range image with "
                << range_image->getNumBeams() << " beams";
          } else {
            EXPECT_FLOAT_EQ(hierarchical_range_image.getLowerBound(index),
                            std::min(hierarchical_range_image.getLowerBound(
                                         first_child_idx),
                                     HierarchicalRangeImage1D::
                                         getUnknownRangeImageValueLowerBound()))
                << "For index " << index.toString() << " and range image with "
                << range_image->getNumBeams() << " beams";
            EXPECT_FLOAT_EQ(hierarchical_range_image.getUpperBound(index),
                            std::max(hierarchical_range_image.getUpperBound(
                                         first_child_idx),
                                     HierarchicalRangeImage1D::
                                         getUnknownRangeImageValueUpperBound()))
                << "For index " << index.toString() << " and range image with "
                << range_image->getNumBeams() << " beams";
          }
        }
      }
    }
  }
}

TEST_F(HierarchicalRangeImage1DTest, RangeBoundQueries) {
  for (int repetition = 0; repetition < 3; ++repetition) {
    // Generate a random hierarchical range image
    auto range_image = std::make_shared<RangeImage1D>(getRandomRangeImage());
    const IndexElement num_beams = range_image->getNumBeams();
    HierarchicalRangeImage1D hierarchical_range_image(range_image);

    // Test range bounds on all sub-intervals and compare to brute force
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
        const RangeImage1D::Data::ConstBlockXpr range =
            range_image->getData().block(0, start_idx, 1, range_length);
        const FloatingPoint lower_bound_brute_force = range.minCoeff();
        const FloatingPoint upper_bound_brute_force = range.maxCoeff();
        EXPECT_LE(lower_bound, lower_bound_brute_force)
            << "On range [" << start_idx << ", " << end_idx
            << "] in range image with " << num_beams << " beams";
        EXPECT_GE(upper_bound, upper_bound_brute_force)
            << "On range [" << start_idx << ", " << end_idx
            << "] in range image with " << num_beams << " beams";
      }
    }
  }
}
}  // namespace wavemap
