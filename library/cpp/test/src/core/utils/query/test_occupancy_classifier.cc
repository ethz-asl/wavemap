#include <gtest/gtest.h>

#include "wavemap/core/utils/query/occupancy_classifier.h"

namespace wavemap {
TEST(OccupancyClassifierTest, Initialization) {
  // Check that default occupancy threshold is zero
  EXPECT_NEAR(OccupancyClassifier{}.getOccupancyThreshold(), 0.f, kEpsilon);
  // Check that non-default occupancy thresholds get set correctly
  EXPECT_NEAR(OccupancyClassifier{1.f}.getOccupancyThreshold(), 1.f, kEpsilon);
}

TEST(OccupancyClassifierTest, CellOccupancy) {
  // Static observed/unobserved testers
  EXPECT_FALSE(OccupancyClassifier::isUnobserved(-1.f));
  EXPECT_TRUE(OccupancyClassifier::isUnobserved(0.f));
  EXPECT_FALSE(OccupancyClassifier::isUnobserved(1.f));
  EXPECT_TRUE(OccupancyClassifier::isObserved(-1.f));
  EXPECT_FALSE(OccupancyClassifier::isObserved(0.f));
  EXPECT_TRUE(OccupancyClassifier::isObserved(1.f));

  // Non-static methods
  // Default threshold
  EXPECT_TRUE(OccupancyClassifier{}.is(-1.f, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier{}.is(-1.f, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier{}.is(-1.f, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier{}.is(-1.f, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier{}.is(0.f, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier{}.is(0.f, Occupancy::kOccupied));
  EXPECT_TRUE(OccupancyClassifier{}.is(0.f, Occupancy::kUnobserved));
  EXPECT_FALSE(OccupancyClassifier{}.is(0.f, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier{}.is(1.f, Occupancy::kFree));
  EXPECT_TRUE(OccupancyClassifier{}.is(1.f, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier{}.is(1.f, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier{}.is(1.f, Occupancy::kObserved));
  // Non-default threshold
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(-1.1f, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(-1.1f, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(-1.1f, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(-1.1f, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(-0.9f, Occupancy::kFree));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(-0.9f, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(-0.9f, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(-0.9f, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(0.f, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(0.f, Occupancy::kOccupied));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(0.f, Occupancy::kUnobserved));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(0.f, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(1.f, Occupancy::kFree));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(1.f, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier{-1.f}.is(1.f, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier{-1.f}.is(1.f, Occupancy::kObserved));
}

TEST(OccupancyClassifierTest, RegionOccupancy) {
  constexpr Occupancy::Mask kFullyFree = Occupancy::toMask(Occupancy::kFree);
  EXPECT_TRUE(OccupancyClassifier::has(kFullyFree, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier::has(kFullyFree, Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier::has(kFullyFree, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier::has(kFullyFree, Occupancy::kObserved));
  EXPECT_TRUE(OccupancyClassifier::isFully(kFullyFree, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier::isFully(kFullyFree, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFullyFree, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier::isFully(kFullyFree, Occupancy::kObserved));

  constexpr Occupancy::Mask kFullyOccupied =
      Occupancy::toMask(Occupancy::kOccupied);
  EXPECT_FALSE(OccupancyClassifier::has(kFullyOccupied, Occupancy::kFree));
  EXPECT_TRUE(OccupancyClassifier::has(kFullyOccupied, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::has(kFullyOccupied, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier::has(kFullyOccupied, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier::isFully(kFullyOccupied, Occupancy::kFree));
  EXPECT_TRUE(
      OccupancyClassifier::isFully(kFullyOccupied, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFullyOccupied, Occupancy::kUnobserved));
  EXPECT_TRUE(
      OccupancyClassifier::isFully(kFullyOccupied, Occupancy::kObserved));

  constexpr Occupancy::Mask kFullyUnobserved =
      Occupancy::toMask(Occupancy::kUnobserved);
  EXPECT_FALSE(OccupancyClassifier::has(kFullyUnobserved, Occupancy::kFree));
  EXPECT_FALSE(
      OccupancyClassifier::has(kFullyUnobserved, Occupancy::kOccupied));
  EXPECT_TRUE(
      OccupancyClassifier::has(kFullyUnobserved, Occupancy::kUnobserved));
  EXPECT_FALSE(
      OccupancyClassifier::has(kFullyUnobserved, Occupancy::kObserved));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFullyUnobserved, Occupancy::kFree));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFullyUnobserved, Occupancy::kOccupied));
  EXPECT_TRUE(
      OccupancyClassifier::isFully(kFullyUnobserved, Occupancy::kUnobserved));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFullyUnobserved, Occupancy::kObserved));

  constexpr Occupancy::Mask kFreeOrUnobserved =
      Occupancy::toMask(Occupancy::kFree) |
      Occupancy::toMask(Occupancy::kUnobserved);
  EXPECT_TRUE(OccupancyClassifier::has(kFreeOrUnobserved, Occupancy::kFree));
  EXPECT_FALSE(
      OccupancyClassifier::has(kFreeOrUnobserved, Occupancy::kOccupied));
  EXPECT_TRUE(
      OccupancyClassifier::has(kFreeOrUnobserved, Occupancy::kUnobserved));
  EXPECT_TRUE(
      OccupancyClassifier::has(kFreeOrUnobserved, Occupancy::kObserved));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrUnobserved, Occupancy::kFree));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrUnobserved, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrUnobserved, Occupancy::kUnobserved));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrUnobserved, Occupancy::kObserved));

  constexpr Occupancy::Mask kOccupiedOrUnobserved =
      Occupancy::toMask(Occupancy::kOccupied) |
      Occupancy::toMask(Occupancy::kUnobserved);
  EXPECT_FALSE(
      OccupancyClassifier::has(kOccupiedOrUnobserved, Occupancy::kFree));
  EXPECT_TRUE(
      OccupancyClassifier::has(kOccupiedOrUnobserved, Occupancy::kOccupied));
  EXPECT_TRUE(
      OccupancyClassifier::has(kOccupiedOrUnobserved, Occupancy::kUnobserved));
  EXPECT_TRUE(
      OccupancyClassifier::has(kOccupiedOrUnobserved, Occupancy::kObserved));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kOccupiedOrUnobserved, Occupancy::kFree));
  EXPECT_FALSE(OccupancyClassifier::isFully(kOccupiedOrUnobserved,
                                            Occupancy::kOccupied));
  EXPECT_FALSE(OccupancyClassifier::isFully(kOccupiedOrUnobserved,
                                            Occupancy::kUnobserved));
  EXPECT_FALSE(OccupancyClassifier::isFully(kOccupiedOrUnobserved,
                                            Occupancy::kObserved));

  constexpr Occupancy::Mask kFreeOrOccupied =
      Occupancy::toMask(Occupancy::kFree) |
      Occupancy::toMask(Occupancy::kOccupied);
  EXPECT_TRUE(OccupancyClassifier::has(kFreeOrOccupied, Occupancy::kFree));
  EXPECT_TRUE(OccupancyClassifier::has(kFreeOrOccupied, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::has(kFreeOrOccupied, Occupancy::kUnobserved));
  EXPECT_TRUE(OccupancyClassifier::has(kFreeOrOccupied, Occupancy::kObserved));
  EXPECT_FALSE(OccupancyClassifier::isFully(kFreeOrOccupied, Occupancy::kFree));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrOccupied, Occupancy::kOccupied));
  EXPECT_FALSE(
      OccupancyClassifier::isFully(kFreeOrOccupied, Occupancy::kUnobserved));
  EXPECT_TRUE(
      OccupancyClassifier::isFully(kFreeOrOccupied, Occupancy::kObserved));
}
}  // namespace wavemap
