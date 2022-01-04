#ifndef WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_
#define WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
class FixtureBase : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr size_t kFixedRandomSeed = 0u;
    random_number_generator_ =
        std::make_unique<RandomNumberGenerator>(kFixedRandomSeed);
  }

  FloatingPoint getRandomResolution() const {
    constexpr FloatingPoint kMinResolution = 1e-3;
    constexpr FloatingPoint kMaxResolution = 1e0;
    return random_number_generator_->getRandomRealNumber(kMinResolution,
                                                         kMaxResolution);
  }

  static Point getRandomPoint() {
    constexpr FloatingPoint kMaxCoordinate = 1e3;
    return kMaxCoordinate * Point::Random();
  }
  FloatingPoint getRandomSignedDistance() {
    constexpr FloatingPoint kMinDistance = -4e1;
    constexpr FloatingPoint kMaxDistance = 4e1;
    return random_number_generator_->getRandomRealNumber(kMinDistance,
                                                         kMaxDistance);
  }
  Vector getRandomTranslation() {
    return {getRandomSignedDistance(), getRandomSignedDistance()};
  }
  FloatingPoint getRandomAngle() const {
    constexpr FloatingPoint kMinAngle = -M_PI;
    constexpr FloatingPoint kMaxAngle = M_PI;
    return random_number_generator_->getRandomRealNumber(kMinAngle, kMaxAngle);
  }

  IndexElement getRandomIndexElement() const {
    constexpr IndexElement kMinCoordinate = -1e3;
    constexpr IndexElement kMaxCoordinate = 1e3;
    return random_number_generator_->getRandomInteger(kMinCoordinate,
                                                      kMaxCoordinate);
  }
  Index getRandomIndex() const {
    return {getRandomIndexElement(), getRandomIndexElement()};
  }
  std::vector<Index> getRandomIndexVector() const {
    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);
    std::vector<Index> random_indices;
    random_indices.reserve(num_indices);
    for (size_t idx = 0u; idx < num_indices; ++idx) {
      random_indices.emplace_back(getRandomIndex());
    }
    return random_indices;
  }

  FloatingPoint getRandomUpdate() const {
    constexpr FloatingPoint kMinUpdate = 1e-2;
    constexpr FloatingPoint kMaxUpdate = 1e2;
    return random_number_generator_->getRandomRealNumber(kMinUpdate,
                                                         kMaxUpdate);
  }
  std::vector<FloatingPoint> getRandomUpdateVector() const {
    constexpr size_t kMinNumUpdates = 0u;
    constexpr size_t kMaxNumUpdates = 100u;
    const size_t num_updates = random_number_generator_->getRandomInteger(
        kMinNumUpdates, kMaxNumUpdates);
    std::vector<FloatingPoint> random_updates(num_updates);
    std::generate(random_updates.begin(), random_updates.end(),
                  [this]() { return getRandomUpdate(); });
    return random_updates;
  }

  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_
