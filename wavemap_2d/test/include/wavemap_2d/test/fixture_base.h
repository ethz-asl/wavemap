#ifndef WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_
#define WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/indexing/quadtree_index.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
// TODO(victorr): Consider organizing the helper methods into classes by topic,
//                s.t. test suites can pull in only the method groups they need
//                using composition
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
    return getRandomIndexElement(kMinCoordinate, kMaxCoordinate);
  }

  IndexElement getRandomIndexElement(const IndexElement min_coordinate,
                                     const IndexElement max_coordinate) const {
    return random_number_generator_->getRandomInteger(min_coordinate,
                                                      max_coordinate);
  }

  Index getRandomIndex() const {
    return {getRandomIndexElement(), getRandomIndexElement()};
  }

  Index getRandomIndex(const Index& min_index, const Index& max_index) const {
    return {getRandomIndexElement(min_index.x(), max_index.x()),
            getRandomIndexElement(min_index.y(), max_index.y())};
  }

  std::vector<Index> getRandomIndexVector() const {
    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);

    std::vector<Index> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(); });
    return random_indices;
  }

  std::vector<Index> getRandomIndexVector(const Index& min_index,
                                          const Index& max_index) const {
    CHECK((min_index.array() < max_index.array()).all());

    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);

    std::vector<Index> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(min_index, max_index); });
    return random_indices;
  }

  NodeIndexElement getRandomQuadtreeIndexDepth() const {
    constexpr NodeIndexElement kMinDepth = 0;
    constexpr NodeIndexElement kMaxDepth = 14;
    return getRandomQuadtreeIndexDepth(kMinDepth, kMaxDepth);
  }

  NodeIndexElement getRandomQuadtreeIndexDepth(
      const NodeIndexElement min_depth,
      const NodeIndexElement max_depth) const {
    return random_number_generator_->getRandomInteger(min_depth, max_depth);
  }

  std::vector<QuadtreeIndex> getRandomQuadtreeIndexVector(
      const Index& min_index, const Index& max_index,
      NodeIndexElement min_depth, NodeIndexElement max_depth) const {
    CHECK((min_index.array() < max_index.array()).all());
    CHECK_LE(min_depth, max_depth);

    constexpr size_t kMinNumIndices = 2u;
    constexpr size_t kMaxNumIndices = 100u;
    const size_t num_indices = random_number_generator_->getRandomInteger(
        kMinNumIndices, kMaxNumIndices);

    std::vector<QuadtreeIndex> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(), [&]() {
      return QuadtreeIndex{
          .depth = getRandomQuadtreeIndexDepth(min_depth, max_depth),
          .position = getRandomIndex(min_index, max_index)};
    });
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
