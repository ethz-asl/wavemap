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

  FloatingPoint getRandomResolution(FloatingPoint min_resolution = 1e-3,
                                    FloatingPoint max_resolution = 1e0) const {
    return random_number_generator_->getRandomRealNumber(min_resolution,
                                                         max_resolution);
  }

  static Point getRandomPoint(FloatingPoint max_distance = 5e2) {
    return max_distance * Point::Random();
  }

  FloatingPoint getRandomSignedDistance(
      FloatingPoint min_distance = -4e1,
      FloatingPoint max_distance = 4e1) const {
    return random_number_generator_->getRandomRealNumber(min_distance,
                                                         max_distance);
  }

  Vector getRandomTranslation() {
    return {getRandomSignedDistance(), getRandomSignedDistance()};
  }

  FloatingPoint getRandomAngle(FloatingPoint min_angle = -M_PI,
                               FloatingPoint max_angle = M_PI) const {
    return random_number_generator_->getRandomRealNumber(min_angle, max_angle);
  }

  IndexElement getRandomIndexElement(
      const IndexElement min_coordinate = -1e3,
      const IndexElement max_coordinate = 1e3) const {
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

  std::vector<Index> getRandomIndexVector(size_t min_num_indices = 2u,
                                          size_t max_num_indices = 100u) const {
    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<Index> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(); });
    return random_indices;
  }

  std::vector<Index> getRandomIndexVector(const Index& min_index,
                                          const Index& max_index,
                                          size_t min_num_indices = 2u,
                                          size_t max_num_indices = 100u) const {
    CHECK((min_index.array() < max_index.array()).all());

    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<Index> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(min_index, max_index); });
    return random_indices;
  }

  NodeIndexElement getRandomQuadtreeIndexDepth(
      const NodeIndexElement min_depth = 0,
      const NodeIndexElement max_depth = 14) const {
    return random_number_generator_->getRandomInteger(min_depth, max_depth);
  }

  std::vector<QuadtreeIndex> getRandomQuadtreeIndexVector(
      const Index& min_index, const Index& max_index,
      NodeIndexElement min_depth, NodeIndexElement max_depth,
      size_t min_num_indices = 2u, size_t max_num_indices = 100u) const {
    CHECK((min_index.array() < max_index.array()).all());
    CHECK_LE(min_depth, max_depth);

    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<QuadtreeIndex> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(), [&]() {
      return QuadtreeIndex{
          .depth = getRandomQuadtreeIndexDepth(min_depth, max_depth),
          .position = getRandomIndex(min_index, max_index)};
    });
    return random_indices;
  }

  FloatingPoint getRandomUpdate(FloatingPoint min_update = 1e-2,
                                FloatingPoint max_update = 1e2) const {
    return random_number_generator_->getRandomRealNumber(min_update,
                                                         max_update);
  }

  std::vector<FloatingPoint> getRandomUpdateVector(
      size_t min_num_updates = 0u, size_t max_num_updates = 100u) const {
    const size_t num_updates = random_number_generator_->getRandomInteger(
        min_num_updates, max_num_updates);
    std::vector<FloatingPoint> random_updates(num_updates);
    std::generate(random_updates.begin(), random_updates.end(),
                  [this]() { return getRandomUpdate(); });
    return random_updates;
  }

  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_BASE_H_
