#ifndef WAVEMAP_COMMON_TEST_FIXTURE_BASE_H_
#define WAVEMAP_COMMON_TEST_FIXTURE_BASE_H_

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/indexing/ndtree_index.h"
#include "wavemap_common/utils/random_number_generator.h"

namespace wavemap {
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

  FloatingPoint getRandomMinCellWidth(
      FloatingPoint min_min_cell_width = 1e-3f,
      FloatingPoint max_min_cell_width = 1e0f) const {
    return random_number_generator_->getRandomRealNumber(min_min_cell_width,
                                                         max_min_cell_width);
  }

  static Point getRandomPoint(FloatingPoint max_distance = 5e2) {
    return max_distance * Point::Random();
  }

  unsigned int getRandomPointcloudSize(unsigned int min_size = 1u,
                                       unsigned int max_size = 1000u) const {
    return random_number_generator_->getRandomInteger(min_size, max_size);
  }

  std::vector<Point> getRandomPointVector() const {
    std::vector<Point> random_point_vector(getRandomPointcloudSize());
    std::generate(random_point_vector.begin(), random_point_vector.end(),
                  []() { return getRandomPoint(); });
    return random_point_vector;
  }

  FloatingPoint getRandomSignedDistance(
      FloatingPoint min_distance = -4e1,
      FloatingPoint max_distance = 4e1) const {
    return random_number_generator_->getRandomRealNumber(min_distance,
                                                         max_distance);
  }

  Vector getRandomTranslation() const {
    return {getRandomSignedDistance(), getRandomSignedDistance()};
  }

  FloatingPoint getRandomAngle(FloatingPoint min_angle = -kPi,
                               FloatingPoint max_angle = kPi) const {
    return random_number_generator_->getRandomRealNumber(min_angle, max_angle);
  }

  IndexElement getRandomIndexElement(
      const IndexElement min_coordinate = -1e3,
      const IndexElement max_coordinate = 1e3) const {
    return random_number_generator_->getRandomInteger(min_coordinate,
                                                      max_coordinate);
  }

  Index getRandomIndex(Index min_index = Index::Constant(-1e3),
                       Index max_index = Index::Constant(1e3)) const {
    return {getRandomIndexElement(min_index.x(), max_index.x()),
            getRandomIndexElement(min_index.y(), max_index.y())};
  }

  std::vector<Index> getRandomIndexVector(
      size_t min_num_indices = 2u, size_t max_num_indices = 100u,
      Index min_index = Index::Constant(-1e3),
      Index max_index = Index::Constant(1e3)) const {
    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<Index> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(min_index, max_index); });
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

  QuadtreeIndex::Element getRandomNdtreeIndexHeight(
      const QuadtreeIndex::Element min_height = 0,
      const QuadtreeIndex::Element max_height = 14) const {
    return random_number_generator_->getRandomInteger(min_height, max_height);
  }

  template <typename NdtreeIndexT>
  std::vector<NdtreeIndexT> getRandomNdtreeIndexVector(
      typename NdtreeIndexT::Position min_index,
      typename NdtreeIndexT::Position max_index,
      typename NdtreeIndexT::Element min_height,
      typename NdtreeIndexT::Element max_height, size_t min_num_indices = 2u,
      size_t max_num_indices = 100u) const {
    CHECK((min_index.array() <= max_index.array()).all());
    CHECK_LE(min_height, max_height);

    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<NdtreeIndexT> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(), [&]() {
      typename NdtreeIndexT::Position position_index;
      for (int i = 0; i < NdtreeIndexT::kDim; ++i) {
        position_index[i] = getRandomIndexElement(min_index[i], max_index[i]);
      }
      return NdtreeIndexT{getRandomNdtreeIndexHeight(min_height, max_height),
                          position_index};
    });
    return random_indices;
  }

  FloatingPoint getRandomUpdate(FloatingPoint min_update = 1e-2f,
                                FloatingPoint max_update = 1e2f) const {
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
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_TEST_FIXTURE_BASE_H_
