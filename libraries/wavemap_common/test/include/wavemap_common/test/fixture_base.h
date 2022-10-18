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

  template <int dim>
  Point<dim> getRandomPoint(FloatingPoint min_distance = 0.f,
                            FloatingPoint max_distance = 5e2) const {
    return getRandomSignedDistance(min_distance, max_distance) *
           Point<dim>::Random().normalized();
  }

  unsigned int getRandomPointcloudSize(unsigned int min_size = 1u,
                                       unsigned int max_size = 1000u) const {
    return random_number_generator_->getRandomInteger(min_size, max_size);
  }

  template <int dim>
  std::vector<Point<dim>> getRandomPointVector() const {
    std::vector<Point<dim>> random_point_vector(getRandomPointcloudSize());
    std::generate(random_point_vector.begin(), random_point_vector.end(),
                  [this]() { return getRandomPoint<dim>(); });
    return random_point_vector;
  }

  FloatingPoint getRandomSignedDistance(
      FloatingPoint min_distance = -4e1,
      FloatingPoint max_distance = 4e1) const {
    return random_number_generator_->getRandomRealNumber(min_distance,
                                                         max_distance);
  }

  template <int dim>
  Vector<dim> getRandomTranslation() const {
    Vector<dim> random_translation;
    for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
      random_translation[dim_idx] = getRandomSignedDistance();
    }
    return random_translation;
  }

  FloatingPoint getRandomAngle(FloatingPoint min_angle = -kPi,
                               FloatingPoint max_angle = kPi) const {
    return random_number_generator_->getRandomRealNumber(min_angle, max_angle);
  }

  template <int dim>
  Transformation<dim> getRandomTransformation() const {
    Transformation<dim> random_transformation;
    random_transformation.setRandom();
    return random_transformation;
  }

  IndexElement getRandomIndexElement(
      const IndexElement min_coordinate = -1e3,
      const IndexElement max_coordinate = 1e3) const {
    return random_number_generator_->getRandomInteger(min_coordinate,
                                                      max_coordinate);
  }

  template <int dim>
  Index<dim> getRandomIndex(
      Index<dim> min_index = Index<dim>::Constant(-1e3),
      Index<dim> max_index = Index<dim>::Constant(1e3)) const {
    Index<dim> random_index;
    for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
      random_index[dim_idx] =
          getRandomIndexElement(min_index[dim_idx], max_index[dim_idx]);
    }
    return random_index;
  }

  template <int dim>
  std::vector<Index<dim>> getRandomIndexVector(
      size_t min_num_indices = 2u, size_t max_num_indices = 100u,
      Index<dim> min_index = Index<dim>::Constant(-1e3),
      Index<dim> max_index = Index<dim>::Constant(1e3)) const {
    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<Index<dim>> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(),
                  [&]() { return getRandomIndex(min_index, max_index); });
    return random_indices;
  }

  template <int dim>
  std::vector<Index<dim>> getRandomIndexVector(
      const Index<dim>& min_index, const Index<dim>& max_index,
      size_t min_num_indices = 2u, size_t max_num_indices = 100u) const {
    CHECK((min_index.array() < max_index.array()).all());

    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<Index<dim>> random_indices(num_indices);
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
