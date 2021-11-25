#include <gtest/gtest.h>

#include "wavemap_2d/integrator/beam_model.h"
#include "wavemap_2d/integrator/grid_iterator.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
class IteratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    random_number_generator_ = std::make_unique<RandomNumberGenerator>();
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

 private:
  std::unique_ptr<RandomNumberGenerator> random_number_generator_;
};

TEST_F(IteratorTest, GridIterator) {
  const Index bottom_left_idx = -getRandomIndex().cwiseAbs();
  const Index top_right_idx = getRandomIndex().cwiseAbs();

  Grid grid(bottom_left_idx, top_right_idx);
  Grid::Iterator grid_it = grid.begin();
  const Grid::Iterator grid_it_end = grid.end();

  size_t count = 0u;
  for (Index index = bottom_left_idx; index.x() <= top_right_idx.x();
       ++index.x()) {
    for (index.y() = bottom_left_idx.y(); index.y() <= top_right_idx.y();
         ++index.y()) {
      EXPECT_NE(grid_it, grid_it_end);
      EXPECT_EQ(*grid_it, index);
      ++grid_it;
      ++count;
    }
  }
  const size_t num_grid_indices =
      (top_right_idx - bottom_left_idx + Index::Ones()).array().prod();
  EXPECT_EQ(count, num_grid_indices);
  EXPECT_EQ(grid_it, grid_it_end);
}
}  // namespace wavemap_2d
