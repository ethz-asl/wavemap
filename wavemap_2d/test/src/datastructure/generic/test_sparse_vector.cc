#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/generic/sparse_vector.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
TEST(SparseVectorTest, Insertion) {
  using ElementType = size_t;
  constexpr uint8_t kMaxSize = 64u;
  constexpr ElementType kDefaultValue = 0u;
  constexpr size_t kFixedRandomSeed = 0u;

  for (int percentage_non_zero = 0; percentage_non_zero <= 100;
       percentage_non_zero += 10) {
    const FloatingPoint prob_non_zero =
        static_cast<FloatingPoint>(percentage_non_zero) / 100.f;
    std::vector<ElementType> dense_vector(kMaxSize, kDefaultValue);
    SparseVector<ElementType, kMaxSize> sparse_vector;

    EXPECT_EQ(sparse_vector.size(), 0);
    EXPECT_EQ(sparse_vector.capacity(), 0);

    size_t num_inserted_elements = 0;
    RandomNumberGenerator random_number_generator(kFixedRandomSeed);
    for (uint8_t idx = 0u; idx < kMaxSize; ++idx) {
      const bool is_non_zero =
          random_number_generator.getRandomBool(prob_non_zero);
      if (is_non_zero) {
        ++num_inserted_elements;
        sparse_vector[idx] = idx;
        dense_vector[idx] = idx;
      }
    }

    for (int idx = 0; idx < kMaxSize; ++idx) {
      EXPECT_EQ(sparse_vector.at(idx), dense_vector[idx]);
    }

    EXPECT_EQ(sparse_vector.size(), num_inserted_elements);
    sparse_vector.shrink_to_fit();
    EXPECT_EQ(sparse_vector.capacity(), num_inserted_elements);
  }
}
}  // namespace wavemap_2d
