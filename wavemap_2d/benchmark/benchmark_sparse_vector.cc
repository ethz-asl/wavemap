#include <benchmark/benchmark.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/datastructure/generic/sparse_vector.h"
#include "wavemap_2d/utils/random_number_generator.h"

namespace wavemap_2d {
static void AccessDenseArray(benchmark::State& state) {
  // Code before the loop is not measured
  RandomNumberGenerator random_number_generator;
  constexpr uint8_t kMaxSize = 64u;

  std::vector<size_t> dense_vector(kMaxSize);
  std::iota(dense_vector.begin(), dense_vector.end(), 0);

  for (auto _ : state) {
    // Code inside this loop is measured repeatedly
    size_t random_idx =
        random_number_generator.getRandomInteger(0, kMaxSize - 1);
    size_t value = dense_vector[random_idx];
    benchmark::DoNotOptimize(value);
  }
}
BENCHMARK(AccessDenseArray);

static void AccessSparseArray(benchmark::State& state) {
  // Code before the loop is not measured
  RandomNumberGenerator random_number_generator;
  constexpr uint8_t kMaxSize = 64u;
  const FloatingPoint kPNonZero =
      static_cast<FloatingPoint>(state.range(0)) / 100.f;

  SparseVector<size_t, kMaxSize> sparse_vector;
  for (uint8_t idx = 0u; idx < kMaxSize; ++idx) {
    const bool is_non_zero = random_number_generator.getRandomBool(kPNonZero);
    if (is_non_zero) {
      sparse_vector[idx] = idx;
    }
  }

  for (auto _ : state) {
    // Code inside this loop is measured repeatedly
    size_t random_idx =
        random_number_generator.getRandomInteger(0, kMaxSize - 1);
    size_t value = sparse_vector.at(random_idx);
    benchmark::DoNotOptimize(value);
  }
}
BENCHMARK(AccessSparseArray)->DenseRange(0, 100, 10);
}  // namespace wavemap_2d
