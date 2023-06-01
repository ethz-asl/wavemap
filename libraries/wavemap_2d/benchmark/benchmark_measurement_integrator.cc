#include <benchmark/benchmark.h>
#include <wavemap_common/common.h>
#include <wavemap_common/iterator/grid_iterator.h>

namespace wavemap {
static void GridIterationNestedForLoop(benchmark::State& state) {
  // Code before the loop is not measured
  const Index2D bottom_left_idx{-1000, -1000};
  const Index2D top_right_idx{1000, 1000};
  for (auto _ : state) {
    // Code inside this loop is measured repeatedly
    int count = 0;
    for (Index2D index = bottom_left_idx; index.x() <= top_right_idx.x();
         ++index.x()) {
      for (index.y() = bottom_left_idx.y(); index.y() <= top_right_idx.y();
           ++index.y()) {
        benchmark::DoNotOptimize(++count);
      }
    }
    CHECK_EQ(count, 2001 * 2001)
        << "Wrong number of iterations for nested for loop";
  }
}
BENCHMARK(GridIterationNestedForLoop);

static void GridIterationRangeBasedLoop(benchmark::State& state) {
  // Code before the loop is not measured
  const Index2D bottom_left_idx{-1000, -1000};
  const Index2D top_right_idx{1000, 1000};
  for (auto _ : state) {
    // Code inside this loop is measured repeatedly
    const Grid range(bottom_left_idx, top_right_idx);
    int count = 0;
    for ([[maybe_unused]] auto idx : range) {
      benchmark::DoNotOptimize(++count);
    }
    CHECK_EQ(count, 2001 * 2001)
        << "Wrong number of iterations for range based iterator";
  }
}
BENCHMARK(GridIterationRangeBasedLoop);

}  // namespace wavemap

BENCHMARK_MAIN();
