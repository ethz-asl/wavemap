#include <benchmark/benchmark.h>

#include "wavemap/core/map/cell_types/haar_coefficients.h"
#include "wavemap/core/map/cell_types/haar_transform.h"
#include "wavemap/core/utils/random_number_generator.h"

namespace wavemap {
template <typename ValueT, int dim>
typename HaarCoefficients<ValueT, dim>::CoefficientsArray
GenerateRandomCoefficientsArray() {
  RandomNumberGenerator random_number_generator;
  typename HaarCoefficients<ValueT, dim>::CoefficientsArray coefficients_array;
  std::generate(coefficients_array.begin(), coefficients_array.end(),
                [&random_number_generator]() {
                  return random_number_generator.template getRandomRealNumber(
                      -1e2, 1e2);
                });
  return coefficients_array;
}

template <typename ValueT, int dim>
static void ForwardParallelTransform(benchmark::State& state) {
  const typename HaarCoefficients<ValueT, dim>::CoefficientsArray child_scales =
      GenerateRandomCoefficientsArray<ValueT, dim>();
  for (auto _ : state) {
    benchmark::DoNotOptimize(ForwardParallel<ValueT, dim>(child_scales));
  }
}

template <typename ValueT, int dim>
static void ForwardLiftedTransform(benchmark::State& state) {
  const typename HaarCoefficients<ValueT, dim>::CoefficientsArray child_scales =
      GenerateRandomCoefficientsArray<ValueT, dim>();
  for (auto _ : state) {
    benchmark::DoNotOptimize(ForwardLifted<ValueT, dim>(child_scales));
  }
}

BENCHMARK_TEMPLATE(ForwardParallelTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(ForwardLiftedTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(ForwardParallelTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(ForwardLiftedTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(ForwardParallelTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(ForwardLiftedTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(ForwardParallelTransform, FloatingPoint, 4);
BENCHMARK_TEMPLATE(ForwardLiftedTransform, FloatingPoint, 4);

template <typename ValueT, int dim>
static void BackwardParallelTransform(benchmark::State& state) {
  const typename HaarCoefficients<ValueT, dim>::Parent parent(
      GenerateRandomCoefficientsArray<ValueT, dim>());
  for (auto _ : state) {
    benchmark::DoNotOptimize(BackwardParallel<ValueT, dim>(parent));
  }
}

template <typename ValueT, int dim>
static void BackwardLiftedTransform(benchmark::State& state) {
  const typename HaarCoefficients<ValueT, dim>::Parent parent(
      GenerateRandomCoefficientsArray<ValueT, dim>());
  for (auto _ : state) {
    benchmark::DoNotOptimize(BackwardLifted<ValueT, dim>(parent));
  }
}

BENCHMARK_TEMPLATE(BackwardParallelTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(BackwardLiftedTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(BackwardParallelTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(BackwardLiftedTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(BackwardParallelTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(BackwardLiftedTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(BackwardParallelTransform, FloatingPoint, 4);
BENCHMARK_TEMPLATE(BackwardLiftedTransform, FloatingPoint, 4);

template <typename ValueT, int dim>
static void ForwardSingleChildTransform(benchmark::State& state) {
  RandomNumberGenerator random_number_generator;
  const typename HaarCoefficients<ValueT, dim>::Scale child_scale =
      random_number_generator.template getRandomRealNumber(-1e2, 1e2);
  const NdtreeIndexRelativeChild child_idx =
      random_number_generator.template getRandomInteger(
          0, HaarCoefficients<ValueT, dim>::kNumCoefficients - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        ForwardSingleChild<ValueT, dim>(child_scale, child_idx));
  }
}

template <typename ValueT, int dim>
static void ForwardSingleChildMemberTransform(benchmark::State& state) {
  RandomNumberGenerator random_number_generator;
  const typename HaarCoefficients<ValueT, dim>::Scale child_scale =
      random_number_generator.template getRandomRealNumber(-1e2, 1e2);
  const NdtreeIndexRelativeChild child_idx =
      random_number_generator.template getRandomInteger(
          0, HaarCoefficients<ValueT, dim>::kNumCoefficients - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        HaarTransform<ValueT, dim>::forwardSingleChild(child_scale, child_idx));
  }
}

// TODO(victorr): Also include and benchmark against manual versions up to 3D to
//                see if GCC is able to optimize the templated vers equally well

BENCHMARK_TEMPLATE(ForwardSingleChildTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(ForwardSingleChildMemberTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(ForwardSingleChildTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(ForwardSingleChildMemberTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(ForwardSingleChildTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(ForwardSingleChildMemberTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(ForwardSingleChildTransform, FloatingPoint, 4);
BENCHMARK_TEMPLATE(ForwardSingleChildMemberTransform, FloatingPoint, 4);

template <typename ValueT, int dim>
static void BackwardSingleChildTransform(benchmark::State& state) {
  RandomNumberGenerator random_number_generator;
  typename HaarCoefficients<ValueT, dim>::Parent parent(
      GenerateRandomCoefficientsArray<ValueT, dim>());
  const NdtreeIndexRelativeChild child_idx =
      random_number_generator.template getRandomInteger(
          0, HaarCoefficients<ValueT, dim>::kNumCoefficients - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        BackwardSingleChild<ValueT, dim>(parent, child_idx));
  }
}

template <typename ValueT, int dim>
static void BackwardSingleChildMemberTransform(benchmark::State& state) {
  RandomNumberGenerator random_number_generator;
  typename HaarCoefficients<ValueT, dim>::Parent parent(
      GenerateRandomCoefficientsArray<ValueT, dim>());
  const NdtreeIndexRelativeChild child_idx =
      random_number_generator.template getRandomInteger(
          0, HaarCoefficients<ValueT, dim>::kNumCoefficients - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        HaarTransform<ValueT, dim>::backwardSingleChild(parent, child_idx));
  }
}

BENCHMARK_TEMPLATE(BackwardSingleChildTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(BackwardSingleChildMemberTransform, FloatingPoint, 1);
BENCHMARK_TEMPLATE(BackwardSingleChildTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(BackwardSingleChildMemberTransform, FloatingPoint, 2);
BENCHMARK_TEMPLATE(BackwardSingleChildTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(BackwardSingleChildMemberTransform, FloatingPoint, 3);
BENCHMARK_TEMPLATE(BackwardSingleChildTransform, FloatingPoint, 4);
BENCHMARK_TEMPLATE(BackwardSingleChildMemberTransform, FloatingPoint, 4);
}  // namespace wavemap

BENCHMARK_MAIN();
