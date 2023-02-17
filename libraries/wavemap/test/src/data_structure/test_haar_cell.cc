#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/container_print_utils.h"

namespace wavemap {
template <typename TypeParamT>
class HaarCellTest : public FixtureBase {
 protected:
  static constexpr FloatingPoint kReconstructionErrorTolerance = 1e-3f;

  FloatingPoint getRandomWaveletCoefficient() const {
    return random_number_generator_->getRandomRealNumber(-1e2f, 1e2f);
  }

  typename HaarCoefficients<typename TypeParamT::ValueType,
                            TypeParamT::kDim>::CoefficientsArray
  getRandomWaveletCoefficientArray() const {
    typename HaarCoefficients<typename TypeParamT::ValueType,
                              TypeParamT::kDim>::CoefficientsArray coefficients;
    std::generate(coefficients.begin(), coefficients.end(),
                  [this]() { return getRandomWaveletCoefficient(); });
    return coefficients;
  }
};

template <typename ValueT, int dim>
struct TypeParamTemplate {
  using ValueType = ValueT;
  static constexpr int kDim = dim;
};
using TypeParams = ::testing::Types<
    TypeParamTemplate<FloatingPoint, 1>, TypeParamTemplate<FloatingPoint, 2>,
    TypeParamTemplate<FloatingPoint, 3>, TypeParamTemplate<FloatingPoint, 4>>;
TYPED_TEST_SUITE(HaarCellTest, TypeParams, );

TYPED_TEST(HaarCellTest, InitializationAndAccessors) {
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;

  // Default (zero) initialization
  {
    typename Coefficients::Parent default_parent;
    EXPECT_EQ(default_parent.scale, 0.f);
    for (int idx = 0; idx < Coefficients::kNumDetailCoefficients; ++idx) {
      EXPECT_EQ(default_parent.details[idx], 0.f);
    }
    EXPECT_EQ(
        static_cast<typename Coefficients::CoefficientsArray>(default_parent),
        typename Coefficients::CoefficientsArray{});
    EXPECT_EQ(default_parent.details, typename Coefficients::Details{});
  }

  // Initialization from array
  constexpr int kNumRepetitions = 10;
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const auto random_parent_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();
    typename Coefficients::Parent random_parent(random_parent_coefficients);
    EXPECT_EQ(
        static_cast<typename Coefficients::CoefficientsArray>(random_parent),
        random_parent_coefficients);
    EXPECT_EQ(random_parent.scale, random_parent_coefficients[0]);
    for (int idx = 0; idx < Coefficients::kNumDetailCoefficients; ++idx) {
      EXPECT_EQ(random_parent.details[idx],
                random_parent_coefficients[idx + 1]);
    }
  }
}

TYPED_TEST(HaarCellTest, ParallelAndLiftedForwardTransformEquivalence) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;

  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const typename Coefficients::CoefficientsArray child_scale_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();
    const typename Coefficients::Parent parallel_parent_coefficients =
        ForwardParallel<typename TypeParam::ValueType, TypeParam::kDim>(
            child_scale_coefficients);
    const typename Coefficients::Parent lifted_parent_coefficients =
        ForwardLifted<typename TypeParam::ValueType, TypeParam::kDim>(
            child_scale_coefficients);

    bool results_are_equal =
        parallel_parent_coefficients.scale - lifted_parent_coefficients.scale <
        TestFixture::kReconstructionErrorTolerance;
    for (NdtreeIndexRelativeChild detail_idx = 0;
         detail_idx < Coefficients::kNumDetailCoefficients; ++detail_idx) {
      if (TestFixture::kReconstructionErrorTolerance <
          parallel_parent_coefficients.details[detail_idx] -
              lifted_parent_coefficients.details[detail_idx]) {
        results_are_equal = false;
      }
    }
    EXPECT_TRUE(results_are_equal)
        << "The parallel implementation's parent coefficients are\n"
        << parallel_parent_coefficients.toString()
        << " and the lifted parent coefficients are\n"
        << lifted_parent_coefficients.toString();
  }
}

TYPED_TEST(HaarCellTest, ParallelAndLiftedBackwardTransformEquivalence) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;

  // Test equivalence of backward transforms
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const typename Coefficients::Parent parent_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();
    const typename Coefficients::CoefficientsArray
        parallel_child_scale_coefficients =
            BackwardParallel<typename TypeParam::ValueType, TypeParam::kDim>(
                parent_coefficients);
    const typename Coefficients::CoefficientsArray
        lifted_child_scale_coefficients =
            BackwardLifted<typename TypeParam::ValueType, TypeParam::kDim>(
                parent_coefficients);

    bool results_are_equal = true;
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < Coefficients::kNumCoefficients; ++child_idx) {
      if (TestFixture::kReconstructionErrorTolerance <
          parallel_child_scale_coefficients[child_idx] -
              lifted_child_scale_coefficients[child_idx]) {
        results_are_equal = false;
      }
    }
    EXPECT_TRUE(results_are_equal)
        << "The parallel implementation's child scales are\n["
        << ToString(parallel_child_scale_coefficients)
        << "] and the lifted child scales are\n["
        << ToString(lifted_child_scale_coefficients) << "]";
  }
}

TYPED_TEST(HaarCellTest, ParentScaleEqualsAverageChildScale) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;
  using Transform =
      HaarTransform<typename TypeParam::ValueType, TypeParam::kDim>;

  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const typename Coefficients::CoefficientsArray child_scale_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();
    const typename Coefficients::Parent parent_coefficients =
        Transform::forward(child_scale_coefficients);

    const FloatingPoint average_of_children =
        std::accumulate(child_scale_coefficients.cbegin(),
                        child_scale_coefficients.cend(), 0.f) /
        static_cast<FloatingPoint>(NdtreeIndex<TypeParam::kDim>::kNumChildren);
    EXPECT_NEAR(parent_coefficients.scale, average_of_children,
                TestFixture::kReconstructionErrorTolerance);
  }
}

TYPED_TEST(HaarCellTest, LosslessReconstruction) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;
  using Transform =
      HaarTransform<typename TypeParam::ValueType, TypeParam::kDim>;

  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const auto child_scale_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();
    const typename Coefficients::Parent parent_coefficients =
        Transform::forward(child_scale_coefficients);
    const typename Coefficients::CoefficientsArray
        roundtrip_child_scale_coefficients =
            Transform::backward(parent_coefficients);

    bool round_trip_equals_original = true;
    for (NdtreeIndexRelativeChild relative_child_idx = 0;
         relative_child_idx < NdtreeIndex<TypeParam::kDim>::kNumChildren;
         ++relative_child_idx) {
      if (TestFixture::kReconstructionErrorTolerance <
          roundtrip_child_scale_coefficients[relative_child_idx] -
              child_scale_coefficients[relative_child_idx]) {
        round_trip_equals_original = false;
      }
    }
    EXPECT_TRUE(round_trip_equals_original)
        << "The full original array was\n["
        << ToString(child_scale_coefficients) << "] and after the roundtrip\n["
        << ToString(roundtrip_child_scale_coefficients) << "]";
  }
}

TYPED_TEST(HaarCellTest, SingleChildForwardTransforms) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;
  using Transform =
      HaarTransform<typename TypeParam::ValueType, TypeParam::kDim>;

  // Test forward transform
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < NdtreeIndex<TypeParam::kDim>::kNumChildren; ++child_idx) {
      const typename Coefficients::Scale child_scale_coefficient =
          TestFixture::getRandomWaveletCoefficient();

      const typename Coefficients::CoefficientsArray parent_from_single_child =
          Transform::forwardSingleChild(child_scale_coefficient, child_idx);

      typename Coefficients::CoefficientsArray child_scale_coefficients{};
      child_scale_coefficients[child_idx] = child_scale_coefficient;
      const typename Coefficients::CoefficientsArray parent_from_sparse_array =
          Transform::forward(child_scale_coefficients);

      bool single_child_transform_equals_regular = true;
      for (NdtreeIndexRelativeChild parent_coefficient_idx = 0;
           parent_coefficient_idx < NdtreeIndex<TypeParam::kDim>::kNumChildren;
           ++parent_coefficient_idx) {
        if (TestFixture::kReconstructionErrorTolerance <
            std::abs(parent_from_single_child[parent_coefficient_idx] -
                     parent_from_sparse_array[parent_coefficient_idx])) {
          single_child_transform_equals_regular = false;
        }
      }
      EXPECT_TRUE(single_child_transform_equals_regular)
          << "The single child transform returned parent coefficients\n["
          << ToString(parent_from_single_child)
          << "] while the regular transform returns\n["
          << ToString(parent_from_sparse_array) << "]";
    }
  }
}

TYPED_TEST(HaarCellTest, SingleChildBackwardTransforms) {
  constexpr int kNumRepetitions = 1000;
  using Coefficients =
      HaarCoefficients<typename TypeParam::ValueType, TypeParam::kDim>;
  using Transform =
      HaarTransform<typename TypeParam::ValueType, TypeParam::kDim>;

  // Test backward transform
  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const typename Coefficients::CoefficientsArray parent_coefficients =
        TestFixture::getRandomWaveletCoefficientArray();

    const typename Coefficients::CoefficientsArray child_scale_array =
        Transform::backward(parent_coefficients);

    for (NdtreeIndexRelativeChild child_idx = 0;
         child_idx < NdtreeIndex<TypeParam::kDim>::kNumChildren; ++child_idx) {
      const typename Coefficients::Scale single_child_scale =
          Transform::backwardSingleChild(parent_coefficients, child_idx);
      EXPECT_NEAR(single_child_scale, child_scale_array[child_idx],
                  TestFixture::kReconstructionErrorTolerance);
    }
  }
}
}  // namespace wavemap
