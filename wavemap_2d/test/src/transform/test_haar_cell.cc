#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/haar_wavelet.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/container_print_utils.h"

namespace wavemap_2d {
class HaarCellTest : public FixtureBase {
 protected:
  using HaarWaveletType = HaarWavelet<FloatingPoint>;
  static constexpr FloatingPoint kReconstructionErrorTolerance = 1e-3f;

  FloatingPoint getRandomWaveletCoefficient() const {
    return random_number_generator_->getRandomRealNumber(-1e2f, 1e2f);
  }

  HaarWaveletType::ChildScaleCoefficients getRandomChildScaleCoefficients()
      const {
    HaarWaveletType::ChildScaleCoefficients child_scale_coefficients;
    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      child_scale_coefficients[relative_child_idx] =
          getRandomWaveletCoefficient();
    }
    return child_scale_coefficients;
  }

  HaarWaveletType::ParentCoefficients getRandomParentCoefficients() const {
    HaarWaveletType::ParentCoefficients parent_coefficients;
    parent_coefficients.scale = getRandomWaveletCoefficient();
    parent_coefficients.details.xx = getRandomWaveletCoefficient();
    parent_coefficients.details.yy = getRandomWaveletCoefficient();
    parent_coefficients.details.xy = getRandomWaveletCoefficient();
    return parent_coefficients;
  }
};

TEST_F(HaarCellTest, Initialization) {
  using WaveletCoefficientsType = WaveletCoefficients<FloatingPoint>;
  WaveletCoefficientsType::Details detail_coefficients;
  EXPECT_EQ(detail_coefficients.xx, 0.f);
  EXPECT_EQ(detail_coefficients.yy, 0.f);
  EXPECT_EQ(detail_coefficients.xy, 0.f);

  HaarWaveletType::ParentCoefficients parent_coefficients;
  EXPECT_EQ(parent_coefficients.scale, 0.f);
  EXPECT_EQ(parent_coefficients.details, WaveletCoefficientsType::Details{});

  HaarWaveletType::ChildScaleCoefficients child_scale_coefficients{};
  ASSERT_EQ(child_scale_coefficients.size(), QuadtreeIndex::kNumChildren);
  for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < QuadtreeIndex::kNumChildren; ++relative_child_idx) {
    EXPECT_EQ(child_scale_coefficients[relative_child_idx], 0.f);
  }
}

TEST_F(HaarCellTest, LosslessReconstruction) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const HaarWaveletType::ChildScaleCoefficients child_scale_coefficients =
        getRandomChildScaleCoefficients();
    const HaarWaveletType::ParentCoefficients parent_coefficients =
        HaarWaveletType::forward(child_scale_coefficients);
    const HaarWaveletType::ChildScaleCoefficients
        roundtrip_child_scale_coefficients =
            HaarWaveletType::backward(parent_coefficients);

    bool check_failed = false;
    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      EXPECT_NEAR(roundtrip_child_scale_coefficients[relative_child_idx],
                  child_scale_coefficients[relative_child_idx],
                  kReconstructionErrorTolerance)
          << (check_failed = true);
    }
    if (check_failed) {
      std::cerr << "The full original array was { "
                << ToString(child_scale_coefficients)
                << " } and after the roundtrip { "
                << ToString(roundtrip_child_scale_coefficients) << " }"
                << std::endl;
    }
  }
}

TEST_F(HaarCellTest, ConservationOfEnergy) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const HaarWaveletType::ChildScaleCoefficients child_scale_coefficients =
        getRandomChildScaleCoefficients();
    const HaarWaveletType::ParentCoefficients parent_coefficients =
        HaarWaveletType::forward(child_scale_coefficients);

    // Check that the parent's scale matches the average of its children
    // NOTE: The average of the children's scale coefficients corresponds to 0.5
    //       times scale coefficient of the parent since we use the orthonormal
    //       2D Haar basis whose scaling function magnitude is 0.5 (i.e. it is
    //       orthogonal and satisfies 4 * 0.5^2 = 1). More generally, in N
    //       dimensions, the scaling function magnitude is (1 / sqrt(2))^N.
    const FloatingPoint average_of_children =
        std::accumulate(child_scale_coefficients.begin(),
                        child_scale_coefficients.end(), 0.f) /
        static_cast<FloatingPoint>(QuadtreeIndex::kNumChildren);
    EXPECT_NEAR(0.5f * parent_coefficients.scale, average_of_children,
                kReconstructionErrorTolerance);

    // Check that the parent's coefficients preserve the energy of its children
    const FloatingPoint child_scale_coefficients_energy = std::inner_product(
        child_scale_coefficients.begin(), child_scale_coefficients.end(),
        child_scale_coefficients.begin(), 1.f);
    const FloatingPoint parent_coefficients_energy =
        parent_coefficients.scale * parent_coefficients.scale +
        parent_coefficients.details.xx * parent_coefficients.details.xx +
        parent_coefficients.details.yy * parent_coefficients.details.yy +
        parent_coefficients.details.xy * parent_coefficients.details.xy;
    EXPECT_NEAR(parent_coefficients_energy, child_scale_coefficients_energy,
                1e-2f * child_scale_coefficients_energy);
  }
}

TEST_F(HaarCellTest, LiftedImplementationEquivalence) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const HaarWaveletType::ChildScaleCoefficients child_scale_coefficients =
        getRandomChildScaleCoefficients();
    const HaarWaveletType::ParentCoefficients naive_parent_coefficients =
        HaarWaveletType::forwardNaive(child_scale_coefficients);
    const HaarWaveletType::ParentCoefficients lifted_parent_coefficients =
        HaarWaveletType::forwardLifted(child_scale_coefficients);

    bool check_failed = false;
    EXPECT_NEAR(lifted_parent_coefficients.scale,
                naive_parent_coefficients.scale, kReconstructionErrorTolerance)
        << (check_failed = true);
    EXPECT_NEAR(lifted_parent_coefficients.details.xx,
                naive_parent_coefficients.details.xx,
                kReconstructionErrorTolerance)
        << (check_failed = true);
    EXPECT_NEAR(lifted_parent_coefficients.details.yy,
                naive_parent_coefficients.details.yy,
                kReconstructionErrorTolerance)
        << (check_failed = true);
    EXPECT_NEAR(lifted_parent_coefficients.details.xy,
                naive_parent_coefficients.details.xy,
                kReconstructionErrorTolerance)
        << (check_failed = true);
    if (check_failed) {
      std::cerr << "The naive implementation's parent coefficients are "
                << naive_parent_coefficients.toString()
                << " and the lifted coefficients are "
                << lifted_parent_coefficients.toString() << std::endl;
    }
  }
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const HaarWaveletType::ParentCoefficients parent_coefficients =
        getRandomParentCoefficients();
    const HaarWaveletType::ChildScaleCoefficients
        naive_child_scale_coefficients =
            HaarWaveletType::backwardNaive(parent_coefficients);
    const HaarWaveletType::ChildScaleCoefficients
        lifted_child_scale_coefficients =
            HaarWaveletType::backwardLifted(parent_coefficients);

    bool check_failed = false;
    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      EXPECT_NEAR(lifted_child_scale_coefficients[relative_child_idx],
                  naive_child_scale_coefficients[relative_child_idx],
                  kReconstructionErrorTolerance)
          << (check_failed = true);
    }
    if (check_failed) {
      std::cerr << "The naive implementation's parent coefficients are { "
                << ToString(naive_child_scale_coefficients)
                << " } and the lifted coefficients are { "
                << ToString(lifted_child_scale_coefficients) << " }"
                << std::endl;
    }
  }
}

TEST_F(HaarCellTest, SingleChildTransforms) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      const HaarWaveletType::Coefficients::Scale child_scale_coefficient =
          getRandomWaveletCoefficient();

      const HaarWaveletType::ParentCoefficients parent_from_single_child =
          HaarWaveletType::forwardSingleChild(child_scale_coefficient,
                                              relative_child_idx);

      HaarWaveletType::Coefficients::ChildScales child_scale_coefficients{};
      child_scale_coefficients[relative_child_idx] = child_scale_coefficient;
      const HaarWaveletType::ParentCoefficients parent_from_sparse_array =
          HaarWaveletType::forward(child_scale_coefficients);

      EXPECT_EQ(parent_from_single_child, parent_from_sparse_array);
    }
  }

  for (int repetition = 0; repetition < 1000; ++repetition) {
    const HaarWaveletType::ParentCoefficients parent_coefficients =
        getRandomParentCoefficients();

    const HaarWaveletType::ChildScaleCoefficients child_scale_array =
        HaarWaveletType::backward(parent_coefficients);

    for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
         relative_child_idx < QuadtreeIndex::kNumChildren;
         ++relative_child_idx) {
      const HaarWaveletType::Coefficients::Scale single_child_scale =
          HaarWaveletType::backwardSingleChild(parent_coefficients,
                                               relative_child_idx);
      EXPECT_EQ(single_child_scale, child_scale_array[relative_child_idx]);
    }
  }
}
}  // namespace wavemap_2d
