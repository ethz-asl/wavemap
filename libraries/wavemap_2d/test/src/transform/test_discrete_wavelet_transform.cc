#include <gtest/gtest.h>

#include "wavemap_2d/test/fixture_wavelet_transforms.h"
#include "wavemap_2d/transform/dense/lifted_cdf_5_3.h"
#include "wavemap_2d/transform/dense/lifted_cdf_9_7.h"
#include "wavemap_2d/transform/dense/naive_haar.h"

namespace wavemap {
template <typename WaveletType>
class DiscreteWaveletTransformTest : public FixtureWaveletTransform {
 protected:
  static constexpr FloatingPoint kReconstructionErrorTolerance = 1e-5f;
};

using WaveletTypes =
    ::testing::Types<NaiveHaar<FloatingPoint>, LiftedCDF53<FloatingPoint>,
                     LiftedCDF97<FloatingPoint>>;
TYPED_TEST_SUITE(DiscreteWaveletTransformTest, WaveletTypes, );

TYPED_TEST(DiscreteWaveletTransformTest, Reconstruction) {
  const TypeParam dwt;
  constexpr int kNSizeIncrements = 9;
  for (int size_idx = 1; size_idx <= kNSizeIncrements; ++size_idx) {
    const auto rows = static_cast<Eigen::Index>(std::exp2(size_idx));
    const auto cols = static_cast<Eigen::Index>(std::exp2(size_idx));

    // Test perfect reconstruction
    for (int pass_idx = 1; pass_idx <= size_idx; ++pass_idx) {
      // Setup the matrix
      const Matrix original_matrix = Matrix::Random(rows, cols);
      Matrix matrix = original_matrix;

      // Transform back and forth
      dwt.forward(matrix, pass_idx);
      if (std::is_same_v<TypeParam, NaiveHaar<typename TypeParam::ValueType>>) {
        // Check if the energy is conserved
        // NOTE: This property is currently only satisfied by the Haar wavelet
        //       implementation. For the other wavelet types, the numerical
        //       errors seem to be too large (or there might be a bug or
        //       incorrect lifting coefficients).
        EXPECT_NEAR(matrix.norm(), original_matrix.norm(),
                    1e-3f * original_matrix.norm());
      }
      dwt.backward(matrix, pass_idx);
      EXPECT_TRUE(
          TestFixture::cwiseNear(matrix - original_matrix, 0,
                                 TestFixture::kReconstructionErrorTolerance))
          << "Expected decode(encode(original_matrix))=original_matrix for "
             "original matrix:\n"
          << original_matrix
          << "\nBut the encoded-decoded matrix equals this instead:\n"
          << matrix << "\nThe non-negligible differences are:\n"
          << (TestFixture::kReconstructionErrorTolerance <=
              ((matrix - original_matrix).array() - 0).abs())
                 .template select(matrix - original_matrix, 0);
      EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
    }
  }
}

TYPED_TEST(DiscreteWaveletTransformTest, Transposes) {
  const TypeParam dwt;
  constexpr int kNSizeIncrements = 7;
  for (int size_idx = 1; size_idx <= kNSizeIncrements; ++size_idx) {
    const auto rows = static_cast<Eigen::Index>(std::exp2(size_idx));
    const auto cols = static_cast<Eigen::Index>(std::exp2(size_idx));

    // Transposing before or after encoding should yield same matrix
    for (int pass_idx = 1; pass_idx <= size_idx; ++pass_idx) {
      const Matrix original_matrix = Matrix::Random(rows, cols);
      Matrix matrix = original_matrix;
      Matrix matrix_transposed = original_matrix.transpose();
      dwt.forward(matrix, pass_idx);
      dwt.forward(matrix_transposed, pass_idx);
      if (std::is_same_v<TypeParam, NaiveHaar<typename TypeParam::ValueType>>) {
        // Check if the energy is conserved
        // NOTE: This property is currently only satisfied by the Haar wavelet
        //       implementation. For the other wavelet types, the numerical
        //       errors seem to be too large (or there might be a bug or
        //       incorrect lifting coefficients).
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
        EXPECT_FLOAT_EQ(matrix_transposed.norm(), original_matrix.norm());
      }
      EXPECT_TRUE(
          TestFixture::cwiseNear(matrix - matrix_transposed.transpose(), 0.f,
                                 TestFixture::kReconstructionErrorTolerance));
    }
  }
}
}  // namespace wavemap
