#include <gtest/gtest.h>

#include "wavemap_2d/test/fixture_wavelet_transforms.h"
#include "wavemap_2d/transform/dense/naive_haar.h"

namespace wavemap_2d {
using NaiveHaarTest = FixtureWaveletTransform;

TEST_F(NaiveHaarTest, KnownPatterns) {
  const NaiveHaar<FloatingPoint> dwt;
  constexpr int kNSizeIncrements = 3;
  for (int size_idx = 1; size_idx <= kNSizeIncrements; ++size_idx) {
    const auto rows = static_cast<Eigen::Index>(std::exp2(size_idx));
    const auto cols = static_cast<Eigen::Index>(std::exp2(size_idx));
    for (int pass_idx = 1; pass_idx <= size_idx; ++pass_idx) {
      DLOG(INFO) << "Evaluating pass " << pass_idx << " / " << size_idx
                 << " for matrix of size [" << rows << ", " << cols << "]"
                 << std::endl;

      // Constant matrices
      for (FloatingPoint constant_value : {0.f, 1.f}) {
        // Setup the matrix
        const Matrix original_matrix =
            Matrix::Constant(rows, cols, constant_value);
        Matrix matrix = original_matrix;

        // Forward test
        dwt.forward(matrix, pass_idx);
        const FloatingPoint power = matrix.norm();
        const FloatingPoint LL_band_num_cells =
            static_cast<FloatingPoint>(matrix.size()) /
            static_cast<FloatingPoint>(std::exp2(2 * pass_idx));
        FloatingPoint LL_band_expected_coef_value =
            std::sqrt(power * power / LL_band_num_cells);
        const BandExpectationMap band_expectation_map{
            {{BandOrientation::LL, pass_idx}, LL_band_expected_coef_value}};
        expectBands(matrix, pass_idx, original_matrix, band_expectation_map,
                    0.f);
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());

        // Backward test
        dwt.backward(matrix, pass_idx);
        EXPECT_TRUE(cwiseNear(matrix - original_matrix, 0))
            << "Expected decode(encode(original_matrix))=original_matrix for "
               "original matrix:\n"
            << original_matrix
            << "\nBut the encoded-decoded matrix equals this instead:\n"
            << matrix;
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
      }

      // Ramp patterns
      const bool is_last_pass = (pass_idx == size_idx);
      if (is_last_pass) {
        for (bool transpose : {false, true}) {
          // Setup the matrix
          Matrix matrix = Matrix::Zero(rows, cols);
          for (Eigen::Index row_idx = 0; row_idx < rows; ++row_idx) {
            for (Eigen::Index col_idx = 0; col_idx < cols; ++col_idx) {
              matrix(row_idx, col_idx) = static_cast<FloatingPoint>(col_idx) -
                                         static_cast<FloatingPoint>(cols) / 2 +
                                         0.5f;
            }
          }
          if (transpose) {
            matrix.transposeInPlace();
          }
          const Matrix original_matrix = matrix;

          // Forward test
          dwt.forward(matrix, pass_idx);
          BandExpectationMap band_expectations_map{
              {{BandOrientation::LL, pass_idx}, 0.f}};
          if (transpose) {
            band_expectations_map[{BandOrientation::HL, 1}] = -1.f;
          } else {
            band_expectations_map[{BandOrientation::LH, 1}] = -1.f;
          }
          expectBands(matrix, pass_idx, original_matrix, band_expectations_map);
          EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());

          // Backward test
          dwt.backward(matrix, pass_idx);
          EXPECT_TRUE(cwiseNear(matrix - original_matrix, 0))
              << "Expected decode(encode(original_matrix))=original_matrix for "
                 "original matrix:\n"
              << original_matrix
              << "\nBut the encoded-decoded matrix equals this instead:\n"
              << matrix;
          EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
        }
      }

      // Striped patterns
      for (bool transpose : {false, true}) {
        // Setup the matrix
        Matrix matrix = Matrix::Zero(rows, cols);
        for (Eigen::Index row_idx = 0; row_idx < rows; ++row_idx) {
          for (Eigen::Index col_idx = 0; col_idx < cols; ++col_idx) {
            matrix(row_idx, col_idx) =
                (static_cast<FloatingPoint>(col_idx % 2) - 0.5f);
          }
        }
        if (transpose) {
          matrix.transposeInPlace();
        }
        const Matrix original_matrix = matrix;

        // Forward test
        dwt.forward(matrix, pass_idx);
        BandExpectationMap band_expectations_map;
        if (transpose) {
          band_expectations_map[{BandOrientation::HL, 1}] = -1.f;
        } else {
          band_expectations_map[{BandOrientation::LH, 1}] = -1.f;
        }
        expectBands(matrix, pass_idx, original_matrix, band_expectations_map,
                    0.f);
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());

        // Backward test
        dwt.backward(matrix, pass_idx);
        EXPECT_TRUE(cwiseNear(matrix - original_matrix, 0))
            << "Expected decode(encode(original_matrix))=original_matrix for "
               "original matrix:\n"
            << original_matrix
            << "\nBut the encoded-decoded matrix equals this instead:\n"
            << matrix;
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
      }

      // Checkerboard patterns
      {
        // Setup the matrix
        Matrix matrix = Matrix::Zero(rows, cols);
        for (Eigen::Index row_idx = 0; row_idx < rows; ++row_idx) {
          for (Eigen::Index col_idx = 0; col_idx < cols; ++col_idx) {
            if ((row_idx % 2) != (col_idx % 2)) {
              matrix(row_idx, col_idx) = 0.5f;
            } else {
              matrix(row_idx, col_idx) = -0.5;
            }
          }
        }
        const Matrix original_matrix = matrix;

        // Forward test
        dwt.forward(matrix, pass_idx);
        BandExpectationMap band_expectations_map{
            {{BandOrientation::HH, 1}, -1.f}};
        expectBands(matrix, pass_idx, original_matrix, band_expectations_map,
                    0.f);
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());

        // Backward test
        dwt.backward(matrix, pass_idx);
        EXPECT_TRUE(cwiseNear(matrix - original_matrix, 0))
            << "Expected decode(encode(original_matrix))=original_matrix for "
               "original matrix:\n"
            << original_matrix
            << "\nBut the encoded-decoded matrix equals this instead:\n"
            << matrix;
        EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
      }
    }
  }
}
}  // namespace wavemap_2d
