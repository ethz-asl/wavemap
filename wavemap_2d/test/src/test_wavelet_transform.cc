#include <gtest/gtest.h>

#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/transform/discrete_wavelet_transform.h"
#include "wavemap_2d/transform/naive_haar.h"

namespace wavemap_2d {
class WaveletTransformTest : public FixtureBase {
 protected:
  template <typename T = FloatingPoint>
  using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
  using MatrixF = Matrix<FloatingPoint>;

  enum class BandOrientation { LL = 0, LH, HL, HH };
  static std::string getBandOrientationStr(BandOrientation orientation) {
    return std::vector({"LL", "LH", "HL", "HH"})[static_cast<int>(orientation)];
  }
  struct BandKey {
    BandOrientation orientation;
    int pass_idx;

    bool operator<(const BandKey& rhs) const {
      if (pass_idx < rhs.pass_idx) {
        return true;
      } else if (pass_idx == rhs.pass_idx) {
        return orientation < rhs.orientation;
      } else {
        return false;
      }
    }
  };
  using BandKeySet = std::set<BandKey>;
  using BandExpectationMap = std::map<BandKey, FloatingPoint>;

  static BandKeySet bandKeysAfterNPasses(int n_passes) {
    std::set<BandKey> band_keys;
    for (int pass_idx = 1; pass_idx <= n_passes; ++pass_idx) {
      band_keys.emplace(BandKey{BandOrientation::LH, pass_idx});
      band_keys.emplace(BandKey{BandOrientation::HL, pass_idx});
      band_keys.emplace(BandKey{BandOrientation::HH, pass_idx});
    }
    band_keys.emplace(BandKey{BandOrientation::LL, n_passes});
    return band_keys;
  }

  static MatrixF::ConstBlockXpr getBlockForBand(BandKey band_key,
                                                const MatrixF& matrix) {
    const Eigen::Index band_rows =
        matrix.rows() / static_cast<Eigen::Index>(std::exp2(band_key.pass_idx));
    const Eigen::Index band_cols =
        matrix.cols() / static_cast<Eigen::Index>(std::exp2(band_key.pass_idx));
    switch (band_key.orientation) {
      case BandOrientation::LL:
        return matrix.block(0, 0, band_rows, band_cols);
      case BandOrientation::LH:
        return matrix.block(0, band_cols, band_rows, band_cols);
      case BandOrientation::HL:
        return matrix.block(band_rows, 0, band_rows, band_cols);
      case BandOrientation::HH:
        return matrix.block(band_rows, band_cols, band_rows, band_cols);
      default:
        LOG(FATAL) << "Tried to get block for invalid orientation.";
        return matrix.block(0, 0, 0, 0);
    }
  }

  static bool cwiseNear(const MatrixF& matrix, FloatingPoint value) {
    return ((matrix.array() - value).abs() < kEpsilon * (1 + std::abs(value)))
        .all();
  }
  static bool cwiseNear(const MatrixF::ConstBlockXpr& block,
                        FloatingPoint value) {
    return ((block.array() - value).abs() < kEpsilon * (1 + std::abs(value)))
        .all();
  }

  static void expectBands(const MatrixF& matrix, int n_passes,
                          const MatrixF& original_matrix,
                          const BandExpectationMap& band_expectation_map,
                          FloatingPoint default_value = NAN) {
    BandKeySet expected_default_value_bands;
    if (!std::isnan(default_value)) {
      expected_default_value_bands = bandKeysAfterNPasses(n_passes);
    }

    for (const auto& band_expectation_kv : band_expectation_map) {
      const BandKey& band_key = band_expectation_kv.first;
      const FloatingPoint band_expected_value = band_expectation_kv.second;
      if (!std::isnan(band_expected_value)) {
        const MatrixF::ConstBlockXpr band_block =
            getBlockForBand(band_key, matrix);
        EXPECT_TRUE(cwiseNear(band_block, band_expected_value))
            << "Expected all coefficients in band "
            << getBandOrientationStr(band_key.orientation) << " at pass "
            << band_key.pass_idx << " to be equal to expected value "
            << band_expected_value << ", but block equals this instead:\n"
            << band_block << "\nThe original (non decomposed) matrix was:\n"
            << original_matrix << "\nAnd the fully compressed matrix is:\n"
            << matrix;
      }
      expected_default_value_bands.erase(band_key);
    }

    for (const auto& band_key : expected_default_value_bands) {
      const MatrixF::ConstBlockXpr band_block =
          getBlockForBand(band_key, matrix);
      EXPECT_TRUE(cwiseNear(band_block, default_value))
          << "Expected all coefficients in band "
          << getBandOrientationStr(band_key.orientation) << " at pass "
          << band_key.pass_idx << " to be equal to default value "
          << default_value << ", but block equals this instead:\n"
          << band_block << "\nThe original (non decomposed) matrix was:\n"
          << original_matrix << "\nAnd the fully compressed matrix is:\n"
          << matrix;
    }
  }
};

// TODO(victorr): Add tests for odd numbers of rows and columns

TEST_F(WaveletTransformTest, KnownPatterns) {
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
        const MatrixF original_matrix =
            MatrixF::Constant(rows, cols, constant_value);
        MatrixF matrix = original_matrix;

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
          MatrixF matrix = MatrixF::Zero(rows, cols);
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
          const MatrixF original_matrix = matrix;

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
        MatrixF matrix = MatrixF::Zero(rows, cols);
        for (Eigen::Index row_idx = 0; row_idx < rows; ++row_idx) {
          for (Eigen::Index col_idx = 0; col_idx < cols; ++col_idx) {
            matrix(row_idx, col_idx) =
                (static_cast<FloatingPoint>(col_idx % 2) - 0.5f);
          }
        }
        if (transpose) {
          matrix.transposeInPlace();
        }
        const MatrixF original_matrix = matrix;

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
        MatrixF matrix = MatrixF::Zero(rows, cols);
        for (Eigen::Index row_idx = 0; row_idx < rows; ++row_idx) {
          for (Eigen::Index col_idx = 0; col_idx < cols; ++col_idx) {
            if ((row_idx % 2) != (col_idx % 2)) {
              matrix(row_idx, col_idx) = 0.5f;
            } else {
              matrix(row_idx, col_idx) = -0.5;
            }
          }
        }
        const MatrixF original_matrix = matrix;

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

TEST_F(WaveletTransformTest, RandomMatrices) {
  const NaiveHaar<FloatingPoint> dwt;
  constexpr int kNSizeIncrements = 7;
  for (int size_idx = 1; size_idx <= kNSizeIncrements; ++size_idx) {
    const auto rows = static_cast<Eigen::Index>(std::exp2(size_idx));
    const auto cols = static_cast<Eigen::Index>(std::exp2(size_idx));

    // Test perfect reconstruction
    for (int pass_idx = 1; pass_idx <= size_idx; ++pass_idx) {
      // Setup the matrix
      const MatrixF original_matrix = MatrixF::Random(rows, cols);
      MatrixF matrix = original_matrix;

      // Transform back and forth
      dwt.forward(matrix, pass_idx);
      EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
      dwt.backward(matrix, pass_idx);
      EXPECT_TRUE(cwiseNear(matrix - original_matrix, 0))
          << "Expected decode(encode(original_matrix))=original_matrix for "
             "original matrix:\n"
          << original_matrix
          << "\nBut the encoded-decoded matrix equals this instead:\n"
          << matrix;
      EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
    }

    // Transposing before or after encoding should yield same matrix
    for (int pass_idx = 1; pass_idx <= size_idx; ++pass_idx) {
      const MatrixF original_matrix = MatrixF::Random(rows, cols);
      MatrixF matrix = original_matrix;
      MatrixF matrix_transposed = original_matrix.transpose();
      dwt.forward(matrix, pass_idx);
      dwt.forward(matrix_transposed, pass_idx);
      EXPECT_FLOAT_EQ(matrix.norm(), original_matrix.norm());
      EXPECT_FLOAT_EQ(matrix_transposed.norm(), original_matrix.norm());
      EXPECT_TRUE(cwiseNear(matrix - matrix_transposed.transpose(), 0.f));
    }
  }
}
}  // namespace wavemap_2d
