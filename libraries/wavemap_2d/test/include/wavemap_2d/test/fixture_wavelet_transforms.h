#ifndef WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_WAVELET_TRANSFORMS_H_
#define WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_WAVELET_TRANSFORMS_H_

#include <map>
#include <set>
#include <string>

#include <wavemap_common/test/fixture_base.h>

namespace wavemap {
class FixtureWaveletTransform : public FixtureBase {
 protected:
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

  static Matrix::ConstBlockXpr getBlockForBand(BandKey band_key,
                                               const Matrix& matrix) {
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

  static bool cwiseNear(const Matrix& matrix, FloatingPoint value,
                        FloatingPoint threshold = kEpsilon) {
    return ((matrix.array() - value).abs() < threshold * (1 + std::abs(value)))
        .all();
  }
  static bool cwiseNear(const typename Matrix::ConstBlockXpr& block,
                        FloatingPoint value,
                        FloatingPoint threshold = kEpsilon) {
    return ((block.array() - value).abs() < threshold * (1 + std::abs(value)))
        .all();
  }

  static void expectBands(const Matrix& matrix, int n_passes,
                          const Matrix& original_matrix,
                          const BandExpectationMap& band_expectation_map,
                          FloatingPoint default_value = NAN) {
    BandKeySet expected_default_value_bands;
    if (!std::isnan(default_value)) {
      expected_default_value_bands = bandKeysAfterNPasses(n_passes);
    }

    for (const auto& [band_key, band_expected_value] : band_expectation_map) {
      if (!std::isnan(band_expected_value)) {
        const Matrix::ConstBlockXpr band_block =
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
      const Matrix::ConstBlockXpr band_block =
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
}  // namespace wavemap

#endif  // WAVEMAP_2D_TEST_INCLUDE_WAVEMAP_2D_TEST_FIXTURE_WAVELET_TRANSFORMS_H_
