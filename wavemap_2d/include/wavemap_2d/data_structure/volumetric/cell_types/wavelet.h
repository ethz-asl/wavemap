#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
// TODO(victorr): Refactor this to make the structure more intuitive
template <typename CoefficientT>
class Wavelet {
 public:
  using CoefficientType = CoefficientT;

  using ScaleCoefficient = CoefficientType;
  struct DetailCoefficients {
    CoefficientType xx{};
    CoefficientType yy{};
    CoefficientType xy{};
    friend bool operator==(const DetailCoefficients& lhs,
                           const DetailCoefficients& rhs) {
      return lhs.xx == rhs.xx && lhs.yy == rhs.yy && lhs.xy == rhs.xy;
    }
    friend DetailCoefficients operator+(const DetailCoefficients& lhs,
                                        const DetailCoefficients& rhs) {
      return {lhs.xx + rhs.xx, lhs.yy + rhs.yy, lhs.xy + rhs.xy};
    }
    DetailCoefficients& operator+=(const DetailCoefficients& rhs) {
      *this = *this + rhs;
      return *this;
    }
    friend DetailCoefficients operator*(FloatingPoint lhs,
                                        const DetailCoefficients& rhs) {
      return {lhs * rhs.xx, lhs * rhs.yy, lhs * rhs.xy};
    }
    friend DetailCoefficients operator*(const DetailCoefficients& lhs,
                                        FloatingPoint rhs) {
      return rhs * lhs;
    }
    DetailCoefficients& operator*=(FloatingPoint rhs) {
      *this = *this * rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[xx = " << xx << ", yy = " << yy << ", xy = " << xy << "]";
      return ss.str();
    }
  };

  struct ParentCoefficients {
    ScaleCoefficient scale{};
    DetailCoefficients details;
    friend bool operator==(const ParentCoefficients& lhs,
                           const ParentCoefficients& rhs) {
      return lhs.scale == rhs.scale && lhs.details == rhs.details;
    }
    friend ParentCoefficients operator+(const ParentCoefficients& lhs,
                                        const ParentCoefficients& rhs) {
      return {lhs.scale + rhs.scale, lhs.details + rhs.details};
    }
    ParentCoefficients& operator+=(const ParentCoefficients& rhs) {
      *this = *this + rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[scale = " << scale << ", details = " << details.toString() << "]";
      return ss.str();
    }
  };

  using CoefficientArray =
      std::array<ScaleCoefficient, QuadtreeIndex::kNumChildren>;
  using ChildScaleCoefficients = CoefficientArray;
};

template <typename CoefficientT = FloatingPoint>
class NaiveHaarWavelet : public Wavelet<CoefficientT> {
 public:
  using typename Wavelet<CoefficientT>::ParentCoefficients;
  using typename Wavelet<CoefficientT>::ChildScaleCoefficients;

  static ParentCoefficients forward(
      const ChildScaleCoefficients& child_scale_coefficients) {
    ParentCoefficients parent_coefficients;
    parent_coefficients.scale =
        static_cast<CoefficientT>(0.5) *
        (child_scale_coefficients[0] + child_scale_coefficients[2] +
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.xx =
        static_cast<CoefficientT>(0.5) *
        (-child_scale_coefficients[0] - child_scale_coefficients[2] +
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.yy =
        static_cast<CoefficientT>(0.5) *
        (-child_scale_coefficients[0] + child_scale_coefficients[2] -
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.xy =
        static_cast<CoefficientT>(0.5) *
        (child_scale_coefficients[0] - child_scale_coefficients[2] -
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    return parent_coefficients;
  }

  static ChildScaleCoefficients backward(
      const ParentCoefficients& parent_coefficients) {
    ChildScaleCoefficients child_scale_coeffs;
    child_scale_coeffs[0] =
        static_cast<CoefficientT>(0.5) *
        (parent_coefficients.scale - parent_coefficients.details.xx -
         parent_coefficients.details.yy + parent_coefficients.details.xy);
    child_scale_coeffs[1] =
        static_cast<CoefficientT>(0.5) *
        (parent_coefficients.scale + parent_coefficients.details.xx -
         parent_coefficients.details.yy - parent_coefficients.details.xy);
    child_scale_coeffs[2] =
        static_cast<CoefficientT>(0.5) *
        (parent_coefficients.scale - parent_coefficients.details.xx +
         parent_coefficients.details.yy - parent_coefficients.details.xy);
    child_scale_coeffs[3] =
        static_cast<CoefficientT>(0.5) *
        (parent_coefficients.scale + parent_coefficients.details.xx +
         parent_coefficients.details.yy + parent_coefficients.details.xy);
    return child_scale_coeffs;
  }
};

template <typename CoefficientT = FloatingPoint>
class LiftedHaarWavelet : public Wavelet<CoefficientT> {
 public:
  using typename Wavelet<CoefficientT>::ParentCoefficients;
  using typename Wavelet<CoefficientT>::ChildScaleCoefficients;

  // Transform 4 scale space coefficients into 1 coarse scale and 3 detail
  // coefficients using the orthonormal Haar basis:
  // fn_scale = 1 / Sqrt2 * [1, 1] and fn_details = 1 / Sqrt2 * [-1, 1]
  static ParentCoefficients forward(
      const ChildScaleCoefficients& child_scale_coefficients) {
    typename Wavelet<CoefficientT>::CoefficientArray coefficient_array =
        child_scale_coefficients;

    // Apply the wavelet transform one row/col at a time using lifting
    // NOTE: This lifts into basis s=[0.5, 0.5]; d=[-1, 1].
    for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
      for (QuadtreeIndex::Element pass = 0; pass < QuadtreeIndex::kDim;
           ++pass) {
        const QuadtreeIndex::Element scale_idx = pass << dim;
        const QuadtreeIndex::Element detail_idx =
            (QuadtreeIndex::kDim >> dim) | (pass << dim);
        coefficient_array[detail_idx] =
            coefficient_array[detail_idx] - coefficient_array[scale_idx];
        coefficient_array[scale_idx] =
            coefficient_array[scale_idx] +
            static_cast<CoefficientT>(0.5) * coefficient_array[detail_idx];
      }
    }
    // Convert to the orthonormal Haar basis
    // NOTE: This step could be avoided by directly lifting with steps
    //       b = 1 / Sqrt2 * (b - a); a = 1 / Sqrt2 * a + b, but that requires
    //       more compute.
    coefficient_array[0b00] *= static_cast<CoefficientT>(2.0);
    coefficient_array[0b11] *= static_cast<CoefficientT>(0.5);

    return {coefficient_array[0],
            {coefficient_array[1], coefficient_array[2], coefficient_array[3]}};
  }

  static ChildScaleCoefficients backward(
      const ParentCoefficients& parent_coefficients) {
    ChildScaleCoefficients child_scale_coefficients{
        parent_coefficients.scale, parent_coefficients.details.xx,
        parent_coefficients.details.yy, parent_coefficients.details.xy};

    child_scale_coefficients[0b00] *= static_cast<CoefficientT>(0.5);
    child_scale_coefficients[0b11] *= static_cast<CoefficientT>(2.0);
    for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
      for (QuadtreeIndex::Element pass = 0; pass < QuadtreeIndex::kDim;
           ++pass) {
        const QuadtreeIndex::Element scale_idx = pass << dim;
        const QuadtreeIndex::Element detail_idx =
            (QuadtreeIndex::kDim >> dim) | (pass << dim);
        child_scale_coefficients[scale_idx] =
            child_scale_coefficients[scale_idx] -
            static_cast<CoefficientT>(0.5) *
                child_scale_coefficients[detail_idx];
        child_scale_coefficients[detail_idx] =
            child_scale_coefficients[detail_idx] +
            child_scale_coefficients[scale_idx];
      }
    }

    return child_scale_coefficients;
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_
