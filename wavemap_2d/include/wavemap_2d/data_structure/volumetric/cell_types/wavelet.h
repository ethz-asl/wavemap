#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_

#include <string>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/ndtree_index.h"

namespace wavemap_2d {
template <typename ValueT>
struct WaveletCoefficients {
  using ValueType = ValueT;

  using Scale = ValueType;
  struct Details {
    ValueType xx{};
    ValueType yy{};
    ValueType xy{};
    friend bool operator==(const Details& lhs, const Details& rhs) {
      return lhs.xx == rhs.xx && lhs.yy == rhs.yy && lhs.xy == rhs.xy;
    }
    friend Details operator+(const Details& lhs, const Details& rhs) {
      return {lhs.xx + rhs.xx, lhs.yy + rhs.yy, lhs.xy + rhs.xy};
    }
    Details& operator+=(const Details& rhs) {
      *this = *this + rhs;
      return *this;
    }
    friend Details operator*(FloatingPoint lhs, const Details& rhs) {
      return {lhs * rhs.xx, lhs * rhs.yy, lhs * rhs.xy};
    }
    friend Details operator*(const Details& lhs, FloatingPoint rhs) {
      return rhs * lhs;
    }
    Details& operator*=(FloatingPoint rhs) {
      *this = *this * rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[xx = " << xx << ", yy = " << yy << ", xy = " << xy << "]";
      return ss.str();
    }
  };

  struct Parent {
    Scale scale{};
    Details details;
    friend bool operator==(const Parent& lhs, const Parent& rhs) {
      return lhs.scale == rhs.scale && lhs.details == rhs.details;
    }
    friend Parent operator+(const Parent& lhs, const Parent& rhs) {
      return {lhs.scale + rhs.scale, lhs.details + rhs.details};
    }
    Parent& operator+=(const Parent& rhs) {
      *this = *this + rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[scale = " << scale << ", details = " << details.toString() << "]";
      return ss.str();
    }
  };

  using Array = std::array<Scale, QuadtreeIndex::kNumChildren>;
  using ChildScales = Array;
};

template <typename ValueT = FloatingPoint>
class HaarWavelet {
 public:
  using ParentCoefficients = typename WaveletCoefficients<ValueT>::Parent;
  using ChildScaleCoefficients =
      typename WaveletCoefficients<ValueT>::ChildScales;

  // Transform 4 scale space coefficients into 1 coarse scale and 3 detail
  // coefficients using the DWT with the orthonormal Haar basis:
  // fn_scale = 1 / Sqrt2 * [1, 1] and fn_details = 1 / Sqrt2 * [-1, 1]
  static ParentCoefficients forward(
      const ChildScaleCoefficients& child_scale_coefficients) {
    // NOTE: We use the lifted implementation because it's as fast as the naive
    //       version but uses less operations, leaving more room for pipelining
    //       of surrounding code, and possibly hyper-threads.
    return forwardLifted(child_scale_coefficients);
  }

  // Transform 1 coarse scale and 3 detail coefficients into 4 scale space
  // coefficients using Inverse DWT with the orthonormal Haar basis:
  // fn_scale = 1 / Sqrt2 * [1, 1] and fn_details = 1 / Sqrt2 * [-1, 1]
  static ChildScaleCoefficients backward(
      const ParentCoefficients& parent_coefficients) {
    // NOTE: We use the naive implementation because it's significantly faster.
    //       Even though the lifted implementation uses less operations, most of
    //       its operations depend on each other's results. This generates long
    //       dependency chains and reduces the potential for pipelining.
    return backwardNaive(parent_coefficients);
  }

 private:
  static ParentCoefficients forwardNaive(
      const ChildScaleCoefficients& child_scale_coefficients) {
    ParentCoefficients parent_coefficients;
    parent_coefficients.scale =
        static_cast<ValueT>(0.5) *
        (child_scale_coefficients[0] + child_scale_coefficients[2] +
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.xx =
        static_cast<ValueT>(0.5) *
        (-child_scale_coefficients[0] - child_scale_coefficients[2] +
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.yy =
        static_cast<ValueT>(0.5) *
        (-child_scale_coefficients[0] + child_scale_coefficients[2] -
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    parent_coefficients.details.xy =
        static_cast<ValueT>(0.5) *
        (child_scale_coefficients[0] - child_scale_coefficients[2] -
         child_scale_coefficients[1] + child_scale_coefficients[3]);
    return parent_coefficients;
  }

  static ChildScaleCoefficients backwardNaive(
      const ParentCoefficients& parent_coefficients) {
    ChildScaleCoefficients child_scale_coeffs;
    child_scale_coeffs[0] =
        static_cast<ValueT>(0.5) *
        (parent_coefficients.scale - parent_coefficients.details.xx -
         parent_coefficients.details.yy + parent_coefficients.details.xy);
    child_scale_coeffs[1] =
        static_cast<ValueT>(0.5) *
        (parent_coefficients.scale + parent_coefficients.details.xx -
         parent_coefficients.details.yy - parent_coefficients.details.xy);
    child_scale_coeffs[2] =
        static_cast<ValueT>(0.5) *
        (parent_coefficients.scale - parent_coefficients.details.xx +
         parent_coefficients.details.yy - parent_coefficients.details.xy);
    child_scale_coeffs[3] =
        static_cast<ValueT>(0.5) *
        (parent_coefficients.scale + parent_coefficients.details.xx +
         parent_coefficients.details.yy + parent_coefficients.details.xy);
    return child_scale_coeffs;
  }

  static ParentCoefficients forwardLifted(
      const ChildScaleCoefficients& child_scale_coefficients) {
    typename WaveletCoefficients<ValueT>::Array coefficient_array =
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
            static_cast<ValueT>(0.5) * coefficient_array[detail_idx];
      }
    }
    // Convert to the orthonormal Haar basis
    // NOTE: This step could be avoided by directly lifting with steps
    //       b = 1 / Sqrt2 * (b - a); a = 1 / Sqrt2 * a + b, but that requires
    //       more compute.
    coefficient_array[0b00] *= static_cast<ValueT>(2.0);
    coefficient_array[0b11] *= static_cast<ValueT>(0.5);

    return {coefficient_array[0],
            {coefficient_array[1], coefficient_array[2], coefficient_array[3]}};
  }

  static ChildScaleCoefficients backwardLifted(
      const ParentCoefficients& parent_coefficients) {
    ChildScaleCoefficients child_scale_coefficients{
        parent_coefficients.scale, parent_coefficients.details.xx,
        parent_coefficients.details.yy, parent_coefficients.details.xy};

    child_scale_coefficients[0b00] *= static_cast<ValueT>(0.5);
    child_scale_coefficients[0b11] *= static_cast<ValueT>(2.0);
    for (QuadtreeIndex::Element dim = 0; dim < QuadtreeIndex::kDim; ++dim) {
      for (QuadtreeIndex::Element pass = 0; pass < QuadtreeIndex::kDim;
           ++pass) {
        const QuadtreeIndex::Element scale_idx = pass << dim;
        const QuadtreeIndex::Element detail_idx =
            (QuadtreeIndex::kDim >> dim) | (pass << dim);
        child_scale_coefficients[scale_idx] =
            child_scale_coefficients[scale_idx] -
            static_cast<ValueT>(0.5) * child_scale_coefficients[detail_idx];
        child_scale_coefficients[detail_idx] =
            child_scale_coefficients[detail_idx] +
            child_scale_coefficients[scale_idx];
      }
    }

    return child_scale_coefficients;
  }

  friend class HaarCellTest_LiftedImplementationEquivalence_Test;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_H_
