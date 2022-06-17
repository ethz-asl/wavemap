#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_COEFFICIENTS_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_COEFFICIENTS_H_

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
    friend Details operator*(ValueType lhs, const Details& rhs) {
      return {lhs * rhs.xx, lhs * rhs.yy, lhs * rhs.xy};
    }
    friend Details operator*(const Details& lhs, ValueType rhs) {
      return rhs * lhs;
    }
    Details& operator*=(ValueType rhs) {
      *this = *this * rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[xx = " << xx << ", yy = " << yy << ", xy = " << xy << "]";
      return ss.str();
    }
  };

  using Array = std::array<Scale, QuadtreeIndex::kNumChildren>;
  using ChildScales = Array;

  class Parent {
   public:
    Scale scale{};
    Details details;

    Parent() = default;
    Parent(Scale scale, const Details& details)
        : scale(scale), details(details) {}
    explicit Parent(const Array& array)
        : scale(array[0]), details{array[1], array[2], array[3]} {}

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
    friend Parent operator*(ValueType lhs, const Parent& rhs) {
      return {lhs * rhs.scale, lhs * rhs.details};
    }
    friend Parent operator*(const Parent& lhs, ValueType rhs) {
      return rhs * lhs;
    }
    Parent& operator*=(ValueType rhs) {
      *this = *this * rhs;
      return *this;
    }
    std::string toString() const {
      std::stringstream ss;
      ss << "[scale = " << scale << ", details = " << details.toString() << "]";
      return ss.str();
    }
  };
};

template <typename ValueT>
typename WaveletCoefficients<ValueT>::ChildScales operator*(
    ValueT lhs, const typename WaveletCoefficients<ValueT>::ChildScales& rhs) {
  typename WaveletCoefficients<ValueT>::ChildScales result = rhs;
  std::for_each(result.begin(), result.end(),
                [lhs](auto& val) { return val *= lhs; });
  return result;
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_WAVELET_COEFFICIENTS_H_
