#ifndef WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_COEFFICIENTS_H_
#define WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_COEFFICIENTS_H_

#include <algorithm>
#include <functional>
#include <string>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/math/int_math.h"

namespace wavemap {
template <typename ValueT, int dim>
struct HaarCoefficients {
  static constexpr int kDim = dim;
  static constexpr int kNumCoefficients = int_math::exp2(dim);
  static constexpr int kNumDetailCoefficients = kNumCoefficients - 1;

  using ValueType = ValueT;
  using CoefficientsArray = std::array<ValueType, kNumCoefficients>;

  using Scale = ValueType;
  class Details : public std::array<ValueType, kNumDetailCoefficients> {
   public:
    friend Details operator+(const Details& lhs, const Details& rhs) {
      Details result;
      std::transform(lhs.cbegin(), lhs.cend(), rhs.cbegin(), result.begin(),
                     std::plus<>());
      return result;
    }
    Details& operator+=(const Details& rhs) {
      *this = *this + rhs;
      return *this;
    }
    friend Details operator-(const Details& lhs, const Details& rhs) {
      Details result;
      std::transform(lhs.cbegin(), lhs.cend(), rhs.cbegin(), result.begin(),
                     std::minus<>());
      return result;
    }
    Details& operator-=(const Details& rhs) {
      *this = *this - rhs;
      return *this;
    }
    friend Details operator*(ValueType lhs, const Details& rhs) {
      Details result;
      std::transform(rhs.cbegin(), rhs.cend(), result.begin(),
                     [lhs](const auto& val) { return lhs * val; });
      return result;
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
      ss << "[";
      for (int coeff_idx = 1; coeff_idx <= kNumDetailCoefficients;
           ++coeff_idx) {
        for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
          ss << (bit_ops::is_bit_set(coeff_idx, dim_idx) ? "H" : "L");
        }
        ss << " = " << this->operator[](coeff_idx - 1) << ", ";
      }
      ss << "\b\b]";
      return ss.str();
    }
  };

  class Parent {
   public:
    Scale scale{};
    Details details{};

    Parent() = default;
    Parent(Scale scale, Details details) : scale(scale), details(details) {}
    Parent(const CoefficientsArray& array) : scale(array[0]) {  // NOLINT
      std::copy(std::next(array.begin()), array.end(), details.begin());
    }
    operator CoefficientsArray() const {  // NOLINT
      CoefficientsArray coefficients_array;
      coefficients_array[0] = scale;
      std::copy(details.cbegin(), details.cend(),
                std::next(coefficients_array.begin()));
      return coefficients_array;
    }

    ValueType& operator[](size_t index) {
      return (index == 0) ? scale : details[index - 1];
    }
    const ValueType& operator[](size_t index) const {
      return (index == 0) ? scale : details[index - 1];
    }

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
    friend Parent operator-(const Parent& lhs, const Parent& rhs) {
      return {lhs.scale - rhs.scale, lhs.details - rhs.details};
    }
    Parent& operator-=(const Parent& rhs) {
      *this = *this - rhs;
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
      ss << "[scale (";
      for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
        ss << "L";
      }
      ss << ") = " << scale << ", details = " << details.toString() << "]";
      return ss.str();
    }
  };
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_CELL_TYPES_HAAR_COEFFICIENTS_H_
