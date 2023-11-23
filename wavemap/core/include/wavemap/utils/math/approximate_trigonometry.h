#ifndef WAVEMAP_UTILS_MATH_APPROXIMATE_TRIGONOMETRY_H_
#define WAVEMAP_UTILS_MATH_APPROXIMATE_TRIGONOMETRY_H_

#include <functional>
#include <limits>

#include "wavemap/common.h"

namespace wavemap::approximate {
struct atan : public std::unary_function<FloatingPoint, FloatingPoint> {
  FloatingPoint operator()(FloatingPoint x) const {
    // Copyright (c) 2021 Francesco Mazzoli <f@mazzo.li>
    //
    // Permission to use, copy, modify, and distribute this software for any
    // purpose with or without fee is hereby granted, provided that the above
    // copyright notice and this permission notice appear in all copies.
    //
    // THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
    // WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
    // MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    // ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    // WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    // ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    // OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
    constexpr FloatingPoint a1 = 0.99997726f;
    constexpr FloatingPoint a3 = -0.33262347f;
    constexpr FloatingPoint a5 = 0.19354346f;
    constexpr FloatingPoint a7 = -0.11643287f;
    constexpr FloatingPoint a9 = 0.05265332f;
    constexpr FloatingPoint a11 = -0.01172120f;

    // Compute approximation using Horner's method
    const FloatingPoint x_sq = x * x;
    FloatingPoint u = std::fma(x_sq, a11, a9);
    u = std::fma(x_sq, u, a7);
    u = std::fma(x_sq, u, a5);
    u = std::fma(x_sq, u, a3);
    u = std::fma(x_sq, u, a1);
    return x * u;
  }
};

struct atan2
    : public std::binary_function<FloatingPoint, FloatingPoint, FloatingPoint> {
  static constexpr FloatingPoint kWorstCaseError = 1.908e-6f;

  FloatingPoint operator()(FloatingPoint y, FloatingPoint x) const {
    // Copyright (c) 2021 Francesco Mazzoli <f@mazzo.li>
    //
    // Permission to use, copy, modify, and distribute this software for any
    // purpose with or without fee is hereby granted, provided that the above
    // copyright notice and this permission notice appear in all copies.
    //
    // THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
    // WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
    // MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    // ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    // WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    // ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    // OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

    // Ensure input is in [-1, +1]
    const bool swap = std::abs(x) < std::abs(y);
    const FloatingPoint atan_input = (swap ? x : y) / (swap ? y : x);

    // Approximate atan
    FloatingPoint res = atan()(atan_input);

    // If swapped, adjust atan output
    res = swap ? std::copysign(kHalfPi, atan_input) - res : res;
    // Adjust the result depending on the input quadrant
    if (x < 0.0f) {
      res = std::copysign(kPi, y) + res;
    }

    // Return the result
    return res;
  }
};
}  // namespace wavemap::approximate

#endif  // WAVEMAP_UTILS_MATH_APPROXIMATE_TRIGONOMETRY_H_
