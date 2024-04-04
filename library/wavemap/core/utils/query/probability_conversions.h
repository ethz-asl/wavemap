#ifndef WAVEMAP_UTILS_QUERY_PROBABILITY_CONVERSIONS_H_
#define WAVEMAP_UTILS_QUERY_PROBABILITY_CONVERSIONS_H_

#include <limits>

#include "wavemap/core/common.h"

namespace wavemap::convert {
constexpr FloatingPoint kLogOddsNumericalMin =
    std::numeric_limits<FloatingPoint>::lowest();
const FloatingPoint kLogOddsNumericalMax =
    std::nextafter(std::log(std::numeric_limits<FloatingPoint>::max()), 0.f);

constexpr FloatingPoint logOddsToProbability(FloatingPoint log_odds) {
  const FloatingPoint odds = std::exp(log_odds);
  const FloatingPoint prob = odds / (1.f + odds);
  return prob;
}

constexpr FloatingPoint probabilityToLogOdds(FloatingPoint probability) {
  const FloatingPoint odds = probability / (1.f - probability);
  return std::log(odds);
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_UTILS_QUERY_PROBABILITY_CONVERSIONS_H_
