#ifndef WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_
#define WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_

#include "wavemap/utils/query/occupancy.h"

namespace wavemap {
class OccupancyClassifier {
 public:
  explicit OccupancyClassifier(FloatingPoint log_odds_occupancy_threshold = 0.f)
      : occupancy_threshold_(log_odds_occupancy_threshold) {}

  static FloatingPoint getUnobservedThreshold() { return kUnobservedThreshold; }
  FloatingPoint getOccupancyThreshold() const { return occupancy_threshold_; }

  constexpr Occupancy::Id classify(FloatingPoint log_odds_occupancy) const;
  constexpr Occupancy::Mask toMask(FloatingPoint log_odds_occupancy) const;

  constexpr bool is(FloatingPoint log_odds_occupancy,
                    Occupancy::Id occupancy_type) const;
  constexpr bool is(FloatingPoint log_odds_occupancy,
                    Occupancy::Mask occupancy_mask) const;

  static constexpr bool has(Occupancy::Mask region_occupancy,
                            Occupancy::Id occupancy_type);
  static constexpr bool has(Occupancy::Mask region_occupancy,
                            Occupancy::Mask occupancy_mask);

  static constexpr bool isFully(Occupancy::Mask region_occupancy,
                                Occupancy::Id occupancy_type);
  static constexpr bool isFully(Occupancy::Mask region_occupancy,
                                Occupancy::Mask occupancy_mask);

  static constexpr bool isUnobserved(FloatingPoint log_odds_occupancy);
  static constexpr bool isObserved(FloatingPoint log_odds_occupancy);

  // Forbid implicit Mask type conversions
  template <typename A, typename B>
  static constexpr bool has(A region_occupancy, B occupancy_mask) = delete;
  template <typename A, typename B>
  static constexpr bool isFully(A region_occupancy, B occupancy_mask) = delete;

 private:
  static constexpr FloatingPoint kUnobservedThreshold = 1e-3f;
  const FloatingPoint occupancy_threshold_;
};
}  // namespace wavemap

#include "wavemap/utils/query/impl/occupancy_classifier_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_
