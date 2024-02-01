#ifndef WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_CLASSIFIER_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_CLASSIFIER_INL_H_

namespace wavemap {
inline constexpr Occupancy::Id OccupancyClassifier::classify(
    FloatingPoint log_odds_occupancy) const {
  if (isUnobserved(log_odds_occupancy)) {
    return Occupancy::kUnobserved;
  }
  const bool is_free = log_odds_occupancy < occupancy_threshold_;
  return is_free ? Occupancy::kFree : Occupancy::kOccupied;
}

inline constexpr Occupancy::Mask OccupancyClassifier::toMask(
    FloatingPoint log_odds_occupancy) const {
  if (isUnobserved(log_odds_occupancy)) {
    return Occupancy::toMask(Occupancy::kUnobserved);
  }
  const bool is_free = log_odds_occupancy < occupancy_threshold_;
  return is_free ? Occupancy::toMask(Occupancy::kFree)
                 : Occupancy::toMask(Occupancy::kOccupied);
}

inline constexpr bool OccupancyClassifier::is(
    FloatingPoint log_odds_occupancy, Occupancy::Id occupancy_type) const {
  return is(log_odds_occupancy, Occupancy::toMask(occupancy_type));
}

inline constexpr bool OccupancyClassifier::is(
    FloatingPoint log_odds_occupancy, Occupancy::Mask occupancy_mask) const {
  return has(toMask(log_odds_occupancy), occupancy_mask);
}

inline constexpr bool OccupancyClassifier::has(Occupancy::Mask region_occupancy,
                                               Occupancy::Id occupancy_type) {
  return has(region_occupancy, Occupancy::toMask(occupancy_type));
}

inline constexpr bool OccupancyClassifier::has(Occupancy::Mask region_occupancy,
                                               Occupancy::Mask occupancy_mask) {
  // Check that at least one bit in the region matches the mask
  return region_occupancy & occupancy_mask;
}

inline constexpr bool OccupancyClassifier::isFully(
    Occupancy::Mask region_occupancy, Occupancy::Id occupancy_type) {
  return isFully(region_occupancy, Occupancy::toMask(occupancy_type));
}

inline constexpr bool OccupancyClassifier::isFully(
    Occupancy::Mask region_occupancy, Occupancy::Mask occupancy_mask) {
  // Set stray bits beyond mask width to 0, as they should not influence result
  const Occupancy::Mask region_occ_trimmed = region_occupancy & 0b111;
  // Check that no bits in the region are set while not being in the mask
  return !(region_occ_trimmed & ~occupancy_mask);
}

inline constexpr bool OccupancyClassifier::isUnobserved(
    FloatingPoint log_odds_occupancy) {
  return std::abs(log_odds_occupancy) < kUnobservedThreshold;
}

inline constexpr bool OccupancyClassifier::isObserved(
    FloatingPoint log_odds_occupancy) {
  return !isUnobserved(log_odds_occupancy);
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_CLASSIFIER_INL_H_
