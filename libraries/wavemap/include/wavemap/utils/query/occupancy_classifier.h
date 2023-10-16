#ifndef WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_
#define WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_

namespace wavemap {
class OccupancyClassifier {
 public:
  explicit OccupancyClassifier(FloatingPoint log_odds_occupancy_threshold = 0.f)
      : occupancy_threshold_(log_odds_occupancy_threshold) {}

  static bool isUnobserved(FloatingPoint log_odds_occupancy_value) {
    return std::abs(log_odds_occupancy_value) < 1e-3f;
  }
  static bool isObserved(FloatingPoint log_odds_occupancy_value) {
    return !isUnobserved(log_odds_occupancy_value);
  }

  bool isFree(FloatingPoint log_odds_occupancy_value) const {
    return log_odds_occupancy_value < occupancy_threshold_;
  }
  bool isOccupied(FloatingPoint log_odds_occupancy_value) const {
    return occupancy_threshold_ < log_odds_occupancy_value;
  }

 private:
  const FloatingPoint occupancy_threshold_;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_OCCUPANCY_CLASSIFIER_H_
