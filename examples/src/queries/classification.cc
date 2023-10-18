#include <wavemap/utils/query/probability_conversions.h>

#include "wavemap_examples/common.h"

using namespace wavemap;
int main(int, char**) {
  // Declare a floating point value representing the occupancy posterior in log
  // odds as queried from the map in one of the previous examples
  const FloatingPoint occupancy_log_odds{};

  // A point is considered unobserved if its occupancy posterior is equal to the
  // prior. Wavemap assumes that an unobserved point is equally likely to be
  // free or occupied. In other words, the prior occupancy probability is 0.5,
  // which corresponds to a log odds value of 0.0. Accounting for numerical
  // noise, checking whether a point is unobserved can be done as follows:
  constexpr FloatingPoint kUnobservedThreshold = 1e-3;
  const bool is_unobserved =
      std::abs(occupancy_log_odds) < kUnobservedThreshold;
  examples::doSomething(is_unobserved);

  // In case you would like to convert log odds into probabilities, we provide
  // the following convenience function:
  const FloatingPoint occupancy_probability =
      convert::logOddsToProbability(occupancy_log_odds);
  examples::doSomething(occupancy_probability);

  // To classify whether a point is estimated to be occupied or free, you need
  // to choose a discrimination threshold. A reasonable default threshold is 0.5
  // (probability), which corresponds to 0.0 log odds.
  constexpr FloatingPoint kOccupancyThresholdProb = 0.5;
  constexpr FloatingPoint kOccupancyThresholdLogOdds = 0.0;

  // NOTE: To taylor the threshold, we recommend running wavemap on a dataset
  //       that is representative of your application and analyzing the Receiver
  //       Operating Characteristic curve.

  // Once a threshold has been chosen, you can either classify in log space
  {
    const bool is_occupied = kOccupancyThresholdLogOdds < occupancy_log_odds;
    const bool is_free = occupancy_log_odds < kOccupancyThresholdLogOdds;
    examples::doSomething(is_occupied);
    examples::doSomething(is_free);
  }

  // Or in probability space
  {
    const bool is_occupied = kOccupancyThresholdProb < occupancy_probability;
    const bool is_free = occupancy_probability < kOccupancyThresholdProb;
    examples::doSomething(is_occupied);
    examples::doSomething(is_free);
  }
}
