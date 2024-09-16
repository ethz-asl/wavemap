import numpy as np
import _dummy_objects

# Declare a floating point value representing the occupancy posterior in log
# odds as queried from the map in one of the previous examples
occupancy_log_odds = _dummy_objects.example_occupancy_log_odds()

# A point is considered unobserved if its occupancy posterior is equal to the
# prior. Wavemap assumes that an unobserved point is equally likely to be
# free or occupied. In other words, the prior occupancy probability is 0.5,
# which corresponds to a log odds value of 0.0. Accounting for numerical
# noise, checking whether a point is unobserved can be done as follows:
kUnobservedThreshold = 1e-3
is_unobserved = np.abs(occupancy_log_odds) < kUnobservedThreshold
print(is_unobserved)


# In case you would like to convert log odds into probabilities, we provide
# the following convenience function:
def log_odds_to_probability(log_odds):
    odds = np.exp(log_odds)
    prob = odds / (1.0 + odds)
    return prob


occupancy_probability = log_odds_to_probability(occupancy_log_odds)
print(occupancy_probability)


# To do the opposite
def probability_to_log_odds(probability):
    odds = probability / (1.0 - probability)
    return np.log(odds)


occupancy_log_odds = probability_to_log_odds(occupancy_probability)
print(occupancy_log_odds)

# To classify whether a point is estimated to be occupied or free, you need
# to choose a discrimination threshold. A reasonable default threshold is 0.5
# (probability), which corresponds to 0.0 log odds.
kOccupancyThresholdProb = 0.5
kOccupancyThresholdLogOdds = 0.0

# NOTE: To tailor the threshold, we recommend running wavemap on a dataset
#       that is representative of your application and analyzing the Receiver
#       Operating Characteristic curve.

# Once a threshold has been chosen, you can either classify in log space
is_occupied = kOccupancyThresholdLogOdds <= occupancy_log_odds
is_free = occupancy_log_odds < kOccupancyThresholdLogOdds
print(is_occupied)
print(is_free)

# Or in probability space
is_occupied = kOccupancyThresholdProb <= occupancy_probability
is_free = occupancy_probability < kOccupancyThresholdProb
print(is_occupied)
print(is_free)
