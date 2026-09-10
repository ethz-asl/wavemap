#ifndef WAVEMAP_LAYERED_LAYER_UPDATE_POLICY_H_
#define WAVEMAP_LAYERED_LAYER_UPDATE_POLICY_H_

#include <algorithm>
#include <functional>
#include <optional>
#include <type_traits>

#include <wavemap/core/common.h>

#include <wavemap/layered/integration/layer_observation.h>
#include <wavemap/layered/types/weighted_mean_state.h>

namespace wavemap::layered {

template <typename PolicyT, typename ValueT>
inline constexpr bool kIsContinuousLayerUpdatePolicy =
    std::is_invocable_r_v<ValueT, const PolicyT&, const ValueT&,
                          const LayerObservation<ValueT>&>;

template <typename PolicyT, typename StateT, typename ValueT>
inline constexpr bool kIsStatefulContinuousLayerUpdatePolicy =
    std::is_invocable_r_v<StateT, const PolicyT&, const StateT&,
                          const LayerObservation<ValueT>&>;

template <typename PolicyT, typename ValueT>
inline constexpr bool kIsDiscreteLayerUpdatePolicy =
    kIsContinuousLayerUpdatePolicy<PolicyT, ValueT> ||
    std::is_invocable_r_v<ValueT, const PolicyT&,
                          const std::optional<ValueT>&,
                          const LayerObservation<ValueT>&>;

template <typename ValueT>
struct ReplaceLayerUpdatePolicy {
  ValueT operator()(const ValueT& /*current_value*/,
                    const LayerObservation<ValueT>& observation) const {
    return observation.value;
  }
};

template <typename ValueT>
struct MinimumLayerUpdatePolicy {
  ValueT operator()(const ValueT& current_value,
                    const LayerObservation<ValueT>& observation) const {
    return std::min(current_value, observation.value);
  }
};

template <typename ValueT>
struct MaximumLayerUpdatePolicy {
  ValueT operator()(const ValueT& current_value,
                    const LayerObservation<ValueT>& observation) const {
    return std::max(current_value, observation.value);
  }
};

template <typename ValueT, typename AddT = std::plus<ValueT>>
struct AccumulateLayerUpdatePolicy {
  ValueT operator()(const ValueT& current_value,
                    const LayerObservation<ValueT>& observation) const {
    return AddT{}(current_value, observation.value);
  }
};

struct LogicalOrLayerUpdatePolicy {
  bool operator()(bool current_value,
                  const LayerObservation<bool>& observation) const {
    return current_value || observation.value;
  }
};

struct LogicalAndLayerUpdatePolicy {
  bool operator()(bool current_value,
                  const LayerObservation<bool>& observation) const {
    return current_value && observation.value;
  }
};

// Schema-friendly exponential average for scalar floating-point layers.
// The compile-time ratio keeps the policy default-constructible, as required
// by the layer integrator, while allowing each layer to select its own gain.
// This is a convex update, so bounded current values and observations remain
// bounded. The zero-initialized voxel acts as the initial prior.
template <int ObservationWeightNumerator,
          int ObservationWeightDenominator = 100>
struct ExponentialScalarLayerUpdatePolicy {
  static_assert(0 < ObservationWeightDenominator,
                "The exponential update denominator must be positive.");
  static_assert(0 <= ObservationWeightNumerator,
                "The exponential observation weight cannot be negative.");
  static_assert(ObservationWeightNumerator <= ObservationWeightDenominator,
                "The exponential observation weight cannot exceed one.");

  static constexpr FloatingPoint observationWeight() {
    return static_cast<FloatingPoint>(ObservationWeightNumerator) /
           static_cast<FloatingPoint>(ObservationWeightDenominator);
  }

  FloatingPoint operator()(
      FloatingPoint current_value,
      const LayerObservation<FloatingPoint>& observation) const {
    const FloatingPoint weight = observationWeight();
    return current_value + weight * (observation.value - current_value);
  }
};

// Runtime form for direct/custom use when a policy instance is explicitly
// owned by the caller rather than default-constructed from a layer schema.
template <typename ValueT, typename ArithmeticPolicyT>
class ExponentialLayerUpdatePolicy {
 public:
  explicit ExponentialLayerUpdatePolicy(FloatingPoint observation_weight)
      : observation_weight_(
            std::clamp(observation_weight, FloatingPoint{0.f},
                       FloatingPoint{1.f})) {}

  ValueT operator()(const ValueT& current_value,
                    const LayerObservation<ValueT>& observation) const {
    return ArithmeticPolicyT::add(
        ArithmeticPolicyT::scale(current_value, 1.f - observation_weight_),
        ArithmeticPolicyT::scale(observation.value, observation_weight_));
  }

 private:
  FloatingPoint observation_weight_;
};

// Uses observation confidence as the blend factor when present. This is an
// exponential update, not a running weighted mean: no historical weight is
// hidden outside the voxel value.
template <typename ValueT, typename ArithmeticPolicyT>
class ConfidenceBlendLayerUpdatePolicy {
 public:
  explicit ConfidenceBlendLayerUpdatePolicy(
      FloatingPoint default_observation_weight = 1.f)
      : default_observation_weight_(std::clamp(
            default_observation_weight, FloatingPoint{0.f},
            FloatingPoint{1.f})) {}

  ValueT operator()(const ValueT& current_value,
                    const LayerObservation<ValueT>& observation) const {
    const FloatingPoint weight = std::clamp(
        observation.confidence.value_or(default_observation_weight_),
        FloatingPoint{0.f}, FloatingPoint{1.f});
    return ArithmeticPolicyT::add(
        ArithmeticPolicyT::scale(current_value, 1.f - weight),
        ArithmeticPolicyT::scale(observation.value, weight));
  }

 private:
  FloatingPoint default_observation_weight_;
};

struct WeightedMeanLayerUpdatePolicy {
  WeightedMeanState operator()(
      const WeightedMeanState& current,
      const LayerObservation<FloatingPoint>& observation) const {
    const FloatingPoint weight =
        std::max(0.f, observation.confidence.value_or(1.f));
    if (weight == 0.f) {
      return current;
    }
    return {current.weighted_sum + weight * observation.value,
            current.total_weight + weight};
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_UPDATE_POLICY_H_
