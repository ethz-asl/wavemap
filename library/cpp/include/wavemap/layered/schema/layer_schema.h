#ifndef WAVEMAP_LAYERED_LAYER_SCHEMA_H_
#define WAVEMAP_LAYERED_LAYER_SCHEMA_H_

#include <array>
#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <wavemap/layered/schema/layer_traits.h>
#include <wavemap/layered/integration/layer_update_policy.h>

namespace wavemap::layered::schema {

template <typename ValueT, typename UpdatePolicyT>
struct ContinuousLayer {
  static_assert(kIsContinuousLayerUpdatePolicy<UpdatePolicyT, ValueT>,
                "A continuous layer policy must return Value and accept "
                "(const Value&, const LayerObservation<Value>&).");
  using Value = ValueT;
  using State = ValueT;
  using UpdatePolicy = UpdatePolicyT;
  static constexpr LayerStorageKind storage_kind =
      LayerStorageKind::kWaveletCompatible;
};

template <typename ValueT, typename StateT, typename UpdatePolicyT>
struct StatefulContinuousLayer {
  static_assert(
      kIsStatefulContinuousLayerUpdatePolicy<UpdatePolicyT, StateT, ValueT>,
      "A stateful continuous policy must return State and accept "
      "(const State&, const LayerObservation<Value>&).");
  using Value = ValueT;
  using State = StateT;
  using UpdatePolicy = UpdatePolicyT;
  static constexpr LayerStorageKind storage_kind =
      LayerStorageKind::kWaveletCompatible;
};

template <typename ValueT, typename UpdatePolicyT>
struct DiscreteLayer {
  static_assert(kIsDiscreteLayerUpdatePolicy<UpdatePolicyT, ValueT>,
                "A discrete layer policy must return Value and accept either "
                "Value or optional<Value> plus LayerObservation<Value>.");
  using Value = ValueT;
  using State = ValueT;
  using UpdatePolicy = UpdatePolicyT;
  static constexpr LayerStorageKind storage_kind =
      LayerStorageKind::kNonWavelet;
};

namespace detail {
template <typename LayerTagT, typename = void>
struct IsLayerTag : std::false_type {};

template <typename LayerTagT>
struct IsLayerTag<
    LayerTagT,
    std::void_t<typename LayerTraits<LayerTagT>::Value,
                typename LayerTraits<LayerTagT>::UpdatePolicy,
                decltype(LayerTraits<LayerTagT>::name),
                decltype(LayerTraits<LayerTagT>::storage_kind)>>
    : std::true_type {};

template <typename... LayerTags>
constexpr bool namesAreUnique() {
  constexpr std::array<std::string_view, sizeof...(LayerTags)> names = {
      LayerTraits<LayerTags>::name...};
  for (size_t lhs = 0u; lhs < sizeof...(LayerTags); ++lhs) {
    if (names[lhs].empty()) {
      return false;
    }
    for (size_t rhs = lhs + 1u; rhs < sizeof...(LayerTags); ++rhs) {
      if (names[lhs] == names[rhs]) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace detail

template <typename... LayerTags>
struct LayerSchema {
  static_assert((detail::IsLayerTag<LayerTags>::value && ...),
                "Every schema entry must be a valid layer tag.");
  static_assert(detail::namesAreUnique<LayerTags...>(),
                "Layer names must be non-empty and unique within a schema.");

  using Layers = std::tuple<LayerTags...>;
  static constexpr size_t size = sizeof...(LayerTags);

  template <typename LayerTagT>
  static constexpr bool contains =
      (std::is_same_v<LayerTagT, LayerTags> || ...);

  template <typename LayerTagT>
  static constexpr bool containsContinuous =
      contains<LayerTagT> && kIsWaveletCompatibleLayer<LayerTagT>;

  template <typename LayerTagT>
  static constexpr bool containsDiscrete =
      contains<LayerTagT> && !kIsWaveletCompatibleLayer<LayerTagT>;

  static constexpr size_t continuousLayerCount =
      (size_t{0u} + ... +
       (kIsWaveletCompatibleLayer<LayerTags> ? size_t{1u} : size_t{0u}));
  static constexpr size_t discreteLayerCount =
      size - continuousLayerCount;
};

}  // namespace wavemap::layered::schema

#endif  // WAVEMAP_LAYERED_LAYER_SCHEMA_H_
