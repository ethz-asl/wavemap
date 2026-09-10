#ifndef WAVEMAP_LAYERED_LAYER_TRAITS_H_
#define WAVEMAP_LAYERED_LAYER_TRAITS_H_

#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <wavemap/layered/types/weighted_mean_state.h>

namespace wavemap::layered {

// Storage follows mathematical semantics, not the value C++ numeric type.
enum class LayerStorageKind {
  kWaveletCompatible,
  kNonWavelet,
};

struct VisualizationColor {
  float r;
  float g;
  float b;
};

struct ScalarLayerVisualization {
  VisualizationColor low;
  VisualizationColor high;
};

struct IntegerCategoryLabel {
  int value;
  std::string_view label;
  std::optional<VisualizationColor> color = std::nullopt;

  constexpr IntegerCategoryLabel(int value_in, std::string_view label_in)
      : value(value_in), label(label_in) {}
  constexpr IntegerCategoryLabel(int value_in, std::string_view label_in,
                                 VisualizationColor color_in)
      : value(value_in), label(label_in), color(color_in) {}
};

struct IntegerCategoryMetadata {
  int value;
  std::string label;
  std::optional<VisualizationColor> color;
};

namespace detail {
template <typename T>
inline constexpr bool kLayerTraitsAlwaysFalse = false;

template <typename LayerTagT, typename = void>
struct HasIntegerCategoryLabels : std::false_type {};

template <typename LayerTagT>
struct HasIntegerCategoryLabels<
    LayerTagT, std::void_t<decltype(LayerTagT::categories)>>
    : std::true_type {};

template <typename LayerTagT, typename = void>
struct HasVisualization : std::false_type {};

template <typename LayerTagT>
struct HasVisualization<LayerTagT,
                        std::void_t<decltype(LayerTagT::visualization)>>
    : std::true_type {};
}  // namespace detail

template <typename LayerTagT>
std::vector<IntegerCategoryMetadata> integerCategoryMetadata() {
  std::vector<IntegerCategoryMetadata> result;
  if constexpr (detail::HasIntegerCategoryLabels<LayerTagT>::value) {
    result.reserve(LayerTagT::categories.size());
    for (const IntegerCategoryLabel& category : LayerTagT::categories) {
      result.push_back(
          {category.value, std::string(category.label), category.color});
    }
  }
  return result;
}

template <typename LayerTagT>
std::optional<ScalarLayerVisualization> scalarLayerVisualization() {
  if constexpr (detail::HasVisualization<LayerTagT>::value) {
    return LayerTagT::visualization;
  }
  return std::nullopt;
}

// Layer tags can inherit from schema::ContinuousLayer or
// schema::DiscreteLayer, in which case this trait is populated automatically.
// Explicit specializations remain supported for backwards compatibility.
template <typename LayerTagT, typename = void>
struct LayerTraits {
  static_assert(detail::kLayerTraitsAlwaysFalse<LayerTagT>,
                "LayerTraits<LayerTagT> must be specialized for each layer.");
};

template <typename LayerTagT>
struct LayerTraits<
    LayerTagT,
    std::void_t<typename LayerTagT::Value, typename LayerTagT::UpdatePolicy,
                decltype(LayerTagT::name), decltype(LayerTagT::storage_kind)>> {
  using Value = typename LayerTagT::Value;
  using UpdatePolicy = typename LayerTagT::UpdatePolicy;
  static constexpr std::string_view name = LayerTagT::name;
  static constexpr LayerStorageKind storage_kind = LayerTagT::storage_kind;
};

template <typename LayerTagT>
using LayerValueT = typename LayerTraits<LayerTagT>::Value;

namespace detail {
template <typename LayerTagT, typename = void>
struct LayerState {
  using Type = LayerValueT<LayerTagT>;
};

template <typename LayerTagT>
struct LayerState<LayerTagT,
                  std::void_t<typename LayerTagT::State>> {
  using Type = typename LayerTagT::State;
};
}  // namespace detail

// Value is the public observation/query type. State is the representation
// actually stored in the map. They are identical for ordinary layers.
template <typename LayerTagT>
using LayerStateT = typename detail::LayerState<LayerTagT>::Type;

template <typename LayerTagT>
using LayerUpdatePolicyT = typename LayerTraits<LayerTagT>::UpdatePolicy;

template <typename LayerTagT>
struct LayerStateConversion {
  static LayerValueT<LayerTagT> value(const LayerStateT<LayerTagT>& state) {
    if constexpr (std::is_same_v<LayerStateT<LayerTagT>,
                                 WeightedMeanState>) {
      static_assert(std::is_same_v<LayerValueT<LayerTagT>, FloatingPoint>);
      return state.valueOr();
    } else {
      static_assert(std::is_same_v<LayerValueT<LayerTagT>,
                                   LayerStateT<LayerTagT>>,
                    "A stateful layer must specialize LayerStateConversion.");
      return state;
    }
  }

  static LayerStateT<LayerTagT> initialState(
      const LayerValueT<LayerTagT>& value) {
    if constexpr (std::is_same_v<LayerStateT<LayerTagT>,
                                 WeightedMeanState>) {
      static_assert(std::is_same_v<LayerValueT<LayerTagT>, FloatingPoint>);
      return {value, 1.f};
    } else {
      static_assert(std::is_same_v<LayerValueT<LayerTagT>,
                                   LayerStateT<LayerTagT>>,
                    "A stateful layer must specialize LayerStateConversion.");
      return value;
    }
  }
};

template <typename LayerTagT>
inline constexpr bool kIsWaveletCompatibleLayer =
    LayerTraits<LayerTagT>::storage_kind ==
    LayerStorageKind::kWaveletCompatible;

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_TRAITS_H_
