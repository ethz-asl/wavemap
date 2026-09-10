#ifndef WAVEMAP_LAYERED_LAYER_BUNDLE_TRAITS_H_
#define WAVEMAP_LAYERED_LAYER_BUNDLE_TRAITS_H_

#include <cstddef>
#include <tuple>
#include <utility>

namespace wavemap::layered {
namespace detail {
template <typename T>
inline constexpr bool kLayerBundleTraitsAlwaysFalse = false;

template <typename TupleT, typename FunctionT, size_t... Indices>
constexpr void forEachLayerTagImpl(FunctionT&& function,
                                   std::index_sequence<Indices...>) {
  (function(std::tuple_element_t<Indices, TupleT>{}), ...);
}
}  // namespace detail

// Users specialize this trait and expose LayerTags as a tuple of layer tags.
// The metadata is storage-independent so it can later drive integration,
// schema validation, ROS conversion, and visualization.
template <typename LayerBundleT>
struct LayerBundleTraits {
  static_assert(
      detail::kLayerBundleTraitsAlwaysFalse<LayerBundleT>,
      "LayerBundleTraits<LayerBundleT> must be specialized for each bundle.");
};

template <typename LayerBundleT, typename FunctionT>
constexpr void forEachLayerTag(FunctionT&& function) {
  using LayerTags = typename LayerBundleTraits<LayerBundleT>::LayerTags;
  detail::forEachLayerTagImpl<LayerTags>(
      std::forward<FunctionT>(function),
      std::make_index_sequence<std::tuple_size_v<LayerTags>>{});
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_BUNDLE_TRAITS_H_
