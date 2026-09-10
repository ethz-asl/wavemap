#ifndef WAVEMAP_LAYERED_LAYER_DESCRIPTOR_H_
#define WAVEMAP_LAYERED_LAYER_DESCRIPTOR_H_

#include <tuple>
#include <type_traits>
#include <utility>

#include <wavemap/layered/schema/layer_traits.h>

namespace wavemap::layered {
namespace detail {
template <typename T>
inline constexpr bool kLayerDescriptorsAlwaysFalse = false;

template <typename TupleT, typename FunctionT, size_t... Indices>
auto transformTupleImpl(TupleT&& tuple, FunctionT&& function,
                        std::index_sequence<Indices...>) {
  return std::make_tuple(
      function(std::get<Indices>(std::forward<TupleT>(tuple)))...);
}
}  // namespace detail

template <typename LayerTagT, typename BundleT, typename MemberT>
struct LayerDescriptor {
  using LayerTag = LayerTagT;
  using Bundle = BundleT;
  using Member = MemberT;

  MemberT BundleT::*member;

  static constexpr std::string_view name() {
    return LayerTraits<LayerTagT>::name;
  }

  MemberT& value(BundleT& bundle) const { return bundle.*member; }
  const MemberT& value(const BundleT& bundle) const { return bundle.*member; }
};

template <typename LayerTagT, typename BundleT, typename MemberT>
constexpr auto layerDescriptor(MemberT BundleT::*member) {
  return LayerDescriptor<LayerTagT, BundleT, MemberT>{member};
}

template <typename LayerTagT, typename BundleT>
struct AccessorLayerDescriptor {
  using LayerTag = LayerTagT;
  using Bundle = BundleT;
  using Member = LayerStateT<LayerTagT>;

  static constexpr std::string_view name() {
    return LayerTraits<LayerTagT>::name;
  }

  decltype(auto) value(BundleT& bundle) const {
    return bundle.template get<LayerTagT>();
  }
  decltype(auto) value(const BundleT& bundle) const {
    return bundle.template get<LayerTagT>();
  }
};

template <typename LayerTagT, typename BundleT>
constexpr auto accessorLayerDescriptor() {
  return AccessorLayerDescriptor<LayerTagT, BundleT>{};
}

// One specialization per bundle is the authoritative ordered layer list.
template <typename BundleT>
struct LayerDescriptors {
  static_assert(detail::kLayerDescriptorsAlwaysFalse<BundleT>,
                "LayerDescriptors<BundleT> must be specialized.");
};

template <typename BundleT>
constexpr auto layerDescriptors() {
  return LayerDescriptors<BundleT>::descriptors();
}

template <typename TupleT, typename FunctionT>
auto transformLayerDescriptors(TupleT&& tuple, FunctionT&& function) {
  constexpr size_t kTupleSize =
      std::tuple_size_v<std::remove_reference_t<TupleT>>;
  return detail::transformTupleImpl(
      std::forward<TupleT>(tuple), std::forward<FunctionT>(function),
      std::make_index_sequence<kTupleSize>{});
}

template <typename BundleT, typename FunctionT>
void forEachLayerDescriptor(BundleT&& bundle, FunctionT&& function) {
  const auto descriptors =
      layerDescriptors<std::remove_cv_t<std::remove_reference_t<BundleT>>>();
  std::apply(
      [&](const auto&... descriptor) {
        (function(descriptor, descriptor.value(bundle)), ...);
      },
      descriptors);
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_DESCRIPTOR_H_
