#ifndef WAVEMAP_LAYERED_LAYER_INTEGRATOR_H_
#define WAVEMAP_LAYERED_LAYER_INTEGRATOR_H_

#include <map>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <wavemap/core/indexing/index_conversions.h>

#include <wavemap/layered/integration/continuous_layer_updater.h>
#include <wavemap/layered/map/discrete_layer.h>
#include <wavemap/layered/schema/layer_descriptor.h>
#include <wavemap/layered/integration/layer_observation.h>
#include <wavemap/layered/schema/layer_traits.h>

namespace wavemap::layered {

struct LayerIntegrationResult {
  size_t received = 0u;
  size_t integrated = 0u;
  size_t rejected = 0u;
  size_t updated_voxels = 0u;
};

namespace detail {

inline bool isFinite(const Point3D& position) {
  return position.array().isFinite().all();
}

template <typename LayerTagT, typename DescriptorT>
inline constexpr bool kDescriptorMatchesLayer =
    std::is_same_v<LayerTagT, typename DescriptorT::LayerTag>;

template <typename LayerTagT, typename TupleT, size_t... Indices>
constexpr size_t descriptorMatchCount(std::index_sequence<Indices...>) {
  return (size_t{0u} + ... +
          (kDescriptorMatchesLayer<
               LayerTagT, std::tuple_element_t<Indices, TupleT>>
               ? size_t{1u}
               : size_t{0u}));
}

template <typename LayerTagT, size_t Index = 0u, typename TupleT>
constexpr auto descriptorForLayer(const TupleT& descriptors) {
  static_assert(Index < std::tuple_size_v<TupleT>,
                "The requested layer is not present in this bundle.");
  using Descriptor = std::tuple_element_t<Index, TupleT>;
  if constexpr (kDescriptorMatchesLayer<LayerTagT, Descriptor>) {
    return std::get<Index>(descriptors);
  } else {
    return descriptorForLayer<LayerTagT, Index + 1u>(descriptors);
  }
}

template <typename LayerTagT, typename BundleT>
constexpr auto descriptorForLayer() {
  constexpr auto descriptors = layerDescriptors<BundleT>();
  using DescriptorTuple = decltype(descriptors);
  constexpr size_t kDescriptorCount = std::tuple_size_v<DescriptorTuple>;
  static_assert(
      descriptorMatchCount<LayerTagT, DescriptorTuple>(
          std::make_index_sequence<kDescriptorCount>{}) == 1u,
      "Each layer tag must occur exactly once in its storage bundle.");
  return descriptorForLayer<LayerTagT>(descriptors);
}

template <typename PolicyT, typename ValueT>
ValueT applyDiscretePolicy(const PolicyT& policy,
                           const std::optional<ValueT>& current_value,
                           const LayerObservation<ValueT>& observation) {
  if constexpr (std::is_invocable_r_v<
                    ValueT, PolicyT, const std::optional<ValueT>&,
                    const LayerObservation<ValueT>&>) {
    return policy(current_value, observation);
  } else {
    return policy(current_value.value_or(ValueT{}), observation);
  }
}

}  // namespace detail

template <typename LayerTagT, typename LayeredMapT>
LayerIntegrationResult integrateLayerObservations(
    LayeredMapT& map,
    const std::vector<LayerObservation<LayerValueT<LayerTagT>>>& observations,
    const LayerUpdatePolicyT<LayerTagT>& policy =
        LayerUpdatePolicyT<LayerTagT>{}) {
  using Value = LayerValueT<LayerTagT>;
  LayerIntegrationResult result;
  result.received = observations.size();

  const FloatingPoint cell_width_inv =
      1.f / map.continuousMap().getMinCellWidth();
  std::map<IndexKey, std::vector<const LayerObservation<Value>*>> grouped;
  for (const auto& observation : observations) {
    if (!detail::isFinite(observation.position)) {
      ++result.rejected;
      continue;
    }
    const Index3D index =
        convert::pointToNearestIndex(observation.position, cell_width_inv);
    grouped[IndexKey(index)].push_back(&observation);
    ++result.integrated;
  }
  result.updated_voxels = grouped.size();

  if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
    constexpr auto descriptor =
        detail::descriptorForLayer<
            LayerTagT, typename LayeredMapT::ContinuousLayers>();
    for (const auto& [index_key, voxel_observations] : grouped) {
      updateContinuousLayersAtVoxel(
          map.continuousMap(), indexFromKey(index_key), [&](auto& layers) {
            auto& state = descriptor.value(layers);
            for (const auto* observation : voxel_observations) {
              state = policy(state, *observation);
            }
          });
    }
  } else {
    constexpr auto descriptor =
        detail::descriptorForLayer<
            LayerTagT, typename LayeredMapT::DiscreteLayers>();
    auto& layer = descriptor.value(map.discreteLayers());
    std::vector<std::pair<Index3D, Value>> updates;
    updates.reserve(grouped.size());
    for (const auto& [index_key, voxel_observations] : grouped) {
      const Index3D index = indexFromKey(index_key);
      std::optional<Value> value = layer.getValue(index);
      for (const auto* observation : voxel_observations) {
        value = detail::applyDiscretePolicy(policy, value, *observation);
      }
      updates.emplace_back(index, *value);
    }
    layer.setValues(updates);
  }

  return result;
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_INTEGRATOR_H_
