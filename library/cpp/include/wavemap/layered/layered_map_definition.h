#ifndef WAVEMAP_LAYERED_LAYERED_MAP_DEFINITION_H_
#define WAVEMAP_LAYERED_LAYERED_MAP_DEFINITION_H_

#include <wavemap/layered/descriptor_layered_map_io.h>
#include <wavemap/layered/layered_map.h>
#include <wavemap/layered/schema_continuous_traits.h>
#include <wavemap/layered/schema_layer_storage.h>

namespace wavemap::layered {
namespace detail {

template <typename LayerTagT>
inline constexpr bool kContinuousLayerHasValueTraits =
    !kIsWaveletCompatibleLayer<LayerTagT> ||
    schema::kHasContinuousValueTraits<LayerStateT<LayerTagT>>;

template <typename LayerTagT>
inline constexpr bool kContinuousLayerHasStreamCodec =
    !kIsWaveletCompatibleLayer<LayerTagT> ||
    kHasLayerStreamCodec<LayerStateT<LayerTagT>>;

template <typename TupleT, size_t... Indices>
constexpr bool continuousLayersHaveValueTraits(
    std::index_sequence<Indices...>) {
  return (kContinuousLayerHasValueTraits<
              std::tuple_element_t<Indices, TupleT>> &&
          ...);
}

template <typename TupleT, size_t... Indices>
constexpr bool continuousLayersHaveStreamCodecs(
    std::index_sequence<Indices...>) {
  return (kContinuousLayerHasStreamCodec<
              std::tuple_element_t<Indices, TupleT>> &&
          ...);
}

}  // namespace detail

// Derives the complete core map representation from a user layer schema.
// Occupancy remains built into LayeredMapVoxel and is therefore intentionally
// not listed in SchemaT.
template <typename SchemaT>
struct LayeredMapDefinition {
  using Schema = SchemaT;
  using LayerTags = typename Schema::Layers;

  static_assert(
      detail::continuousLayersHaveValueTraits<LayerTags>(
          std::make_index_sequence<std::tuple_size_v<LayerTags>>{}),
      "Every continuous layer value needs ContinuousValueTraits providing "
      "add, subtract, scale, threshold, and magnitude operations.");
  static_assert(
      detail::continuousLayersHaveStreamCodecs<LayerTags>(
          std::make_index_sequence<std::tuple_size_v<LayerTags>>{}),
      "Every continuous layer value needs a LayerStreamCodec for map "
      "persistence.");

  using ContinuousLayers = schema::ContinuousLayerBundle<Schema>;
  using ContinuousArithmetic = schema::ContinuousBundleArithmetic<Schema>;
  using DiscreteLayers = schema::DiscreteLayerBundle<Schema>;

  using Map =
      LayeredMap<ContinuousLayers, ContinuousArithmetic, DiscreteLayers>;
  using Config = typename Map::Config;
  using Voxel = typename Map::ContinuousVoxel;
  using ContinuousMap = typename Map::ContinuousMap;

  using VoxelSerializer = DescriptorVoxelSerializer<Voxel>;
  using MapIo = io::LayeredMapIo<Map, VoxelSerializer>;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYERED_MAP_DEFINITION_H_
