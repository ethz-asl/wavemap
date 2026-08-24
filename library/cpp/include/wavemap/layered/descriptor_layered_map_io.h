#ifndef WAVEMAP_LAYERED_DESCRIPTOR_LAYERED_MAP_IO_H_
#define WAVEMAP_LAYERED_DESCRIPTOR_LAYERED_MAP_IO_H_

#include <type_traits>
#include <vector>

#include <wavemap/io/stream_conversions.h>
#include <wavemap/layered/layer_descriptor.h>
#include <wavemap/layered/layer_stream_codec.h>
#include <wavemap/layered/layered_map_io.h>
#include <wavemap/layered/layered_map_schema.h>
#include <wavemap/layered/schema_layer_storage.h>

namespace wavemap::layered {

template <typename ContinuousLayersT>
std::vector<LayerSchemaEntry> descriptorContinuousLayerSchema() {
  std::vector<LayerSchemaEntry> schema;
  const auto descriptors = layerDescriptors<ContinuousLayersT>();
  schema.reserve(std::tuple_size_v<decltype(descriptors)>);
  std::apply(
      [&](const auto&... descriptor) {
        (schema.emplace_back(
             LayerSchemaEntry{
                 std::string(descriptor.name()),
                 LayerStreamCodec<
                     std::remove_cv_t<std::remove_reference_t<decltype(
                         descriptor.value(
                             std::declval<const ContinuousLayersT&>()))>>>::
                     typeName()}),
         ...);
      },
      descriptors);
  return schema;
}

template <typename VoxelT>
struct DescriptorVoxelSerializer {
  static void write(std::ostream& ostream, const VoxelT& voxel) {
    wavemap::io::StreamableCellData<FloatingPoint>::write(ostream, voxel.occupancy);
    forEachLayerDescriptor(
        voxel.data, [&](const auto& /*descriptor*/, const auto& value) {
          using Value =
              std::remove_cv_t<std::remove_reference_t<decltype(value)>>;
          LayerStreamCodec<Value>::write(ostream, value);
        });
  }

  static VoxelT read(std::istream& istream) {
    VoxelT voxel;
    voxel.occupancy =
        wavemap::io::StreamableCellData<FloatingPoint>::read(istream);
    forEachLayerDescriptor(
        voxel.data, [&](const auto& /*descriptor*/, auto& value) {
          using Value =
              std::remove_cv_t<std::remove_reference_t<decltype(value)>>;
          value = LayerStreamCodec<Value>::read(istream);
        });
    return voxel;
  }
};

template <typename DiscreteLayersT>
auto descriptorNamedDiscreteLayers(DiscreteLayersT& layers) {
  return transformLayerDescriptors(
      layerDescriptors<DiscreteLayersT>(), [&](const auto& descriptor) {
        return io::namedDiscreteLayer(std::string(descriptor.name()),
                                      descriptor.value(layers));
      });
}

template <typename DiscreteLayersT>
auto descriptorNamedDiscreteLayers(const DiscreteLayersT& layers) {
  return transformLayerDescriptors(
      layerDescriptors<DiscreteLayersT>(), [&](const auto& descriptor) {
        return io::namedDiscreteLayer(std::string(descriptor.name()),
                                      descriptor.value(layers));
      });
}

template <typename... LayerTags>
struct ContinuousLayerSchemaTraits<
    schema::ContinuousLayerBundle<schema::LayerSchema<LayerTags...>>> {
  using Bundle =
      schema::ContinuousLayerBundle<schema::LayerSchema<LayerTags...>>;

  static std::vector<LayerSchemaEntry> layers() {
    return descriptorContinuousLayerSchema<Bundle>();
  }
};

}  // namespace wavemap::layered

namespace wavemap::layered::io {

template <typename... LayerTags>
struct DiscreteLayerBundleTraits<
    schema::DiscreteLayerBundle<schema::LayerSchema<LayerTags...>>> {
  using Bundle =
      schema::DiscreteLayerBundle<schema::LayerSchema<LayerTags...>>;

  static auto layers(Bundle& bundle) {
    return descriptorNamedDiscreteLayers(bundle);
  }

  static auto layers(const Bundle& bundle) {
    return descriptorNamedDiscreteLayers(bundle);
  }
};

}  // namespace wavemap::layered::io

#endif  // WAVEMAP_LAYERED_DESCRIPTOR_LAYERED_MAP_IO_H_
