#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_CONFIG_H_

#include <wavemap/layered/layered_map.h>
#include <wavemap/layered/layered_map_io.h>
#include "layered_voxel_config.h"

using wavemap::layered::DiscreteCompressionConfig;
using wavemap::layered::DiscreteLayer;
using wavemap::layered::LayeredMap;

struct ExampleDiscreteLayers {
  explicit ExampleDiscreteLayers(
      DiscreteCompressionConfig config = DiscreteCompressionConfig())
      : semantic(config), changed(config) {}

  DiscreteLayer<int> semantic;
  DiscreteLayer<bool> changed;
};

using ExampleLayeredMap =
    LayeredMap<ContinuousLayers, ContinuousLayersPolicy,
               ExampleDiscreteLayers>;
using ExampleLayeredMapConfig = ExampleLayeredMap::Config;
using ExampleLayeredMapIo =
    wavemap::layered::io::LayeredMapIo<ExampleLayeredMap, LayeredVoxelSerializer>;

namespace wavemap::layered::io {
template <>
struct DiscreteLayerBundleTraits<::ExampleDiscreteLayers> {
  static auto layers(ExampleDiscreteLayers& layers) {
    return std::make_tuple(namedDiscreteLayer("semantic", layers.semantic),
                           namedDiscreteLayer("changed", layers.changed));
  }

  static auto layers(const ExampleDiscreteLayers& layers) {
    return std::make_tuple(namedDiscreteLayer("semantic", layers.semantic),
                           namedDiscreteLayer("changed", layers.changed));
  }
};
}  // namespace wavemap::layered::io

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_CONFIG_H_
