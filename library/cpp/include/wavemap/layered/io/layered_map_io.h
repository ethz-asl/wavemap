#ifndef WAVEMAP_LAYERED_LAYERED_MAP_IO_H_
#define WAVEMAP_LAYERED_LAYERED_MAP_IO_H_

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include <wavemap/io/stream_conversions.h>

#include <wavemap/layered/map/discrete_layer.h>
#include <wavemap/layered/map/layered_map.h>
#include <wavemap/layered/io/layered_map_schema.h>

namespace wavemap::layered::io {
namespace detail {
template <typename T>
inline constexpr bool kAlwaysFalse = false;

constexpr char kMagic[] = "LWVMP";
constexpr uint32_t kVersion = 1u;
constexpr char kSchemaSection[] = "schema";
constexpr char kContinuousMapSection[] = "continuous_map";
constexpr char kDiscreteLayersSection[] = "discrete_layers";

inline bool writeBytes(std::ostream& ostream, const char* data, size_t size) {
  ostream.write(data, static_cast<std::streamsize>(size));
  return static_cast<bool>(ostream);
}

inline bool readBytes(std::istream& istream, char* data, size_t size) {
  istream.read(data, static_cast<std::streamsize>(size));
  return static_cast<bool>(istream);
}

template <typename T>
bool writePod(std::ostream& ostream, const T& value) {
  static_assert(std::is_trivially_copyable_v<T>);
  return writeBytes(ostream, reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool readPod(std::istream& istream, T& value) {
  static_assert(std::is_trivially_copyable_v<T>);
  return readBytes(istream, reinterpret_cast<char*>(&value), sizeof(T));
}

inline bool writeString(std::ostream& ostream, const std::string& value) {
  const uint64_t size = value.size();
  return writePod(ostream, size) && writeBytes(ostream, value.data(), value.size());
}

inline bool readString(std::istream& istream, std::string& value) {
  uint64_t size = 0u;
  if (!readPod(istream, size)) {
    return false;
  }
  value.resize(size);
  return readBytes(istream, value.data(), value.size());
}

inline bool writeSectionMarker(std::ostream& ostream,
                               const std::string& section_name) {
  return writeString(ostream, section_name);
}

inline bool readSectionMarker(std::istream& istream,
                              const std::string& expected_section_name) {
  std::string section_name;
  return readString(istream, section_name) &&
         section_name == expected_section_name;
}

inline bool writeFileHeader(std::ostream& ostream) {
  return writeString(ostream, kMagic) && writePod(ostream, kVersion);
}

inline bool readFileHeader(std::istream& istream, uint32_t& version) {
  std::string magic;
  version = 0u;
  return readString(istream, magic) && magic == kMagic &&
         readPod(istream, version) && version == kVersion;
}
}  // namespace detail

inline bool writeLayerSchemaEntries(
    std::ostream& ostream, const std::vector<LayerSchemaEntry>& entries) {
  const uint64_t entry_count = entries.size();
  if (!detail::writePod(ostream, entry_count)) {
    return false;
  }
  for (const auto& entry : entries) {
    if (!detail::writeString(ostream, entry.name) ||
        !detail::writeString(ostream, entry.type)) {
      return false;
    }
  }
  return static_cast<bool>(ostream);
}

inline bool readLayerSchemaEntries(
    std::istream& istream, std::vector<LayerSchemaEntry>& entries) {
  uint64_t entry_count = 0u;
  if (!detail::readPod(istream, entry_count)) {
    return false;
  }
  entries.clear();
  entries.reserve(entry_count);
  for (uint64_t entry_i = 0u; entry_i < entry_count; ++entry_i) {
    auto& entry = entries.emplace_back();
    if (!detail::readString(istream, entry.name) ||
        !detail::readString(istream, entry.type)) {
      return false;
    }
  }
  return true;
}

inline bool writeLayeredMapSchema(std::ostream& ostream,
                                  const LayeredMapSchema& schema) {
  return writeLayerSchemaEntries(ostream, schema.continuous_layers) &&
         writeLayerSchemaEntries(ostream, schema.discrete_layers);
}

inline bool readLayeredMapSchema(std::istream& istream,
                                 LayeredMapSchema& schema) {
  return readLayerSchemaEntries(istream, schema.continuous_layers) &&
         readLayerSchemaEntries(istream, schema.discrete_layers);
}

inline bool readLayeredMapSchema(const std::filesystem::path& file_path,
                                 LayeredMapSchema& schema) {
  std::ifstream istream(file_path, std::ios::binary);
  if (!istream.is_open()) {
    return false;
  }

  uint32_t version = 0u;
  if (!detail::readFileHeader(istream, version)) {
    return false;
  }

  return detail::readSectionMarker(istream, detail::kSchemaSection) &&
         readLayeredMapSchema(istream, schema);
}

template <typename ValueT>
struct DiscreteValueSerializer {
  static constexpr const char* typeName() {
    static_assert(detail::kAlwaysFalse<ValueT>,
                  "DiscreteValueSerializer<T> must be specialized before "
                  "DiscreteLayer<T> can be saved or loaded. Add a serializer "
                  "for this value type, for example by following the int/bool "
                  "specializations in layered_map_io.h.");
    return "unsupported";
  }

  static bool write(std::ostream&, const ValueT&) {
    static_assert(detail::kAlwaysFalse<ValueT>,
                  "DiscreteValueSerializer<T>::write is not specialized for "
                  "this discrete value type.");
    return false;
  }

  static bool read(std::istream&, ValueT&) {
    static_assert(detail::kAlwaysFalse<ValueT>,
                  "DiscreteValueSerializer<T>::read is not specialized for "
                  "this discrete value type.");
    return false;
  }
};

template <>
struct DiscreteValueSerializer<int> {
  static constexpr const char* typeName() { return "int"; }

  static bool write(std::ostream& ostream, int value) {
    const int32_t serialized = value;
    return detail::writePod(ostream, serialized);
  }

  static bool read(std::istream& istream, int& value) {
    int32_t serialized = 0;
    if (!detail::readPod(istream, serialized)) {
      return false;
    }
    value = serialized;
    return true;
  }
};

template <>
struct DiscreteValueSerializer<bool> {
  static constexpr const char* typeName() { return "bool"; }

  static bool write(std::ostream& ostream, bool value) {
    const uint8_t serialized = value ? 1u : 0u;
    return detail::writePod(ostream, serialized);
  }

  static bool read(std::istream& istream, bool& value) {
    uint8_t serialized = 0u;
    if (!detail::readPod(istream, serialized)) {
      return false;
    }
    value = serialized != 0u;
    return true;
  }
};

template <typename ValueT>
struct NamedDiscreteLayer {
  std::string name;
  DiscreteLayer<ValueT>& layer;
};

template <typename ValueT>
struct ConstNamedDiscreteLayer {
  std::string name;
  const DiscreteLayer<ValueT>& layer;
};

template <typename ValueT>
NamedDiscreteLayer<ValueT> namedDiscreteLayer(const std::string& name,
                                              DiscreteLayer<ValueT>& layer) {
  return {name, layer};
}

template <typename ValueT>
ConstNamedDiscreteLayer<ValueT> namedDiscreteLayer(
    const std::string& name, const DiscreteLayer<ValueT>& layer) {
  return {name, layer};
}

template <typename DiscreteLayersT>
struct DiscreteLayerBundleTraits {
  static auto layers(DiscreteLayersT&) {
    static_assert(detail::kAlwaysFalse<DiscreteLayersT>,
                  "DiscreteLayerBundleTraits<T> must be specialized for your "
                  "discrete layer bundle. Return a tuple of "
                  "namedDiscreteLayer(name, layers.layer_field) entries.");
    return std::tuple<>();
  }

  static auto layers(const DiscreteLayersT&) {
    static_assert(detail::kAlwaysFalse<DiscreteLayersT>,
                  "DiscreteLayerBundleTraits<T> must be specialized for your "
                  "discrete layer bundle. Return a tuple of "
                  "namedDiscreteLayer(name, layers.layer_field) entries.");
    return std::tuple<>();
  }
};

template <typename ValueT>
struct DiscreteLayerSerializer {
  static bool write(std::ostream& ostream, const DiscreteLayer<ValueT>& layer) {
    const int32_t block_height = layer.config().block_height;
    if (!detail::writePod(ostream, block_height)) {
      return false;
    }

    const uint64_t parent_count = layer.cells().size();
    if (!detail::writePod(ostream, parent_count)) {
      return false;
    }

    for (const auto& [parent_key, cell] : layer.cells()) {
      const int32_t x = parent_key.x;
      const int32_t y = parent_key.y;
      const int32_t z = parent_key.z;
      if (!detail::writePod(ostream, x) || !detail::writePod(ostream, y) ||
          !detail::writePod(ostream, z) ||
          !DiscreteValueSerializer<ValueT>::write(ostream, cell.dominant_value)) {
        return false;
      }

      const uint64_t observed_count = cell.observed_offsets.size();
      if (!detail::writePod(ostream, observed_count)) {
        return false;
      }
      for (const int offset : cell.observed_offsets) {
        const int32_t serialized_offset = offset;
        if (!detail::writePod(ostream, serialized_offset)) {
          return false;
        }
      }

      const uint64_t exception_count = cell.exceptions.size();
      if (!detail::writePod(ostream, exception_count)) {
        return false;
      }
      for (const auto& [offset, value] : cell.exceptions) {
        const int32_t serialized_offset = offset;
        if (!detail::writePod(ostream, serialized_offset) ||
            !DiscreteValueSerializer<ValueT>::write(ostream, value)) {
          return false;
        }
      }
    }
    return static_cast<bool>(ostream);
  }

  static bool read(std::istream& istream, DiscreteLayer<ValueT>& layer) {
    int32_t block_height = 0;
    uint64_t parent_count = 0u;
    if (!detail::readPod(istream, block_height) ||
        !detail::readPod(istream, parent_count)) {
      return false;
    }

    std::map<IndexKey, typename DiscreteLayer<ValueT>::Cell> cells;
    for (uint64_t parent_i = 0u; parent_i < parent_count; ++parent_i) {
      int32_t x = 0;
      int32_t y = 0;
      int32_t z = 0;
      typename DiscreteLayer<ValueT>::Cell cell;
      if (!detail::readPod(istream, x) || !detail::readPod(istream, y) ||
          !detail::readPod(istream, z) ||
          !DiscreteValueSerializer<ValueT>::read(istream, cell.dominant_value)) {
        return false;
      }

      uint64_t observed_count = 0u;
      if (!detail::readPod(istream, observed_count)) {
        return false;
      }
      for (uint64_t observed_i = 0u; observed_i < observed_count; ++observed_i) {
        int32_t offset = 0;
        if (!detail::readPod(istream, offset)) {
          return false;
        }
        cell.observed_offsets.insert(offset);
      }

      uint64_t exception_count = 0u;
      if (!detail::readPod(istream, exception_count)) {
        return false;
      }
      for (uint64_t exception_i = 0u; exception_i < exception_count;
           ++exception_i) {
        int32_t offset = 0;
        ValueT value{};
        if (!detail::readPod(istream, offset) ||
            !DiscreteValueSerializer<ValueT>::read(istream, value)) {
          return false;
        }
        cell.exceptions[offset] = value;
      }

      cells[IndexKey(wavemap::Index3D(x, y, z))] = cell;
    }

    layer.replaceCellsForLoad(DiscreteCompressionConfig{block_height},
                              std::move(cells));
    return true;
  }
};

template <typename TupleT, typename FuncT, size_t... Indices>
bool forEachTupleElementImpl(TupleT&& tuple, FuncT&& func,
                             std::index_sequence<Indices...>) {
  bool ok = true;
  ((ok = ok && func(std::get<Indices>(tuple))), ...);
  return ok;
}

template <typename TupleT, typename FuncT>
bool forEachTupleElement(TupleT&& tuple, FuncT&& func) {
  constexpr size_t tuple_size =
      std::tuple_size_v<std::remove_reference_t<TupleT>>;
  return forEachTupleElementImpl(std::forward<TupleT>(tuple),
                                 std::forward<FuncT>(func),
                                 std::make_index_sequence<tuple_size>{});
}

template <typename ValueT>
LayerSchemaEntry discreteLayerSchemaEntry(
    const ConstNamedDiscreteLayer<ValueT>& named_layer) {
  return {named_layer.name, DiscreteValueSerializer<ValueT>::typeName()};
}

template <typename ValueT>
LayerSchemaEntry discreteLayerSchemaEntry(
    const NamedDiscreteLayer<ValueT>& named_layer) {
  return {named_layer.name, DiscreteValueSerializer<ValueT>::typeName()};
}

template <typename DiscreteLayersT>
std::vector<LayerSchemaEntry> discreteLayerBundleSchema(
    const DiscreteLayersT& layers) {
  std::vector<LayerSchemaEntry> schema;
  const auto named_layers =
      DiscreteLayerBundleTraits<DiscreteLayersT>::layers(layers);
  schema.reserve(
      std::tuple_size_v<std::remove_reference_t<decltype(named_layers)>>);
  forEachTupleElement(named_layers, [&](const auto& named_layer) {
    schema.emplace_back(discreteLayerSchemaEntry(named_layer));
    return true;
  });
  return schema;
}

template <typename LayeredMapT>
LayeredMapSchema layeredMapSchema(const LayeredMapT& map) {
  return {continuousLayerSchema<typename LayeredMapT::ContinuousLayers>(),
          discreteLayerBundleSchema(map.discreteLayers())};
}

template <typename DiscreteLayersT>
bool writeDiscreteLayerBundle(std::ostream& ostream,
                              const DiscreteLayersT& layers) {
  const auto named_layers = DiscreteLayerBundleTraits<DiscreteLayersT>::layers(layers);
  constexpr uint64_t layer_count =
      std::tuple_size_v<std::remove_reference_t<decltype(named_layers)>>;
  if (!detail::writePod(ostream, layer_count)) {
    return false;
  }
  return forEachTupleElement(named_layers, [&](const auto& named_layer) {
    using ValueT = typename std::remove_reference_t<decltype(named_layer.layer)>::Value;
    return detail::writeString(ostream, named_layer.name) &&
           detail::writeString(ostream, DiscreteValueSerializer<ValueT>::typeName()) &&
           DiscreteLayerSerializer<ValueT>::write(ostream, named_layer.layer);
  });
}

template <typename DiscreteLayersT>
bool readDiscreteLayerBundle(std::istream& istream, DiscreteLayersT& layers) {
  const auto named_layers = DiscreteLayerBundleTraits<DiscreteLayersT>::layers(layers);
  constexpr uint64_t expected_layer_count =
      std::tuple_size_v<std::remove_reference_t<decltype(named_layers)>>;
  uint64_t layer_count = 0u;
  if (!detail::readPod(istream, layer_count) ||
      layer_count != expected_layer_count) {
    return false;
  }

  return forEachTupleElement(named_layers, [&](auto named_layer) {
    using ValueT = typename std::remove_reference_t<decltype(named_layer.layer)>::Value;
    std::string name;
    std::string type_name;
    if (!detail::readString(istream, name) ||
        !detail::readString(istream, type_name)) {
      return false;
    }
    if (name != named_layer.name ||
        type_name != DiscreteValueSerializer<ValueT>::typeName()) {
      return false;
    }
    return DiscreteLayerSerializer<ValueT>::read(istream, named_layer.layer);
  });
}

template <typename LayeredMapT, typename ContinuousCellSerializerT>
bool saveLayeredMap(const std::filesystem::path& file_path,
                    const LayeredMapT& map) {
  std::ofstream ostream(file_path, std::ios::binary);
  if (!ostream.is_open()) {
    return false;
  }

  const LayeredMapSchema schema = layeredMapSchema(map);
  if (!detail::writeFileHeader(ostream) ||
      !detail::writeSectionMarker(ostream, detail::kSchemaSection) ||
      !writeLayeredMapSchema(ostream, schema) ||
      !detail::writeSectionMarker(ostream, detail::kContinuousMapSection)) {
    return false;
  }
  if (!wavemap::io::mapToStream<typename LayeredMapT::ContinuousVoxel,
                                ContinuousCellSerializerT>(
          map.continuousMap(), ostream)) {
    return false;
  }
  if (!detail::writeSectionMarker(ostream, detail::kDiscreteLayersSection)) {
    return false;
  }
  return writeDiscreteLayerBundle(ostream, map.discreteLayers());
}

template <typename LayeredMapT, typename ContinuousCellSerializerT>
bool loadLayeredMap(const std::filesystem::path& file_path, LayeredMapT& map,
                    std::string* error_message = nullptr) {
  std::ifstream istream(file_path, std::ios::binary);
  if (!istream.is_open()) {
    if (error_message) {
      *error_message = "Could not open LayeredMap file: " + file_path.string();
    }
    return false;
  }

  uint32_t version = 0u;
  if (!detail::readFileHeader(istream, version)) {
    if (error_message) {
      *error_message = "Invalid LayeredMap file header or unsupported version.";
    }
    return false;
  }

  LayeredMapSchema schema;
  if (!detail::readSectionMarker(istream, detail::kSchemaSection) ||
      !readLayeredMapSchema(istream, schema)) {
    if (error_message) {
      *error_message = "Failed to read LayeredMap schema section.";
    }
    return false;
  }

  const LayeredMapSchema expected_schema = layeredMapSchema(map);
  if (!checkLayeredMapSchemaCompatibility(schema, expected_schema,
                                          error_message)) {
    return false;
  }

  if (!detail::readSectionMarker(istream, detail::kContinuousMapSection)) {
    if (error_message) {
      *error_message = "Failed to find LayeredMap continuous map section.";
    }
    return false;
  }

  typename LayeredMapT::ContinuousMap::Ptr continuous_map;
  if (!wavemap::io::streamToMap<typename LayeredMapT::ContinuousVoxel,
                                ContinuousCellSerializerT>(istream,
                                                           continuous_map) ||
      !continuous_map) {
    if (error_message) {
      *error_message = "Failed to deserialize LayeredMap continuous map data.";
    }
    return false;
  }

  typename LayeredMapT::DiscreteLayers discrete_layers;
  if (!detail::readSectionMarker(istream, detail::kDiscreteLayersSection) ||
      !readDiscreteLayerBundle(istream, discrete_layers)) {
    if (error_message) {
      *error_message = "Failed to deserialize LayeredMap discrete layer data.";
    }
    return false;
  }

  // Keep the destination map object and its continuous-map allocation alive.
  // The ROS server's occupancy pipeline and layered integrators may retain a
  // shared pointer to this exact octree. Replacing LayeredMap here would leave
  // those components writing into the old allocation and would also discard
  // the destination's runtime threshold and pruning policies, which are not
  // part of the serialized map payload.
  auto& destination_continuous_map = map.continuousMap();
  if (continuous_map->getConfig() != destination_continuous_map.getConfig()) {
    if (error_message) {
      *error_message =
          "Loaded LayeredMap geometry is incompatible with the configured "
          "destination map.";
    }
    return false;
  }

  destination_continuous_map.getHashMap() =
      std::move(continuous_map->getHashMap());
  map.discreteLayers() = std::move(discrete_layers);
  return true;
}


template <typename LayeredMapT, typename ContinuousCellSerializerT>
struct LayeredMapIo {
  static bool save(const std::filesystem::path& file_path,
                   const LayeredMapT& map) {
    return saveLayeredMap<LayeredMapT, ContinuousCellSerializerT>(file_path,
                                                                  map);
  }

  static bool load(const std::filesystem::path& file_path, LayeredMapT& map,
                   std::string* error_message = nullptr) {
    return loadLayeredMap<LayeredMapT, ContinuousCellSerializerT>(
        file_path, map, error_message);
  }
};

}  // namespace wavemap::layered::io

#endif  // WAVEMAP_LAYERED_LAYERED_MAP_IO_H_
