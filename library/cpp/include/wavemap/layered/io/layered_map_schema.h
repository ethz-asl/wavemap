#ifndef WAVEMAP_LAYERED_LAYERED_MAP_SCHEMA_H_
#define WAVEMAP_LAYERED_LAYERED_MAP_SCHEMA_H_

#include <sstream>
#include <string>
#include <vector>

namespace wavemap::layered {
namespace detail {
template <typename T>
inline constexpr bool kLayeredMapSchemaAlwaysFalse = false;
}

struct LayerSchemaEntry {
  std::string name;
  std::string type;

  bool operator==(const LayerSchemaEntry& other) const {
    return name == other.name && type == other.type;
  }

  bool operator!=(const LayerSchemaEntry& other) const {
    return !(*this == other);
  }
};

struct LayeredMapSchema {
  std::vector<LayerSchemaEntry> continuous_layers;
  std::vector<LayerSchemaEntry> discrete_layers;

  bool operator==(const LayeredMapSchema& other) const {
    return continuous_layers == other.continuous_layers &&
           discrete_layers == other.discrete_layers;
  }

  bool operator!=(const LayeredMapSchema& other) const {
    return !(*this == other);
  }
};

inline std::string layerSchemaEntriesToString(
    const std::vector<LayerSchemaEntry>& entries) {
  std::ostringstream stream;
  if (entries.empty()) {
    stream << "none";
    return stream.str();
  }

  for (size_t entry_i = 0u; entry_i < entries.size(); ++entry_i) {
    if (entry_i != 0u) {
      stream << ", ";
    }
    stream << entries[entry_i].name << " (" << entries[entry_i].type << ")";
  }
  return stream.str();
}

inline std::string layeredMapSchemaToString(const LayeredMapSchema& schema) {
  std::ostringstream stream;
  stream << "continuous: "
         << layerSchemaEntriesToString(schema.continuous_layers)
         << "; discrete: "
         << layerSchemaEntriesToString(schema.discrete_layers);
  return stream.str();
}

inline bool checkLayeredMapSchemaCompatibility(
    const LayeredMapSchema& file_schema, const LayeredMapSchema& expected_schema,
    std::string* error_message = nullptr) {
  if (file_schema == expected_schema) {
    return true;
  }

  if (error_message) {
    std::ostringstream stream;
    stream << "Layered map schema mismatch. File has ["
           << layeredMapSchemaToString(file_schema) << "]; expected ["
           << layeredMapSchemaToString(expected_schema) << "].";
    *error_message = stream.str();
  }
  return false;
}

template <typename ContinuousLayersT>
struct ContinuousLayerSchemaTraits {
  static std::vector<LayerSchemaEntry> layers() {
    static_assert(detail::kLayeredMapSchemaAlwaysFalse<ContinuousLayersT>,
                  "ContinuousLayerSchemaTraits<T> must be specialized for "
                  "your continuous layer bundle. Return the custom "
                  "continuous layers, excluding occupancy, because "
                  "occupancy is part of every LayeredMap.");
    return {};
  }
};

inline std::vector<LayerSchemaEntry> occupancySchemaEntry() {
  return {{"occupancy", "float32"}};
}

template <typename ContinuousLayersT>
std::vector<LayerSchemaEntry> continuousLayerSchema() {
  auto layers = occupancySchemaEntry();
  const auto custom_layers =
      ContinuousLayerSchemaTraits<ContinuousLayersT>::layers();
  layers.insert(layers.end(), custom_layers.begin(), custom_layers.end());
  return layers;
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYERED_MAP_SCHEMA_H_
