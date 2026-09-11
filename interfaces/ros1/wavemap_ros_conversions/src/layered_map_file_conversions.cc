#include "wavemap_ros_conversions/layered_map_file_conversions.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <wavemap/io/streamable_types.h>
#include <wavemap/layered/io/layered_map_schema.h>
#include <wavemap_msgs/DiscreteLayerCell.h>
#include <wavemap_msgs/Index3D.h>
#include <wavemap_msgs/Layer.h>
#include <wavemap_msgs/LayeredHashedWaveletOctree.h>
#include <wavemap_msgs/LayeredHashedWaveletOctreeBlock.h>
#include <wavemap_msgs/LayeredWaveletOctreeNode.h>

namespace wavemap::convert {
namespace {
using layered::LayerSchemaEntry;
using layered::LayeredMapSchema;

constexpr uint64_t kMaxLayerCount = 1024u;
constexpr uint64_t kMaxNameLength = 4096u;
constexpr uint64_t kMaxBlockCount = 10'000'000u;
constexpr uint64_t kMaxNodeCount = 100'000'000u;
constexpr uint64_t kMaxDiscreteCellCount = 100'000'000u;
constexpr uint64_t kMaxOffsetsPerCell = 100'000'000u;

bool fail(const std::string& message, std::string* error_message) {
  if (error_message) {
    *error_message = message;
  }
  return false;
}

template <typename T>
bool readPod(std::istream& stream, T& value) {
  static_assert(std::is_trivially_copyable_v<T>);
  stream.read(reinterpret_cast<char*>(&value), sizeof(value));
  return static_cast<bool>(stream);
}

bool readString(std::istream& stream, std::string& value) {
  uint64_t size = 0u;
  if (!readPod(stream, size) || kMaxNameLength < size) {
    return false;
  }
  value.resize(static_cast<size_t>(size));
  stream.read(value.data(), static_cast<std::streamsize>(size));
  return static_cast<bool>(stream);
}

bool readSection(std::istream& stream, const std::string& expected) {
  std::string actual;
  return readString(stream, actual) && actual == expected;
}

bool readSchemaEntries(std::istream& stream,
                       std::vector<LayerSchemaEntry>& entries) {
  uint64_t count = 0u;
  if (!readPod(stream, count) || kMaxLayerCount < count) {
    return false;
  }
  entries.clear();
  entries.reserve(static_cast<size_t>(count));
  for (uint64_t index = 0u; index < count; ++index) {
    auto& entry = entries.emplace_back();
    if (!readString(stream, entry.name) || !readString(stream, entry.type)) {
      return false;
    }
  }
  return true;
}

bool readSchema(std::istream& stream, LayeredMapSchema& schema) {
  return readSchemaEntries(stream, schema.continuous_layers) &&
         readSchemaEntries(stream, schema.discrete_layers);
}

std::string rosTypeForStreamType(const std::string& stream_type) {
  if (stream_type == "float32_weighted_mean_state") {
    return "float32";
  }
  return stream_type;
}

bool isSupportedContinuousType(const std::string& type) {
  return type == "float32" || type == "float32_rgb" ||
         type == "float32_weighted_mean_state";
}

std::vector<wavemap_msgs::Layer> makeLayers(
    const std::vector<LayerSchemaEntry>& layers) {
  std::vector<wavemap_msgs::Layer> result;
  result.reserve(layers.size());
  for (const auto& layer : layers) {
    auto& layer_msg = result.emplace_back();
    layer_msg.name = layer.name;
    layer_msg.type = rosTypeForStreamType(layer.type);
  }
  return result;
}

bool appendContinuousValue(std::istream& stream,
                           const LayerSchemaEntry& schema,
                           wavemap_msgs::Layer& layer) {
  if (schema.type == "float32") {
    float value = 0.f;
    if (!readPod(stream, value)) {
      return false;
    }
    layer.float32_values.emplace_back(value);
    return true;
  }
  if (schema.type == "float32_rgb") {
    std::array<float, 3> value{};
    for (float& component : value) {
      if (!readPod(stream, component)) {
        return false;
      }
      layer.float32_values.emplace_back(component);
    }
    return true;
  }
  if (schema.type == "float32_weighted_mean_state") {
    float weighted_sum = 0.f;
    float total_weight = 0.f;
    if (!readPod(stream, weighted_sum) || !readPod(stream, total_weight)) {
      return false;
    }
    layer.float32_values.emplace_back(
        0.f < total_weight ? weighted_sum / total_weight : 0.f);
    return true;
  }
  return false;
}

bool readContinuousCell(std::istream& stream,
                        const std::vector<LayerSchemaEntry>& custom_layers,
                        float& occupancy,
                        std::vector<wavemap_msgs::Layer>& layers) {
  if (!readPod(stream, occupancy)) {
    return false;
  }
  for (size_t layer_index = 0u; layer_index < custom_layers.size();
       ++layer_index) {
    if (!appendContinuousValue(stream, custom_layers[layer_index],
                               layers[layer_index])) {
      return false;
    }
  }
  return true;
}

wavemap_msgs::Index3D toRosIndex(
    const wavemap::io::streamable::Index3D& index) {
  wavemap_msgs::Index3D msg;
  msg.x = index.x;
  msg.y = index.y;
  msg.z = index.z;
  return msg;
}

unsigned int countSetBits(uint8_t value) {
  unsigned int count = 0u;
  while (value != 0u) {
    count += value & 1u;
    value >>= 1u;
  }
  return count;
}

bool readContinuousMap(std::istream& stream,
                       const std::vector<LayerSchemaEntry>& custom_layers,
                       wavemap_msgs::LayeredHashedWaveletOctree& msg,
                       std::string* error_message) {
  const auto storage_format =
      wavemap::io::streamable::StorageFormat::read(stream);
  if (!stream || storage_format !=
                     wavemap::io::streamable::StorageFormat::
                         kLayeredHashedWaveletOctree) {
    return fail("The file does not contain a supported layered hashed-wavelet "
                "continuous map.", error_message);
  }

  const auto header =
      wavemap::io::streamable::HashedWaveletOctreeHeader::read(stream);
  if (!stream || !std::isfinite(header.min_cell_width) ||
      header.min_cell_width <= 0.f || !std::isfinite(header.min_log_odds) ||
      !std::isfinite(header.max_log_odds) ||
      header.max_log_odds < header.min_log_odds || header.tree_height < 0 ||
      30 < header.tree_height ||
      kMaxBlockCount < header.num_blocks) {
    return fail("The layered continuous-map header is invalid.", error_message);
  }
  msg.min_cell_width = header.min_cell_width;
  msg.min_log_odds = header.min_log_odds;
  msg.max_log_odds = header.max_log_odds;
  msg.tree_height = header.tree_height;
  msg.layer_names.reserve(custom_layers.size());
  msg.layer_types.reserve(custom_layers.size());
  for (const auto& layer : custom_layers) {
    msg.layer_names.emplace_back(layer.name);
    msg.layer_types.emplace_back(rosTypeForStreamType(layer.type));
  }
  uint64_t total_node_count = 0u;
  for (uint64_t block_index = 0u; block_index < header.num_blocks;
       ++block_index) {
    const auto serialized_index =
        wavemap::io::streamable::Index3D::read(stream);
    if (!stream) {
      return fail("The layered map ended while reading a block index.",
                  error_message);
    }
    const auto ros_index = toRosIndex(serialized_index);
    msg.allocated_block_indices.emplace_back(ros_index);
    auto& block = msg.blocks.emplace_back();
    block.root_node_offset = ros_index;
    block.root_node_layers = makeLayers(custom_layers);
    if (!readContinuousCell(stream, custom_layers,
                            block.root_node_occupancy_scale_coefficient,
                            block.root_node_layers)) {
      return fail("The layered map ended while reading a block root value.",
                  error_message);
    }

    std::vector<int32_t> pending_node_heights{header.tree_height};
    while (!pending_node_heights.empty()) {
      if (kMaxNodeCount <= total_node_count) {
        return fail("The layered map contains too many wavelet nodes.",
                    error_message);
      }
      ++total_node_count;
      const int32_t node_height = pending_node_heights.back();
      pending_node_heights.pop_back();
      auto& node = block.nodes.emplace_back();
      node.detail_layers = makeLayers(custom_layers);
      for (size_t coefficient_index = 0u;
           coefficient_index + 1u <
               node.occupancy_detail_coefficients.size();
           ++coefficient_index) {
        if (!readContinuousCell(
                stream, custom_layers,
                node.occupancy_detail_coefficients[coefficient_index],
                node.detail_layers)) {
          return fail("The layered map ended while reading wavelet "
                      "coefficients.", error_message);
        }
      }
      if (!readPod(stream, node.allocated_children_bitset)) {
        return fail("The layered map ended while reading its wavelet tree.",
                    error_message);
      }
      const unsigned int child_count =
          countSetBits(node.allocated_children_bitset);
      if (node_height == 0 && child_count != 0u) {
        return fail("The layered map has children below its minimum cell "
                    "height.", error_message);
      }
      pending_node_heights.insert(pending_node_heights.end(), child_count,
                                  node_height - 1);
    }
  }
  return true;
}

template <typename ValueT>
bool readDiscreteCells(std::istream& stream,
                       wavemap_msgs::DiscreteLayer& layer,
                       std::string* error_message) {
  int32_t block_height = 0;
  uint64_t cell_count = 0u;
  if (!readPod(stream, block_height) || block_height < 0 ||
      10 < block_height || !readPod(stream, cell_count) ||
      kMaxDiscreteCellCount < cell_count) {
    return fail("A discrete-layer header is invalid.", error_message);
  }
  layer.block_height = block_height;
  const int32_t max_offset = 1 << (3 * block_height);
  for (uint64_t cell_index = 0u; cell_index < cell_count; ++cell_index) {
    auto& cell = layer.cells.emplace_back();
    if (!readPod(stream, cell.parent_index.x) ||
        !readPod(stream, cell.parent_index.y) ||
        !readPod(stream, cell.parent_index.z)) {
      return fail("The file ended while reading a discrete cell index.",
                  error_message);
    }
    ValueT dominant{};
    if (!readPod(stream, dominant)) {
      return fail("The file ended while reading a discrete cell value.",
                  error_message);
    }
    if constexpr (std::is_same_v<ValueT, int32_t>) {
      cell.dominant_int32 = dominant;
    } else {
      if (1u < dominant) {
        return fail("A boolean discrete layer contains a non-boolean value.",
                    error_message);
      }
      cell.dominant_uint8 = dominant;
    }

    uint64_t observed_count = 0u;
    if (!readPod(stream, observed_count) ||
        kMaxOffsetsPerCell < observed_count) {
      return fail("A discrete cell's observed-offset list is invalid.",
                  error_message);
    }
    cell.observed_offsets.resize(static_cast<size_t>(observed_count));
    for (int32_t& offset : cell.observed_offsets) {
      if (!readPod(stream, offset) || offset < 0 || max_offset <= offset) {
        return fail("A discrete cell contains an invalid observed offset.",
                    error_message);
      }
    }

    uint64_t exception_count = 0u;
    if (!readPod(stream, exception_count) ||
        kMaxOffsetsPerCell < exception_count) {
      return fail("A discrete cell's exception list is invalid.",
                  error_message);
    }
    cell.exception_offsets.resize(static_cast<size_t>(exception_count));
    if constexpr (std::is_same_v<ValueT, int32_t>) {
      cell.exception_int32_values.reserve(static_cast<size_t>(exception_count));
    } else {
      cell.exception_uint8_values.reserve(static_cast<size_t>(exception_count));
    }
    for (size_t exception_index = 0u;
         exception_index < static_cast<size_t>(exception_count);
         ++exception_index) {
      ValueT value{};
      if (!readPod(stream, cell.exception_offsets[exception_index]) ||
          cell.exception_offsets[exception_index] < 0 ||
          max_offset <= cell.exception_offsets[exception_index] ||
          !readPod(stream, value)) {
        return fail("A discrete cell contains an invalid exception.",
                    error_message);
      }
      if constexpr (std::is_same_v<ValueT, int32_t>) {
        cell.exception_int32_values.emplace_back(value);
      } else {
        if (1u < value) {
          return fail("A boolean discrete layer contains a non-boolean value.",
                      error_message);
        }
        cell.exception_uint8_values.emplace_back(value);
      }
    }
  }
  return true;
}

bool readDiscreteLayers(std::istream& stream, const LayeredMapSchema& schema,
                        std::vector<wavemap_msgs::DiscreteLayer>& layers,
                        std::string* error_message) {
  uint64_t layer_count = 0u;
  if (!readPod(stream, layer_count) ||
      layer_count != schema.discrete_layers.size()) {
    return fail("The discrete-layer data does not match the file schema.",
                error_message);
  }
  layers.reserve(static_cast<size_t>(layer_count));
  for (size_t layer_index = 0u; layer_index < static_cast<size_t>(layer_count);
       ++layer_index) {
    std::string name;
    std::string type;
    if (!readString(stream, name) || !readString(stream, type) ||
        name != schema.discrete_layers[layer_index].name ||
        type != schema.discrete_layers[layer_index].type) {
      return fail("The discrete-layer data does not match the file schema.",
                  error_message);
    }
    auto& layer = layers.emplace_back();
    layer.name = name;
    layer.value_type = type;
    if (type == "int") {
      if (!readDiscreteCells<int32_t>(stream, layer, error_message)) {
        return false;
      }
    } else if (type == "bool") {
      if (!readDiscreteCells<uint8_t>(stream, layer, error_message)) {
        return false;
      }
    } else {
      return fail("Unsupported discrete codec '" + type + "' for layer '" +
                      name + "'. Rebuild RViz with that layer's schema and "
                             "codec registered.",
                  error_message);
    }
  }
  return true;
}
}  // namespace

bool layeredMapFileToRosMsg(const std::filesystem::path& file_path,
                            const std::string& frame_id,
                            const ros::Time& stamp,
                            wavemap_msgs::LayeredMap& msg,
                            std::string* error_message) {
  std::ifstream stream(file_path, std::ios::binary);
  if (!stream.is_open()) {
    return fail("Could not open layered map file: " + file_path.string(),
                error_message);
  }

  std::string magic;
  uint32_t version = 0u;
  if (!readString(stream, magic) || magic != "LWVMP" ||
      !readPod(stream, version)) {
    return fail("Invalid layered map file header.", error_message);
  }
  if (version != 1u) {
    return fail("Unsupported layered map file version " +
                    std::to_string(version) + ".",
                error_message);
  }

  LayeredMapSchema schema;
  if (!readSection(stream, "schema") || !readSchema(stream, schema)) {
    return fail("Failed to read the layered map schema.", error_message);
  }
  if (schema.continuous_layers.empty() ||
      schema.continuous_layers.front() !=
          LayerSchemaEntry{"occupancy", "float32"}) {
    return fail("The file does not have the required occupancy layer.",
                error_message);
  }
  const std::vector<LayerSchemaEntry> custom_layers(
      schema.continuous_layers.begin() + 1,
      schema.continuous_layers.end());
  for (const auto& layer : custom_layers) {
    if (!isSupportedContinuousType(layer.type)) {
      return fail("Unsupported continuous codec '" + layer.type +
                      "' for layer '" + layer.name +
                      "'. Rebuild RViz with that layer's schema and codec "
                      "registered.",
                  error_message);
    }
  }
  for (const auto& layer : schema.discrete_layers) {
    if (layer.type != "int" && layer.type != "bool") {
      return fail("Unsupported discrete codec '" + layer.type +
                      "' for layer '" + layer.name +
                      "'. Rebuild RViz with that layer's schema and codec "
                      "registered.",
                  error_message);
    }
  }

  wavemap_msgs::LayeredMap loaded;
  loaded.header.frame_id = frame_id;
  loaded.header.stamp = stamp;
  loaded.continuous_map.header = loaded.header;
  loaded.is_full_update = true;
  if (!readSection(stream, "continuous_map")) {
    return fail("Failed to find the layered continuous-map section.",
                error_message);
  }
  auto& continuous =
      loaded.continuous_map.layered_hashed_wavelet_octree.emplace_back();
  if (!readContinuousMap(stream, custom_layers, continuous, error_message)) {
    return false;
  }
  if (!readSection(stream, "discrete_layers")) {
    return fail("Failed to find the layered discrete-layers section.",
                error_message);
  }
  if (!readDiscreteLayers(stream, schema, loaded.discrete_layers,
                          error_message)) {
    return false;
  }
  msg = std::move(loaded);
  return true;
}

}  // namespace wavemap::convert
