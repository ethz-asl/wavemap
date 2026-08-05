#ifndef WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_MSG_CONVERSIONS_H_

#include <map>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <ros/time.h>
#include <wavemap/core/common.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_msgs/DiscreteLayer.h>
#include <wavemap_msgs/DiscreteLayerCell.h>
#include <wavemap_msgs/Index3D.h>
#include <wavemap_msgs/LayeredMap.h>

namespace wavemap::convert {
namespace detail {
template <typename T>
inline constexpr bool kLayeredMapRosAlwaysFalse = false;

template <typename IndexKeyT>
inline wavemap_msgs::Index3D indexKeyToRosMsg(const IndexKeyT& key) {
  wavemap_msgs::Index3D msg;
  msg.x = key.x;
  msg.y = key.y;
  msg.z = key.z;
  return msg;
}

template <typename IndexKeyT>
inline IndexKeyT rosMsgToIndexKey(const wavemap_msgs::Index3D& msg) {
  return IndexKeyT(wavemap::Index3D(msg.x, msg.y, msg.z));
}
}  // namespace detail

template <typename LayerT>
struct NamedDiscreteLayerForRos {
  std::string name;
  LayerT& layer;
};

template <typename LayerT>
NamedDiscreteLayerForRos<LayerT> namedDiscreteLayerForRos(
    std::string name, LayerT& layer) {
  return {std::move(name), layer};
}

template <typename DiscreteLayersT>
struct DiscreteLayerBundleRosTraits {
  static auto layers(DiscreteLayersT&) {
    static_assert(detail::kLayeredMapRosAlwaysFalse<DiscreteLayersT>,
                  "DiscreteLayerBundleRosTraits<T> must be specialized for "
                  "this discrete layer bundle type.");
  }
};

template <typename ValueT>
struct DiscreteRosValueConverter {
  static const char* typeName() {
    static_assert(detail::kLayeredMapRosAlwaysFalse<ValueT>,
                  "DiscreteRosValueConverter<T> must be specialized for this "
                  "discrete layer value type.");
    return "unsupported";
  }
};

template <>
struct DiscreteRosValueConverter<int> {
  static const char* typeName() { return "int"; }

  static void setDominantValue(wavemap_msgs::DiscreteLayerCell& msg,
                               int value) {
    msg.dominant_int32 = value;
  }

  static bool readDominantValue(const wavemap_msgs::DiscreteLayerCell& msg,
                                int& value) {
    value = msg.dominant_int32;
    return true;
  }

  static void appendExceptionValue(wavemap_msgs::DiscreteLayerCell& msg,
                                   int value) {
    msg.exception_int32_values.emplace_back(value);
  }

  static bool readExceptionValue(const wavemap_msgs::DiscreteLayerCell& msg,
                                 size_t index, int& value) {
    if (msg.exception_int32_values.size() <= index) {
      return false;
    }
    value = msg.exception_int32_values[index];
    return true;
  }

  static bool hasExpectedExceptionValueCount(
      const wavemap_msgs::DiscreteLayerCell& msg) {
    return msg.exception_int32_values.size() == msg.exception_offsets.size();
  }
};

template <>
struct DiscreteRosValueConverter<bool> {
  static const char* typeName() { return "bool"; }

  static void setDominantValue(wavemap_msgs::DiscreteLayerCell& msg,
                               bool value) {
    msg.dominant_uint8 = value ? 1u : 0u;
  }

  static bool readDominantValue(const wavemap_msgs::DiscreteLayerCell& msg,
                                bool& value) {
    value = msg.dominant_uint8 != 0u;
    return true;
  }

  static void appendExceptionValue(wavemap_msgs::DiscreteLayerCell& msg,
                                   bool value) {
    msg.exception_uint8_values.emplace_back(value ? 1u : 0u);
  }

  static bool readExceptionValue(const wavemap_msgs::DiscreteLayerCell& msg,
                                 size_t index, bool& value) {
    if (msg.exception_uint8_values.size() <= index) {
      return false;
    }
    value = msg.exception_uint8_values[index] != 0u;
    return true;
  }

  static bool hasExpectedExceptionValueCount(
      const wavemap_msgs::DiscreteLayerCell& msg) {
    return msg.exception_uint8_values.size() == msg.exception_offsets.size();
  }
};

template <typename DiscreteLayerT>
void discreteLayerToRosMsg(const std::string& name,
                           const DiscreteLayerT& layer,
                           wavemap_msgs::DiscreteLayer& msg) {
  using ValueT = typename DiscreteLayerT::Value;

  msg.name = name;
  msg.value_type = DiscreteRosValueConverter<ValueT>::typeName();
  msg.block_height = layer.config().block_height;
  msg.cells.clear();
  msg.cells.reserve(layer.cells().size());

  for (const auto& [parent_key, cell] : layer.cells()) {
    wavemap_msgs::DiscreteLayerCell cell_msg;
    cell_msg.parent_index = detail::indexKeyToRosMsg(parent_key);
    DiscreteRosValueConverter<ValueT>::setDominantValue(
        cell_msg, cell.dominant_value);

    cell_msg.observed_offsets.reserve(cell.observed_offsets.size());
    for (const int offset : cell.observed_offsets) {
      cell_msg.observed_offsets.emplace_back(offset);
    }

    cell_msg.exception_offsets.reserve(cell.exceptions.size());
    for (const auto& [offset, value] : cell.exceptions) {
      cell_msg.exception_offsets.emplace_back(offset);
      DiscreteRosValueConverter<ValueT>::appendExceptionValue(cell_msg, value);
    }

    msg.cells.emplace_back(std::move(cell_msg));
  }
}

template <typename DiscreteLayerT>
bool rosMsgToDiscreteLayer(const wavemap_msgs::DiscreteLayer& msg,
                           const std::string& expected_name,
                           DiscreteLayerT& layer) {
  using ValueT = typename DiscreteLayerT::Value;
  using CellsT = std::remove_cv_t<std::remove_reference_t<decltype(layer.cells())>>;
  using IndexKeyT = typename CellsT::key_type;
  using ConfigT = std::remove_cv_t<std::remove_reference_t<decltype(layer.config())>>;

  if (msg.name != expected_name ||
      msg.value_type != DiscreteRosValueConverter<ValueT>::typeName()) {
    return false;
  }

  std::map<IndexKeyT, typename DiscreteLayerT::Cell> cells;
  for (const wavemap_msgs::DiscreteLayerCell& cell_msg : msg.cells) {
    if (!DiscreteRosValueConverter<ValueT>::hasExpectedExceptionValueCount(
            cell_msg)) {
      return false;
    }

    typename DiscreteLayerT::Cell cell;
    if (!DiscreteRosValueConverter<ValueT>::readDominantValue(
            cell_msg, cell.dominant_value)) {
      return false;
    }

    for (const int offset : cell_msg.observed_offsets) {
      cell.observed_offsets.insert(offset);
    }

    for (size_t exception_i = 0u;
         exception_i < cell_msg.exception_offsets.size(); ++exception_i) {
      ValueT value{};
      if (!DiscreteRosValueConverter<ValueT>::readExceptionValue(
              cell_msg, exception_i, value)) {
        return false;
      }
      cell.exceptions[cell_msg.exception_offsets[exception_i]] = value;
    }

    cells[detail::rosMsgToIndexKey<IndexKeyT>(cell_msg.parent_index)] = cell;
  }

  layer.replaceCellsForLoad(ConfigT{msg.block_height}, std::move(cells));
  return true;
}

template <typename TupleT, typename FuncT, size_t... Indices>
bool forEachRosTupleElementImpl(TupleT&& tuple, FuncT&& func,
                                std::index_sequence<Indices...>) {
  bool ok = true;
  ((ok = ok && func(std::get<Indices>(tuple))), ...);
  return ok;
}

template <typename TupleT, typename FuncT>
bool forEachRosTupleElement(TupleT&& tuple, FuncT&& func) {
  constexpr size_t tuple_size =
      std::tuple_size_v<std::remove_reference_t<TupleT>>;
  return forEachRosTupleElementImpl(std::forward<TupleT>(tuple),
                                    std::forward<FuncT>(func),
                                    std::make_index_sequence<tuple_size>{});
}

template <typename DiscreteLayersT>
bool discreteLayerBundleToRosMsg(const DiscreteLayersT& layers,
                                 std::vector<wavemap_msgs::DiscreteLayer>& msg) {
  const auto named_layers =
      DiscreteLayerBundleRosTraits<DiscreteLayersT>::layers(layers);
  msg.clear();
  msg.reserve(std::tuple_size_v<std::remove_reference_t<decltype(named_layers)>>);

  return forEachRosTupleElement(named_layers, [&](const auto& named_layer) {
    wavemap_msgs::DiscreteLayer layer_msg;
    discreteLayerToRosMsg(named_layer.name, named_layer.layer, layer_msg);
    msg.emplace_back(std::move(layer_msg));
    return true;
  });
}

template <typename DiscreteLayersT>
bool rosMsgToDiscreteLayerBundle(
    const std::vector<wavemap_msgs::DiscreteLayer>& msg,
    DiscreteLayersT& layers) {
  const auto named_layers =
      DiscreteLayerBundleRosTraits<DiscreteLayersT>::layers(layers);
  constexpr size_t expected_layer_count =
      std::tuple_size_v<std::remove_reference_t<decltype(named_layers)>>;
  if (msg.size() != expected_layer_count) {
    return false;
  }

  size_t layer_index = 0u;
  return forEachRosTupleElement(named_layers, [&](auto named_layer) {
    return rosMsgToDiscreteLayer(msg[layer_index++], named_layer.name,
                                 named_layer.layer);
  });
}

template <typename LayeredMapT, typename ContinuousRosConverterT>
bool layeredMapToRosMsg(const LayeredMapT& map, const std::string& frame_id,
                        const ros::Time& stamp,
                        wavemap_msgs::LayeredMap& msg) {
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  if (!mapToRosMsg<typename LayeredMapT::ContinuousVoxel,
                   ContinuousRosConverterT>(map.continuousMap(), frame_id,
                                            stamp, msg.continuous_map)) {
    return false;
  }
  return discreteLayerBundleToRosMsg(map.discreteLayers(),
                                     msg.discrete_layers);
}

template <typename LayeredMapT, typename ContinuousRosConverterT>
bool rosMsgToLayeredMap(const wavemap_msgs::LayeredMap& msg,
                        LayeredMapT& map) {
  typename LayeredMapT::ContinuousMap::Ptr continuous_map;
  if (!rosMsgToMap<typename LayeredMapT::ContinuousVoxel,
                   ContinuousRosConverterT>(msg.continuous_map,
                                            continuous_map) ||
      !continuous_map) {
    return false;
  }

  typename LayeredMapT::DiscreteLayers discrete_layers;
  if (!rosMsgToDiscreteLayerBundle(msg.discrete_layers, discrete_layers)) {
    return false;
  }

  map = LayeredMapT(std::move(continuous_map), std::move(discrete_layers));
  return true;
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_MSG_CONVERSIONS_H_
