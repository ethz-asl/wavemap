#ifndef WAVEMAP_LAYERED_LAYER_OBSERVATION_BATCH_H_
#define WAVEMAP_LAYERED_LAYER_OBSERVATION_BATCH_H_

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include <wavemap/layered/layer_observation.h>
#include <wavemap/layered/layer_schema.h>

namespace wavemap::layered {
namespace detail {

template <typename LayerTagT>
struct LayerObservationSlot {
  std::vector<LayerObservation<LayerValueT<LayerTagT>>> observations;
};

template <typename LayerTagT>
struct LayerTagIdentity {
  using Type = LayerTagT;
};

}  // namespace detail

// A heterogeneous, sensor-independent batch generated directly from a layer
// schema. One sensor callback can populate any subset of the configured layers
// without defining a new adapter or storage type.
template <typename SchemaT>
class LayerObservationBatch;

template <typename... LayerTags>
class LayerObservationBatch<schema::LayerSchema<LayerTags...>>
    : private detail::LayerObservationSlot<LayerTags>... {
 public:
  using Schema = schema::LayerSchema<LayerTags...>;

  template <typename LayerTagT>
  void add(LayerObservation<LayerValueT<LayerTagT>> observation) {
    static_assert(Schema::template contains<LayerTagT>,
                  "The observation layer is not present in this map schema.");
    observations<LayerTagT>().emplace_back(std::move(observation));
  }

  template <typename LayerTagT>
  void add(Point3D position, LayerValueT<LayerTagT> value) {
    add<LayerTagT>({std::move(position), std::move(value)});
  }

  template <typename LayerTagT>
  auto& observations() {
    static_assert(Schema::template contains<LayerTagT>,
                  "The requested layer is not present in this map schema.");
    return static_cast<detail::LayerObservationSlot<LayerTagT>&>(*this)
        .observations;
  }

  template <typename LayerTagT>
  const auto& observations() const {
    static_assert(Schema::template contains<LayerTagT>,
                  "The requested layer is not present in this map schema.");
    return static_cast<const detail::LayerObservationSlot<LayerTagT>&>(*this)
        .observations;
  }

  bool empty() const { return (observations<LayerTags>().empty() && ...); }
  size_t size() const {
    return (size_t{0u} + ... + observations<LayerTags>().size());
  }

  template <typename FunctionT>
  void forEachLayer(FunctionT&& function) {
    (function(detail::LayerTagIdentity<LayerTags>{},
              observations<LayerTags>()),
     ...);
  }

  template <typename FunctionT>
  void forEachLayer(FunctionT&& function) const {
    (function(detail::LayerTagIdentity<LayerTags>{},
              observations<LayerTags>()),
     ...);
  }

  void transformPositions(const Transformation3D& transform) {
    forEachLayer([&transform](auto /*layer_tag*/, auto& layer_observations) {
      for (auto& observation : layer_observations) {
        observation.position = transform * observation.position;
      }
    });
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_OBSERVATION_BATCH_H_
