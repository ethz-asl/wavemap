#ifndef WAVEMAP_LAYERED_SCHEMA_LAYER_STORAGE_H_
#define WAVEMAP_LAYERED_SCHEMA_LAYER_STORAGE_H_

#include <tuple>
#include <type_traits>
#include <utility>

#include <wavemap/layered/discrete_layer.h>
#include <wavemap/layered/layer_descriptor.h>
#include <wavemap/layered/layer_schema.h>

namespace wavemap::layered::schema {
namespace detail {

template <typename LayerTagT, bool Enabled>
struct ContinuousSlot {
  bool operator==(const ContinuousSlot&) const { return true; }
};

template <typename LayerTagT>
struct ContinuousSlot<LayerTagT, true> {
  LayerStateT<LayerTagT> value{};

  bool operator==(const ContinuousSlot& other) const {
    return value == other.value;
  }
};

template <typename LayerTagT, bool Enabled>
struct DiscreteSlot {
  explicit DiscreteSlot(const DiscreteCompressionConfig&) {}
};

template <typename LayerTagT>
struct DiscreteSlot<LayerTagT, true> {
  explicit DiscreteSlot(const DiscreteCompressionConfig& config)
      : value(config) {}

  layered::DiscreteLayer<LayerValueT<LayerTagT>> value;
};

template <typename LayerTagT, typename BundleT>
constexpr auto continuousDescriptorTuple() {
  if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
    return std::make_tuple(
        accessorLayerDescriptor<LayerTagT, BundleT>());
  } else {
    return std::tuple<>();
  }
}

template <typename LayerTagT, typename BundleT>
constexpr auto discreteDescriptorTuple() {
  if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
    return std::tuple<>();
  } else {
    return std::make_tuple(
        accessorLayerDescriptor<LayerTagT, BundleT>());
  }
}

}  // namespace detail

template <typename SchemaT>
class ContinuousLayerBundle;

template <typename... LayerTags>
class ContinuousLayerBundle<LayerSchema<LayerTags...>>
    : private detail::ContinuousSlot<
          LayerTags, kIsWaveletCompatibleLayer<LayerTags>>... {
 public:
  using Schema = LayerSchema<LayerTags...>;

  template <typename LayerTagT>
  LayerStateT<LayerTagT>& get() {
    static_assert(Schema::template containsContinuous<LayerTagT>,
                  "Requested layer is not a continuous layer in this schema.");
    return static_cast<detail::ContinuousSlot<LayerTagT, true>&>(*this).value;
  }

  template <typename LayerTagT>
  const LayerStateT<LayerTagT>& get() const {
    static_assert(Schema::template containsContinuous<LayerTagT>,
                  "Requested layer is not a continuous layer in this schema.");
    return static_cast<const detail::ContinuousSlot<LayerTagT, true>&>(*this)
        .value;
  }

  bool operator==(const ContinuousLayerBundle& other) const {
    return ((!
                 kIsWaveletCompatibleLayer<LayerTags> ||
             getIfContinuous<LayerTags>() ==
                 other.template getIfContinuous<LayerTags>()) &&
            ...);
  }

 private:
  template <typename LayerTagT>
  const auto& getIfContinuous() const {
    return static_cast<const detail::ContinuousSlot<
        LayerTagT, kIsWaveletCompatibleLayer<LayerTagT>>&>(*this);
  }
};

template <typename SchemaT>
class DiscreteLayerBundle;

template <typename... LayerTags>
class DiscreteLayerBundle<LayerSchema<LayerTags...>>
    : private detail::DiscreteSlot<
          LayerTags, !kIsWaveletCompatibleLayer<LayerTags>>... {
 public:
  using Schema = LayerSchema<LayerTags...>;

  explicit DiscreteLayerBundle(
      const DiscreteCompressionConfig& config = DiscreteCompressionConfig())
      : detail::DiscreteSlot<
            LayerTags, !kIsWaveletCompatibleLayer<LayerTags>>(config)... {}

  template <typename LayerTagT>
  layered::DiscreteLayer<LayerValueT<LayerTagT>>& get() {
    static_assert(Schema::template containsDiscrete<LayerTagT>,
                  "Requested layer is not a discrete layer in this schema.");
    return static_cast<detail::DiscreteSlot<LayerTagT, true>&>(*this).value;
  }

  template <typename LayerTagT>
  const layered::DiscreteLayer<LayerValueT<LayerTagT>>& get() const {
    static_assert(Schema::template containsDiscrete<LayerTagT>,
                  "Requested layer is not a discrete layer in this schema.");
    return static_cast<const detail::DiscreteSlot<LayerTagT, true>&>(*this)
        .value;
  }
};

}  // namespace wavemap::layered::schema

namespace wavemap::layered {

template <typename... LayerTags>
struct LayerDescriptors<
    schema::ContinuousLayerBundle<schema::LayerSchema<LayerTags...>>> {
  using Bundle =
      schema::ContinuousLayerBundle<schema::LayerSchema<LayerTags...>>;

  static constexpr auto descriptors() {
    return std::tuple_cat(
        schema::detail::continuousDescriptorTuple<LayerTags, Bundle>()...);
  }
};

template <typename... LayerTags>
struct LayerDescriptors<
    schema::DiscreteLayerBundle<schema::LayerSchema<LayerTags...>>> {
  using Bundle =
      schema::DiscreteLayerBundle<schema::LayerSchema<LayerTags...>>;

  static constexpr auto descriptors() {
    return std::tuple_cat(
        schema::detail::discreteDescriptorTuple<LayerTags, Bundle>()...);
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_SCHEMA_LAYER_STORAGE_H_
