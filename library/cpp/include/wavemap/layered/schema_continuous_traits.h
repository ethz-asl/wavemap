#ifndef WAVEMAP_LAYERED_SCHEMA_CONTINUOUS_TRAITS_H_
#define WAVEMAP_LAYERED_SCHEMA_CONTINUOUS_TRAITS_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include <wavemap/core/map/cell_types/cell_data_traits.h>
#include <wavemap/layered/schema_layer_storage.h>
#include <wavemap/layered/types/rgb.h>

namespace wavemap::layered::schema {

template <typename ValueT>
struct ContinuousValueTraits;

namespace detail {
template <typename ValueT, typename = void>
struct HasContinuousValueTraits : std::false_type {};

template <typename ValueT>
struct HasContinuousValueTraits<
    ValueT,
    std::void_t<
        typename ContinuousValueTraits<ValueT>::ThresholdConfig,
        decltype(ContinuousValueTraits<ValueT>::add(
            std::declval<const ValueT&>(), std::declval<const ValueT&>())),
        decltype(ContinuousValueTraits<ValueT>::subtract(
            std::declval<const ValueT&>(), std::declval<const ValueT&>())),
        decltype(ContinuousValueTraits<ValueT>::scale(
            std::declval<const ValueT&>(), std::declval<FloatingPoint>())),
        decltype(ContinuousValueTraits<ValueT>::threshold(
            std::declval<ValueT&>(),
            std::declval<const typename ContinuousValueTraits<
                ValueT>::ThresholdConfig&>())),
        decltype(ContinuousValueTraits<ValueT>::magnitude(
            std::declval<const ValueT&>()))>> : std::true_type {};
}  // namespace detail

template <typename ValueT>
inline constexpr bool kHasContinuousValueTraits =
    detail::HasContinuousValueTraits<ValueT>::value;

template <>
struct ContinuousValueTraits<FloatingPoint> {
  struct ThresholdConfig {
    FloatingPoint min = std::numeric_limits<FloatingPoint>::lowest();
    FloatingPoint max = std::numeric_limits<FloatingPoint>::max();
  };

  static FloatingPoint add(FloatingPoint lhs, FloatingPoint rhs) {
    return lhs + rhs;
  }
  static FloatingPoint subtract(FloatingPoint lhs, FloatingPoint rhs) {
    return lhs - rhs;
  }
  static FloatingPoint scale(FloatingPoint value, FloatingPoint factor) {
    return factor * value;
  }
  static void threshold(FloatingPoint& value, const ThresholdConfig& config) {
    value = std::clamp(value, config.min, config.max);
  }
  static FloatingPoint magnitude(FloatingPoint value) {
    return std::abs(value);
  }
};

template <>
struct ContinuousValueTraits<Rgb> {
  struct ThresholdConfig {
    Rgb min{std::numeric_limits<FloatingPoint>::lowest(),
            std::numeric_limits<FloatingPoint>::lowest(),
            std::numeric_limits<FloatingPoint>::lowest()};
    Rgb max{std::numeric_limits<FloatingPoint>::max(),
            std::numeric_limits<FloatingPoint>::max(),
            std::numeric_limits<FloatingPoint>::max()};
  };

  static Rgb add(const Rgb& lhs, const Rgb& rhs) {
    return RgbArithmetic::add(lhs, rhs);
  }
  static Rgb subtract(const Rgb& lhs, const Rgb& rhs) {
    return RgbArithmetic::subtract(lhs, rhs);
  }
  static Rgb scale(const Rgb& value, FloatingPoint factor) {
    return RgbArithmetic::scale(value, factor);
  }
  static void threshold(Rgb& value, const ThresholdConfig& config) {
    value = clampRgb(value, config.min, config.max);
  }
  static FloatingPoint magnitude(const Rgb& value) { return maxAbs(value); }
};

template <>
struct ContinuousValueTraits<WeightedMeanState> {
  struct ThresholdConfig {
    FloatingPoint min = std::numeric_limits<FloatingPoint>::lowest();
    FloatingPoint max = std::numeric_limits<FloatingPoint>::max();
    FloatingPoint max_total_weight =
        std::numeric_limits<FloatingPoint>::max();
  };

  static WeightedMeanState add(const WeightedMeanState& lhs,
                               const WeightedMeanState& rhs) {
    return {lhs.weighted_sum + rhs.weighted_sum,
            lhs.total_weight + rhs.total_weight};
  }
  static WeightedMeanState subtract(const WeightedMeanState& lhs,
                                    const WeightedMeanState& rhs) {
    return {lhs.weighted_sum - rhs.weighted_sum,
            lhs.total_weight - rhs.total_weight};
  }
  static WeightedMeanState scale(const WeightedMeanState& value,
                                 FloatingPoint factor) {
    return {factor * value.weighted_sum, factor * value.total_weight};
  }
  static void threshold(WeightedMeanState& state,
                        const ThresholdConfig& config) {
    if (state.total_weight <= 0.f) {
      state = {};
      return;
    }
    if (config.max_total_weight < state.total_weight) {
      const FloatingPoint factor =
          config.max_total_weight / state.total_weight;
      state = scale(state, factor);
    }
    const FloatingPoint bounded_value =
        std::clamp(state.valueOr(), config.min, config.max);
    state.weighted_sum = bounded_value * state.total_weight;
  }
  static FloatingPoint magnitude(const WeightedMeanState& state) {
    return std::max(std::abs(state.weighted_sum),
                    std::abs(state.total_weight));
  }
};

namespace detail {
template <typename LayerTagT>
using ThresholdConfigFor =
    typename ContinuousValueTraits<LayerStateT<LayerTagT>>::ThresholdConfig;

template <typename LayerTagT>
using PruningConfigFor = wavemap::FieldPruningConfig;

template <typename LayerTagT, template <typename> class ConfigT, bool Enabled>
struct ContinuousConfigSlot {};

template <typename LayerTagT, template <typename> class ConfigT>
struct ContinuousConfigSlot<LayerTagT, ConfigT, true> {
  ConfigT<LayerTagT> value{};
};
}  // namespace detail

template <typename SchemaT, template <typename> class ConfigT>
class ContinuousConfigBundle;

template <typename... LayerTags, template <typename> class ConfigT>
class ContinuousConfigBundle<LayerSchema<LayerTags...>, ConfigT>
    : private detail::ContinuousConfigSlot<
          LayerTags, ConfigT, kIsWaveletCompatibleLayer<LayerTags>>... {
 public:
  using Schema = LayerSchema<LayerTags...>;

  template <typename LayerTagT>
  ConfigT<LayerTagT>& get() {
    static_assert(Schema::template containsContinuous<LayerTagT>);
    return static_cast<
        detail::ContinuousConfigSlot<LayerTagT, ConfigT, true>&>(*this)
        .value;
  }

  template <typename LayerTagT>
  const ConfigT<LayerTagT>& get() const {
    static_assert(Schema::template containsContinuous<LayerTagT>);
    return static_cast<const detail::ContinuousConfigSlot<
        LayerTagT, ConfigT, true>&>(*this)
        .value;
  }
};

template <typename SchemaT>
using ContinuousThresholdConfig =
    ContinuousConfigBundle<SchemaT, detail::ThresholdConfigFor>;

template <typename SchemaT>
struct ContinuousPruningConfig
    : ContinuousConfigBundle<SchemaT, detail::PruningConfigFor> {
  FloatingPoint combined_threshold = 1.f;
};

template <typename SchemaT>
struct ContinuousBundleArithmetic;

template <typename... LayerTags>
struct ContinuousBundleArithmetic<LayerSchema<LayerTags...>> {
  using Schema = LayerSchema<LayerTags...>;
  using Bundle = ContinuousLayerBundle<Schema>;

  static Bundle add(const Bundle& lhs, const Bundle& rhs) {
    Bundle result;
    (applyAdd<LayerTags>(result, lhs, rhs), ...);
    return result;
  }

  static Bundle subtract(const Bundle& lhs, const Bundle& rhs) {
    Bundle result;
    (applySubtract<LayerTags>(result, lhs, rhs), ...);
    return result;
  }

  static Bundle scale(const Bundle& value, FloatingPoint factor) {
    Bundle result;
    if constexpr (sizeof...(LayerTags) == 0u) {
      (void)value;
      (void)factor;
    }
    (applyScale<LayerTags>(result, value, factor), ...);
    return result;
  }

 private:
  template <typename LayerTagT>
  static void applyAdd(Bundle& result, const Bundle& lhs, const Bundle& rhs) {
    if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
      result.template get<LayerTagT>() =
          ContinuousValueTraits<LayerStateT<LayerTagT>>::add(
              lhs.template get<LayerTagT>(), rhs.template get<LayerTagT>());
    }
  }

  template <typename LayerTagT>
  static void applySubtract(Bundle& result, const Bundle& lhs,
                            const Bundle& rhs) {
    if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
      result.template get<LayerTagT>() =
          ContinuousValueTraits<LayerStateT<LayerTagT>>::subtract(
              lhs.template get<LayerTagT>(), rhs.template get<LayerTagT>());
    }
  }

  template <typename LayerTagT>
  static void applyScale(Bundle& result, const Bundle& value,
                         FloatingPoint factor) {
    if constexpr (kIsWaveletCompatibleLayer<LayerTagT>) {
      result.template get<LayerTagT>() =
          ContinuousValueTraits<LayerStateT<LayerTagT>>::scale(
              value.template get<LayerTagT>(), factor);
    } else {
      (void)result;
      (void)value;
      (void)factor;
    }
  }
};

}  // namespace wavemap::layered::schema

namespace wavemap {

template <typename... LayerTags>
struct CellDataTraits<layered::schema::ContinuousLayerBundle<
    layered::schema::LayerSchema<LayerTags...>>> {
  using Schema = layered::schema::LayerSchema<LayerTags...>;
  using Bundle = layered::schema::ContinuousLayerBundle<Schema>;
  using ThresholdConfig = layered::schema::ContinuousThresholdConfig<Schema>;
  using PruningConfig = layered::schema::ContinuousPruningConfig<Schema>;

  static void threshold(Bundle& bundle, const ThresholdConfig& config) {
    (thresholdOne<LayerTags>(bundle, config), ...);
  }

  static FloatingPoint pruningScore(const Bundle& bundle,
                                    const PruningConfig& config) {
    return (FloatingPoint{0.f} + ... + scoreOne<LayerTags>(bundle, config));
  }

  static bool isNonzero(const Bundle& bundle, const PruningConfig& config) {
    return config.combined_threshold < pruningScore(bundle, config);
  }

 private:
  template <typename LayerTagT>
  static void thresholdOne(Bundle& bundle, const ThresholdConfig& config) {
    if constexpr (layered::kIsWaveletCompatibleLayer<LayerTagT>) {
      layered::schema::ContinuousValueTraits<
          layered::LayerStateT<LayerTagT>>::threshold(
          bundle.template get<LayerTagT>(),
          config.template get<LayerTagT>());
    }
  }

  template <typename LayerTagT>
  static FloatingPoint scoreOne(const Bundle& bundle,
                                const PruningConfig& config) {
    if constexpr (layered::kIsWaveletCompatibleLayer<LayerTagT>) {
      return weightedPruningScore(
          layered::schema::ContinuousValueTraits<
              layered::LayerStateT<LayerTagT>>::magnitude(
              bundle.template get<LayerTagT>()),
          config.template get<LayerTagT>());
    } else {
      return 0.f;
    }
  }
};

}  // namespace wavemap

#endif  // WAVEMAP_LAYERED_SCHEMA_CONTINUOUS_TRAITS_H_
