#ifndef WAVEMAP_LAYERED_LAYER_STREAM_CODEC_H_
#define WAVEMAP_LAYERED_LAYER_STREAM_CODEC_H_

#include <istream>
#include <ostream>
#include <type_traits>
#include <utility>

#include <wavemap/io/stream_conversions.h>
#include <wavemap/layered/types/rgb.h>
#include <wavemap/layered/types/weighted_mean_state.h>

namespace wavemap::layered {

template <typename ValueT>
struct LayerStreamCodec;

namespace detail {
template <typename ValueT, typename = void>
struct HasLayerStreamCodec : std::false_type {};

template <typename ValueT>
struct HasLayerStreamCodec<
    ValueT,
    std::void_t<decltype(LayerStreamCodec<ValueT>::typeName()),
                decltype(LayerStreamCodec<ValueT>::write(
                    std::declval<std::ostream&>(),
                    std::declval<const ValueT&>())),
                decltype(LayerStreamCodec<ValueT>::read(
                    std::declval<std::istream&>()))>> : std::true_type {};
}  // namespace detail

template <typename ValueT>
inline constexpr bool kHasLayerStreamCodec =
    detail::HasLayerStreamCodec<ValueT>::value;

template <>
struct LayerStreamCodec<FloatingPoint> {
  static constexpr const char* typeName() { return "float32"; }

  static void write(std::ostream& ostream, FloatingPoint value) {
    wavemap::io::StreamableCellData<FloatingPoint>::write(ostream, value);
  }

  static FloatingPoint read(std::istream& istream) {
    return wavemap::io::StreamableCellData<FloatingPoint>::read(istream);
  }
};

template <>
struct LayerStreamCodec<Rgb> {
  static constexpr const char* typeName() { return "float32_rgb"; }

  static void write(std::ostream& ostream, const Rgb& value) {
    LayerStreamCodec<FloatingPoint>::write(ostream, value.r);
    LayerStreamCodec<FloatingPoint>::write(ostream, value.g);
    LayerStreamCodec<FloatingPoint>::write(ostream, value.b);
  }

  static Rgb read(std::istream& istream) {
    return {LayerStreamCodec<FloatingPoint>::read(istream),
            LayerStreamCodec<FloatingPoint>::read(istream),
            LayerStreamCodec<FloatingPoint>::read(istream)};
  }
};

template <>
struct LayerStreamCodec<WeightedMeanState> {
  static constexpr const char* typeName() {
    return "float32_weighted_mean_state";
  }

  static void write(std::ostream& ostream, const WeightedMeanState& state) {
    LayerStreamCodec<FloatingPoint>::write(ostream, state.weighted_sum);
    LayerStreamCodec<FloatingPoint>::write(ostream, state.total_weight);
  }

  static WeightedMeanState read(std::istream& istream) {
    return {LayerStreamCodec<FloatingPoint>::read(istream),
            LayerStreamCodec<FloatingPoint>::read(istream)};
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LAYER_STREAM_CODEC_H_
