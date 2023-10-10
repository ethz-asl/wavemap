#ifndef WAVEMAP_UTILS_BITS_MORTON_ENCODING_H_
#define WAVEMAP_UTILS_BITS_MORTON_ENCODING_H_

#include <limits>

#include "wavemap/utils/bits/bit_operations.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap::morton {
template <int dim>
constexpr int kMaxTreeHeight = 8 * sizeof(MortonIndex) / dim - 1;

template <int dim>
constexpr IndexElement kMaxSingleCoordinate =
    (dim < 3) ? std::numeric_limits<IndexElement>::max()
              : (1ul << kMaxTreeHeight<dim>)-1ul;

template <int dim>
constexpr MortonIndex kMaxMortonIndex =
    (dim < 3) ? std::numeric_limits<MortonIndex>::max()
              : (1ul << (dim * kMaxTreeHeight<dim>)) - 1ul;

template <int dim>
struct MortonByteLut {
 private:
  static constexpr std::array<uint_fast32_t, 256> generate_encoder() {
    std::array<uint_fast32_t, 256> lut{};
    for (int byte = 0; byte < 256; ++byte) {
      for (int bit_idx = 0; bit_idx < 8; ++bit_idx) {
        lut[byte] |= ((byte >> bit_idx) & 1) << (dim * bit_idx);
      }
    }
    return lut;
  }

  static constexpr std::array<uint_fast8_t, 256> generate_decoder() {
    std::array<uint_fast8_t, 256> lut{};
    for (int byte = 0; byte < 256; ++byte) {
      for (int bit_idx = 0; bit_idx < 8; bit_idx += dim) {
        lut[byte] |= ((byte >> bit_idx) & 1) << (bit_idx / dim);
      }
    }
    return lut;
  }

 public:
  static constexpr std::array<uint_fast32_t, 256> encode{generate_encoder()};
  static constexpr std::array<uint_fast8_t, 256> decode{generate_decoder()};
};

/**
 * Method to convert regular n-dimensional indices into Morton indices.
 * @tparam dim Dimension of the index.
 * @tparam check_sign Whether to check that each index coefficient is positive.
 *         Note that negative signs are not preserved when encoding and decoding
 *         Morton indices. The check can be enabled to throw an error in cases
 *         where round trip conversions would not yield an identical index.
 *         Since we often only perform one way conversions and use the Morton
 *         indices as relative offsets, the check is disabled by default.
 *         Note that since the check is a DCHECK, it only triggers when the code
 *         is built with the DCHECK_ALWAYS_ON flag enabled or in debug mode.
 */
template <int dim, bool check_sign = false>
MortonIndex encode(const Index<dim>& index) {
  // Check if the index coordinates are within the supported range
  // NOTE: This check is only performed in debug mode, or if DCHECK_ALWAYS_ON is
  //       set. Otherwise, the loop is empty and optimized out.
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    if constexpr (check_sign) {
      DCHECK_GE(index[dim_idx], 0);
    } else {
      DCHECK_GT(index[dim_idx], -kMaxSingleCoordinate<dim>);
    }
    DCHECK_LE(index[dim_idx], kMaxSingleCoordinate<dim>);
  }

  // Perform the morton encoding, using bitwise expansion if supported by the
  // target CPU architecture and LUTs otherwise
  uint64_t morton = 0u;
#ifdef BIT_EXPAND_AVAILABLE
  constexpr auto pattern = bit_ops::repeat_block<uint64_t>(dim, 0b1);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    morton |= bit_ops::expand<uint64_t>(index[dim_idx], pattern << dim_idx);
  }
#else
  constexpr std::make_unsigned_t<IndexElement> kByteMask = (1 << 8) - 1;
  // TODO(victorr): Check if we can iterate over less bytes
  for (int byte_idx = sizeof(IndexElement) - 1; 0 <= byte_idx; --byte_idx) {
    const int shift = 8 * byte_idx;
    morton <<= 8 * dim;
    for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
      morton |=
          MortonByteLut<dim>::encode[(index[dim_idx] >> shift) & kByteMask]
          << dim_idx;
    }
  }
#endif
  return morton;
}

// Decode a single coordinate from a Morton index using LUTs
// NOTE: This method is only used if bitwise compression is not supported by the
//       target CPU architecture.
template <int dim>
IndexElement decode_coordinate(MortonIndex morton, int coordinate_idx) {
  constexpr MortonIndex kByteMask = (1 << 8) - 1;
  constexpr int kChunkSize = dim * int_math::div_round_up(8, dim);
  constexpr int kNumBitsPerChunk = kChunkSize / dim;
  constexpr int kNumChunks = 8 * sizeof(MortonIndex) / kChunkSize;
  // TODO(victorr): Check if we can iterate over less bytes
  MortonIndex coordinate = 0u;
  for (int chunk_idx = 0; chunk_idx < kNumChunks; ++chunk_idx) {
    coordinate |= static_cast<MortonIndex>(
        MortonByteLut<dim>::decode
            [(morton >> (kChunkSize * chunk_idx + coordinate_idx)) & kByteMask]
        << (kNumBitsPerChunk * chunk_idx));
  }
  return static_cast<IndexElement>(coordinate);
}

template <int dim>
Index<dim> decode(MortonIndex morton) {
  // Check if the Morton index is within the supported range
  DCHECK_LE(morton, kMaxMortonIndex<dim>);

  // Perform the morton decoding, using bitwise compression if supported by the
  // target CPU architecture and LUTs otherwise
  Index<dim> index;
#ifdef BIT_COMPRESS_AVAILABLE
  constexpr auto pattern = bit_ops::repeat_block<uint64_t>(dim, 0b1);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    index[dim_idx] = bit_ops::compress<uint64_t>(morton, pattern << dim_idx);
  }
#else
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    index[dim_idx] = decode_coordinate<dim>(morton, dim_idx);
  }
#endif
  return index;
}
}  // namespace wavemap::morton

#endif  // WAVEMAP_UTILS_BITS_MORTON_ENCODING_H_
