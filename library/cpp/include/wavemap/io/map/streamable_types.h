#ifndef WAVEMAP_IO_MAP_STREAMABLE_TYPES_H_
#define WAVEMAP_IO_MAP_STREAMABLE_TYPES_H_

#include <istream>
#include <ostream>

#include "wavemap/core/config/type_selector.h"

namespace wavemap::io::streamable {
// NOTE: This file defines the serialization format for all types that might be
//       used in wavemap map files. The idea is that these are used as common
//       building blocks (e.g. 3D indices are always (de)serialized using
//       streamable::Index3D) and modified as little as possible to avoid
//       breaking backward compatibility with previously saved maps.

using UInt8 = uint8_t;
using UInt64 = uint64_t;
using Int32 = int32_t;
using Float = float;

struct Index3D {
  Int32 x{};
  Int32 y{};
  Int32 z{};

  inline void write(std::ostream& ostream) const;
  inline static Index3D read(std::istream& istream);
};

struct HashedBlockHeader {
  Index3D block_offset{};

  inline void write(std::ostream& ostream) const;
  inline static HashedBlockHeader read(std::istream& istream);
};

struct HashedBlocksHeader {
  Float min_cell_width{};
  Float min_log_odds{};
  Float max_log_odds{};

  UInt64 num_blocks{};

  inline void write(std::ostream& ostream) const;
  inline static HashedBlocksHeader read(std::istream& istream);
};

struct WaveletOctreeNode {
  std::array<Float, 7> detail_coefficients{};
  UInt8 allocated_children_bitset{};

  inline void write(std::ostream& ostream) const;
  inline static WaveletOctreeNode read(std::istream& istream);
};

struct WaveletOctreeHeader {
  Float min_cell_width{};
  Float min_log_odds{};
  Float max_log_odds{};

  Int32 tree_height{};
  Float root_node_scale_coefficient{};

  inline void write(std::ostream& ostream) const;
  inline static WaveletOctreeHeader read(std::istream& istream);
};

struct HashedWaveletOctreeBlockHeader {
  Index3D root_node_offset{};
  Float root_node_scale_coefficient{};

  inline void write(std::ostream& ostream) const;
  inline static HashedWaveletOctreeBlockHeader read(std::istream& istream);
};

struct HashedWaveletOctreeHeader {
  Float min_cell_width{};
  Float min_log_odds{};
  Float max_log_odds{};

  Int32 tree_height{};
  UInt64 num_blocks{};

  inline void write(std::ostream& ostream) const;
  inline static HashedWaveletOctreeHeader read(std::istream& istream);
};

struct StorageFormat : TypeSelector<StorageFormat> {
  using TypeSelector<StorageFormat>::TypeSelector;

  enum Id : TypeId { kWaveletOctree, kHashedWaveletOctree, kHashedBlocks };

  static constexpr std::array names = {
      "wavelet_octree", "hashed_wavelet_octree", "hashed_blocks"};

  inline void write(std::ostream& ostream) const;
  inline static StorageFormat read(std::istream& istream);
  inline static StorageFormat peek(std::istream& istream);
};
}  // namespace wavemap::io::streamable

#include "wavemap/io/map/impl/streamable_types_impl.h"

#endif  // WAVEMAP_IO_MAP_STREAMABLE_TYPES_H_
