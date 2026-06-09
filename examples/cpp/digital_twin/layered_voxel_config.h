#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_

#include <iostream>
#include <istream>
#include <ostream>

#include <wavemap/core/map/cell_types/voxel_data.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree_block.h>
#include <wavemap/io/stream_conversions.h>

struct LayeredData {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;
  float traversability = 0.f;

  bool operator==(const LayeredData& other) const {
    return r == other.r && g == other.g && b == other.b && traversability == other.traversability;
  }
};

struct LayeredDataPolicy {
  static LayeredData add(const LayeredData& lhs, const LayeredData& rhs) {
    return {lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b, lhs.traversability + rhs.traversability};
  }

  static LayeredData subtract(const LayeredData& lhs, const LayeredData& rhs) {
    return {lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b, lhs.traversability - rhs.traversability};
  }

  static LayeredData scale(const LayeredData& data, wavemap::FloatingPoint factor) {
    return {factor * data.r, factor * data.g, factor * data.b, factor * data.traversability};
  }
};

using LayeredVoxel = wavemap::VoxelData<LayeredData, LayeredDataPolicy>;
using LayeredBlock = wavemap::HashedWaveletOctreeBlockT<LayeredVoxel>;
using LayeredMap = wavemap::HashedWaveletOctreeT<LayeredVoxel>;

struct LayeredVoxelSerializer {
  static void write(std::ostream& ostream, const LayeredVoxel& voxel) {
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.occupancy);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.r);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.g);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.b);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.traversability);
  }

  static LayeredVoxel read(std::istream& istream) {
    LayeredVoxel voxel;
    voxel.occupancy = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.r = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.g = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.b = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.traversability = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    return voxel;
  }
};

inline void printVoxel(const char* label, const LayeredVoxel& voxel) {
  std::cout << label << " occupancy: " << voxel.occupancy << "\n";
  std::cout << label << " color: " << voxel.data.r << " " << voxel.data.g << " " << voxel.data.b << "\n";
  std::cout << label << " traversability: " << voxel.data.traversability << "\n";
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_
