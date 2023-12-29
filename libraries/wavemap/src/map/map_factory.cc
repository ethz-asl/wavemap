#include "wavemap/map/map_factory.h"

#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/map/volumetric_octree.h"
#include "wavemap/map/wavelet_octree.h"

namespace wavemap {
MapBase::Ptr MapFactory::create(const param::Value& params,
                                std::optional<MapType> default_map_type) {
  if (const auto type = MapType::from(params); type) {
    return create(type.value(), params);
  }

  if (default_map_type.has_value()) {
    LOG(WARNING) << "Default type \"" << default_map_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_map_type.value(), params);
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

MapBase::Ptr MapFactory::create(MapType map_type, const param::Value& params) {
  switch (map_type) {
    case MapType::kHashedBlocks: {
      if (const auto config = MapBaseConfig::from(params); config) {
        return std::make_shared<HashedBlocks>(config.value());
      } else {
        LOG(ERROR) << "Hashed blocks volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case MapType::kOctree: {
      if (const auto config = VolumetricOctreeConfig::from(params); config) {
        return std::make_shared<VolumetricOctree>(config.value());
      } else {
        LOG(ERROR)
            << "Octree volumetric data structure config could not be loaded.";
        return nullptr;
      }
    }
    case MapType::kWaveletOctree: {
      if (const auto config = WaveletOctreeConfig::from(params); config) {
        return std::make_shared<WaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Wavelet octree volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case MapType::kHashedWaveletOctree: {
      if (const auto config = HashedWaveletOctreeConfig::from(params); config) {
        return std::make_shared<HashedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed wavelet octree volumetric data structure config "
                      "could not be loaded.";
        return nullptr;
      }
    }
    case MapType::kHashedChunkedWaveletOctree: {
      if (const auto config = HashedChunkedWaveletOctreeConfig::from(params);
          config) {
        return std::make_shared<HashedChunkedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed chunked wavelet octree volumetric data structure "
                      "config could not be loaded.";
        return nullptr;
      }
    }
    default:
      LOG(ERROR) << "Attempted to create data structure with unknown type ID: "
                 << map_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
