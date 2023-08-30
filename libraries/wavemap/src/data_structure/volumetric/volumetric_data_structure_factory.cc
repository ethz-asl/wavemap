#include "wavemap/data_structure/volumetric/volumetric_data_structure_factory.h"

#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/volumetric_octree.h"
#include "wavemap/data_structure/volumetric/wavelet_octree.h"

namespace wavemap {
VolumetricDataStructureBase::Ptr VolumetricDataStructureFactory::create(
    const param::Map& params,
    std::optional<VolumetricDataStructureType> default_data_structure_type) {
  std::string error_msg;
  auto type = VolumetricDataStructureType::fromParamMap(params, error_msg);
  if (type.isValid()) {
    return create(type, params);
  }

  if (default_data_structure_type.has_value()) {
    type = default_data_structure_type.value();
    LOG(WARNING) << error_msg << " Default type \"" << type.toStr()
                 << "\" will be created instead.";
    return create(default_data_structure_type.value(), params);
  }

  LOG(ERROR) << error_msg << "No default was set. Returning nullptr.";
  return nullptr;
}

VolumetricDataStructureBase::Ptr VolumetricDataStructureFactory::create(
    VolumetricDataStructureType data_structure_type, const param::Map& params) {
  switch (data_structure_type.toTypeId()) {
    case VolumetricDataStructureType::kHashedBlocks: {
      const auto config = VolumetricDataStructureConfig::from(params);
      if (config.has_value()) {
        return std::make_shared<HashedBlocks>(config.value());
      } else {
        LOG(ERROR) << "Hashed blocks volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kOctree: {
      const auto config = VolumetricOctreeConfig::from(params);
      if (config.has_value()) {
        return std::make_shared<VolumetricOctree>(config.value());
      } else {
        LOG(ERROR)
            << "Octree volumetric data structure config could not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kWaveletOctree: {
      const auto config = WaveletOctreeConfig::from(params);
      if (config.has_value()) {
        return std::make_shared<WaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Wavelet octree volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kHashedWaveletOctree: {
      const auto config = HashedWaveletOctreeConfig::from(params);
      if (config.has_value()) {
        return std::make_shared<HashedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed wavelet octree volumetric data structure config "
                      "could not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kHashedChunkedWaveletOctree: {
      const auto config = HashedChunkedWaveletOctreeConfig::from(params);
      if (config.has_value()) {
        return std::make_shared<HashedChunkedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed chunked wavelet octree volumetric data structure "
                      "config could not be loaded.";
        return nullptr;
      }
    }
    default:
      LOG(ERROR) << "Attempted to create data structure with unknown type ID: "
                 << data_structure_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
