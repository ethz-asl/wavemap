#ifndef WAVEMAP_MAP_MAP_FACTORY_H_
#define WAVEMAP_MAP_MAP_FACTORY_H_

#include <string>

#include "wavemap/core/map/map_base.h"

namespace wavemap {
class MapFactory {
 public:
  static MapBase::Ptr create(
      const param::Value& params,
      std::optional<MapType> default_map_type = std::nullopt);

  static MapBase::Ptr create(MapType map_type, const param::Value& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_MAP_MAP_FACTORY_H_
