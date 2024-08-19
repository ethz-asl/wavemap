#ifndef WAVEMAP_CORE_MAP_MAP_FACTORY_H_
#define WAVEMAP_CORE_MAP_MAP_FACTORY_H_

#include <memory>
#include <string>

#include "wavemap/core/map/map_base.h"

namespace wavemap {
class MapFactory {
 public:
  static std::unique_ptr<MapBase> create(
      const param::Value& params,
      std::optional<MapType> default_map_type = std::nullopt);

  static std::unique_ptr<MapBase> create(MapType map_type,
                                         const param::Value& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_MAP_FACTORY_H_
