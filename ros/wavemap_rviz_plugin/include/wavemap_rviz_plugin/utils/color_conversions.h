#ifndef WAVEMAP_RVIZ_PLUGIN_UTILS_COLOR_CONVERSIONS_H_
#define WAVEMAP_RVIZ_PLUGIN_UTILS_COLOR_CONVERSIONS_H_

#include <OGRE/OgreColourValue.h>
#include <wavemap/common.h>

namespace wavemap::rviz_plugin {
// Map a voxel's log-odds value to a color (grey value)
inline Ogre::ColourValue logOddsToColor(FloatingPoint log_odds) {
  Ogre::ColourValue color;
  color.a = 1.f;

  const FloatingPoint cell_odds = std::exp(log_odds);
  const FloatingPoint cell_prob = cell_odds / (1.f + cell_odds);
  const FloatingPoint cell_free_prob = 1.f - cell_prob;
  color.r = cell_free_prob;
  color.g = cell_free_prob;
  color.b = cell_free_prob;
  return color;
}

// Map a voxel's position to a color (HSV color map)
Ogre::ColourValue positionToColor(const Point3D& center_point);

// Map a voxel's raw value to a color after normalizing it (grey value)
inline Ogre::ColourValue scalarToColor(FloatingPoint value,
                                       FloatingPoint min_value,
                                       FloatingPoint max_value) {
  Ogre::ColourValue color;
  color.a = 1.f;

  const FloatingPoint rescaled_odds =
      (value - min_value) / (max_value - min_value);
  color.a = 1.f;
  color.r = rescaled_odds;
  color.g = rescaled_odds;
  color.b = rescaled_odds;
  return color;
}
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_UTILS_COLOR_CONVERSIONS_H_
