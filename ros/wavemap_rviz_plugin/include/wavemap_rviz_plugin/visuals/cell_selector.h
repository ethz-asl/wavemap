#ifndef WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_SELECTOR_H_
#define WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_SELECTOR_H_

#ifndef Q_MOC_RUN
#include <ros/console.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <wavemap/config/type_selector.h>
#include <wavemap/map/hashed_wavelet_octree.h>
#include <wavemap/utils/query/query_accelerator.h>
#endif

namespace wavemap::rviz_plugin {
struct CellSelectionMode : public TypeSelector<CellSelectionMode> {
  using TypeSelector<CellSelectionMode>::TypeSelector;

  enum Id : TypeId { kSurface, kBand };

  static constexpr std::array names = {"Surface", "Band"};
};

class CellSelector : public QObject {
  Q_OBJECT
 public:  // NOLINT
  CellSelector(rviz::Property* submenu_root_property,
               std::function<void()> redraw_map);

  void initializePropertyMenu();

  void setMap(const VolumetricDataStructureBase::ConstPtr& map);

  bool shouldBeDrawn(const OctreeIndex& cell_index,
                     FloatingPoint cell_log_odds) const;
  bool isUnknown(FloatingPoint log_odds) const {
    return std::abs(log_odds) < unknown_occupancy_threshold_;
  }
  bool hasFreeNeighbor(const OctreeIndex& cell_index) const;

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void cellSelectionModeUpdateCallback();
  void thresholdUpdateCallback();

 private:
  std::function<void()> redraw_map_;
  mutable std::optional<QueryAccelerator<HashedWaveletOctree>>
      query_accelerator_;

  // Selection mode and thresholds
  CellSelectionMode cell_selection_mode_ = CellSelectionMode::kSurface;
  FloatingPoint surface_occupancy_threshold_ = 1e-3f;
  FloatingPoint band_min_occupancy_threshold_ = 1e-3f;
  FloatingPoint band_max_occupancy_threshold_ = 1e6f;
  FloatingPoint unknown_occupancy_threshold_ = 1e-4f;

  // User-editable property variables, contained in the visual's submenu
  rviz::EnumProperty cell_selection_mode_property_;
  rviz::FloatProperty surface_occupancy_threshold_property_;
  rviz::FloatProperty band_min_occupancy_threshold_property_;
  rviz::FloatProperty band_max_occupancy_threshold_property_;
  rviz::FloatProperty unknown_occupancy_threshold_property_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_VISUALS_CELL_SELECTOR_H_
