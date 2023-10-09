#include "wavemap_rviz_plugin/visuals/cell_selector.h"

#include <wavemap/utils/iterate/grid_iterator.h>

namespace wavemap::rviz_plugin {
std::array<Index3D, 26> generateNeighborIndexOffsets() {
  std::array<Index3D, 26> neighbor_offsets{};
  size_t array_idx = 0u;
  for (const Index3D& index : Grid<3>(-Index3D::Ones(), Index3D::Ones())) {
    if (index != Index3D::Zero()) {
      neighbor_offsets[array_idx] = index;
      ++array_idx;
    }
  }
  std::sort(
      neighbor_offsets.begin(), neighbor_offsets.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.norm() < rhs.norm(); });
  return neighbor_offsets;
}
static const auto kNeighborOffsets = generateNeighborIndexOffsets();

CellSelector::CellSelector(rviz::Property* submenu_root_property,
                           std::function<void()> redraw_map)
    : redraw_map_(std::move(redraw_map)),
      cell_selection_mode_property_(
          "Cell selector", "", "Mode determining what cells to draw.",
          submenu_root_property, SLOT(cellSelectionModeUpdateCallback()), this),
      surface_occupancy_threshold_property_(
          "Log odds threshold", surface_occupancy_threshold_,
          "Classification threshold above which a cell is considered occupied. "
          "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(thresholdUpdateCallback()), this),
      band_min_occupancy_threshold_property_(
          "Min log odds", band_min_occupancy_threshold_,
          "Only show cells whose occupancy exceeds this threshold. "
          "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(thresholdUpdateCallback()), this),
      band_max_occupancy_threshold_property_(
          "Max log odds", band_max_occupancy_threshold_,
          "Only show cells whose occupancy falls below this threshold. "
          "Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(thresholdUpdateCallback()), this),
      unknown_occupancy_threshold_property_(
          "Unknown log odds", unknown_occupancy_threshold_,
          "Hide cells whose absolute occupancy falls below this threshold, "
          "considering them as unknown. Ranges from -Inf to Inf.",
          submenu_root_property, SLOT(thresholdUpdateCallback()), this) {
  // Initialize the property menu
  initializePropertyMenu();
}

void CellSelector::initializePropertyMenu() {
  cell_selection_mode_property_.clearOptions();
  for (const auto& name : CellSelectionMode::names) {
    cell_selection_mode_property_.addOption(name);
  }
  cell_selection_mode_property_.setStringStd(cell_selection_mode_.toStr());
  surface_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                  CellSelectionMode::kSurface);
  band_min_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                   CellSelectionMode::kBand);
  band_max_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                   CellSelectionMode::kBand);
}

void CellSelector::setMap(
    const VolumetricDataStructureBase::ConstPtr& hashed_map) {
  hashed_map_ =
      std::dynamic_pointer_cast<const HashedWaveletOctree>(hashed_map);
  if (!hashed_map_ && cell_selection_mode_ == CellSelectionMode::kSurface) {
    ROS_WARN(
        "Cell selection mode 'Surface' only supports HashedWaveletOctree maps. "
        "Falling back to mode 'Band'.");
    cell_selection_mode_ = CellSelectionMode::kBand;
    initializePropertyMenu();
  }
}

bool CellSelector::shouldBeDrawn(const OctreeIndex& cell_index,
                                 FloatingPoint cell_log_odds) const {
  switch (cell_selection_mode_.toTypeId()) {
    case CellSelectionMode::kSurface:
      // Skip free cells
      if (cell_log_odds < surface_occupancy_threshold_ ||
          isUnknown(cell_log_odds)) {
        return false;
      }
      // Skip cells that are occluded by neighbors on all sides
      if (!hasFreeNeighbor(cell_index)) {
        return false;
      }
      break;
    case CellSelectionMode::kBand:
      // Skip cells that don't meet the occupancy threshold
      if (cell_log_odds < band_min_occupancy_threshold_ ||
          band_max_occupancy_threshold_ < cell_log_odds ||
          isUnknown(cell_log_odds)) {
        return false;
      }
      break;
  }
  return true;
}

bool CellSelector::hasFreeNeighbor(const OctreeIndex& cell_index) const {
  for (const auto& offset : kNeighborOffsets) {  // NOLINT
    const OctreeIndex neighbor_index = {cell_index.height,
                                        cell_index.position + offset};
    const FloatingPoint neighbor_log_odds =
        hashed_map_->getCellValue(neighbor_index);
    // Check if the neighbor is free and observed
    if (neighbor_log_odds < surface_occupancy_threshold_ &&
        !isUnknown(neighbor_log_odds)) {
      return true;
    }
  }
  return false;
}

void CellSelector::cellSelectionModeUpdateCallback() {
  // Update the cached cell selection mode value
  const CellSelectionMode old_cell_selection_mode = cell_selection_mode_;
  cell_selection_mode_ =
      CellSelectionMode(cell_selection_mode_property_.getStdString());

  // Show/hide properties that only affect certain modes
  surface_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                  CellSelectionMode::kSurface);
  band_min_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                   CellSelectionMode::kBand);
  band_max_occupancy_threshold_property_.setHidden(cell_selection_mode_ !=
                                                   CellSelectionMode::kBand);

  // Redraw the map if the color mode changed
  if (cell_selection_mode_ != old_cell_selection_mode) {
    std::invoke(redraw_map_);
  }
}

void CellSelector::thresholdUpdateCallback() {
  // Update the thresholds
  surface_occupancy_threshold_ =
      surface_occupancy_threshold_property_.getFloat();
  band_min_occupancy_threshold_ =
      band_min_occupancy_threshold_property_.getFloat();
  band_max_occupancy_threshold_ =
      band_max_occupancy_threshold_property_.getFloat();
  unknown_occupancy_threshold_ =
      unknown_occupancy_threshold_property_.getFloat();

  // Redraw the map
  std::invoke(redraw_map_);
}
}  // namespace wavemap::rviz_plugin
