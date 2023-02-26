#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_

#include <memory>
#include <utility>

#include "wavemap/data_structure/volumetric/volumetric_data_structure_base.h"
#include "wavemap/data_structure/volumetric/wavelet_octree_interface.h"
#include "wavemap/integrator/projective/coarse_to_fine/range_image_intersector.h"
#include "wavemap/integrator/projective/projective_integrator.h"

namespace wavemap {
class WaveletIntegrator : public ProjectiveIntegrator {
 public:
  WaveletIntegrator(const ProjectiveIntegratorConfig& config,
                    ProjectorBase::ConstPtr projection_model,
                    PosedImage<>::Ptr posed_range_image,
                    Image<Vector2D>::Ptr beam_offset_image,
                    MeasurementModelBase::ConstPtr measurement_model,
                    WaveletOctreeInterface::Ptr occupancy_map)
      : ProjectiveIntegrator(
            config, std::move(projection_model), std::move(posed_range_image),
            std::move(beam_offset_image), std::move(measurement_model)),
        occupancy_map_(std::move(CHECK_NOTNULL(occupancy_map))),
        min_cell_width_(occupancy_map_->getMinCellWidth()) {}

 private:
  const WaveletOctreeInterface::Ptr occupancy_map_;
  const FloatingPoint min_cell_width_;

  std::shared_ptr<RangeImageIntersector> range_image_intersector_;
  static constexpr auto kUnitCubeHalfDiagonal =
      constants<FloatingPoint>::kSqrt3 / 2.f;

  WaveletOctreeInterface::Coefficients::Scale recursiveSamplerCompressor(
      const OctreeIndex& node_index, FloatingPoint node_value,
      WaveletOctreeInterface::NodeType& parent_node,
      OctreeIndex ::RelativeChild relative_child_index);

  void updateMap() override;
};
}  // namespace wavemap

#include "wavemap/integrator/projective/coarse_to_fine/impl/wavelet_integrator_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_WAVELET_INTEGRATOR_H_
