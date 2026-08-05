#include <array>

#include <wavemap/core/map/cell_types/haar_coefficients.h>
#include <wavemap/core/map/cell_types/haar_transform.h>

#include "../common/layered_voxel_config.h"

namespace {
using Coefficients = wavemap::HaarCoefficients<LayeredVoxel, 3>;
using Transform = wavemap::HaarTransform<LayeredVoxel, 3>;
}  // namespace

int main() {
  Coefficients::CoefficientsArray child_voxels{
      LayeredVoxel{0.1f, ContinuousLayers{rgb(1.f, 0.f, 0.f), 0.1f}},
      LayeredVoxel{0.2f, ContinuousLayers{rgb(0.f, 1.f, 0.f), 0.2f}},
      LayeredVoxel{0.3f, ContinuousLayers{rgb(0.f, 0.f, 1.f), 0.3f}},
      LayeredVoxel{0.4f, ContinuousLayers{rgb(1.f, 1.f, 0.f), 0.4f}},
      LayeredVoxel{0.5f, ContinuousLayers{rgb(1.f, 0.f, 1.f), 0.5f}},
      LayeredVoxel{0.6f, ContinuousLayers{rgb(0.f, 1.f, 1.f), 0.6f}},
      LayeredVoxel{0.7f, ContinuousLayers{rgb(0.5f, 0.5f, 0.5f), 0.7f}},
      LayeredVoxel{0.8f, ContinuousLayers{rgb(1.f, 1.f, 1.f), 0.8f}},
  };

  const Coefficients::Parent parent_coefficients =
      Transform::forward(child_voxels);
  const Coefficients::CoefficientsArray reconstructed_children =
      Transform::backward(parent_coefficients);

  printVoxel("Parent scale", parent_coefficients.scale);
  printVoxel("First reconstructed child", reconstructed_children[0]);
  printVoxel("Last reconstructed child", reconstructed_children[7]);

  return 0;
}
