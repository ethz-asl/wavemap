#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/integrator/measurement_model/constant_ray.h"
#include "wavemap/integrator/measurement_model/continuous_beam.h"
#include "wavemap/integrator/measurement_model/continuous_ray.h"
#include "wavemap/iterator/grid_iterator.h"
#include "wavemap/iterator/ray_iterator.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
using MeasurementModelTest = FixtureBase;

TEST_F(MeasurementModelTest, DISABLED_ConstantRay) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}

TEST_F(MeasurementModelTest, DISABLED_ContinuousRay) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}

TEST_F(MeasurementModelTest, DISABLED_ContinuousBeam) {
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}
}  // namespace wavemap
