#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_

#include "wavemap/integrator/projection_model/projector_base.h"

namespace wavemap {
class ProjectorFactory {
 public:
  static ProjectorBase::Ptr create(
      const param::Map& params,
      std::optional<ProjectorType> default_projector_type = std::nullopt);

  static ProjectorBase::Ptr create(ProjectorType projector_type,
                                   const param::Map& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_
