#ifndef WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_
#define WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_

#include "wavemap/core/common.h"
#include "wavemap/core/config/type_selector.h"

namespace wavemap {
struct Adjacency : TypeSelector<Adjacency> {
  using TypeSelector<Adjacency>::TypeSelector;
  using Mask = uint8_t;

  enum Id : TypeId {
    kSharedVertex,
    kSharedEdge,
    kSharedFace,
    kSharedCube,
    kAnyDisjoint,
    kAny,
    kNone
  };
  static constexpr std::array names = {
      "shared_vertex", "shared_edge", "shared_face", "shared_cube",
      "any_disjoint",  "any",         "none"};

  template <int dim>
  static constexpr Mask toMask(Id type_id);

  template <int dim>
  constexpr Mask toMask() const;
};
}  // namespace wavemap

#include "wavemap/core/utils/neighbors/impl/adjacency_inl.h"

#endif  // WAVEMAP_UTILS_NEIGHBORS_ADJACENCY_H_
