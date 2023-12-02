#ifndef WAVEMAP_UTILS_NEIGHBORS_IMPL_ADJACENCY_INL_H_
#define WAVEMAP_UTILS_NEIGHBORS_IMPL_ADJACENCY_INL_H_

namespace wavemap {
template <int dim>
constexpr Adjacency::Mask Adjacency::toMask(Adjacency::Id type_id) {
  static_assert(dim <= 3);
  if (0 <= type_id && type_id <= 3) {
    return 1 << type_id;
  }
  switch (type_id) {
    case kAnyDisjoint:
      return (1 << dim) - 1;
    case kAny:
      return static_cast<Mask>(-1);
    case kNone:
    default:
      return 0;
  }
}

template <int dim>
constexpr Adjacency::Mask Adjacency::toMask() const {
  return toMask<dim>(id_);
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_NEIGHBORS_IMPL_ADJACENCY_INL_H_
