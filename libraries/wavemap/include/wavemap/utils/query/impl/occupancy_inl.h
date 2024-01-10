#ifndef WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_INL_H_
#define WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_INL_H_

namespace wavemap {
constexpr Occupancy::Mask Occupancy::toMask(Occupancy::Id type_id) {
  DCHECK(0 <= type_id);
  DCHECK(type_id < static_cast<TypeId>(names.size()));
  if (type_id < 3) {
    return 1 << type_id;
  } else {
    return 0b011;
  }
}
constexpr Occupancy::Mask Occupancy::toMask(bool free, bool occupied,
                                            bool unobserved) {
  return free | occupied << 1 | unobserved << 2;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_QUERY_IMPL_OCCUPANCY_INL_H_
