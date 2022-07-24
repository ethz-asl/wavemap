#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_CELL_TRAITS_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_CELL_TRAITS_H_

namespace wavemap_2d {
template <typename SpecializedType, typename BaseFloatType,
          typename BaseIntType>
struct CellTraits {
  using Specialized = SpecializedType;
  using BaseFloat = BaseFloatType;
  using BaseInt = BaseIntType;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_CELL_TRAITS_H_
