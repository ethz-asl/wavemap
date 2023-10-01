#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_

#include <utility>

namespace wavemap {
inline Index2D ProjectorBase::imageToNearestIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .round()
      .cast<IndexElement>();
}

inline Index2D ProjectorBase::imageToFloorIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .floor()
      .cast<IndexElement>();
}

inline Index2D ProjectorBase::imageToCeilIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .ceil()
      .cast<IndexElement>();
}

inline std::pair<Index2D, Vector2D> ProjectorBase::imageToNearestIndexAndOffset(
    const ImageCoordinates& image_coordinates) const {
  const Vector2D index = imageToIndexReal(image_coordinates);
  const Vector2D index_rounded = index.array().round();
  Vector2D image_coordinate_offset =
      (index - index_rounded).cwiseProduct(index_to_image_scale_factor_);
  return {index_rounded.cast<IndexElement>(),
          std::move(image_coordinate_offset)};
}

inline std::array<Index2D, 4> ProjectorBase::imageToNearestIndices(
    const ImageCoordinates& image_coordinates) const {
  const Vector2D index = imageToIndexReal(image_coordinates);
  const Vector2D index_lower = index.array().floor();
  const Vector2D index_upper = index.array().ceil();

  std::array<Index2D, 4> indices{};
  for (int neighbox_idx = 0; neighbox_idx < 4; ++neighbox_idx) {
    const Vector2D index_rounded{
        neighbox_idx & 0b01 ? index_upper[0] : index_lower[0],
        neighbox_idx & 0b10 ? index_upper[1] : index_lower[1]};
    indices[neighbox_idx] = index_rounded.cast<IndexElement>();
  }

  return indices;
}

inline std::pair<std::array<Index2D, 4>, std::array<Vector2D, 4>>
ProjectorBase::imageToNearestIndicesAndOffsets(
    const ImageCoordinates& image_coordinates) const {
  const Vector2D index = imageToIndexReal(image_coordinates);
  const Vector2D index_lower = index.array().floor();
  const Vector2D index_upper = index.array().ceil();

  std::array<Index2D, 4> indices{};
  std::array<Vector2D, 4> offsets{};
  for (int neighbox_idx = 0; neighbox_idx < 4; ++neighbox_idx) {
    const Vector2D index_rounded{
        neighbox_idx & 0b01 ? index_upper[0] : index_lower[0],
        neighbox_idx & 0b10 ? index_upper[1] : index_lower[1]};
    indices[neighbox_idx] = index_rounded.cast<IndexElement>();
    offsets[neighbox_idx][0] =
        index_to_image_scale_factor_[0] * (index[0] - index_rounded[0]);
    offsets[neighbox_idx][1] =
        index_to_image_scale_factor_[1] * (index[1] - index_rounded[1]);
  }

  return {std::move(indices), std::move(offsets)};
}

inline ImageCoordinates ProjectorBase::indexToImage(
    const Index2D& index) const {
  return index.cast<FloatingPoint>().cwiseProduct(
             index_to_image_scale_factor_) +
         image_offset_;
}

inline Vector2D ProjectorBase::imageToIndexReal(
    const ImageCoordinates& image_coordinates) const {
  return (image_coordinates - image_offset_)
      .cwiseProduct(image_to_index_scale_factor_);
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_
