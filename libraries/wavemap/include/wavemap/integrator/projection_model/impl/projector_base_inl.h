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
  const Vector2D index_real = imageToIndexReal(image_coordinates);
  const Vector2D index_rounded = index_real.array().round();
  Vector2D image_coordinate_offset =
      (index_rounded - index_real).cwiseProduct(index_to_image_scale_factor_);
  return {index_rounded.cast<IndexElement>(),
          std::move(image_coordinate_offset)};
}

inline ProjectorBase::NearestIndexArray ProjectorBase::imageToNearestIndices(
    const ImageCoordinates& image_coordinates) const {
  NearestIndexArray indices;

  const Vector2D index_real = imageToIndexReal(image_coordinates);
  indices.col(0) = index_real.array().floor().cast<IndexElement>();
  indices.col(3) = index_real.array().ceil().cast<IndexElement>();
  indices(0, 1) = indices(0, 3);
  indices(1, 1) = indices(1, 0);
  indices(0, 2) = indices(0, 0);
  indices(1, 2) = indices(1, 3);

  return indices;
}

inline std::pair<ProjectorBase::NearestIndexArray,
                 ProjectorBase::CellToBeamOffsetArray>
ProjectorBase::imageToNearestIndicesAndOffsets(
    const ImageCoordinates& image_coordinates) const {
  std::pair<NearestIndexArray, CellToBeamOffsetArray> result;
  auto& indices = result.first;
  auto& offsets = result.second;

  const Vector2D index_real = imageToIndexReal(image_coordinates);
  offsets.col(0) = index_real.array().floor();
  offsets.col(3) = index_real.array().ceil();
  offsets(0, 1) = offsets(0, 3);
  offsets(1, 1) = offsets(1, 0);
  offsets(0, 2) = offsets(0, 0);
  offsets(1, 2) = offsets(1, 3);

  indices = offsets.cast<IndexElement>();
  offsets = index_to_image_scale_factor_.asDiagonal() *
            (offsets.colwise() - index_real);

  return result;
}

inline ImageCoordinates ProjectorBase::indexToImage(
    const Index2D& index) const {
  return index.cast<FloatingPoint>().cwiseProduct(
             index_to_image_scale_factor_) +
         image_offset_;
}

inline FloatingPoint ProjectorBase::imageOffsetToErrorNorm(
    const ImageCoordinates& linearization_point, const Vector2D& offset) const {
  return std::sqrt(imageOffsetToErrorSquaredNorm(linearization_point, offset));
}

inline std::array<FloatingPoint, 4> ProjectorBase::imageOffsetsToErrorNorms(
    const ImageCoordinates& linearization_point,
    const ProjectorBase::CellToBeamOffsetArray& offsets) const {
  auto error_norms =
      imageOffsetsToErrorSquaredNorms(linearization_point, offsets);
  for (auto& error_norm : error_norms) {
    error_norm = std::sqrt(error_norm);
  }
  return error_norms;
}

inline Vector2D ProjectorBase::imageToIndexReal(
    const ImageCoordinates& image_coordinates) const {
  return (image_coordinates - image_offset_)
      .cwiseProduct(image_to_index_scale_factor_);
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_
