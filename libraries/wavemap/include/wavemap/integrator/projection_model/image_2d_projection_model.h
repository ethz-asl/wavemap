#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PROJECTION_MODEL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PROJECTION_MODEL_H_

#include <utility>

#include "wavemap/common.h"
#include "wavemap/data_structure/aabb.h"

namespace wavemap {
class Image2DProjectionModel {
 public:
  using ImageCoordinates = Vector2D;

  Image2DProjectionModel(Vector2D index_to_image_scale_factor,
                         Vector2D image_offset)
      : index_to_image_scale_factor_(std::move(index_to_image_scale_factor)),
        image_offset_(std::move(image_offset)) {}
  virtual ~Image2DProjectionModel() = default;

  virtual IndexElement getNumRows() const = 0;
  virtual IndexElement getNumColumns() const = 0;
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }
  virtual ImageCoordinates getMinImageCoordinates() const = 0;
  virtual ImageCoordinates getMaxImageCoordinates() const = 0;
  virtual Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const = 0;
  virtual Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const = 0;

  // Coordinate transforms between Cartesian and sensor space
  virtual Vector3D cartesianToSensor(const Point3D& C_point) const = 0;
  virtual Point3D sensorToCartesian(const Vector3D& coordinates) const = 0;
  virtual Point3D sensorToCartesian(const ImageCoordinates& image_coordinates,
                                    FloatingPoint range) const = 0;

  // Projection from Cartesian space onto the sensor's image surface
  virtual ImageCoordinates cartesianToImage(const Point3D& C_point) const = 0;
  virtual FloatingPoint cartesianToSensorZ(const Point3D& C_point) const = 0;

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  Index2D imageToNearestIndex(const ImageCoordinates& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .round()
        .cast<IndexElement>();
  }
  Index2D imageToFloorIndex(const ImageCoordinates& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .floor()
        .cast<IndexElement>();
  }
  Index2D imageToCeilIndex(const ImageCoordinates& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .ceil()
        .cast<IndexElement>();
  }
  std::pair<Index2D, ImageCoordinates> imageToNearestIndexAndOffset(
      const ImageCoordinates& image_coordinates) const {
    const Vector2D index = imageToIndexReal(image_coordinates);
    const Vector2D index_rounded = index.array().round();
    const ImageCoordinates image_coordinate_offset =
        (index - index_rounded).cwiseProduct(index_to_image_scale_factor_);
    return {index_rounded.cast<IndexElement>(), image_coordinate_offset};
  }
  std::pair<std::array<Index2D, 4>, std::array<ImageCoordinates, 4>>
  imageToNearestIndicesAndOffsets(
      const ImageCoordinates& image_coordinates) const {
    const Vector2D index = imageToIndexReal(image_coordinates);
    const Vector2D index_lower = index.array().floor();
    const Vector2D index_upper = index.array().ceil();

    std::array<Index2D, 4> indices{};
    std::array<ImageCoordinates, 4> offsets{};
    for (int neighbox_idx = 0; neighbox_idx < 4; ++neighbox_idx) {
      const Vector2D index_rounded{
          neighbox_idx & 0b01 ? index_upper[0] : index_lower[0],
          neighbox_idx & 0b10 ? index_upper[1] : index_lower[1]};
      indices[neighbox_idx] = index_rounded.cast<IndexElement>();
      offsets[neighbox_idx] =
          (index - index_rounded).cwiseProduct(index_to_image_scale_factor_);
    }

    return {indices, offsets};
  }
  ImageCoordinates indexToImage(const Index2D& index) const {
    return index.cast<FloatingPoint>().cwiseProduct(
               index_to_image_scale_factor_) +
           image_offset_;
  }

  // Compute the error norm in the image plane based on an offset vector (in
  // image plane) and a linearization point
  // NOTE: For spherical models, this error norm corresponds to the relative
  //       angle between the two rays whose offset is given. For camera models,
  //       it corresponds to the reprojection error in pixels.
  virtual FloatingPoint imageOffsetToErrorNorm(
      const ImageCoordinates& linearization_point,
      ImageCoordinates offset) const = 0;

  // Convenience functions combining multiple of the above methods
  Index2D cartesianToNearestIndex(const Point3D& C_point) const {
    return imageToNearestIndex(cartesianToImage(C_point));
  }

  virtual AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const = 0;

 protected:
  const Vector2D index_to_image_scale_factor_;
  const Vector2D image_to_index_scale_factor_ =
      Vector2D::Ones().cwiseQuotient(index_to_image_scale_factor_);
  const Vector2D image_offset_;

  Vector2D imageToIndexReal(const ImageCoordinates& image_coordinates) const {
    return (image_coordinates - image_offset_)
        .cwiseProduct(image_to_index_scale_factor_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PROJECTION_MODEL_H_
