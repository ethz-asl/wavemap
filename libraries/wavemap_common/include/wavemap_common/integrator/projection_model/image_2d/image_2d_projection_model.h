#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_IMAGE_2D_PROJECTION_MODEL_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_IMAGE_2D_PROJECTION_MODEL_H_

#include <utility>

#include "wavemap_common/common.h"

namespace wavemap {
class Image2DProjectionModel {
 public:
  Image2DProjectionModel(Vector2D index_to_image_scale_factor,
                         Vector2D image_offset)
      : index_to_image_scale_factor_(std::move(index_to_image_scale_factor)),
        image_offset_(std::move(image_offset)) {}

  virtual IndexElement getNumRows() const = 0;
  virtual IndexElement getNumColumns() const = 0;
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }
  virtual Vector2D getMinImageCoordinates() const = 0;
  virtual Vector2D getMaxImageCoordinates() const = 0;

  // Coordinate transforms between Cartesian and sensor space
  virtual Vector3D cartesianToSensor(const Point3D& C_point) const = 0;
  virtual Point3D sensorToCartesian(const Vector3D& coordinates) const = 0;
  virtual Point3D sensorToCartesian(const Vector2D& image_coordinates,
                                    FloatingPoint range) const = 0;

  // Projection from Cartesian space onto the sensor's image surface
  virtual Vector2D cartesianToImage(const Point3D& C_point) const = 0;

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  Index2D imageToNearestIndex(const Vector2D& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .round()
        .cast<IndexElement>();
  }
  Index2D imageToFloorIndex(const Vector2D& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .floor()
        .cast<IndexElement>();
  }
  Index2D imageToCeilIndex(const Vector2D& image_coordinates) const {
    return imageToIndexReal(image_coordinates)
        .array()
        .ceil()
        .cast<IndexElement>();
  }
  std::pair<Index2D, Vector2D> imageToNearestIndexAndOffset(
      const Vector2D& image_coordinates) const {
    const Vector2D index = imageToIndexReal(image_coordinates);
    const Vector2D index_rounded = index.array().round();
    const Vector2D image_coordinate_offset =
        (index - index_rounded).cwiseProduct(index_to_image_scale_factor_);
    return {index_rounded.cast<IndexElement>(), image_coordinate_offset};
  }
  Vector2D indexToImage(const Index2D& index) const {
    return index.cast<FloatingPoint>().cwiseProduct(
               index_to_image_scale_factor_) +
           image_offset_;
  }

  // Convenience functions combining multiple of the above methods
  Index2D cartesianToNearestIndex(const Point3D& C_point) const {
    return imageToNearestIndex(cartesianToImage(C_point));
  }

 protected:
  const Vector2D index_to_image_scale_factor_;
  const Vector2D image_to_index_scale_factor_ =
      Vector2D::Ones().cwiseQuotient(index_to_image_scale_factor_);
  const Vector2D image_offset_;

  Vector2D imageToIndexReal(const Vector2D& image_coordinates) const {
    return (image_coordinates - image_offset_)
        .cwiseProduct(image_to_index_scale_factor_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_IMAGE_2D_PROJECTION_MODEL_H_
