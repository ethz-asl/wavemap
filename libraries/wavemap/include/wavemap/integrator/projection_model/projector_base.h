#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_BASE_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_BASE_H_

#include <memory>
#include <utility>

#include "wavemap/common.h"
#include "wavemap/config/type_selector.h"
#include "wavemap/config/value_with_unit.h"
#include "wavemap/data_structure/aabb.h"

namespace wavemap {
struct ProjectorType : TypeSelector<ProjectorType> {
  using TypeSelector<ProjectorType>::TypeSelector;

  enum Id : TypeId {
    kSphericalProjector,
    kOusterProjector,
    kPinholeCameraProjector
  };

  static constexpr std::array names = {
      "spherical_projector", "ouster_projector", "pinhole_camera_projector"};
};

class ProjectorBase {
 public:
  using Ptr = std::shared_ptr<ProjectorBase>;
  using ConstPtr = std::shared_ptr<const ProjectorBase>;

  ProjectorBase(Index2D dimensions, Vector2D index_to_image_scale_factor,
                Vector2D image_offset)
      : dimensions_(std::move(dimensions)),
        index_to_image_scale_factor_(std::move(index_to_image_scale_factor)),
        image_offset_(std::move(image_offset)) {}
  virtual ~ProjectorBase() = default;

  IndexElement getNumRows() const { return dimensions_.x(); }
  IndexElement getNumColumns() const { return dimensions_.y(); }
  Index2D getDimensions() const { return dimensions_; }
  virtual Vector2D getMinImageCoordinates() const = 0;
  virtual Vector2D getMaxImageCoordinates() const = 0;
  virtual Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const = 0;
  virtual Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const = 0;
  virtual SiUnit getImageCoordinatesUnit() const = 0;

  // Coordinate transforms between Cartesian and sensor space
  virtual Vector3D cartesianToSensor(const Point3D& C_point) const = 0;
  virtual Point3D sensorToCartesian(const Vector3D& coordinates) const = 0;
  virtual Point3D sensorToCartesian(const Vector2D& image_coordinates,
                                    FloatingPoint range) const = 0;

  // Projection from Cartesian space onto the sensor's image surface
  virtual Vector2D cartesianToImage(const Point3D& C_point) const = 0;
  virtual FloatingPoint cartesianToSensorZ(const Point3D& C_point) const = 0;

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  Index2D imageToNearestIndex(const Vector2D& image_coordinates) const;
  Index2D imageToFloorIndex(const Vector2D& image_coordinates) const;
  Index2D imageToCeilIndex(const Vector2D& image_coordinates) const;
  std::array<Index2D, 4> imageToNearestIndices(
      const Vector2D& image_coordinates) const;

  std::pair<Index2D, Vector2D> imageToNearestIndexAndOffset(
      const Vector2D& image_coordinates) const;
  std::pair<std::array<Index2D, 4>, std::array<Vector2D, 4>>
  imageToNearestIndicesAndOffsets(const Vector2D& image_coordinates) const;

  Vector2D indexToImage(const Index2D& index) const;

  // Compute the error norm in the image plane based on an offset vector (in
  // image plane) and a linearization point
  // NOTE: For spherical models, this error norm corresponds to the relative
  //       angle between the two rays whose offset is given. For camera models,
  //       it corresponds to the reprojection error in pixels.
  virtual FloatingPoint imageOffsetToErrorNorm(
      const Vector2D& linearization_point, const Vector2D& offset) const = 0;
  using CellToBeamOffsetArray = std::array<Vector2D, 4>;
  virtual std::array<FloatingPoint, 4> imageOffsetsToErrorNorms(
      const Vector2D& linearization_point,
      const CellToBeamOffsetArray& offsets) const = 0;

  // Convenience functions combining multiple of the above methods
  Index2D cartesianToNearestIndex(const Point3D& C_point) const {
    return imageToNearestIndex(cartesianToImage(C_point));
  }

  virtual AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const = 0;

 protected:
  const Index2D dimensions_;
  const Vector2D index_to_image_scale_factor_;
  const Vector2D image_to_index_scale_factor_ =
      Vector2D::Ones().cwiseQuotient(index_to_image_scale_factor_);
  const Vector2D image_offset_;

  Vector2D imageToIndexReal(const Vector2D& image_coordinates) const;
};
}  // namespace wavemap

#include "wavemap/integrator/projection_model/impl/projector_base_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_BASE_H_
