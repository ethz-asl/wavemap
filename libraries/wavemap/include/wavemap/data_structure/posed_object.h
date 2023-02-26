#ifndef WAVEMAP_DATA_STRUCTURE_POSED_OBJECT_H_
#define WAVEMAP_DATA_STRUCTURE_POSED_OBJECT_H_

#include <memory>
#include <utility>

#include "wavemap/common.h"

namespace wavemap {
template <typename ObjectT>
class PosedObject : public ObjectT {
 public:
  using Ptr = std::shared_ptr<PosedObject<ObjectT>>;
  using ConstPtr = std::shared_ptr<const PosedObject<ObjectT>>;

  using ObjectT::ObjectT;

  template <typename... Args>
  explicit PosedObject(Transformation3D T_W_C, Args... args)
      : ObjectT(std::forward<Args>(args)...) {
    setPose(T_W_C);
  }

  void setPose(const Transformation3D& T_W_C) {
    T_W_C_ = T_W_C;
    R_W_C_ = T_W_C_.getRotationMatrix();
    T_C_W_ = T_W_C.inverse();
    R_C_W_ = T_C_W_.getRotationMatrix();
  }

  const Point3D& getOrigin() const { return T_W_C_.getPosition(); }
  const Transformation3D& getPose() const { return T_W_C_; }
  const Transformation3D& getPoseInverse() const { return T_C_W_; }

  const Rotation3D& getRotation() const { return T_W_C_.getRotation(); }
  const Transformation3D::RotationMatrix& getRotationMatrix() const {
    return R_W_C_;
  }
  const Transformation3D::RotationMatrix& getRotationMatrixInverse() const {
    return R_C_W_;
  }

 private:
  Transformation3D T_W_C_;
  Transformation3D T_C_W_;
  Transformation3D::RotationMatrix R_W_C_;
  Transformation3D::RotationMatrix R_C_W_;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_POSED_OBJECT_H_
