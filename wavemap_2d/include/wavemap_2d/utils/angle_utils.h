#ifndef WAVEMAP_2D_UTILS_ANGLE_UTILS_H_
#define WAVEMAP_2D_UTILS_ANGLE_UTILS_H_

namespace wavemap_2d::angle_math {
inline FloatingPoint normalize(FloatingPoint angle) {
  return std::remainder(angle, M_PIf32);
}

inline FloatingPoint normalize_near(FloatingPoint angle) {
  if (angle < -M_PIf32) {
    angle += 2.f * M_PIf32;
  } else if (M_PIf32 < angle) {
    angle -= 2.f * M_PIf32;
  }
  return angle;
}
}  // namespace wavemap_2d::angle_math

#endif  // WAVEMAP_2D_UTILS_ANGLE_UTILS_H_
