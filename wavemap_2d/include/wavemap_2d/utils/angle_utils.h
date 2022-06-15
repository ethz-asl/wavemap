#ifndef WAVEMAP_2D_UTILS_ANGLE_UTILS_H_
#define WAVEMAP_2D_UTILS_ANGLE_UTILS_H_

namespace wavemap_2d::angle_math {
inline FloatingPoint normalize(FloatingPoint angle) {
  return std::remainder(angle, kPi);
}

inline FloatingPoint normalize_near(FloatingPoint angle) {
  if (angle < -kPi) {
    angle += kTwoPi;
  } else if (kPi < angle) {
    angle -= kTwoPi;
  }
  return angle;
}
}  // namespace wavemap_2d::angle_math

#endif  // WAVEMAP_2D_UTILS_ANGLE_UTILS_H_
