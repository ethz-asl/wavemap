#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_TYPES_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_TYPES_H_

namespace wavemap::examples::field_map {

enum class ClassLabel : int {
  kUnknown = 0,
  kGround = 1,
  kObstacle = 2,
};

}  // namespace wavemap::examples::field_map

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_TYPES_H_
