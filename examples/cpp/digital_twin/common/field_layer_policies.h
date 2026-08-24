#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_POLICIES_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_POLICIES_H_

#include <string_view>

#include <wavemap/layered/layer_schema.h>
#include <wavemap/layered/layer_update_policy.h>

#include "field_layer_types.h"

namespace wavemap::examples::field_map {

using ReflectivityUpdatePolicy = layered::ReplaceLayerUpdatePolicy<float>;
using ClassUpdatePolicy = layered::ReplaceLayerUpdatePolicy<int>;

struct ReflectivityLayer
    : layered::schema::ContinuousLayer<float, ReflectivityUpdatePolicy> {
  static constexpr std::string_view name = "reflectivity";
};

struct ClassLayer
    : layered::schema::DiscreteLayer<int, ClassUpdatePolicy> {
  static constexpr std::string_view name = "class";
};

using Schema =
    layered::schema::LayerSchema<ReflectivityLayer, ClassLayer>;

static_assert(Schema::size == 2u);
static_assert(Schema::continuousLayerCount == 1u);
static_assert(Schema::discreteLayerCount == 1u);
static_assert(Schema::containsContinuous<ReflectivityLayer>);
static_assert(Schema::containsDiscrete<ClassLayer>);

}  // namespace wavemap::examples::field_map

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYER_POLICIES_H_
