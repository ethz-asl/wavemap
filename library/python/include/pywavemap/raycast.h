#ifndef PYWAVEMAP_RAYCAST_H_
#define PYWAVEMAP_RAYCAST_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_raycast_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_RAYCAST_H_
