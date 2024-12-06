#ifndef PYWAVEMAP_RENDER_H_
#define PYWAVEMAP_RENDER_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_render_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_RENDER_H_
