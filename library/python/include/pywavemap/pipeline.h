#ifndef PYWAVEMAP_PIPELINE_H_
#define PYWAVEMAP_PIPELINE_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_pipeline_bindings(nb::module_& m);
}  // namespace wavemap

#endif  // PYWAVEMAP_PIPELINE_H_
