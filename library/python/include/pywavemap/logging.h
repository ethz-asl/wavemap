#ifndef PYWAVEMAP_LOGGING_H_
#define PYWAVEMAP_LOGGING_H_

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace wavemap {
void add_logging_module(nb::module_& m_logging);
}  // namespace wavemap

#endif  // PYWAVEMAP_LOGGING_H_
