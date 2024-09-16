#include "pywavemap/indices.h"

#include <nanobind/eigen/dense.h>
#include <wavemap/core/indexing/ndtree_index.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_index_bindings(nb::module_& m) {
  nb::class_<OctreeIndex>(m, "OctreeIndex",
                          "A class representing indices of octree nodes.")
      .def(nb::init<>())
      .def(nb::init<OctreeIndex::Element, OctreeIndex::Position>(), "height"_a,
           "position"_a)
      .def_rw("height", &OctreeIndex::height, "height"_a = 0,
              "The node's resolution level in the octree. A height of 0 "
              "corresponds to the map’s maximum resolution. In a fully "
              "allocated tree, all leaf nodes are at height 0. Increasing the "
              "height by 1 doubles the node size along each dimension. The "
              "root node corresponds to the map's lowest resolution, and the "
              "root node's height matches the configured tree height.")
      .def_rw("position", &OctreeIndex::position, "position"_a,
              "The node's XYZ position in the octree’s grid at the resolution "
              "level set by *height*.")
      .def("compute_parent_index",
           nb::overload_cast<>(&OctreeIndex::computeParentIndex, nb::const_),
           "Compute the index of the node's direct parent.")
      .def("compute_parent_index",
           nb::overload_cast<OctreeIndex::Element>(
               &OctreeIndex::computeParentIndex, nb::const_),
           "parent_height"_a,
           "Compute the index of the node's parent (or ancestor) at "
           "*parent_height*.")
      .def("compute_child_index", &OctreeIndex::computeChildIndex,
           "relative_child_index"_a,
           "Compute the index of the node's n-th child, where n ranges from 0 "
           "to 7.");
}
}  // namespace wavemap
