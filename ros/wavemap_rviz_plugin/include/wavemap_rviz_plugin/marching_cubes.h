#ifndef WAVEMAP_RVIZ_PLUGIN_MARCHING_CUBES_H_
#define WAVEMAP_RVIZ_PLUGIN_MARCHING_CUBES_H_

#include <vector>

#include <wavemap/common.h>

// All credits for this code go to OpenChisel:
// https://github.com/personalrobotics/OpenChisel

namespace wavemap {
class MarchingCubes {
 public:
  static int kTriangleTable[256][16];
  static int kEdgeIndexPairs[12][2];
  static Index3D kCornerOffsets[8];

  static void meshCube(
      const Eigen::Matrix<FloatingPoint, 3, 8>& corner_coordinates,
      const std::array<FloatingPoint, 8>& corner_values,
      std::vector<Point3D>& vertices) {
    const int index = calculateVertexConfiguration(corner_values);

    Eigen::Matrix<FloatingPoint, 3, 12> edge_vertex_coordinates;
    interpolateEdgeVertices(corner_coordinates, corner_values,
                            edge_vertex_coordinates);

    const int* table_row = kTriangleTable[index];

    int table_col = 0;
    while (table_row[table_col] != -1) {
      const Vector3D& p0 = edge_vertex_coordinates.col(table_row[table_col]);
      const Vector3D& p1 =
          edge_vertex_coordinates.col(table_row[table_col + 1]);
      const Vector3D& p2 =
          edge_vertex_coordinates.col(table_row[table_col + 2]);

      vertices.emplace_back(p0);
      vertices.emplace_back(p1);
      vertices.emplace_back(p2);

      table_col += 3;
    }
  }

  static int calculateVertexConfiguration(
      const std::array<FloatingPoint, 8>& corner_values) {
    return (corner_values[0] < 0.f ? (1 << 0) : 0) |
           (corner_values[1] < 0.f ? (1 << 1) : 0) |
           (corner_values[2] < 0.f ? (1 << 2) : 0) |
           (corner_values[3] < 0.f ? (1 << 3) : 0) |
           (corner_values[4] < 0.f ? (1 << 4) : 0) |
           (corner_values[5] < 0.f ? (1 << 5) : 0) |
           (corner_values[6] < 0.f ? (1 << 6) : 0) |
           (corner_values[7] < 0.f ? (1 << 7) : 0);
  }

  static void interpolateEdgeVertices(
      const Eigen::Matrix<FloatingPoint, 3, 8>& corner_coordinates,
      const std::array<FloatingPoint, 8>& corner_values,
      Eigen::Matrix<FloatingPoint, 3, 12>& edge_coordinates) {
    for (int i = 0; i < 12; ++i) {
      const int* pairs = kEdgeIndexPairs[i];
      const int edge0 = pairs[0];
      const int edge1 = pairs[1];
      // Only interpolate along edges where there is a zero crossing.
      if ((corner_values[edge0] < 0 && corner_values[edge1] >= 0) ||
          (corner_values[edge0] >= 0 && corner_values[edge1] < 0))
        edge_coordinates.col(i) = interpolateVertex(
            corner_coordinates.col(edge0), corner_coordinates.col(edge1),
            corner_values[edge0], corner_values[edge1]);
    }
  }

  // Performs linear interpolation on two cube corners to find the approximate
  // zero crossing (surface) value.
  static Point3D interpolateVertex(const Point3D& corner_1,
                                   const Point3D& corner_2,
                                   FloatingPoint value_1,
                                   FloatingPoint value_2) {
    const FloatingPoint value_difference = value_1 - value_2;
    if (std::abs(value_difference) < 1e-4f) {
      return 0.5f * (corner_1 + corner_2);
    }
    const FloatingPoint t = value_1 / value_difference;
    CHECK_GE(t, 0.f);
    CHECK_LE(t, 1.f);
    return (1.f - t) * corner_1 + t * corner_2;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_RVIZ_PLUGIN_MARCHING_CUBES_H_
