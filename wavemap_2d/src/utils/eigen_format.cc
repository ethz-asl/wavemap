#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
const Eigen::IOFormat EigenFormat::kOneLine =
    Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", " [", "]");
}
