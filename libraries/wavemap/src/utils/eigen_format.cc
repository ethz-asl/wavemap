#include "wavemap/utils/eigen_format.h"

namespace wavemap {
const Eigen::IOFormat EigenFormat::kOneLine =
    Eigen::IOFormat(4, Eigen::DontAlignCols, ", ", ", ", "", "", " [", "]");
}
