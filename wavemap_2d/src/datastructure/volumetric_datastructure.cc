#include "wavemap_2d/datastructure/volumetric/volumetric_datastructure.h"

#include "wavemap_2d/utils/image_utils.h"

namespace wavemap_2d {
void DataStructureBase::showImage(bool use_color, int delay_ms) const {
  ShowImage(getImage(use_color), delay_ms);
}

void DataStructureBase::saveImage(const std::string& file_path,
                                  bool use_color) const {
  cv::imwrite(file_path, getImage(use_color));
}
}  // namespace wavemap_2d
