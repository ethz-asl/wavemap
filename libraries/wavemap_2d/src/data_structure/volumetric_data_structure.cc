#include "wavemap_2d/data_structure/volumetric_data_structure.h"

#include "wavemap_2d/utils/image_utils.h"

namespace wavemap {
void VolumetricDataStructure::showImage(bool use_color, int delay_ms) const {
  ShowImage(getImage(use_color), delay_ms);
}

void VolumetricDataStructure::saveImage(const std::string& file_path,
                                        bool use_color) const {
  cv::imwrite(file_path, getImage(use_color));
}
}  // namespace wavemap
