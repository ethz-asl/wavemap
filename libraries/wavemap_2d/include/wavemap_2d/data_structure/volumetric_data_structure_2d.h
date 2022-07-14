#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_H_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <wavemap_common/data_structure/volumetric_data_structure_base.h>

#include "wavemap_2d/utils/image_utils.h"

namespace wavemap {
class VolumetricDataStructure2D
    : public virtual VolumetricDataStructureBase<2> {
 public:
  using Ptr = std::shared_ptr<VolumetricDataStructure2D>;

  using VolumetricDataStructureBase<2>::VolumetricDataStructureBase;

  virtual cv::Mat getImage(bool use_color) const = 0;
  void showImage(bool use_color, int delay_ms = 1) const {
    ShowImage(getImage(use_color), delay_ms);
  }
  void saveImage(const std::string& file_path, bool use_color = false) const {
    cv::imwrite(file_path, getImage(use_color));
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_DATA_STRUCTURE_2D_H_
