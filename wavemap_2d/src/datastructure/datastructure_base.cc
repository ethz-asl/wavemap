#include "wavemap_2d/datastructure/datastructure_base.h"

namespace wavemap_2d {
void DataStructureBase::showImage(bool use_color, int delay_ms) const {
  if (empty()) {
    return;
  }

  cv::namedWindow("Grid map", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Grid map", CV_WND_PROP_FULLSCREEN,
                        CV_WINDOW_FULLSCREEN);
  cv::imshow("Grid map", getImage(use_color));
  cv::waitKey(delay_ms);
}

void DataStructureBase::saveImage(const std::string& file_path,
                                  bool use_color) const {
  cv::imwrite(file_path, getImage(use_color));
}
}  // namespace wavemap_2d
