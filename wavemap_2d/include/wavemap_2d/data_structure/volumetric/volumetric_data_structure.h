#ifndef WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_H_
#define WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_H_

#include <algorithm>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"

namespace wavemap_2d {
class VolumetricDataStructure {
 public:
  using Ptr = std::shared_ptr<VolumetricDataStructure>;

  explicit VolumetricDataStructure(const FloatingPoint resolution)
      : resolution_(resolution) {}
  virtual ~VolumetricDataStructure() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void clear() = 0;

  FloatingPoint getResolution() const { return resolution_; }
  virtual size_t getMemoryUsage() const = 0;

  virtual Index getMinIndex() const = 0;
  virtual Index getMaxIndex() const = 0;

  virtual bool hasCell(const Index& index) const = 0;
  virtual FloatingPoint getCellValue(const Index& index) const = 0;
  virtual void setCellValue(const Index& index, FloatingPoint new_value) = 0;
  virtual void addToCellValue(const Index& index, FloatingPoint update) = 0;

  void printSize() const { LOG(INFO) << "Size:\n" << size(); }

  virtual cv::Mat getImage(bool use_color) const = 0;
  void showImage(bool use_color = false, int delay_ms = 1) const;
  void saveImage(const std::string& file_path, bool use_color = false) const;
  virtual bool save(const std::string& file_path_prefix,
                    bool use_floating_precision) const = 0;
  // TODO(victorr): Automatically determine whether floating or fixed precision
  //                was used from the file format once it has been designed
  virtual bool load(const std::string& file_path_prefix,
                    bool used_floating_precision) = 0;

 protected:
  const FloatingPoint resolution_;

  static std::string getHeaderFilePath(const std::string& file_path_prefix) {
    return file_path_prefix + "_header";
  }
  static std::string getDataFilePath(const std::string& file_path_prefix,
                                     const bool use_floating_precision) {
    return file_path_prefix + "_data." +
           (use_floating_precision ? "tiff" : "jp2");
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_H_
