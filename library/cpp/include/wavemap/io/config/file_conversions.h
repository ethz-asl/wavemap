#ifndef WAVEMAP_IO_CONFIG_FILE_CONVERSIONS_H_
#define WAVEMAP_IO_CONFIG_FILE_CONVERSIONS_H_

#include <filesystem>

#include <wavemap/core/config/param.h>

namespace wavemap::io {
std::optional<param::Value> yamlFileToParams(
    const std::filesystem::path& file_path);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_CONFIG_FILE_CONVERSIONS_H_
