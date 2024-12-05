#ifndef WAVEMAP_IO_CONFIG_STREAM_CONVERSIONS_H_
#define WAVEMAP_IO_CONFIG_STREAM_CONVERSIONS_H_

#include <istream>

#include <wavemap/core/config/param.h>

namespace wavemap::io {
bool yamlStreamToParams(std::istream& istream, param::Value& params);
}

#endif  // WAVEMAP_IO_CONFIG_STREAM_CONVERSIONS_H_
