#ifndef WAVEMAP_FILE_CONVERSIONS_PROTO_UTILS_H_
#define WAVEMAP_FILE_CONVERSIONS_PROTO_UTILS_H_

#include <fstream>
#include <istream>

#include <google/protobuf/message.h>
#include <google/protobuf/message_lite.h>
#include <wavemap/common.h>

namespace wavemap::proto {
bool readProtoMsgFromStream(std::istream* stream_in,
                            google::protobuf::Message* message);

bool writeProtoMsgToStream(const google::protobuf::Message& message,
                           std::ofstream* stream_out);
}  // namespace wavemap::proto

#endif  // WAVEMAP_FILE_CONVERSIONS_PROTO_UTILS_H_
