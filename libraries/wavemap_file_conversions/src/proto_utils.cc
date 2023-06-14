#include "wavemap_file_conversions/proto_utils.h"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace wavemap::proto {
bool readProtoMsgFromStream(std::istream* stream_in,
                            google::protobuf::Message* message) {
  CHECK_NOTNULL(stream_in);
  CHECK_NOTNULL(message);

  stream_in->clear();
  stream_in->seekg(0, std::ios::beg);
  google::protobuf::io::IstreamInputStream raw_in(stream_in);
  google::protobuf::io::CodedInputStream coded_in(&raw_in);

  uint32_t message_size;
  if (!coded_in.ReadVarint32(&message_size)) {
    LOG(ERROR) << "Could not read protobuf message size.";
    return false;
  }

  if (message_size == 0u) {
    LOG(ERROR) << "Empty protobuf message!";
    return false;
  }
  google::protobuf::io::CodedInputStream::Limit limit =
      coded_in.PushLimit(message_size);
  if (!message->ParseFromCodedStream(&coded_in)) {
    LOG(ERROR) << "Could not parse stream.";
    return false;
  }
  if (!coded_in.ConsumedEntireMessage()) {
    LOG(ERROR) << "Could not consume protobuf message.";
    return false;
  }
  coded_in.PopLimit(limit);
  return true;
}

bool writeProtoMsgToStream(const google::protobuf::Message& message,
                           std::ofstream* stream_out) {
  CHECK_NOTNULL(stream_out);
  CHECK(stream_out->is_open());
  google::protobuf::io::OstreamOutputStream raw_out(stream_out);
  google::protobuf::io::CodedOutputStream coded_out(&raw_out);
  const uint32_t size_bytes = message.ByteSize();
  coded_out.WriteVarint32(size_bytes);
  uint8_t* buffer = coded_out.GetDirectBufferForNBytesAndAdvance(size_bytes);
  if (buffer != nullptr) {
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    message.SerializeWithCachedSizes(&coded_out);
    if (coded_out.HadError()) {
      return false;
    }
  }

  return true;
}
}  // namespace wavemap::proto
