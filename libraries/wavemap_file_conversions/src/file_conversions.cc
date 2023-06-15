#include "wavemap_file_conversions/file_conversions.h"

#include <fstream>

namespace wavemap::convert {
bool mapToFile(const VolumetricDataStructureBase& map,
               const std::string& file_path) {
  CHECK(!file_path.empty());

  // Open the file for writing
  std::ofstream file_ostream(file_path, std::ofstream::out);
  if (!file_ostream.is_open()) {
    LOG(WARNING) << "Could not open file '" << file_path << "' for writing.";
    return false;
  }

  // Serialize
  proto::Map map_proto;
  mapToProto(map, &map_proto);

  // Write to bytestream
  return map_proto.SerializeToOstream(&file_ostream);
}

bool fileToMap(const std::string& file_path,
               VolumetricDataStructureBase::Ptr& map) {
  CHECK(!file_path.empty());
  CHECK_NOTNULL(map);

  // Open the file for reading
  std::ifstream file_istream(file_path, std::ifstream::in);
  if (!file_istream.is_open()) {
    LOG(WARNING) << "Could not open file '" << file_path << "' for reading.";
    return false;
  }

  // Read from bytestream
  proto::Map map_proto;
  if (!map_proto.ParseFromIstream(&file_istream)) {
    LOG(WARNING) << "Failed to parse map proto from file '" << file_path
                 << "'.";
    return false;
  }

  // Deserialize
  protoToMap(map_proto, map);

  return true;
}
}  // namespace wavemap::convert
