#ifndef WAVEMAP_IO_IMPL_STREAMABLE_TYPES_IMPL_H_
#define WAVEMAP_IO_IMPL_STREAMABLE_TYPES_IMPL_H_

namespace wavemap::io::streamable {
void Index3D::write(std::ostream& ostream) const {
  ostream.write(reinterpret_cast<const char*>(&x), sizeof(x));
  ostream.write(reinterpret_cast<const char*>(&y), sizeof(y));
  ostream.write(reinterpret_cast<const char*>(&z), sizeof(z));
}

Index3D Index3D::read(std::istream& istream) {
  Index3D instance;
  istream.read(reinterpret_cast<char*>(&instance.x), sizeof(x));
  istream.read(reinterpret_cast<char*>(&instance.y), sizeof(y));
  istream.read(reinterpret_cast<char*>(&instance.z), sizeof(z));
  return instance;
}

void HashedBlockHeader::write(std::ostream& ostream) const {
  block_offset.write(ostream);
}

HashedBlockHeader HashedBlockHeader::read(std::istream& istream) {
  HashedBlockHeader instance;
  instance.block_offset = Index3D::read(istream);
  return instance;
}

void HashedBlocksHeader::write(std::ostream& ostream) const {
  ostream.write(reinterpret_cast<const char*>(&min_cell_width),
                sizeof(min_cell_width));
  ostream.write(reinterpret_cast<const char*>(&min_log_odds),
                sizeof(min_log_odds));
  ostream.write(reinterpret_cast<const char*>(&max_log_odds),
                sizeof(max_log_odds));
  ostream.write(reinterpret_cast<const char*>(&num_blocks), sizeof(num_blocks));
}

HashedBlocksHeader HashedBlocksHeader::read(std::istream& istream) {
  HashedBlocksHeader instance;
  istream.read(reinterpret_cast<char*>(&instance.min_cell_width),
               sizeof(min_cell_width));
  istream.read(reinterpret_cast<char*>(&instance.min_log_odds),
               sizeof(min_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.max_log_odds),
               sizeof(max_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.num_blocks),
               sizeof(num_blocks));
  return instance;
}

void WaveletOctreeNode::write(std::ostream& ostream) const {
  for (Float coefficient : detail_coefficients) {
    ostream.write(reinterpret_cast<const char*>(&coefficient),
                  sizeof(coefficient));
  }
  ostream.write(reinterpret_cast<const char*>(&allocated_children_bitset),
                sizeof(allocated_children_bitset));
}

WaveletOctreeNode WaveletOctreeNode::read(std::istream& istream) {
  WaveletOctreeNode instance;
  for (Float& coefficient : instance.detail_coefficients) {
    istream.read(reinterpret_cast<char*>(&coefficient), sizeof(coefficient));
  }
  istream.read(reinterpret_cast<char*>(&instance.allocated_children_bitset),
               sizeof(allocated_children_bitset));
  return instance;
}

void WaveletOctreeHeader::write(std::ostream& ostream) const {
  ostream.write(reinterpret_cast<const char*>(&min_cell_width),
                sizeof(min_cell_width));
  ostream.write(reinterpret_cast<const char*>(&min_log_odds),
                sizeof(min_log_odds));
  ostream.write(reinterpret_cast<const char*>(&max_log_odds),
                sizeof(max_log_odds));
  ostream.write(reinterpret_cast<const char*>(&tree_height),
                sizeof(tree_height));
  ostream.write(reinterpret_cast<const char*>(&root_node_scale_coefficient),
                sizeof(root_node_scale_coefficient));
}

WaveletOctreeHeader WaveletOctreeHeader::read(std::istream& istream) {
  WaveletOctreeHeader instance;
  istream.read(reinterpret_cast<char*>(&instance.min_cell_width),
               sizeof(min_cell_width));
  istream.read(reinterpret_cast<char*>(&instance.min_log_odds),
               sizeof(min_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.max_log_odds),
               sizeof(max_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.tree_height),
               sizeof(tree_height));
  istream.read(reinterpret_cast<char*>(&instance.root_node_scale_coefficient),
               sizeof(root_node_scale_coefficient));
  return instance;
}

void HashedWaveletOctreeBlockHeader::write(std::ostream& ostream) const {
  // TODO(victorr): Write root_node_offset using
  //                root_node_offset.write(ostream) for better portability
  ostream.write(reinterpret_cast<const char*>(&root_node_offset),
                sizeof(root_node_offset));
  ostream.write(reinterpret_cast<const char*>(&root_node_scale_coefficient),
                sizeof(root_node_scale_coefficient));
}

HashedWaveletOctreeBlockHeader HashedWaveletOctreeBlockHeader::read(
    std::istream& istream) {
  // TODO(victorr): Read root_node_offset using
  //                instance.root_node_offset = Index3D::read(istream) for
  //                better portability
  HashedWaveletOctreeBlockHeader instance;
  istream.read(reinterpret_cast<char*>(&instance.root_node_offset),
               sizeof(root_node_offset));
  istream.read(reinterpret_cast<char*>(&instance.root_node_scale_coefficient),
               sizeof(root_node_scale_coefficient));
  return instance;
}

void HashedWaveletOctreeHeader::write(std::ostream& ostream) const {
  ostream.write(reinterpret_cast<const char*>(&min_cell_width),
                sizeof(min_cell_width));
  ostream.write(reinterpret_cast<const char*>(&min_log_odds),
                sizeof(min_log_odds));
  ostream.write(reinterpret_cast<const char*>(&max_log_odds),
                sizeof(max_log_odds));
  ostream.write(reinterpret_cast<const char*>(&tree_height),
                sizeof(tree_height));
  ostream.write(reinterpret_cast<const char*>(&num_blocks), sizeof(num_blocks));
}

HashedWaveletOctreeHeader HashedWaveletOctreeHeader::read(
    std::istream& istream) {
  HashedWaveletOctreeHeader instance;
  istream.read(reinterpret_cast<char*>(&instance.min_cell_width),
               sizeof(min_cell_width));
  istream.read(reinterpret_cast<char*>(&instance.min_log_odds),
               sizeof(min_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.max_log_odds),
               sizeof(max_log_odds));
  istream.read(reinterpret_cast<char*>(&instance.tree_height),
               sizeof(tree_height));
  istream.read(reinterpret_cast<char*>(&instance.num_blocks),
               sizeof(num_blocks));
  return instance;
}

void StorageFormat::write(std::ostream& ostream) const {
  ostream.write(reinterpret_cast<const char*>(&id_), sizeof(id_));
}

StorageFormat StorageFormat::read(std::istream& istream) {
  StorageFormat instance;
  istream.read(reinterpret_cast<char*>(&instance.id_), sizeof(id_));
  return instance;
}

StorageFormat StorageFormat::peek(std::istream& istream) {
  const auto pos = istream.tellg();
  auto instance = read(istream);
  istream.seekg(pos);
  return instance;
}
}  // namespace wavemap::io::streamable

#endif  // WAVEMAP_IO_IMPL_STREAMABLE_TYPES_IMPL_H_
