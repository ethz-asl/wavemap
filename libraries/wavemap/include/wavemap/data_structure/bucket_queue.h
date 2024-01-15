#ifndef WAVEMAP_DATA_STRUCTURE_BUCKET_QUEUE_H_
#define WAVEMAP_DATA_STRUCTURE_BUCKET_QUEUE_H_

#include <cmath>
#include <queue>
#include <vector>

#include <wavemap/common.h>

namespace wavemap {
/**
 * Bucketed priority queue, mostly following L. Yatziv et al in
 * O(N) Implementation of the Fast Marching Algorithm, though skipping the
 * circular aspect (don't care about a bit more memory used for this).
 * Based on voxblox's bucket queue, see:
 * https://github.com/ethz-asl/voxblox/blob/master/voxblox/include/voxblox/utils/bucket_queue.h
 */
template <typename ValueT>
class BucketQueue {
 public:
  explicit BucketQueue(int num_buckets, FloatingPoint max_key);

  /// WARNING: Calling this method clears the queue.
  void setNumBuckets(int num_buckets, FloatingPoint max_key);

  size_t size() const { return num_elements_; }
  bool empty() { return num_elements_ == 0; }
  void clear();

  void push(FloatingPoint key, const ValueT& value);
  void pop();
  ValueT front();

  int computeBucketIndex(FloatingPoint key) const {
    int bucket_index =
        std::floor(std::abs(key) / max_key_ * (num_buckets_ - 1));
    if (num_buckets_ <= bucket_index) {
      bucket_index = num_buckets_ - 1;
    }
    return bucket_index;
  }

 private:
  int num_buckets_;
  FloatingPoint max_key_;
  std::vector<std::queue<ValueT>> buckets_;

  /// Used to speed up retrievals
  int last_bucket_index_;

  /// Used to speed up empty checks
  size_t num_elements_;
};
}  // namespace wavemap

#include "wavemap/data_structure/impl/bucket_queue_inl.h"

#endif  // WAVEMAP_DATA_STRUCTURE_BUCKET_QUEUE_H_
