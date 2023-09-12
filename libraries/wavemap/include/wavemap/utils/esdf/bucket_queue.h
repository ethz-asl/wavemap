#ifndef WAVEMAP_UTILS_ESDF_BUCKET_QUEUE_H_
#define WAVEMAP_UTILS_ESDF_BUCKET_QUEUE_H_

#include <cmath>
#include <queue>
#include <vector>

#include <glog/logging.h>

namespace wavemap {
/**
 * Bucketed priority queue, mostly following L. Yatziv et al in
 * O(N) Implementation of the Fast Marching Algorithm, though skipping the
 * circular aspect (don't care about a bit more memory used for this).
 * Adapted from voxblox, see:
 * https://github.com/ethz-asl/voxblox/blob/master/voxblox/include/voxblox/utils/bucket_queue.h
 */
template <typename T>
class BucketQueue {
 public:
  explicit BucketQueue(int num_buckets, double max_val)
      : num_buckets_(num_buckets),
        max_val_(max_val),
        last_bucket_index_(0),
        num_elements_(0) {
    buckets_.resize(num_buckets_);
  }

  /// WARNING: This clears the queue.
  void setNumBuckets(int num_buckets, double max_val) {
    max_val_ = max_val;
    num_buckets_ = num_buckets;
    buckets_.clear();
    buckets_.resize(num_buckets_);
    num_elements_ = 0;
  }

  void push(const T& key, double value) {
    CHECK_NE(num_buckets_, 0);
    if (value > max_val_) {
      value = max_val_;
    }
    int bucket_index =
        std::floor(std::abs(value) / max_val_ * (num_buckets_ - 1));
    if (bucket_index >= num_buckets_) {
      bucket_index = num_buckets_ - 1;
    }
    if (bucket_index < last_bucket_index_) {
      last_bucket_index_ = bucket_index;
    }
    buckets_[bucket_index].push(key);
    num_elements_++;
  }

  void pop() {
    if (empty()) {
      return;
    }
    while (last_bucket_index_ < num_buckets_ &&
           buckets_[last_bucket_index_].empty()) {
      last_bucket_index_++;
    }
    if (last_bucket_index_ < num_buckets_) {
      buckets_[last_bucket_index_].pop();
      num_elements_--;
    }
  }

  T front() {
    CHECK_NE(num_buckets_, 0);
    CHECK(!empty());
    while (last_bucket_index_ < num_buckets_ &&
           buckets_[last_bucket_index_].empty()) {
      last_bucket_index_++;
    }
    return buckets_[last_bucket_index_].front();
  }

  bool empty() { return num_elements_ == 0; }

  void clear() {
    buckets_.clear();
    buckets_.resize(num_buckets_);
    last_bucket_index_ = 0;
    num_elements_ = 0;
  }

 private:
  int num_buckets_;
  double max_val_;
  std::vector<std::queue<T>> buckets_;

  /// Speeds up retrievals
  int last_bucket_index_;
  /// This speeds up empty checks
  size_t num_elements_;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_ESDF_BUCKET_QUEUE_H_
