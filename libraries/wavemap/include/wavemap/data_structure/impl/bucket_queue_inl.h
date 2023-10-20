#ifndef WAVEMAP_DATA_STRUCTURE_IMPL_BUCKET_QUEUE_INL_H_
#define WAVEMAP_DATA_STRUCTURE_IMPL_BUCKET_QUEUE_INL_H_

#include <glog/logging.h>

namespace wavemap {
template <typename ValueT>
BucketQueue<ValueT>::BucketQueue(int num_buckets, FloatingPoint max_key)
    : num_buckets_(num_buckets),
      max_key_(max_key),
      last_bucket_index_(0),
      num_elements_(0) {
  buckets_.resize(num_buckets_);
}

template <typename ValueT>
void BucketQueue<ValueT>::setNumBuckets(int num_buckets,
                                        FloatingPoint max_key) {
  max_key_ = max_key;
  num_buckets_ = num_buckets;
  buckets_.clear();
  buckets_.resize(num_buckets_);
  num_elements_ = 0;
}

template <typename ValueT>
void BucketQueue<ValueT>::clear() {
  buckets_.clear();
  buckets_.resize(num_buckets_);
  last_bucket_index_ = 0;
  num_elements_ = 0;
}

template <typename ValueT>
void BucketQueue<ValueT>::push(FloatingPoint key, const ValueT& value) {
  CHECK_NE(num_buckets_, 0);
  if (key > max_key_) {
    key = max_key_;
  }
  int bucket_index = std::floor(std::abs(key) / max_key_ * (num_buckets_ - 1));
  if (bucket_index >= num_buckets_) {
    bucket_index = num_buckets_ - 1;
  }
  if (bucket_index < last_bucket_index_) {
    last_bucket_index_ = bucket_index;
  }
  buckets_[bucket_index].push(value);
  num_elements_++;
}

template <typename ValueT>
void BucketQueue<ValueT>::pop() {
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

template <typename ValueT>
ValueT BucketQueue<ValueT>::front() {
  CHECK_NE(num_buckets_, 0);
  CHECK(!empty());
  while (last_bucket_index_ < num_buckets_ &&
         buckets_[last_bucket_index_].empty()) {
    last_bucket_index_++;
  }
  return buckets_[last_bucket_index_].front();
}
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_IMPL_BUCKET_QUEUE_INL_H_
