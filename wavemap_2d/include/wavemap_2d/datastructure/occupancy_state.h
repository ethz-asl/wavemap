#ifndef WAVEMAP_2D_DATASTRUCTURE_OCCUPANCY_STATE_H_
#define WAVEMAP_2D_DATASTRUCTURE_OCCUPANCY_STATE_H_

namespace wavemap_2d {
class OccupancyState {
 public:
  static OccupancyState Unknown() { return {false, false}; }
  static OccupancyState Free() { return {true, false}; }
  static OccupancyState Occupied() { return {true, true}; }

  OccupancyState() : OccupancyState(Unknown()) {}

  void setUnknown() { *this = Unknown(); }
  void setFree() { *this = Free(); }
  void setOccupied() { *this = Occupied(); }

  bool isUnknown() const { return !observed_; }
  bool isObserved() const { return observed_; }
  bool isFree() const { return observed_ && !occupied_; }
  bool isOccupied() const { return observed_ && occupied_; }

  static OccupancyState fromValue(FloatingPoint cell_value) {
    if (std::abs(cell_value) < kEpsilon) {
      return Unknown();
    } else if (cell_value < 0.f) {
      return Free();
    } else {
      return Occupied();
    }
  }

 protected:
  OccupancyState(bool observed, bool occupied)
      : observed_(observed), occupied_(occupied) {}

  bool observed_;
  bool occupied_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_OCCUPANCY_STATE_H_
