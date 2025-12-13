#include "ballbot/plant/planar/input.h"

#include "drake/common/never_destroyed.h"

namespace drake::ballbot::planar {
const int BallbotInputIndicies::kNumCoordinates;
const int BallbotInputIndicies::kTau;

const std::vector<std::string>& BallbotInputIndicies::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{"tau"});
  return coordinates.access();
}

}  // namespace drake::ballbot::planar
