#include "ballbot/plant/planar/state.h"

#include "drake/common/never_destroyed.h"

namespace drake::ballbot::planar {
const int BallbotStateIndicies::kNumCoordinates;
const int BallbotStateIndicies::kBallAngle;
const int BallbotStateIndicies::kBallVelocity;
const int BallbotStateIndicies::kLeanAngle;
const int BallbotStateIndicies::kLeanVelocity;

const std::vector<std::string>& BallbotStateIndicies::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{"ball_angle", "ball_velocity", "lean_angle",
                               "lean_velocity"});
  return coordinates.access();
}
}  // namespace drake::ballbot::planar
