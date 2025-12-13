#include "ballbot/plant/planar/params.h"

#include <string>
#include <vector>

#include "drake/common/never_destroyed.h"

namespace drake::ballbot::planar {

const int BallbotParamsIndicies::kNumCoordinates;
const int BallbotParamsIndicies::kMassK;
const int BallbotParamsIndicies::kMassA;
const int BallbotParamsIndicies::kMassW;
const int BallbotParamsIndicies::kRadiusK;
const int BallbotParamsIndicies::kRadiusW;
const int BallbotParamsIndicies::kRadiusA;
const int BallbotParamsIndicies::kL;
const int BallbotParamsIndicies::kThetaK;
const int BallbotParamsIndicies::kThetaW;
const int BallbotParamsIndicies::kThetaA;
const int BallbotParamsIndicies::kGravity;

const std::vector<std::string>& BallbotParamsIndicies::GetCoordinateNames() {
  static const never_destroyed<std::vector<std::string>> coordinates(
      std::vector<std::string>{"mass_k", "mass_a", "mass_w", "radius_k",
                               "radius_w", "radius_a", "l", "Theta_k",
                               "Theta_w", "Theta_a", "gravity"});
  return coordinates.access();
}
}  // namespace drake::ballbot::planar
