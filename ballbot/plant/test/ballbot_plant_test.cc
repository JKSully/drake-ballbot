#include <memory>

#include <gtest/gtest.h>

#include "ballbot/plant/plant.h"

namespace drake {
TEST(PlantTest, Constructor) {
  auto plant = std::make_unique<ballbot::BallbotPlant<double>>();

  EXPECT_EQ(plant->get_action_input_port().size(), 1);

  EXPECT_EQ(plant->get_state_output_port().size(), 4);
}

TEST(PlantTest, SetEthBallbotParameters) {
  auto plant = std::make_unique<ballbot::BallbotPlant<double>>();

  auto context = plant->CreateDefaultContext();
  auto& params = plant->get_mutable_parameters(context.get());

  double const length_old = params.l();

  plant->SetEthBallbotParameters(&params);

  double const length_new = params.l();

  EXPECT_NE(length_new, length_old);
}
}  // namespace drake
