#include <memory>

#include <gtest/gtest.h>

#include "ballbot/plant/plant.h"

namespace drake {
TEST(PlantTest, Constructor) {
  auto plant = std::make_unique<ballbot::BallbotPlant<double>>();

  EXPECT_EQ(plant->get_action_input_port().size(), 1);

  EXPECT_EQ(plant->get_state_output_port().size(), 4);
}
}  // namespace drake
