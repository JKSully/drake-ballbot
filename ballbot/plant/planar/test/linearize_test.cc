#include <memory>

#include <gtest/gtest.h>

#include "ballbot/plant/planar/plant.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake::ballbot::planar {
class TestLinearization : public ::testing::Test {
 protected:
  void SetUp() {
    plant_ = std::make_unique<BallbotPlant<double>>();
    x0_ = VectorX<double>::Zero(plant_->get_state_output_port().size());
    u0_ = VectorX<double>::Zero(plant_->get_action_input_port().size());
  }

  std::unique_ptr<BallbotPlant<double>> plant_;
  VectorX<double> x0_;
  VectorX<double> u0_;
};

TEST_F(TestLinearization, Linearize) {
  auto context = plant_->CreateDefaultContext();
  plant_->get_action_input_port().FixValue(context.get(), u0_);
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);

  std::unique_ptr<systems::LinearSystem<double>> linear_system;

  EXPECT_NO_THROW({ linear_system = Linearize(*plant_, *context); });
}
}  // namespace drake::ballbot::planar
