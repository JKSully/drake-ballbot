#include <memory>

#include <gtest/gtest.h>

#include "ballbot/plant/planar/plant.h"

#include "drake/planning/trajectory_optimization/direct_transcription.h"
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

  static constexpr double TIME_PERIOD = 0.1;
  static constexpr double TIME_HORIZON = 10.0;
};

TEST_F(TestLinearization, Linearize) {
  auto context = plant_->CreateDefaultContext();
  plant_->get_action_input_port().FixValue(context.get(), u0_);
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);

  std::unique_ptr<systems::LinearSystem<double>> linear_system;

  EXPECT_NO_THROW({ linear_system = Linearize(*plant_, *context); });
}

TEST_F(TestLinearization, DirectTranscription) {
  using planning::trajectory_optimization::DirectTranscription,
      planning::trajectory_optimization::TimeStep;

  auto context = plant_->CreateDefaultContext();
  plant_->get_action_input_port().FixValue(context.get(), u0_);
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);

  std::unique_ptr<systems::LinearSystem<double>> linear_system =
      Linearize(*plant_, *context);
  std::unique_ptr<systems::Context<double>> linear_context =
      linear_system->CreateDefaultContext();

  int const num_sample_times =
      static_cast<int>((TIME_HORIZON / TIME_PERIOD) + 0.5);

  EXPECT_NO_THROW({
    DirectTranscription dirtran(linear_system.get(), *linear_context,
                                num_sample_times, TimeStep(TIME_PERIOD));
  });
}
}  // namespace drake::ballbot::planar
