#include <memory>

#include <gtest/gtest.h>

#include "ballbot/mpc/planar/nlmpc.h"
#include "ballbot/plant/planar/plant.h"

namespace drake::ballbot::planar {
class TestBallbotMpc : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<BallbotPlant<double>>();
    x0_ = VectorX<double>::Zero(plant_->get_state_output_port().size());
    u0_ = VectorX<double>::Zero(plant_->get_action_input_port().size());
  }

  const double kTimeStep = 0.1;     // discrete time step.
  const double kTimeHorizon = 10.;  // Time horizon.
  const int kNumTimeSteps = static_cast<int>((kTimeHorizon / kTimeStep) + 0.5);

  std::unique_ptr<BallbotPlant<double>> plant_;
  VectorX<double> x0_;
  VectorX<double> u0_;
};

TEST_F(TestBallbotMpc, Constructor) {
  auto context = plant_->CreateDefaultContext();
  plant_->get_action_input_port().FixValue(context.get(), u0_);
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);

  Matrix4<double> Q = Matrix4<double>::Zero();
  Q(0, 0) = 1.0;
  Q(2, 2) = 1'000.0;

  MatrixX<double> R = MatrixX<double>::Identity(1, 1);
  R(0, 0) = 1.0;

  EXPECT_NO_THROW(BallbotNLMPC<double> mpc(std::move(plant_),
                                           std::move(context), Q, R,
                                           kNumTimeSteps, kTimeHorizon););
}

}  // namespace drake::ballbot::planar
