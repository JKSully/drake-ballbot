#include <exception>
#include <memory>

#include <Eigen/Core>
#include <fmt/ostream.h>

#include "ballbot/mpc/nlmpc.h"
#include "ballbot/plant/plant.h"

#include "drake/common/eigen_types.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/query_object.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log.h"
#include "drake/systems/primitives/vector_log_sink.h"

namespace drake::ballbot {
int do_main() {
  systems::DiagramBuilder<double> builder;

  auto ballbot = builder.AddSystem<BallbotPlant>();

  ballbot->set_name("ballbot");

  Matrix4<double> Q;
  Q(0, 0) = 1.0;
  Q(2, 2) = 1'000.0;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 1.0;

  auto controller = builder.AddSystem<BallbotNLMPC>(
      std::make_shared<BallbotPlant<double>>(), Q, R, 21, 10.0);
  std::unique_ptr<systems::Context<double>> controller_context =
      controller->CreateDefaultContext();

  builder.Connect(controller->get_action_output_port(),
                  ballbot->get_action_input_port());
  builder.Connect(ballbot->get_state_output_port(),
                  controller->get_state_input_port());

  auto state_logger =
      systems::LogVectorOutput(ballbot->get_state_output_port(), &builder);
  state_logger->set_name("state_logger");
  auto action_logger =
      systems::LogVectorOutput(controller->get_action_output_port(), &builder);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();

  diagram->ForcedPublish(*context);

  auto sim = std::make_unique<systems::Simulator<double>>(*diagram);
  systems::Context<double>& sim_context = sim->get_mutable_context();
  sim->set_target_realtime_rate(1.0);
  sim->Initialize();

  Vector4<double> const initial_state = Vector4<double>::Zero();
  Vector4<double> goal_state = Vector4<double>::Zero();
  goal_state(0) = 1.0;

  systems::Context<double>& plant_context =
      ballbot->GetMyMutableContextFromRoot(&sim_context);
  plant_context.SetContinuousState(initial_state);

  controller_context->get_time();

  controller->get_goal_input_port().FixValue(
      &controller->GetMyMutableContextFromRoot(&sim_context), goal_state);

  sim->AdvanceTo(sim->get_context().get_time() + 10.);

  systems::VectorLog<double> const& state_log =
      state_logger->FindLog(sim_context);
  systems::VectorLog<double> const& action_log =
      action_logger->FindLog(sim_context);

  auto times = state_log.sample_times();
  auto states = state_log.data();
  auto actions = action_log.data();

  return 0;
}
}  // namespace drake::ballbot

int main() {
  static_cast<void>(drake::ballbot::do_main());
  return 0;
}
