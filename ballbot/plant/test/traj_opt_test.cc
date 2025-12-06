#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "ballbot/plant/plant.h"

#include "drake/common/eigen_types.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {

GTEST_TEST(TrajectoryOptimizationTest, DirectCollocation) {
  // Create a plant
  auto plant = std::make_unique<drake::ballbot::BallbotPlant<double>>();
  auto context = plant->CreateDefaultContext();

  GTEST_LOG_(INFO) << fmt::format("Plant input port size: {}",
                                  plant->get_input_port().size());
  GTEST_LOG_(INFO) << fmt::format("Plant output port size: {}",
                                  plant->get_output_port().size());

  // Create a trajectory optimization problem
  constexpr int N = 21;
  constexpr double t_f = 10.0;
  constexpr double time_step = t_f / (N - 1);
  auto traj_opt =
      std::make_unique<planning::trajectory_optimization::DirectCollocation>(
          plant.get(), *context, N, time_step, time_step);
  auto& prog = traj_opt->prog();

  traj_opt->AddEqualTimeIntervalsConstraints();

  auto const initial_state =
      VectorX<double>::Zero(plant->get_output_port().size());
  auto final_state = VectorX<double>(plant->get_output_port().size());
  final_state(0, 0) = 1.0;

  prog.AddBoundingBoxConstraint(initial_state, initial_state,
                                traj_opt->initial_state());
  prog.AddBoundingBoxConstraint(final_state, final_state,
                                traj_opt->final_state());

  auto const state_error = traj_opt->state() - final_state;
  auto const input_error = traj_opt->input();

  Matrix4<double> Q;
  Q(0, 0) = 1.0;
  Q(2, 2) = 1'000.0;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 1.0;

  traj_opt->AddRunningCost(state_error.transpose() * Q * state_error +
                           input_error.transpose() * R * input_error);

  auto solver = solvers::SnoptSolver();
  auto result = solver.Solve(prog);

  GTEST_ASSERT_TRUE(result.is_success());
}
}  // namespace drake
