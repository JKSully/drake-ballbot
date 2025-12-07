#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "ballbot/plant/plant.h"

#include "drake/common/eigen_types.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {

TEST(TrajectoryOptimizationTest, DirectCollocation) {
  // Create a plant
  auto plant = std::make_unique<drake::ballbot::BallbotPlant<double>>();
  auto context = plant->CreateDefaultContext();

  // Create a trajectory optimization problem
  constexpr int N = 21;
  constexpr double T_F = 10.0;
  constexpr double TIME_STEP = T_F / (N - 1);
  auto traj_opt =
      std::make_unique<planning::trajectory_optimization::DirectCollocation>(
          plant.get(), *context, N, TIME_STEP, TIME_STEP);
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

  Matrix4<double> q;
  q(0, 0) = 1.0;
  q(2, 2) = 1'000.0;

  Eigen::Matrix<double, 1, 1> r;
  r(0, 0) = 1.0;

  traj_opt->AddRunningCost(state_error.transpose() * q * state_error +
                           input_error.transpose() * r * input_error);

  auto solver = solvers::SnoptSolver();
  auto result = solver.Solve(prog);

  GTEST_ASSERT_TRUE(result.is_success());
}
}  // namespace drake
