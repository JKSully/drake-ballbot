#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "ballbot/plant/plant.h"

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/snopt_solver.h"

template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
bool approxEq(T a, T b) {
  return std::abs(a - b) < std::numeric_limits<T>::epsilon();
}

namespace drake {
TEST(TrajectoryOptimizationTest, DirectCollocation) {
  // Create a plant
  auto plant = std::make_unique<drake::ballbot::BallbotPlant<double>>();
  auto context = plant->CreateDefaultContext();

  auto& params = plant->get_mutable_parameters(context.get());
  plant->SetEthBallbotParameters(&params);

  // Create a trajectory optimization problem
  constexpr int N = 21;
  constexpr double T_F = 10.0;
  constexpr double TIME_STEP = T_F / (N - 1);
  auto traj_opt =
      std::make_unique<planning::trajectory_optimization::DirectCollocation>(
          plant.get(), *context, N, TIME_STEP, TIME_STEP);
  auto& prog = traj_opt->prog();

  traj_opt->AddEqualTimeIntervalsConstraints();

  int const state_dim = plant->get_state_output_port().size();  // 4
  int const input_dim = plant->get_action_input_port().size();  // 1

  auto const initial_state = VectorX<double>::Zero(state_dim);
  auto final_state = VectorX<double>(state_dim);
  final_state(0, 0) = 1.0;

  prog.AddBoundingBoxConstraint(initial_state, initial_state,
                                traj_opt->initial_state());
  prog.AddBoundingBoxConstraint(final_state, final_state,
                                traj_opt->final_state());

  auto const state_error = traj_opt->state() - final_state;
  auto const input_error = traj_opt->input();

  MatrixX<double> q = Matrix4<double>::Zero(state_dim, state_dim);
  q(0, 0) = 1.0;
  q(2, 2) = 1'000.0;

  MatrixX<double> r = MatrixX<double>::Identity(input_dim, input_dim);

  traj_opt->AddRunningCost(state_error.transpose() * q * state_error +
                           input_error.transpose() * r * input_error);

  // Set initial guess
  if (::approxEq<double>(context->get_time(), 0.0)) {
    VectorX<double> breaks(N);
    for (int i = 0; i < N; ++i) {
      breaks(i) = i * TIME_STEP;
    }

    MatrixX<double> state_samples(state_dim, N);
    for (int i = 0; i < N; ++i) {
      double alpha = static_cast<double>(i) / (N - 1);
      state_samples.col(i) =
          initial_state * (1.0 - alpha) + final_state * alpha;
    }

    auto state_guess =
        trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            breaks, state_samples);

    MatrixX<double> input_samples = MatrixX<double>::Zero(input_dim, N);
    auto input_guess =
        trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            breaks, input_samples);

    traj_opt->SetInitialTrajectory(input_guess, state_guess);
  }

  auto solver = solvers::SnoptSolver();
  auto result = solver.Solve(prog);

  ASSERT_TRUE(result.is_success());
}
}  // namespace drake
