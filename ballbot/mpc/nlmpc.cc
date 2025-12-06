#include "ballbot/mpc/nlmpc.h"

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <fmt/core.h>

#include "ballbot/plant/input.h"
#include "ballbot/plant/state.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/common/value.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/state.h"

namespace drake {

using trajectories::PiecewisePolynomial;
template <typename T>
BallbotNLMPC<T>::BallbotNLMPC(std::shared_ptr<systems::System<double>> model,
                              MatrixX<double> const& Q,
                              MatrixX<double> const& R, int N, double T_f)
    : model_(std::move(model)),
      Q_(Q),
      R_(R),
      N_(N),
      T_f_(T_f),
      sample_time_(T_f / (N - 1)),
      model_context_(model_->CreateDefaultContext()) {
  DRAKE_DEMAND(model_ != nullptr);
  DRAKE_DEMAND(N_ > 0);
  DRAKE_DEMAND(T_f_ > 0.);

  int const num_states = model_->get_output_port(0).size();
  int const num_inputs = model_->get_input_port(0).size();

  state_input_port_index_ =
      this->DeclareVectorInputPort("state", BallbotState<T>()).get_index();
  goal_input_port_index_ =
      this->DeclareVectorInputPort("goal", BallbotState<T>()).get_index();
  action_output_port_index_ =
      this->DeclareVectorOutputPort("action", BallbotInput<T>(),
                                    &BallbotNLMPC<T>::DoCalcAction_)
          .get_index();

  DRAKE_DEMAND(num_states == Q_.rows() && num_states == Q_.cols());
  DRAKE_DEMAND(num_inputs == R_.rows() && num_inputs == R_.cols());

  Eigen::LLT<Eigen::MatrixXd> R_chol(R_);
  DRAKE_ASSERT(R_chol.info() == Eigen::Success);

  VectorX<double> breaks(2);
  breaks << 0., 1.;

  auto const state_samples = MatrixX<double>::Zero(num_states, 2);
  auto const input_samples = MatrixX<double>::Zero(num_inputs, 2);

  state_trajectory_index_ =
      this->DeclareAbstractState(Value<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(breaks, state_samples)));
  input_trajectory_index_ =
      this->DeclareAbstractState(Value<PiecewisePolynomial<double>>(
          PiecewisePolynomial<double>::FirstOrderHold(breaks, input_samples)));
  time_offset_index_ = this->DeclareAbstractState(Value<double>(0.));

  solver_ = std::make_unique<solvers::IpoptSolver>();

  SetupTrajectoryOptimization_();

  this->DeclarePeriodicUnrestrictedUpdateEvent(
      T_f_, 0., &BallbotNLMPC<T>::UpdateAndSolve_);
}

template <typename T>
void BallbotNLMPC<T>::UpdateAndSolve_(const systems::Context<T>& context,
                                      systems::State<T>* state) const {
  VectorX<T> const& initial_state =
      this->get_input_port(state_input_port_index_).Eval(context);
  VectorX<T> const& goal_state =
      this->get_input_port(goal_input_port_index_).Eval(context);

  if (initial_state_constraint_ != std::nullopt) {
    initial_state_constraint_->evaluator()->set_bounds(initial_state,
                                                       initial_state);
  }
  if (goal_state_constraint_ != std::nullopt) {
    goal_state_constraint_->evaluator()->set_bounds(goal_state, goal_state);
  }

  T const sim_time = context.get_time();

  if (sim_time != 0.) {
    // TOOD: Need to clone these two
    auto const& initial_state_trajectory =
        context.template get_abstract_state<PiecewisePolynomial<double>>(
            state_trajectory_index_);

    auto const& initial_input_trajectory =
        context.template get_abstract_state<PiecewisePolynomial<double>>(
            input_trajectory_index_);

    dircol_->SetInitialTrajectory(initial_input_trajectory,
                                  initial_state_trajectory);
  } else {
    VectorX<double> breaks(2);
    breaks << 0., T_f_;

    int const state_size = initial_state.size();
    MatrixX<double> samples(state_size, 2);
    samples << initial_state, initial_state;

    auto const state_trajectory_guess =
        PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
    dircol_->SetInitialTrajectory(PiecewisePolynomial<double>(),
                                  state_trajectory_guess);
  }

  solvers::MathematicalProgramResult const result =
      solver_->Solve(dircol_->prog());

  if (!result.is_success()) {
    auto infeasible = result.GetInfeasibleConstraints(dircol_->prog());
    for (auto const& constraint : infeasible) {
      std::cout << constraint << '\n';
    }
  }

  auto const input_trajectory = dircol_->ReconstructInputTrajectory(result);
  auto const state_trajectory = dircol_->ReconstructStateTrajectory(result);

  state->get_abstract_state();
}

template <typename T>
void BallbotNLMPC<T>::DoCalcAction_(const systems::Context<T>& context,
                                    BallbotInput<T>* output) const {
  auto const sim_time = context.get_time();
  auto const time_offset =
      context.template get_abstract_state<T>(time_offset_index_);
  auto const& input_trajectory =
      context.template get_abstract_state<PiecewisePolynomial<T>>(
          input_trajectory_index_);

  T const time_delta = sim_time - time_offset;
  auto V = input_trajectory.scalarValue(time_delta);
  output->set_tau(V);
}

template <typename T>
void BallbotNLMPC<T>::SetupTrajectoryOptimization_() {
  dircol_ =
      std::make_unique<planning::trajectory_optimization::DirectCollocation>(
          model_.get(), *model_context_, N_, sample_time_, sample_time_);
  auto& prog = dircol_->prog();

  dircol_->AddEqualTimeIntervalsConstraints();

  auto const dummy_state =
      VectorX<double>::Zero(model_->get_output_port().size());
  initial_state_constraint_ = prog.AddBoundingBoxConstraint(
      dummy_state, dummy_state, dircol_->initial_state());

  goal_state_constraint_ = prog.AddBoundingBoxConstraint(
      dummy_state, dummy_state, dircol_->final_state());

  auto const state_error = dircol_->state() - dummy_state;
  auto const input_error = dircol_->input();

  dircol_->AddRunningCost(state_error.transpose() * Q_ * state_error +
                          input_error.transpose() * R_ * input_error);
}

// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class BallbotNLMPC);
template class BallbotNLMPC<double>;
}  // namespace drake
