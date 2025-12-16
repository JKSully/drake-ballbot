#include "ballbot/mpc/planar/nlmpc.h"

#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <fmt/core.h>

#include "ballbot/plant/planar/input.h"
#include "ballbot/plant/planar/state.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/value.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/state.h"

namespace drake::ballbot::planar {

using trajectories::PiecewisePolynomial;
template <typename T>
BallbotNLMPC<T>::BallbotNLMPC(std::shared_ptr<systems::System<double>> model,
                              MatrixX<double> const& Q,
                              MatrixX<double> const& R, int N, double T_f,
                              BallbotConstraints const& constraints)
    : model_(std::move(model)),
      Q_(Q),
      R_(R),
      N_(N),
      T_f_(T_f),
      sample_time_(T_f / (N - 1)),
      constraints_(constraints),
      model_context_(model_->CreateDefaultContext()) {
  DRAKE_DEMAND(model_ != nullptr);
  // Require at least two collocation points (N must be >= 2) so that the
  // denominator (N - 1) used to compute `sample_time_` is non-zero.
  DRAKE_DEMAND(N_ >= 2);
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

  Eigen::LLT<Eigen::MatrixXd> r_cholesky(R_);
  DRAKE_THROW_UNLESS(r_cholesky.info() == Eigen::Success);

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
  auto const solver_id = solver_->solver_id();

  solver_options_.SetOption(solver_id, "max_iter", 100);
  solver_options_.SetOption(solver_id, "tol", 1e-4);

  SetupTrajectoryOptimization_();

  dircol_->prog().SetSolverOptions(solver_options_);

  this->DeclarePeriodicUnrestrictedUpdateEvent(
      sample_time_, 0., &BallbotNLMPC<T>::UpdateAndSolve_);
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
    VectorX<double> breaks(N_);
    for (int i = 0; i < N_; ++i) {
      breaks(i) = i * sample_time_;
    }

    MatrixX<double> state_samples(get_state_input_port().size(), N_);
    for (int i = 0; i < N_; ++i) {
      double alpha = static_cast<double>(i) / (N_ - 1);
      state_samples.col(i) =
          (initial_state * (1.0 - alpha)) + (goal_state * alpha);
    }

    auto state_guess =
        trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            breaks, state_samples);

    MatrixX<double> input_samples =
        MatrixX<double>::Zero(get_action_output_port().size(), N_);
    auto input_guess =
        trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            breaks, input_samples);

    dircol_->SetInitialTrajectory(input_guess, state_guess);
  }

  solvers::MathematicalProgramResult const result =
      solver_->Solve(dircol_->prog());

  if (!result.is_success()) {
    auto infeasible = result.GetInfeasibleConstraints(dircol_->prog());
    for (auto const& constraint : infeasible) {
      fmt::println("Infeasible constraint: {}", constraint);
    }
  }

  auto const input_trajectory = dircol_->ReconstructInputTrajectory(result);
  auto const state_trajectory = dircol_->ReconstructStateTrajectory(result);

  state->template get_mutable_abstract_state<PiecewisePolynomial<double>>(
      input_trajectory_index_) = input_trajectory;
  state->template get_mutable_abstract_state<PiecewisePolynomial<double>>(
      state_trajectory_index_) = state_trajectory;
  state->template get_mutable_abstract_state<double>(time_offset_index_) =
      sim_time;
}

template <typename T>
void BallbotNLMPC<T>::DoCalcAction_(const systems::Context<T>& context,
                                    BallbotInput<T>* output) const {
  auto const sim_time = context.get_time();
  auto const time_offset =
      context.template get_abstract_state<double>(time_offset_index_);
  auto const& input_trajectory =
      context.template get_abstract_state<PiecewisePolynomial<T>>(
          input_trajectory_index_);

  double const time_delta = sim_time - time_offset;
  double const knot = input_trajectory.scalarValue(time_delta);
  output->set_tau(knot);
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

  goal_vars_ =
      prog.NewContinuousVariables(model_->get_output_port().size(), "goal");

  goal_state_constraint_ =
      prog.AddBoundingBoxConstraint(dummy_state, dummy_state, goal_vars_);

  auto const u_constraint = VectorX<double>::Constant(
      model_->get_input_port().size(), constraints_.u);
  force_constraints_ = dircol_->AddConstraintToAllKnotPoints(
      std::make_shared<solvers::BoundingBoxConstraint>(-u_constraint,
                                                       u_constraint),
      dircol_->input());

  auto const theta_constraint =
      VectorX<double>::Constant(1, constraints_.theta);
  theta_constraints_ = dircol_->AddConstraintToAllKnotPoints(
      std::make_shared<solvers::BoundingBoxConstraint>(-theta_constraint,
                                                       theta_constraint),
      dircol_->state().segment(BallbotStateIndicies::kLeanAngle, 1));

  auto const dphi_constraint = VectorX<double>::Constant(1, constraints_.dphi);
  dphi_constraints_ = dircol_->AddConstraintToAllKnotPoints(
      std::make_shared<solvers::BoundingBoxConstraint>(-dphi_constraint,
                                                       dphi_constraint),
      dircol_->state().segment(BallbotStateIndicies::kWheelVelocity, 1));

  auto const state_error = dircol_->state() - goal_vars_;
  auto const input_error = dircol_->input();

  dircol_->AddRunningCost(state_error.transpose() * Q_ * state_error +
                          input_error.transpose() * R_ * input_error);

  auto const final_state_error = dircol_->final_state() - goal_vars_;
  dircol_->AddFinalCost(final_state_error.transpose() * Q_ * final_state_error);
}

// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//     class BallbotNLMPC);
template class BallbotNLMPC<double>;
}  // namespace drake::ballbot::planar
