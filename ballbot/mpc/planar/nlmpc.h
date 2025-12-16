#pragma once

#include <memory>

#include <numbers>

#include "ballbot/plant/planar/input.h"
#include "ballbot/plant/planar/state.h"

#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solver_base.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake::ballbot::planar {

using planning::trajectory_optimization::DirectCollocation;
using solvers::VectorXDecisionVariable, solvers::SolverOptions,
    solvers::SolverBase, solvers::Binding, solvers::BoundingBoxConstraint;
using systems::InputPort, systems::InputPortIndex, systems::OutputPort,
    systems::OutputPortIndex, systems::System, systems::AbstractStateIndex,
    systems::Context, systems::State, systems::LeafSystem;
using trajectories::PiecewisePolynomial;

/*
 * Constraints for the Ballbot NLMPC.
 * @param u: magnitude of the max allowable torque
 * @param theta: maximum allowable lean angle
 * @param dphi: maximum angular velocity of the ball
 */
struct BallbotConstraints {
  double u{15.0};
  double theta{(100.0 * std::numbers::pi) / 180.0};
  double dphi{100.0};
};

/*
 * Non-Linear Model Predictive Control (NLMPC) for the Ballbot.
 * @param model: the ballbot model, in continuous time
 * @param Q: cost matrix for the state
 * @param R: cost matrix for the control input
 * @param N: number of control intervals
 * @param T_f: time horizon
 * @param constraints: constraints for the NLMPC
 */
template <typename T>
class BallbotNLMPC final : public LeafSystem<T> {
 public:
  BallbotNLMPC(std::shared_ptr<System<double>> model, MatrixX<double> const& Q,
               MatrixX<double> const& R, int N, double T_f,
               BallbotConstraints const& constraints = BallbotConstraints{});

  const InputPort<T>& get_state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  const InputPort<T>& get_goal_input_port() const {
    return this->get_input_port(goal_input_port_index_);
  }

  const OutputPort<T>& get_action_output_port() const {
    return this->get_output_port(action_output_port_index_);
  }

 private:
  std::shared_ptr<System<double>> model_;
  MatrixX<double> Q_;
  MatrixX<double> R_;
  int N_;
  double T_f_;
  double sample_time_;
  BallbotConstraints constraints_;

  std::unique_ptr<Context<double>> model_context_;

  InputPortIndex state_input_port_index_{0};
  InputPortIndex goal_input_port_index_{0};
  OutputPortIndex action_output_port_index_{0};
  AbstractStateIndex state_trajectory_index_{0};
  AbstractStateIndex input_trajectory_index_{0};
  AbstractStateIndex time_offset_index_{0};

  VectorXDecisionVariable goal_vars_{};

  std::unique_ptr<DirectCollocation> dircol_;
  std::unique_ptr<SolverBase> solver_;
  SolverOptions solver_options_{};

  std::optional<Binding<BoundingBoxConstraint>> initial_state_constraint_;
  std::optional<Binding<BoundingBoxConstraint>> goal_state_constraint_;
  std::optional<std::vector<Binding<BoundingBoxConstraint>>> force_constraints_;
  std::optional<std::vector<Binding<BoundingBoxConstraint>>> theta_constraints_;
  std::optional<std::vector<Binding<BoundingBoxConstraint>>> dphi_constraints_;

  void DoCalcAction_(const Context<T>& context, BallbotInput<T>* output) const;

  void UpdateAndSolve_(const Context<T>& context, State<T>* state) const;

  void SetupTrajectoryOptimization_();
};
}  // namespace drake::ballbot::planar
