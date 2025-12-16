#pragma once

#include <memory>

#include <numbers>

#include "ballbot/plant/planar/input.h"

#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solver_base.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake::ballbot::planar {

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

template <typename T>
class BallbotNLMPC final : public systems::LeafSystem<T> {
 public:
  BallbotNLMPC(std::shared_ptr<systems::System<double>> model,
               MatrixX<double> const& Q, MatrixX<double> const& R, int N,
               double T_f,
               BallbotConstraints const& constraints = BallbotConstraints{});

  const systems::InputPort<T>& get_state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  const systems::InputPort<T>& get_goal_input_port() const {
    return this->get_input_port(goal_input_port_index_);
  }

  const systems::OutputPort<T>& get_action_output_port() const {
    return this->get_output_port(action_output_port_index_);
  }

 private:
  std::shared_ptr<systems::System<double>> model_;
  MatrixX<double> Q_;
  MatrixX<double> R_;
  int N_;
  double T_f_;
  double sample_time_;
  BallbotConstraints constraints_;

  std::unique_ptr<systems::Context<double>> model_context_;

  systems::InputPortIndex state_input_port_index_{0};
  systems::InputPortIndex goal_input_port_index_{0};
  systems::OutputPortIndex action_output_port_index_{0};
  systems::AbstractStateIndex state_trajectory_index_{0};
  systems::AbstractStateIndex input_trajectory_index_{0};
  systems::AbstractStateIndex time_offset_index_{0};

  solvers::VectorXDecisionVariable goal_vars_;

  std::unique_ptr<planning::trajectory_optimization::DirectCollocation> dircol_;
  std::unique_ptr<solvers::SolverBase> solver_;
  solvers::SolverOptions solver_options_{};

  std::optional<solvers::Binding<solvers::BoundingBoxConstraint>>
      initial_state_constraint_;
  std::optional<solvers::Binding<solvers::BoundingBoxConstraint>>
      goal_state_constraint_;
  std::optional<std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>>
      force_constraints_;
  std::optional<std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>>
      theta_constraints_;
  std::optional<std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>>
      dphi_constraints_;

  void DoCalcAction_(const systems::Context<T>& context,
                     BallbotInput<T>* output) const;

  void UpdateAndSolve_(const systems::Context<T>& context,
                       systems::State<T>* state) const;

  void SetupTrajectoryOptimization_();
};
}  // namespace drake::ballbot::planar
