#pragma once

#include <Eigen/Core>

#include "ballbot/plant/params.h"
#include "ballbot/plant/state.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake::ballbot {
template <typename T>
class BallbotPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallbotPlant);

  BallbotPlant();

  template <typename U>
  explicit BallbotPlant(const BallbotPlant<U>&);

  ~BallbotPlant() final;

  const systems::InputPort<T>& get_action_input_port() const {
    return this->get_input_port(u_index_);
  }

  const systems::OutputPort<T>& get_state_output_port() const {
    return this->get_output_port(state_index_);
  }

  void SetEthBallbotParameters(BallbotParams<T>* parameters) const;

  Matrix2<T> MassMatrix(const systems::Context<T>& context) const;

  Vector2<T> Coriolis(const systems::Context<T>& context) const;

  Vector2<T> Gravity(const systems::Context<T>& context) const;

  Vector2<T> NonPotentialForce(const systems::Context<T>& context) const;

  const T get_tau(const systems::Context<T>& context) const {
    const systems::BasicVector<T>* u_vec = this->EvalVectorInput(context, 0);
    return u_vec ? u_vec->GetAtIndex(0) : 0.0;
  }

  static const BallbotState<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const BallbotState<T>&>(cstate.get_vector());
  }

  static const BallbotState<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static BallbotState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<BallbotState<T>&>(cstate->get_mutable_vector());
  }

  static BallbotState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const BallbotParams<T>& get_parameters(
      systems::Context<T> const& context) const {
    return this->template GetNumericParameter<BallbotParams>(context, 0);
  }

  BallbotParams<T>& get_mutable_parameters(systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<BallbotParams>(context, 0);
  }

 private:
  systems::InputPortIndex u_index_{0};
  systems::OutputPortIndex state_index_{0};

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
};
}  // namespace drake::ballbot
