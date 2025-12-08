#include "ballbot/plant/plant.h"

#include <cmath>

#include "ballbot/plant/input.h"
#include "ballbot/plant/params.h"
#include "ballbot/plant/state.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/linear_solve.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_type_tag.h"

using std::cos;
using std::pow;
using std::sin;

namespace drake::ballbot {
template <typename T>
BallbotPlant<T>::BallbotPlant()
    : systems::LeafSystem<T>(systems::SystemTypeTag<BallbotPlant>{}) {
  this->DeclareNumericParameter(BallbotParams<T>());
  u_index_ = this->DeclareVectorInputPort("u", BallbotInput<T>()).get_index();

  auto state_index = this->DeclareContinuousState(BallbotState<T>(), 2, 2, 0);
  state_index_ = this->DeclareStateOutputPort("state", state_index).get_index();
}

template <typename T>
template <typename U>
BallbotPlant<T>::BallbotPlant(const BallbotPlant<U>&) : BallbotPlant<T>() {}

template <typename T>
BallbotPlant<T>::~BallbotPlant() = default;

template <typename T>
void BallbotPlant<T>::SetEthBallbotParameters(
    BallbotParams<T>* parameters) const {
  DRAKE_DEMAND(parameters != nullptr);
  parameters->set_mass_k(2.29);
  parameters->set_mass_a(9.2);
  parameters->set_mass_w(3.0);
  parameters->set_radius_k(0.125);
  parameters->set_radius_w(0.06);
  parameters->set_radius_a(0.1);
  parameters->set_l(0.339);
  parameters->set_theta_k(0.0239);
  parameters->set_theta_w(0.00236);
  parameters->set_theta_a(4.76);
  parameters->set_gravity(9.81);
}

template <typename T>
Matrix2<T> BallbotPlant<T>::MassMatrix(
    const systems::Context<T>& context) const {
  BallbotParams<T> const& params = this->get_parameters(context);
  BallbotState<T> const& state = this->get_state(context);
  T const mass_total = params.mass_a() + params.mass_k() + params.mass_w();
  T const radius_total = params.radius_k() + params.radius_w();
  T const gam =
      (params.l() * params.mass_a()) + (radius_total * params.mass_w());

  T const m11 =
      (mass_total * params.radius_k() * params.radius_k()) + params.theta_k() +
      (pow(params.radius_k() / params.radius_w(), 2) * params.theta_w());
  T const m12 = (-(params.radius_k() / pow(params.radius_w(), 2)) *
                 radius_total * params.theta_w()) +
                (gam * params.radius_k() * cos(state.lean_angle()));
  T const& m21 = m12;
  T const m22 = (pow(radius_total / params.radius_w(), 2) * params.theta_w()) +
                params.theta_a() + (params.mass_a() * params.l() * params.l()) +
                (params.mass_w() * radius_total * radius_total);

  Matrix2<T> mass_matrix;
  mass_matrix << m11, m12, m21, m22;

  return mass_matrix;
}

template <typename T>
Vector2<T> BallbotPlant<T>::Coriolis(const systems::Context<T>& context) const {
  BallbotParams<T> const& params = this->get_parameters(context);
  BallbotState<T> const& state = this->get_state(context);

  T const radius_total = params.radius_k() + params.radius_w();
  T const gam =
      (params.l() * params.mass_a()) + (radius_total * params.mass_w());

  Vector2<T> c = Vector2<T>::Zero();

  T const c0 = -params.radius_k() * gam * sin(state.lean_angle()) *
               pow(state.lean_velocity(), 2);

  c(0) = c0;
  return c;
}

template <typename T>
Vector2<T> BallbotPlant<T>::Gravity(const systems::Context<T>& context) const {
  BallbotParams<T> const& params = this->get_parameters(context);
  BallbotState<T> const& state = this->get_state(context);

  T const radius_total = params.radius_k() + params.radius_w();
  T const gam =
      (params.l() * params.mass_a()) + (radius_total * params.mass_w());

  Vector2<T> gravity_matrix = Vector2<T>::Zero();

  T const g1 = -params.gravity() * sin(state.lean_angle()) * gam;

  gravity_matrix(1) = g1;
  return gravity_matrix;
}

template <typename T>
Vector2<T> BallbotPlant<T>::NonPotentialForce(
    const systems::Context<T>& context) const {
  BallbotParams<T> const& params = this->get_parameters(context);
  T const u = this->get_tau(context);

  Vector2<T> force_np = Vector2<T>::Zero();

  T const f0 = (params.radius_k() - params.radius_w()) * u;

  force_np << f0, -f0;

  return force_np;
}

template <typename T>
void BallbotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  BallbotState<T> const& state = get_state(context);
  BallbotState<T>& derivative_vector = get_mutable_state(derivatives);

  Matrix2<T> const mass = MassMatrix(context);
  Vector2<T> const coriolis = Coriolis(context);
  Vector2<T> const gravity = Gravity(context);
  Vector2<T> const force_np = NonPotentialForce(context);
  Vector2<T> const b = force_np - coriolis - gravity;

  math::LinearSolver<Eigen::LLT, Eigen::Matrix<T, 2, 2>> const solver(mass);
  Vector2<T> const qddot = solver.Solve(b);

  derivative_vector.set_ball_angle(state.ball_velocity());
  derivative_vector.set_ball_velocity(qddot(0));
  derivative_vector.set_lean_angle(state.lean_velocity());
  derivative_vector.set_lean_velocity(qddot(1));
}

}  // namespace drake::ballbot
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::ballbot::BallbotPlant);
