#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_bool.h"
#include "drake/common/name_value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake::ballbot::planar {
struct BallbotStateIndicies {
  static int const kNumCoordinates = 4;

  static int const kWheelAngle = 0;
  static int const kWheelVelocity = 1;
  static int const kLeanAngle = 2;
  static int const kLeanVelocity = 3;

  static const std::vector<std::string>& GetCoordinateNames();
};

template <typename T>
class BallbotState final : public systems::BasicVector<T> {
 public:
  typedef BallbotStateIndicies K;

  BallbotState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_wheel_angle(0.0);
    this->set_wheel_velocity(0.0);
    this->set_lean_angle(0.0);
    this->set_lean_velocity(0.0);
  }

  BallbotState(const BallbotState& other)
      : systems::BasicVector<T>(other.values()) {}
  BallbotState(BallbotState&& other) noexcept
      : systems::BasicVector<T>(std::move(other.values())) {}

  BallbotState& operator=(const BallbotState& other) {
    this->values() = other.values();
    return *this;
  }

  BallbotState& operator=(BallbotState&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }

  template <typename U = T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
    this->set_wheel_angle(symbolic::Variable("ball_angle"));
    this->set_wheel_velocity(symbolic::Variable("ball_velocity"));
    this->set_lean_angle(symbolic::Variable("lean_angle"));
    this->set_lean_velocity(symbolic::Variable("lean_velocity"));
  }

  [[nodiscard]] BallbotState<T>* DoClone() const final {
    return new BallbotState;
  }

  const T& wheel_angle() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kWheelAngle);
  }

  void set_wheel_angle(const T& ball_angle) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kWheelAngle, ball_angle);
  }

  [[nodiscard]] BallbotState<T> with_wheel_angle(const T& ball_angle) const {
    BallbotState<T> result(*this);
    result.set_wheel_angle(ball_angle);
    return result;
  }

  const T& wheel_velocity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kWheelVelocity);
  }

  void set_wheel_velocity(const T& ball_velocity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kWheelVelocity, ball_velocity);
  }

  [[nodiscard]] BallbotState<T> with_wheel_velocity(
      const T& ball_velocity) const {
    BallbotState<T> result(*this);
    result.set_wheel_velocity(ball_velocity);
    return result;
  }

  const T& lean_angle() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLeanAngle);
  }

  void set_lean_angle(const T& lean_angle) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLeanAngle, lean_angle);
  }

  [[nodiscard]] BallbotState<T> with_lean_angle(const T& lean_angle) const {
    BallbotState<T> result(*this);
    result.set_lean_angle(lean_angle);
    return result;
  }

  const T& lean_velocity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kLeanVelocity);
  }

  void set_lean_velocity(const T& lean_velocity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kLeanVelocity, lean_velocity);
  }

  [[nodiscard]] BallbotState<T> with_lean_velocity(
      const T& lean_velocity) const {
    BallbotState<T> result(*this);
    result.set_lean_velocity(lean_velocity);
    return result;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    T& ball_angle_ref = this->GetAtIndex(K::kWheelAngle);
    a->Visit(MakeNameValue("wheel_angle", &ball_angle_ref));
    T& ball_velocity_ref = this->GetAtIndex(K::kWheelVelocity);
    a->Visit(MakeNameValue("wheel_velocity", &ball_velocity_ref));
    T& lean_angle_ref = this->GetAtIndex(K::kLeanAngle);
    a->Visit(MakeNameValue("lean_angle", &lean_angle_ref));
    T& lean_velocity_ref = this->GetAtIndex(K::kLeanVelocity);
    a->Visit(MakeNameValue("lean_velocity", &lean_velocity_ref));
  }

  static const std::vector<std::string>& GetCoordinateNames() {
    return BallbotStateIndicies::GetCoordinateNames();
  }

  boolean<T> IsValid() const {
    using std::isnan;
    boolean<T> result(true);
    result = result && !isnan(this->wheel_angle());
    result = result && !isnan(this->wheel_velocity());
    result = result && !isnan(this->lean_angle());
    result = result && !isnan(this->lean_velocity());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->size() == 0) {
      throw std::out_of_range("BallbotState is empty");
    }
  }
};
}  // namespace drake::ballbot::planar

#
