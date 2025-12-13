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

  // TODO: rename these to match paper
  static int const kBallAngle = 0;
  static int const kBallVelocity = 1;
  static int const kLeanAngle = 2;
  static int const kLeanVelocity = 3;

  static const std::vector<std::string>& GetCoordinateNames();
};

template <typename T>
class BallbotState final : public systems::BasicVector<T> {
 public:
  typedef BallbotStateIndicies K;

  BallbotState() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_ball_angle(0.0);
    this->set_ball_velocity(0.0);
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
    this->set_ball_angle(symbolic::Variable("ball_angle"));
    this->set_ball_velocity(symbolic::Variable("ball_velocity"));
    this->set_lean_angle(symbolic::Variable("lean_angle"));
    this->set_lean_velocity(symbolic::Variable("lean_velocity"));
  }

  [[nodiscard]] BallbotState<T>* DoClone() const final {
    return new BallbotState;
  }

  const T& ball_angle() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kBallAngle);
  }

  void set_ball_angle(const T& ball_angle) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kBallAngle, ball_angle);
  }

  [[nodiscard]] BallbotState<T> with_ball_angle(const T& ball_angle) const {
    BallbotState<T> result(*this);
    result.set_ball_angle(ball_angle);
    return result;
  }

  const T& ball_velocity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kBallVelocity);
  }

  void set_ball_velocity(const T& ball_velocity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kBallVelocity, ball_velocity);
  }

  [[nodiscard]] BallbotState<T> with_ball_velocity(
      const T& ball_velocity) const {
    BallbotState<T> result(*this);
    result.set_ball_velocity(ball_velocity);
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
    T& ball_angle_ref = this->GetAtIndex(K::kBallAngle);
    a->Visit(MakeNameValue("ball_angle", &ball_angle_ref));
    T& ball_velocity_ref = this->GetAtIndex(K::kBallVelocity);
    a->Visit(MakeNameValue("ball_velocity", &ball_velocity_ref));
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
    result = result && !isnan(this->ball_angle());
    result = result && !isnan(this->ball_velocity());
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
