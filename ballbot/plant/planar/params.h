#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/name_value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake::ballbot::planar {
struct BallbotParamsIndicies {
  static int const kNumCoordinates = 11;

  static int const kMassK = 0;
  static int const kMassA = 1;
  static int const kMassW = 2;
  static int const kRadiusK = 3;
  static int const kRadiusW = 4;
  static int const kRadiusA = 5;
  static int const kL = 6;
  static int const kThetaK = 7;
  static int const kThetaW = 8;
  static int const kThetaA = 9;
  static int const kGravity = 10;

  static std::vector<std::string> const& GetCoordinateNames();
};

template <typename T>
class BallbotParams final : public systems::BasicVector<T> {
 public:
  typedef BallbotParamsIndicies K;

  BallbotParams() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_mass_k(2.29);
    this->set_mass_a(9.2);
    this->set_mass_w(3.0);
    this->set_radius_k(0.125);
    this->set_radius_w(0.06);
    this->set_radius_a(0.1);
    this->set_l(0.338);
    this->set_theta_k(0.0239);
    this->set_theta_w(0.00236);
    this->set_theta_a(4.76);
    this->set_gravity(9.81);
  }

  BallbotParams(const BallbotParams& other)
      : systems::BasicVector<T>(other.values()) {}

  BallbotParams(BallbotParams&& other) noexcept
      : systems::BasicVector<T>(std::move(other.values())) {}

  BallbotParams& operator=(const BallbotParams& other) {
    this->values() = other.values();
    return *this;
  }

  BallbotParams& operator=(BallbotParams&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }

  template <typename U = T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
    this->set_mass_k(symbolic::Variable("mass_k"));
    this->set_mass_a(symbolic::Variable("mass_a"));
    this->set_mass_w(symbolic::Variable("mass_w"));
    this->set_radius_k(symbolic::Variable("radius_k"));
    this->set_radius_w(symbolic::Variable("radius_w"));
    this->set_radius_a(symbolic::Variable("radius_a"));
    this->set_l(symbolic::Variable("l"));
    this->set_theta_k(symbolic::Variable("Theta_k"));
    this->set_theta_w(symbolic::Variable("Theta_w"));
    this->set_theta_a(symbolic::Variable("Theta_a"));
    this->set_gravity(symbolic::Variable("gravity"));
  }

  [[nodiscard]] BallbotParams<T>* DoClone() const final {
    return new BallbotParams;
  }

  const T& mass_k() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMassK);
  }

  void set_mass_k(const T& mass_k) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMassK, mass_k);
  }

  [[nodiscard]] BallbotParams<T> with_mass_k(const T& mass_k) const {
    BallbotParams<T> result(*this);
    result.set_mass_k(mass_k);
    return result;
  }

  const T& mass_a() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMassA);
  }

  void set_mass_a(const T& mass_a) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMassA, mass_a);
  }

  [[nodiscard]] BallbotParams<T> with_mass_a(const T& mass_a) const {
    BallbotParams<T> result(*this);
    result.set_mass_a(mass_a);
    return result;
  }

  const T& mass_w() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kMassW);
  }

  void set_mass_w(const T& mass_w) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kMassW, mass_w);
  }

  [[nodiscard]] BallbotParams<T> with_mass_w(const T& mass_w) const {
    BallbotParams<T> result(*this);
    result.set_mass_w(mass_w);
    return result;
  }

  const T& radius_k() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kRadiusK);
  }

  void set_radius_k(const T& radius_k) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kRadiusK, radius_k);
  }

  [[nodiscard]] BallbotParams<T> with_radius_k(const T& radius_k) const {
    BallbotParams<T> result(*this);
    result.set_radius_k(radius_k);
    return result;
  }

  const T& radius_w() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kRadiusW);
  }

  void set_radius_w(const T& radius_w) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kRadiusW, radius_w);
  }

  [[nodiscard]] BallbotParams<T> with_radius_w(const T& radius_w) const {
    BallbotParams<T> result(*this);
    result.set_radius_w(radius_w);
    return result;
  }

  const T& radius_a() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kRadiusA);
  }

  void set_radius_a(const T& radius_a) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kRadiusA, radius_a);
  }

  [[nodiscard]] BallbotParams<T> with_radius_a(const T& radius_a) const {
    BallbotParams<T> result(*this);
    result.set_radius_a(radius_a);
    return result;
  }

  const T& l() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kL);
  }

  void set_l(const T& l) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kL, l);
  }

  [[nodiscard]] BallbotParams<T> with_l(const T& l) const {
    BallbotParams<T> result(*this);
    result.set_l(l);
    return result;
  }

  const T& theta_k() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kThetaK);
  }

  void set_theta_k(const T& theta_k) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kThetaK, theta_k);
  }

  [[nodiscard]] BallbotParams<T> with_theta_k(const T& theta_k) const {
    BallbotParams<T> result(*this);
    result.set_theta_k(theta_k);
    return result;
  }

  const T& theta_w() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kThetaW);
  }

  void set_theta_w(const T& theta_w) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kThetaW, theta_w);
  }

  [[nodiscard]] BallbotParams<T> with_theta_w(const T& theta_w) const {
    BallbotParams<T> result(*this);
    result.set_theta_w(theta_w);
    return result;
  }

  const T& theta_a() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kThetaA);
  }

  void set_theta_a(const T& theta_a) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kThetaA, theta_a);
  }

  [[nodiscard]] BallbotParams<T> with_theta_a(const T& theta_a) const {
    BallbotParams<T> result(*this);
    result.set_theta_a(theta_a);
    return result;
  }

  const T& gravity() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kGravity);
  }

  void set_gravity(const T& gravity) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kGravity, gravity);
  }

  [[nodiscard]] BallbotParams<T> with_gravity(const T& gravity) const {
    BallbotParams<T> result(*this);
    result.set_gravity(gravity);
    return result;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    T& mass_k_ref = this->GetAtIndex(K::kMassK);
    a->Visit(MakeNameValue("mass_k", &mass_k_ref));
    T& mass_a_ref = this->GetAtIndex(K::kMassA);
    a->Visit(MakeNameValue("mass_a", &mass_a_ref));
    T& mass_w_ref = this->GetAtIndex(K::kMassW);
    a->Visit(MakeNameValue("mass_w", &mass_w_ref));
    T& radius_k_ref = this->GetAtIndex(K::kRadiusK);
    a->Visit(MakeNameValue("radius_k", &radius_k_ref));
    T& radius_w_ref = this->GetAtIndex(K::kRadiusW);
    a->Visit(MakeNameValue("radius_w", &radius_w_ref));
    T& radius_a_ref = this->GetAtIndex(K::kRadiusA);
    a->Visit(MakeNameValue("radius_a", &radius_a_ref));
    T& l_ref = this->GetAtIndex(K::kL);
    a->Visit(MakeNameValue("l", &l_ref));
    T& theta_k_ref = this->GetAtIndex(K::kThetaK);
    a->Visit(MakeNameValue("Theta_k", &theta_k_ref));
    T& theta_w_ref = this->GetAtIndex(K::kThetaW);
    a->Visit(MakeNameValue("Theta_w", &theta_w_ref));
    T& theta_a_ref = this->GetAtIndex(K::kThetaA);
    a->Visit(MakeNameValue("Theta_a", &theta_a_ref));
    T& gravity_ref = this->GetAtIndex(K::kGravity);
    a->Visit(MakeNameValue("gravity", &gravity_ref));
  }

  static const std::vector<std::string>& GetCoordinateNames() {
    return BallbotParamsIndicies::GetCoordinateNames();
  }

  boolean<T> IsValid() const {
    using std::isnan;
    boolean<T> result(true);
    result = result && !isnan(this->mass_k());
    result = result && (this->mass_k() >= T(0.0));
    result = result && !isnan(this->mass_a());
    result = result && (this->mass_a() >= T(0.0));
    result = result && !isnan(this->mass_w());
    result = result && (this->mass_w() >= T(0.0));
    result = result && !isnan(this->radius_k());
    result = result && (this->radius_k() >= T(0.0));
    result = result && !isnan(this->radius_w());
    result = result && (this->radius_w() >= T(0.0));
    result = result && !isnan(this->radius_a());
    result = result && (this->radius_a() >= T(0.0));
    result = result && !isnan(this->l());
    result = result && (this->l() >= T(0.0));
    result = result && !isnan(this->theta_k());
    result = result && (this->theta_k() >= T(0.0));
    result = result && !isnan(this->theta_w());
    result = result && (this->theta_w() >= T(0.0));
    result = result && !isnan(this->theta_a());
    result = result && (this->theta_a() >= T(0.0));
    result = result && !isnan(this->gravity());
    result = result && (this->gravity() >= T(0.0));
    return result;
  }

  void GetElementBounds(Eigen::VectorXd* lower,
                        Eigen::VectorXd* upper) const final {
    const double kInf = std::numeric_limits<double>::infinity();
    *lower = Eigen::Matrix<double, 11, 1>::Constant(-kInf);
    *upper = Eigen::Matrix<double, 11, 1>::Constant(kInf);
    (*lower)(K::kMassK) = 0.0;
    (*lower)(K::kMassA) = 0.0;
    (*lower)(K::kMassW) = 0.0;
    (*lower)(K::kRadiusK) = 0.0;
    (*lower)(K::kRadiusW) = 0.0;
    (*lower)(K::kRadiusA) = 0.0;
    (*lower)(K::kL) = 0.0;
    (*lower)(K::kThetaK) = 0.0;
    (*lower)(K::kThetaW) = 0.0;
    (*lower)(K::kThetaA) = 0.0;
    (*lower)(K::kGravity) = 0.0;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->size() == 0) {
      throw std::out_of_range(
          "The BallbotParams vector has been moved-from; "
          "accessor methods may no longer be used");
    }
  }
};
}  // namespace drake::ballbot::planar
