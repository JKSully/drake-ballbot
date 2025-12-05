#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/name_value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {

struct BallbotInputIndicies {
  static int const kNumCoordinates = 1;

  static int const kTau = 0;

  static const std::vector<std::string>& GetCoordinateNames();
};

template <typename T>
class BallbotInput final : public systems::BasicVector<T> {
 public:
  using K = BallbotInputIndicies;

  BallbotInput() : systems::BasicVector<T>(K::kNumCoordinates) {
    this->set_tau(0.0);
  }

  BallbotInput(const BallbotInput& other)
      : systems::BasicVector<T>(other.values()) {}

  BallbotInput(BallbotInput&& other) noexcept
      : systems::BasicVector<T>(std::move(other.values())) {}

  BallbotInput& operator=(const BallbotInput& other) {
    this->values() = other.values();
    return *this;
  }

  BallbotInput& operator=(BallbotInput&& other) noexcept {
    this->values() = std::move(other.values());
    other.values().resize(0);
    return *this;
  }

  template <typename U = T>
  typename std::enable_if_t<std::is_same_v<U, symbolic::Expression>>
  SetToNamedVariables() {
    this->set_tau(symbolic::Variable("tau"));
  }

  [[nodiscard]] BallbotInput<T>* DoClone() const final {
    return new BallbotInput;
  }

  const T& tau() const {
    ThrowIfEmpty();
    return this->GetAtIndex(K::kTau);
  }

  void set_tau(const T& tau) {
    ThrowIfEmpty();
    this->SetAtIndex(K::kTau, tau);
  }

  [[nodiscard]] BallbotInput<T> with_tau(const T& tau) const {
    BallbotInput<T> result(*this);
    result.set_tau(tau);
    return result;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    T& tau_ref = this->GetAtIndex(K::kTau);
    a->Visit(MakeNameValue("tau", &tau_ref));
  }

  static const std::vector<std::string>& GetCoordinateNames() {
    return BallbotInputIndicies::GetCoordinateNames();
  }

  boolean<T> IsValid() const {
    using std::isnan;
    boolean<T> result(true);
    result = result && !isnan(this->tau());
    return result;
  }

 private:
  void ThrowIfEmpty() const {
    if (this->size() == 0) {
      throw std::logic_error("BallbotInput is empty");
    }
  }
};
}  // namespace drake
