// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <numbers>
#include <type_traits>
#include <units/dimensionless.h>

namespace frc {

  /**
   * This is a 2D vector struct that supports basic vector operations.
   */
  template<typename T>
  struct Vector2d {
    static_assert(units::traits::is_unit_t<T>::value, "T must be a unit_t!");
    Vector2d() = default;
    Vector2d(T x, T y) {
      this->x = x;
      this->y = y;
    };

    /**
     * Rotate a vector in Cartesian space.
     *
     * @param angle angle in degrees by which to rotate vector counter-clockwise.
     */
    void Rotate(units::radian_t angle) {
      double cosA = units::math::cos(angle);
      double sinA = units::math::sin(angle);
      T out[2];
      out[0] = (x * cosA) - (y * sinA);
      out[1] = (x * sinA) + (y * cosA);
      x = out[0];
      y = out[1];
    };

    /**
     * Returns dot product of this vector with argument.
     *
     * @param vec Vector with which to perform dot product.
     */
    units::meters_per_second_t DotVel(const Vector2d<units::scalar_t>& vec) const {
      return (x * vec.x) + (y * vec.y);
    }

    units::newton_t DotForce(const Vector2d<units::scalar_t>& vec) const {
      return (x * vec.x) + (y * vec.y);
    }

    /**
     * Returns magnitude of vector.
     */
    double Magnitude() const {
      return std::sqrt((x * x) + (y * y));
    };

    /**
     * Returns scalar projection of this vector onto argument.
     *
     * @param vec Vector onto which to project this vector.
     */
    double ScalarProject(const Vector2d& vec) const {
      return Dot(vec) / vec.Magnitude();
    };

    units::newton_meter_t Cross(const Vector2d<units::newton_t>& vec) const {
      return (x * vec.y) - (y * vec.x);
    }

    T x{};
    T y{};
  };

}   // namespace frc