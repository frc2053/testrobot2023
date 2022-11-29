#pragma once

#include "str/Units.h"
#include <frc/system/plant/DCMotor.h>
#include <units/length.h>
#include <units/velocity.h>

namespace frc {
  class MotorGearboxWheelSim {
  public:
    DCMotor motor;
    double gearRatio;
    units::meter_t wheelRadius;
    units::newton_t groundForce;
    units::radian_t wheelRotations;
    double gearboxFrictionCoefNmPerRadPerSec;
    units::radians_per_second_t prevWheelRotationSpeed;
    units::revolutions_per_minute_t wheelSpeed;
    units::revolutions_per_minute_t motorSpeed;
    MotorGearboxWheelSim(DCMotor motorIn, double gearRatioIn, units::meter_t wheelRadiusIn, double gearBoxFrictionIn) :
      motor(motorIn), gearRatio(gearRatioIn), wheelRadius(wheelRadiusIn),
      gearboxFrictionCoefNmPerRadPerSec(gearBoxFrictionIn){};
    void Update(units::meters_per_second_t groundVelocity, units::volt_t motorVoltage, units::second_t dt) {
      units::radians_per_second_t wheelRotationalSpeed =
        str::Units::ConvertLinearVelocityToAngularVelocity(groundVelocity, wheelRadius);
      units::radians_per_second_t motorRotationalSpeed = wheelRotationalSpeed * gearRatio;
      units::newton_meter_t motorTorqueNm = motor.Kt * motor.Current(motorRotationalSpeed, motorVoltage);
      units::newton_meter_t gearboxFrictionalTorque =
        units::newton_meter_t{motorRotationalSpeed.to<double>() * gearboxFrictionCoefNmPerRadPerSec};
      units::newton_meter_t currentWheelTorque = motorTorqueNm * gearRatio - gearboxFrictionalTorque;
      groundForce = currentWheelTorque / wheelRadius / 2;
      wheelRotations = wheelRotations + (wheelRotationalSpeed + prevWheelRotationSpeed) / 2 * dt;
      prevWheelRotationSpeed = wheelRotationalSpeed;
      wheelSpeed = wheelRotationalSpeed;
      motorSpeed = motorRotationalSpeed;
    };
    units::radian_t GetPosition() {
      return wheelRotations;
    }
    units::radians_per_second_t GetVelocity() {
      return wheelSpeed;
    }
    units::newton_t GetGroundForce() {
      return groundForce;
    }

  private:
  };
}   // namespace frc