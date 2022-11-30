#pragma once

#include "MotorGearboxWheelSim.h"
#include "SimpleMotorWithMassModel.h"
#include "Vector2d.h"
#include "constants/SwerveConstants.h"
#include "str/Units.h"
#include <Eigen/Core>
#include <frc/ForceAtPose2d.h>
#include <units/dimensionless.h>
#include <units/math.h>

namespace frc {
  class SwerveModuleSim {
  public:
    SwerveModuleSim() :
      steerMotor(
        str::swerve_physical_dims::STEER_GEARBOX,
        str::swerve_physical_dims::STEER_GEARBOX_RATIO,
        str::swerve_physical_dims::MODULE_MOI
      ),
      driveMotor(
        str::swerve_physical_dims::DRIVE_GEARBOX,
        str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
        str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2,
        0.01
      ){};

    void SetInputVoltages(units::volt_t steerVoltage, units::volt_t driveVoltage) {
      currentSteerVoltage = steerVoltage;
      currentDriveVoltage = driveVoltage;
    };

    units::turn_t GetSteerEncoderPosition() {
      return steerMotor.GetPosition() * str::swerve_physical_dims::STEER_ENCODER_RATIO;
    };

    units::turn_t GetDriveEncoderPosition() {
      return driveMotor.GetPosition() * str::swerve_physical_dims::DRIVE_GEARBOX_RATIO;
    };

    units::radians_per_second_t GetDriveEncoderVelocity() {
      return driveMotor.GetVelocity() * str::swerve_physical_dims::DRIVE_GEARBOX_RATIO;
    };

    void Reset(frc::Pose2d initPose) {
      prevModulePose = initPose;
      currentModulePose = initPose;
      currentLinearSpeed = 0_mps;
      currentSteerAngle = frc::Rotation2d(0_rad);
    };

    void Update(units::second_t dt) {
      Vector2d<units::scalar_t> steerUnitVec(1, 0);
      steerUnitVec.Rotate(currentSteerAngle.Radians());
      units::meters_per_second_t velocityAlongSteer = GetModuleRelativeTranslationVelocity(dt).DotVel(steerUnitVec);
      driveMotor.Update(velocityAlongSteer, currentDriveVoltage, dt);
      steerMotor.Update(currentSteerVoltage, dt);
      currentSteerAngle = Rotation2d(steerMotor.GetPosition());
    };

    Vector2d<units::meters_per_second_t> GetModuleRelativeTranslationVelocity(units::second_t dt) {
      units::meters_per_second_t xVel = (currentModulePose.Translation().X() - prevModulePose.Translation().X()) / dt;
      units::meters_per_second_t yVel = (currentModulePose.Translation().Y() - prevModulePose.Translation().Y()) / dt;
      Vector2d<units::meters_per_second_t> moduleTranslationVec(xVel, yVel);
      moduleTranslationVec.Rotate(-1.0 * currentModulePose.Rotation().Radians());
      return moduleTranslationVec;
    };

    ForceAtPose2d GetCrossTreadFrictionalForce(Force2d netForce, units::second_t dt) {
      Vector2d<units::scalar_t> crossTreadUnitVector(0, 1);
      crossTreadUnitVector.Rotate(currentSteerAngle.Radians());
      crossTreadVelMag = GetModuleRelativeTranslationVelocity(dt).DotVel(crossTreadUnitVector);
      crossTreadForceMag = netForce.GetVector().DotForce(crossTreadUnitVector);
      Force2d fricForce{};
      if(units::math::fabs(crossTreadForceMag) > treadStaticFricForce || units::math::fabs(crossTreadVelMag) > 0.001_mps) {
        crossTreadFricForceMag =
          -1.0 * str::Units::sgn<units::meters_per_second_t>(crossTreadVelMag) * treadKineticFricForce;
      } else {
        crossTreadFricForceMag = -1.0 * crossTreadForceMag;
      }
      fricForce = Force2d(
        units::newton_t{crossTreadUnitVector.x.to<double>()},
        units::newton_t{crossTreadUnitVector.y.to<double>()}
      );
      fricForce = fricForce.Times(crossTreadFricForceMag.to<double>());
      return ForceAtPose2d(fricForce, currentModulePose);
    };

    ForceAtPose2d GetWheelMotiveForce() {
      return ForceAtPose2d(Force2d(driveMotor.GetGroundForce(), currentSteerAngle), currentModulePose);
    }

    void SetModulePose(Pose2d curPose) {
      if(first) {
        prevModulePose = curPose;
        first = false;
      } else {
        prevModulePose = currentModulePose;
      }

      currentModulePose = curPose;
    }

    Pose2d GetModulePose() {
      return currentModulePose;
    }

  private:
    SimpleMotorWithMassModel steerMotor;
    MotorGearboxWheelSim driveMotor;
    units::volt_t currentSteerVoltage{};
    units::volt_t currentDriveVoltage{};
    units::meters_per_second_t currentLinearSpeed{};
    Rotation2d currentSteerAngle{};
    Pose2d prevModulePose{};
    Pose2d currentModulePose{};
    units::newton_t crossTreadFricForceMag{};
    units::meters_per_second_t crossTreadVelMag{};
    units::newton_t crossTreadForceMag{};
    units::newton_t normalForce{(str::swerve_physical_dims::ROBOT_MASS * 9.81_mps_sq) / 4};
    units::newton_t treadStaticFricForce{str::swerve_physical_dims::TREAD_STATIC_COEF_FRIC * normalForce};
    units::newton_t treadKineticFricForce{str::swerve_physical_dims::TREAD_KINETIC_COEF_FRIC * normalForce};
    bool first{true};
  };
}   // namespace frc