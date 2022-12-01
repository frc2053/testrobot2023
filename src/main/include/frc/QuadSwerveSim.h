#pragma once

#include "constants/SwerveConstants.h"
#include <array>
#include <frc/SwerveModuleSim.h>
#include <frc/Vector2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

namespace frc {
  class QuadSwerveSim {
  public:
    QuadSwerveSim() = default;

    void ModelReset(Pose2d pose) {
      prevAccel = Vector2d<units::meters_per_second_squared>();
      prevVel = Vector2d<units::meters_per_second>();
      prevRotAccel = units::radians_per_second_squared_t{0};
      prevRotVel = 0_rad_per_s;
      for(int i = 0; i < 4; i++) {
        simModules[i].Reset(pose.TransformBy(moduleTransforms[i]));
      }
      currentPose = pose;
    }

    Pose2d GetCurrentPose() {
      return currentPose;
    }

    std::array<SwerveModuleSim, 4>& GetModules() {
      return simModules;
    }

    void Update(units::second_t dt) {
      Pose2d fieldReferenceFrame{};
      Transform2d fieldToRobotTrans{fieldReferenceFrame, currentPose};

      for(int i = 0; i < 4; i++) {
        Pose2d modPose = fieldReferenceFrame.TransformBy(fieldToRobotTrans).TransformBy(moduleTransforms[i]);
        simModules[i].SetModulePose(modPose);
        simModules[i].Update(dt);
      }

      std::array<ForceAtPose2d, 4> wheelMotiveForces{};
      for(int i = 0; i < 4; i++) {
        wheelMotiveForces[i] = simModules[i].GetWheelMotiveForce();
      }

      Force2d preFricNetForce{};
      for(int i = 0; i < 4; i++) {
        preFricNetForce.Accum(wheelMotiveForces[i].GetForceInRefFrame(currentPose));
      }

      Force2d sidekickForce{};

      preFricNetForce.Accum(sidekickForce);

      ForceAtPose2d preFricNetForceRobotCenter{preFricNetForce, currentPose};

      std::array<ForceAtPose2d, 4> netXTreadFricForces{};
      for(int i = 0; i < 4; i++) {
        units::scalar_t perWheelForceFrac = 1.0 / 4.0;
        Force2d preFricForceAtModule =
          preFricNetForceRobotCenter.GetForceInRefFrame(simModules[i].GetModulePose()).Times(perWheelForceFrac);
        netXTreadFricForces[i] = simModules[i].GetCrossTreadFrictionalForce(preFricForceAtModule, dt);
      }

      Force2d forceOnRobotCenter = preFricNetForce;
      for(int i = 0; i < 4; i++) {
        forceOnRobotCenter.Accum(netXTreadFricForces[i].GetForceInRefFrame(currentPose));
      }

      ForceAtPose2d netForce{forceOnRobotCenter, currentPose};

      Force2d robotForceInFieldRefFrame = netForce.GetForceInRefFrame(fieldReferenceFrame);

      units::newton_meter_t sumOfTorque{};

      for(int i = 0; i < 4; i++) {
        sumOfTorque = sumOfTorque + wheelMotiveForces[i].GetTorque(currentPose);
        sumOfTorque = sumOfTorque + netXTreadFricForces[i].GetTorque(currentPose);
      }

      Vector2d<units::newtons> temp =
        robotForceInFieldRefFrame.Times(1 / str::swerve_physical_dims::ROBOT_MASS.to<double>()).GetVector();

      Vector2d<units::meters_per_second_squared> accel{
        units::meters_per_second_squared_t{temp.x.to<double>()},
        units::meters_per_second_squared_t{temp.y.to<double>()}};

      Vector2d<units::meters_per_second> velocity{
        prevVel.x + (accel.x + prevAccel.x) / 2 * dt,
        prevVel.y + (accel.y + prevAccel.y) / 2 * dt};

      Translation2d posChange{(velocity.x + prevVel.x) / 2 * dt, (velocity.y + prevVel.y) / 2 * dt};

      prevVel = velocity;
      prevAccel = accel;

      units::radians_per_second_squared_t rotAccel{
        sumOfTorque.to<double>() / str::swerve_physical_dims::ROBOT_MOI.to<double>()};
      units::radians_per_second_t rotVel{prevRotVel + (rotAccel + prevRotAccel) / 2 * dt};
      units::radian_t rotPosChange{(rotVel + prevRotVel) / 2 * dt};

      prevRotVel = rotVel;
      prevRotAccel = rotAccel;

      posChange = posChange.RotateBy(-currentPose.Rotation());
      Twist2d motionThisLoop{posChange.X(), posChange.Y(), rotPosChange};
      currentPose = currentPose.Exp(motionThisLoop);
    }

  private:
    std::array<SwerveModuleSim, 4> simModules{};
    std::array<Translation2d, 4> modulePositions{
      Translation2d{str::swerve_physical_dims::WHEELBASE_WIDTH / 2, str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
      Translation2d{str::swerve_physical_dims::WHEELBASE_WIDTH / 2, -str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
      Translation2d{-str::swerve_physical_dims::WHEELBASE_WIDTH / 2, str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
      Translation2d{-str::swerve_physical_dims::WHEELBASE_WIDTH / 2, -str::swerve_physical_dims::WHEELBASE_LENGTH / 2}};
    std::array<Transform2d, 4> moduleTransforms{
      Transform2d{modulePositions[0], Rotation2d{0_deg}},
      Transform2d{modulePositions[1], Rotation2d{0_deg}},
      Transform2d{modulePositions[2], Rotation2d{0_deg}},
      Transform2d{modulePositions[3], Rotation2d{0_deg}}};
    Vector2d<units::meters_per_second_squared> prevAccel{};
    Vector2d<units::meters_per_second> prevVel{};
    units::radians_per_second_squared_t prevRotAccel{};
    units::radians_per_second_t prevRotVel{};
    Pose2d currentPose{};
  };
};   // namespace frc