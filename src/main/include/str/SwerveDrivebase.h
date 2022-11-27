#pragma once

#include "str/IMU.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/velocity.h>

namespace str {
  class SwerveDrivebase {
  public:
    SwerveDrivebase();
    frc::Rotation2d GetRobotYaw();
    frc::Pose2d GetRobotPoseBasedOnOdometry();
    void Periodic();
    void SimulationPeriodic();
    void ResetPose(const frc::Pose2d& newPose = frc::Pose2d());

  private:
    str::IMU imu{};
  };
}   // namespace str
