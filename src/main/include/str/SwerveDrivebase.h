#pragma once

#include "constants/SwerveConstants.h"
#include "str/IMU.h"
#include "str/SwerveModule.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
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
    void Drive(double fow, double side, double rot);

  private:
    str::IMU imu{};

    frc::Translation2d flLocation{
      str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      str::swerve_physical_dims::WHEELBASE_WIDTH / 2};
    frc::Translation2d frLocation{
      str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      -str::swerve_physical_dims::WHEELBASE_WIDTH / 2};
    frc::Translation2d blLocation{
      -str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      str::swerve_physical_dims::WHEELBASE_WIDTH / 2};
    frc::Translation2d brLocation{
      -str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      -str::swerve_physical_dims::WHEELBASE_WIDTH / 2};

    str::SwerveModule flModule{
      str::swerve_can_ids::FRONT_LEFT_DRIVE_TALON_ID,
      str::swerve_can_ids::FRONT_LEFT_STEER_TALON_ID};
    str::SwerveModule frModule{
      str::swerve_can_ids::FRONT_RIGHT_DRIVE_TALON_ID,
      str::swerve_can_ids::FRONT_RIGHT_STEER_TALON_ID};
    str::SwerveModule blModule{
      str::swerve_can_ids::REAR_LEFT_DRIVE_TALON_ID,
      str::swerve_can_ids::REAR_LEFT_STEER_TALON_ID};
    str::SwerveModule brModule{
      str::swerve_can_ids::REAR_RIGHT_DRIVE_TALON_ID,
      str::swerve_can_ids::REAR_RIGHT_STEER_TALON_ID};
  };
}   // namespace str
