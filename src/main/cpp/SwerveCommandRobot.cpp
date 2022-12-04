#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

void SwerveCommandRobot::ConfigureBindings() {
  // Initialize all of your commands and subsystems here
  // driveSubsystem.SetDefaultCommand(driveSubsystem.ArcadeDriveFactory(
  //   [this] {
  //     return driverController.GetLeftY();
  //   },
  //   [this] {
  //     return driverController.GetRightX();
  //   }
  // ));

  driveSubsystem.SetDefaultCommand(driveSubsystem.DriveFactory(
    [this] {
      return frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
    },
    [this] {
      return frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
    },
    [this] {
      return frc::ApplyDeadband<double>(-driverController.GetRightX(), 0.2);
    }
  ));

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);

  frc::SmartDashboard::PutData(
    "Reset Drivetrain Pose",
    driveSubsystem
      .ResetOdomFactory(
        [this] {
          return frc::SmartDashboard::GetNumber("ResetPose/x_ft", 0);
        },
        [this] {
          return frc::SmartDashboard::GetNumber("ResetPose/y_ft", 0);
        },
        [this] {
          return frc::SmartDashboard::GetNumber("ResetPose/rot_deg", 0);
        }
      )
      .get()
  );
}

frc2::CommandPtr SwerveCommandRobot::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return driveSubsystem.FollowPathFactory(
    15_fps,
    200_mps_sq,
    frc::Pose2d{0_m, 13_ft, 0_rad},
    {frc::Translation2d{4_ft, 9_ft}},
    frc::Pose2d{8_ft, 13_ft, 90_deg},
    true
  );
}
