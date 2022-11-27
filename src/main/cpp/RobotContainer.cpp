#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_subsystem.SetDefaultCommand(m_subsystem.ArcadeDriveFactory(
    [this] {
      return m_driverController.GetLeftY();
    },
    [this] {
      return m_driverController.GetRightX();
    }
  ));

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);

  frc::SmartDashboard::PutData(
    "Reset Drivetrain Pose",
    m_subsystem
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

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
