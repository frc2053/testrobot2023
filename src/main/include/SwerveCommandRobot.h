#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc/XboxController.h>
#include <frc2/command/Command.h>

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  frc2::CommandPtr GetAutonomousCommand();
private:
  frc::XboxController driverController{str::oi::DRIVER_CONTROLLER};
  
  DrivebaseSubsystem driveSubsystem;
};