#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <str/PDP.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  frc2::Command* GetAutonomousCommand();
private:
  frc::XboxController driverController{str::oi::DRIVER_CONTROLLER};
  DrivebaseSubsystem driveSubsystem;
  frc::SendableChooser<frc2::Command*> autoChooser;
};