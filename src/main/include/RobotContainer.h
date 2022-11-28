#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc/XboxController.h>
#include <frc2/command/Command.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

private:
  frc::XboxController m_driverController{str::oi::DRIVER_CONTROLLER};

  // The robot's subsystems and commands are defined here...
  DrivebaseSubsystem m_subsystem;

  void ConfigureButtonBindings();
};
