#include "subsystems/DrivebaseSubsystem.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc2/command/RunCommand.h>

DrivebaseSubsystem::DrivebaseSubsystem() {
}

void DrivebaseSubsystem::Periodic() {
  drivebase.Periodic();
}

void DrivebaseSubsystem::SimulationPeriodic() {
  drivebase.SimulationPeriodic();
}

frc2::CommandPtr DrivebaseSubsystem::ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot) {
  return frc2::RunCommand(
           [this, fow, rot]() {
             drivebase.ArcadeDrive(fow(), rot());
           },
           {this}
  )
    .ToPtr();
}
