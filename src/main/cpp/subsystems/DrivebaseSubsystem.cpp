#include "subsystems/DrivebaseSubsystem.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <units/length.h>

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

frc2::CommandPtr DrivebaseSubsystem::ResetOdomFactory(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  return frc2::InstantCommand(
           [this, x_ft, y_ft, rot_deg]() {
             drivebase.ResetPose(frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg())));
           },
           {this}
  )
    .ToPtr();
}
