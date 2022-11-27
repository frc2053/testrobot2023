#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <functional>
#include <str/DiffDrivebase.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  frc2::CommandPtr ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot);
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );

private:
  str::DiffDrivebase drivebase{
    str::can_ids::FRONT_LEFT_DRIVEBASE_TALON_ID,
    str::can_ids::FRONT_RIGHT_DRIVEBASE_TALON_ID,
    str::can_ids::REAR_LEFT_DRIVEBASE_TALON_ID,
    str::can_ids::REAR_RIGHT_DRIVEBASE_TALON_ID};
};
