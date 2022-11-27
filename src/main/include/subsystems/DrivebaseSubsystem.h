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

private:
  str::DiffDrivebase drivebase{};
};
