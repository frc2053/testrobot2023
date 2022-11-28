#pragma once

#include "str/DiffDrivebase.h"
#include "str/SwerveDrivebase.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <functional>

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
  frc2::CommandPtr DriveFactory(std::function<double()> fow, std::function<double()> side, std::function<double()> rot);

  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );

private:
  // str::DiffDrivebase diffDrivebase{};
  str::SwerveDrivebase swerveDrivebase{};
};