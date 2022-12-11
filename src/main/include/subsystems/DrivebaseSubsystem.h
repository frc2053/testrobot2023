#pragma once

#include "frc/EigenCore.h"
#include "str/DiffDrivebase.h"
#include "str/SwerveDrivebase.h"
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <functional>
#include <frc/trajectory/Trajectory.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot);
  frc2::CommandPtr DriveFactory(std::function<double()> fow, std::function<double()> side, std::function<double()> rot);
  frc2::CommandPtr FollowPathFactory(
    units::meters_per_second_t maxSpeed,
    units::meters_per_second_squared_t maxAccel,
    frc::Pose2d startPose,
    std::vector<frc::Pose2d> middlePoints,
    frc::Pose2d endPose,
    bool flipPath180
  );
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );
private:
  std::vector<frc::Rotation2d> CreateRotationVectorForPath(
    std::vector<frc::Pose2d> allPoses,
    frc::Trajectory trajectory
  );
  bool CompareTranslations(const frc::Translation2d& trans1, const frc::Translation2d& trans2);
  std::vector<units::second_t> FindTimeOfSwitchingRotation(const frc::Trajectory& traj, std::vector<frc::Pose2d> pointsToFind);
  // str::DiffDrivebase diffDrivebase{};
  str::SwerveDrivebase swerveDrivebase{};
  int index{0};
};
