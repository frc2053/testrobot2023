#include "subsystems/DrivebaseSubsystem.h"
#include "str/Field.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/length.h>

DrivebaseSubsystem::DrivebaseSubsystem() {
}

void DrivebaseSubsystem::Periodic() {
  // diffDrivebase.Periodic();
  swerveDrivebase.Periodic();
}

void DrivebaseSubsystem::SimulationPeriodic() {
  // diffDrivebase.SimulationPeriodic();
  swerveDrivebase.SimulationPeriodic();
}

frc2::CommandPtr DrivebaseSubsystem::ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot) {
  return frc2::RunCommand(
    [this, fow, rot]() {
      // diffDrivebase.ArcadeDrive(fow(), rot());
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<double()> rot
) {
  return frc2::RunCommand(
    [this, fow, side, rot]() {
      swerveDrivebase.Drive(
        fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED,
        true,
        true,
        false
      );
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
      //  diffDrivebase.ResetPose(
      //    frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      //  );
      swerveDrivebase.ResetPose(
        frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      );
    },
    {this}
  )
    .ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::FollowPathFactory(
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel,
  frc::Pose2d startPose,
  std::vector<frc::Translation2d> middlePoints,
  frc::Pose2d endPose,
  bool flipPath180
) {
  frc::TrajectoryConfig config(maxSpeed, maxAccel);
  config.SetKinematics(swerveDrivebase.GetKinematics());
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, middlePoints, endPose, config);
  if(flipPath180) {
    trajectory = trajectory.RelativeTo(frc::Pose2d(frc::Translation2d(54_ft, 27_ft), frc::Rotation2d(180_deg)));
  }
  str::Field::GetInstance().DrawTraj("Auto Path", trajectory);
  frc2::SwerveControllerCommand<4> controllerCmd(
    trajectory,
    [this]() {
      return swerveDrivebase.GetRobotPose();
    },
    swerveDrivebase.GetKinematics(),
    frc::PIDController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, 0},
    frc::PIDController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, 0},
    frc::ProfiledPIDController<units::radians>{
      str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP,
      0,
      0,
      str::swerve_drive_consts::GLOBAL_THETA_CONTROLLER_CONSTRAINTS},
    [this](auto states) {
      swerveDrivebase.DirectSetModuleStates(states[0], states[1], states[2], states[3]);
    },
    {this}
  );
  return frc2::SequentialCommandGroup(
    frc2::InstantCommand(
      [this, trajectory]() {
        swerveDrivebase.ResetPose(trajectory.InitialPose());
      },
      {}
    ),
    std::move(controllerCmd),
    frc2::InstantCommand(
      [this]() {
        swerveDrivebase.Drive(0_mps, 0_mps, 0_rad_per_s, false, false, true);
      },
      {}
    )
  ).ToPtr();
}
