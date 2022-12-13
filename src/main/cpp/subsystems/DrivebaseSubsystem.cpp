#include "subsystems/DrivebaseSubsystem.h"
#include "str/Field.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <str/BetterSwerveControllerCommand.h>
#include <units/length.h>
#include <cmath>
#include <iostream>
#include <frc/Filesystem.h>
#include "Constants.h"
#include "frc/ComputerVisionUtil.h"

DrivebaseSubsystem::DrivebaseSubsystem() : tagLayout{frc::filesystem::GetDeployDirectory()  + "\\" + std::string{str::vision::TAG_LAYOUT_FILENAME}}{
  for(const int& tagId : tagIdList) {
    frc::Pose3d tagPose = tagLayout.GetTagPose(tagId).value();
    str::Field::GetInstance().SetObjectPosition("tag-" + std::to_string(tagId), tagPose.ToPose2d());
    system.AddSimVisionTarget(photonlib::SimVisionTarget{tagPose, 6_in, 6_in, tagId});
  }
}

void DrivebaseSubsystem::Periodic() {
  // diffDrivebase.Periodic();
  swerveDrivebase.Periodic();

  system.ProcessFrame(swerveDrivebase.GetRobotPose());
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  if(hasTargets) {
    auto targets = result.GetTargets();
    for(const auto& target : targets) {
      frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();
      int aprilTagId = target.GetFiducialId();
      frc::Pose3d tagPose = tagLayout.GetTagPose(aprilTagId).value();
      frc::Pose3d estimatedRobotPose = frc::ObjectToRobotPose(tagPose, bestCameraToTarget, str::vision::CAMERA_TO_ROBOT);
      swerveDrivebase.AddVisionMeasurementToPoseEstimator(estimatedRobotPose.ToPose2d(), result.GetTimestamp());
    }
  }
}

void DrivebaseSubsystem::SimulationPeriodic() {
  // diffDrivebase.SimulationPeriodic();
  swerveDrivebase.SimulationPeriodic();
}

bool DrivebaseSubsystem::CompareTranslations(const frc::Translation2d& trans1, const frc::Translation2d& trans2) {
  return units::math::abs(trans1.X() - trans2.X()) <= 1_in && units::math::abs(trans1.Y() - trans2.Y()) <= 1_in;
}

std::vector<units::second_t> DrivebaseSubsystem::FindTimeOfSwitchingRotation(const frc::Trajectory& traj, std::vector<frc::Pose2d> pointsToFind) {
  std::vector<units::second_t> retVal{};

  int searchIndex = 0;
  for(units::second_t sampleTime = 0_s; sampleTime <= traj.TotalTime(); sampleTime = sampleTime + 20_ms) {
    if(searchIndex > pointsToFind.size() - 1) {
      break;
    }
    frc::Trajectory::State currentState = traj.Sample(sampleTime);
    if(CompareTranslations(currentState.pose.Translation(), pointsToFind[searchIndex].Translation())) {
      retVal.push_back(sampleTime);
      searchIndex++;
    }
  }

  return retVal;
}


std::vector<frc::Rotation2d> DrivebaseSubsystem::CreateRotationVectorForPath(  
  std::vector<frc::Pose2d> allPoses,
  frc::Trajectory trajectory
) {
  std::vector<frc::Rotation2d> retVal{};
  std::vector<units::second_t> whereToLerp = FindTimeOfSwitchingRotation(trajectory, std::vector<frc::Pose2d>(allPoses.begin() + 1, allPoses.end()));

  int targetIndex = 1;

  units::radian_t startAngle = allPoses[0].Rotation().Radians();
  units::radian_t goalAngle = allPoses[targetIndex].Rotation().Radians();

  for(const units::second_t& lerpPoint : whereToLerp) {
    for(units::second_t time = 0_s; time < lerpPoint; time = time + 20_ms) {
      units::radian_t lerpResult{std::lerp(startAngle.to<double>(), goalAngle.to<double>(), time.to<double>() / lerpPoint.to<double>())};
      frc::Rotation2d rotVal{lerpResult};
      retVal.push_back(rotVal);
    }
    startAngle = goalAngle;
    targetIndex++;
    if(targetIndex > allPoses.size() - 1) {
      break;
    }
    goalAngle = allPoses[targetIndex].Rotation().Radians();
  }

  return retVal;
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
  std::vector<frc::Pose2d> middlePoints,
  frc::Pose2d endPose,
  bool flipPath180
) {
  frc::TrajectoryConfig config(maxSpeed, maxAccel);
  config.SetKinematics(swerveDrivebase.GetKinematics());
  std::vector<frc::Translation2d> middlePointTranslations{middlePoints.size()};
  std::transform(middlePoints.begin(), middlePoints.end(), middlePointTranslations.begin(), [](frc::Pose2d point){ return point.Translation(); });
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, middlePointTranslations, endPose, config);
  if(flipPath180) {
    trajectory = trajectory.RelativeTo(frc::Pose2d(frc::Translation2d(54_ft, 27_ft), frc::Rotation2d(180_deg)));
  }

  std::vector<frc::Pose2d> allPoses{};
  allPoses.push_back(startPose);
  for(const auto& point : middlePoints) {
    allPoses.push_back(point);
  }
  allPoses.push_back(endPose);
  std::vector rotationValues = CreateRotationVectorForPath(allPoses, trajectory);

  str::Field::GetInstance().DrawTraj("Auto Path", trajectory);
  frc2::BetterSwerveControllerCommand<4> controllerCmd(
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
    // [this, rotationValues] {
    //   std::cout << "Index: " << index << "\n";
    //   return rotationValues[index++]; 
    // },
    [this](auto states) {
      swerveDrivebase.DirectSetModuleStates(states[0], states[1], states[2], states[3]);
    },
    {this}
  );
  return frc2::SequentialCommandGroup(
    frc2::InstantCommand(
      [this, trajectory]() {
        index = 0;
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
