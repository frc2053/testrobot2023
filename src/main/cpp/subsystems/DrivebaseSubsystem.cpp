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
#include <cmath>
#include <iostream>

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

bool DrivebaseSubsystem::CompareTranslations(const frc::Translation2d& trans1, const frc::Translation2d& trans2) {
  return units::math::abs(trans1.X() - trans2.X()) <= 1_in && units::math::abs(trans1.Y() - trans2.Y()) <= 1_in;
}

std::vector<int> DrivebaseSubsystem::FindIndicesOfSwitchingRotation(const frc::Trajectory& traj, std::vector<frc::Pose2d> pointsToFind) {
  std::vector<int> retVal{};

  int searchIndex = 0;
  int foundIndex = 0;
  for(units::second_t time = 0_s; time < traj.TotalTime(); time = time + 20_ms) {

    if(searchIndex > pointsToFind.size() - 1) {
      break;
    }

    frc::Trajectory::State currentState = traj.Sample(time);
    if(CompareTranslations(currentState.pose.Translation(), pointsToFind[searchIndex].Translation())) {
      retVal.push_back(foundIndex);
      searchIndex++;
    }
    foundIndex++;
  }

  return retVal;
}


std::vector<frc::Rotation2d> DrivebaseSubsystem::CreateRotationVectorForPath(  
  std::vector<frc::Pose2d> allPoses,
  frc::Trajectory trajectory
) {
  std::vector<frc::Rotation2d> retVal{trajectory.States().size()};
  units::second_t currentTime = 0.00_s;

  int index = 0;
  int counter = 0;
  frc::Rotation2d startAngle{allPoses[0].Rotation()};

  std::vector<int> testResult = FindIndicesOfSwitchingRotation(trajectory, allPoses);

  for(frc::Rotation2d& angleAtTime : retVal) {

    frc::Pose2d targetPose{allPoses[index]};
    
    frc::Rotation2d targetAngle{targetPose.Rotation()};
    frc::Trajectory::State currentState = trajectory.Sample(currentTime);

    std::cout << "Target Pose: " << targetPose.X().to<double>() << ", " << targetPose.Y().to<double>() << ", " << targetPose.Rotation().Degrees().to<double>() << "\n";
    std::cout << "Current State: " << currentState.pose.X().to<double>() << ", " << currentState.pose.Y().to<double>() << "\n";

    units::radian_t result{std::lerp(startAngle.Radians().to<double>(), targetAngle.Radians().to<double>(), (double)counter / (double)testResult[index])};
    angleAtTime = frc::Rotation2d{result};
    std::cout << "Result Angle: " << angleAtTime.Degrees().to<double>() << "\n";
    if(CompareTranslations(currentState.pose.Translation(), targetPose.Translation())) {
      startAngle = targetAngle;
      index++;
    }
    currentTime += 20_ms;
    counter++;
    std::cout << "--------------\n";
  }

  units::second_t printTime = 0_s;
  for(int i = 0; i < retVal.size(); i++) {
    std::cout << "Time: " << printTime.to<double>() << " Rotation: " << retVal[i].Degrees().to<double>() << "\n";
    printTime += 20_ms;
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
    [this, rotationValues] {
      return rotationValues[index++]; 
    },
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
