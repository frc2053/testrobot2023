#pragma once

#include "constants/CanIds.h"
#include "constants/DriveConstants.h"
#include "constants/PhysicalDims.h"
#include "str/IMU.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

namespace str {
  class DiffDrivebase {
  public:
    DiffDrivebase(int flCanId, int frCanId, int rLCanId, int rRCanId);
    frc::Rotation2d GetRobotYaw();
    frc::Pose2d GetRobotPoseBasedOnOdometry();
    void Periodic();
    void SimulationPeriodic();
    void ArcadeDrive(double fow, double rot);
    void ResetPose(const frc::Pose2d& newPose = frc::Pose2d());

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotors();
    void ResetEncoders();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    str::IMU imu{};

    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontLeftController;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontRightController;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearLeftController;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearRightController;

    ctre::phoenix::motorcontrol::TalonFXSimCollection leftSideSim{frontLeftController};
    ctre::phoenix::motorcontrol::TalonFXSimCollection rightSideSim{frontRightController};

    frc::DifferentialDrive drive{frontLeftController, frontRightController};

    frc::DifferentialDriveOdometry driveOdometry{0_deg, 0_m, 0_m};
    frc::DifferentialDrivePoseEstimator driveEstimator{
      frc::Rotation2d{0_deg},
      0_m,
      0_m,
      frc::Pose2d{},
      {0.01, 0.01, 0.01, 0.01, 0.01},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 0.1}};

    frc::sim::DifferentialDrivetrainSim drivetrainSimulator{
      str::drive_consts::DRIVE_TRAIN_PLANT,
      str::physical_dims::WHEELBASE_WIDTH,
      str::physical_dims::DRIVEBASE_GEARBOX,
      str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
      str::physical_dims::DRIVE_WHEEL_DIAMETER / 2,
      {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}};
  };
}   // namespace str