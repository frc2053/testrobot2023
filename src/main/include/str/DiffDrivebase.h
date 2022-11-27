#pragma once

#include "constants/CanIds.h"
#include "constants/DriveConstants.h"
#include "constants/PhysicalDims.h"
#include "str/IMU.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

namespace str {
  class DiffDrivebase {
  public:
    DiffDrivebase();
    frc::Rotation2d GetRobotYaw();
    frc::Pose2d GetRobotPoseBasedOnOdometry();
    void Periodic();
    void SimulationPeriodic();
    void ArcadeDrive(double fow, double rot);

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotors();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);

    str::IMU imu{};

    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontLeftController{str::can_ids::FRONT_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontRightController{str::can_ids::FRONT_RIGHT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearLeftController{str::can_ids::REAR_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearRightController{str::can_ids::REAR_RIGHT_DRIVEBASE_TALON_ID};

    ctre::phoenix::motorcontrol::TalonFXSimCollection leftSideSim{frontLeftController};
    ctre::phoenix::motorcontrol::TalonFXSimCollection rightSideSim{frontRightController};

    frc::DifferentialDrive drive{frontLeftController, frontRightController};

    frc::DifferentialDriveOdometry driveOdometry{0_deg, 0_m, 0_m};

    frc::sim::DifferentialDrivetrainSim drivetrainSimulator{
      str::drive_consts::DRIVE_TRAIN_PLANT,
      str::physical_dims::WHEELBASE_WIDTH,
      str::physical_dims::DRIVEBASE_GEARBOX,
      str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
      str::physical_dims::DRIVE_WHEEL_DIAMETER / 2,
      {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}};
  };
}   // namespace str