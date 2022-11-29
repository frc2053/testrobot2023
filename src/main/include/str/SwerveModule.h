#pragma once

#include "constants/SwerveConstants.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/system/plant/DCMotor.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>

namespace str {
  class SwerveModule {
  public:
    SwerveModule(int driveCanId, int rotationCanId);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop);
    void Periodic();
    void SimulationPeriodic();
    void ResetEncoders();

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotor();
    void ConfigureSteeringMotor();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);
    double MapZeroThreeSixtyToOneEighty(double in);

    ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotorController;
    ctre::phoenix::motorcontrol::TalonFXSimCollection driveMotorSim;
    frc::SimpleMotorFeedforward<units::meters> driveFF{
      str::swerve_drive_consts::DRIVE_KS,
      str::swerve_drive_consts::DRIVE_KV,
      str::swerve_drive_consts::DRIVE_KA};

    units::meters_per_second_t prevModuleSpeed{0};
    units::second_t prevTime{0};
    frc::Timer ffTimer;

    rev::CANSparkMax steeringMotorController;
    std::unique_ptr<rev::SparkMaxPIDController> steeringPIDController;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> steeringEncoder;

    frc::sim::FlywheelSim driveSim{
      str::swerve_physical_dims::DRIVE_GEARBOX,
      str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      0.025_kg_sq_m};

    frc::sim::FlywheelSim steerSim{
      str::swerve_physical_dims::STEER_GEARBOX,
      str::swerve_physical_dims::STEER_GEARBOX_RATIO,
      0.004096955_kg_sq_m};

    units::meter_t driveTotalDistance{0};
    units::radian_t steerTotalAngle{0};
    HAL_SimDeviceHandle steerMotorSim;
    hal::SimDouble steerMotorSimPosition;
    hal::SimDouble steerMotorSimVelocity;
    hal::SimDouble steerMotorOutput;
    frc2::PIDController steerSimPid{
      str::swerve_drive_consts::STEER_KP,
      str::swerve_drive_consts::STEER_KI,
      str::swerve_drive_consts::STEER_KD};
  };
}   // namespace str
