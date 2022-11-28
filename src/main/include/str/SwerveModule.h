#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <units/velocity.h>

namespace str {
  class SwerveModule {
  public:
    SwerveModule(int driveCanId, int rotationCanId);
    frc::SwerveModuleState GetState() const;
    frc::SwerveModulePosition GetPosition() const;
    void SetDesiredState(const frc::SwerveModuleState& state);
    void Periodic();
    void SimulationPeriodic();

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotor();
    void ConfigureSteeringMotor();
    void ResetEncoders();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);

    ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotorController;

    ctre::phoenix::motorcontrol::TalonFXSimCollection driveMotorSim;

    rev::CANSparkMax steeringMotorController;
    std::unique_ptr<rev::SparkMaxPIDController> steeringPIDController;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> steeringEncoder;
  };
}   // namespace str
