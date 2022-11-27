#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <units/velocity.h>

namespace str {
  class SwerveModule {
  public:
    SwerveModule(int driveCanId, int rotationCanId);

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotor();
    void ConfigureSteeringMotor();
    void ResetEncoders();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);

    ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotorController;
    rev::CANSparkMax steeringMotorController;
    std::unique_ptr<rev::SparkMaxPIDController> steeringPIDController;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> steeringEncoder;
  };
}   // namespace str
