#include "str/SwerveModule.h"
#include "constants/DriveConstants.h"
#include "constants/Encoders.h"
#include "constants/PhysicalDims.h"
#include "str/Units.h"
#include <frc/RobotBase.h>

str::SwerveModule::SwerveModule(int driveCanId, int rotationCanId) :
  driveMotorController(driveCanId),
  steeringMotorController(rotationCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless) {
  steeringPIDController = std::make_unique<rev::SparkMaxPIDController>(steeringMotorController.GetPIDController());
  steeringEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(steeringMotorController.GetEncoder());
  ConfigureSteeringMotor();
  ConfigureDriveMotor();
}

ctre::phoenix::motorcontrol::can::TalonFXConfiguration str::SwerveModule::ConfigureBaseMotorControllerSettings() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;

  // Disable limit switch inputs as we are using these to drive and we've had
  // issues where the limit switch pin gets enabled with a small piece of metal
  // inside the falcon port
  config.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
  config.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
  config.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
  config.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;
  config.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;

  // PIDs for velocity control
  config.slot0.kF = str::drive_consts::kF;
  config.slot0.kP = str::drive_consts::kP;
  config.slot0.kI = str::drive_consts::kI;
  config.slot0.kD = str::drive_consts::kD;

  // sets how often the falcon calculates the velocity. We want this as fast as
  // possible to minimize sensor delay
  config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
  config.velocityMeasurementWindow = 1;

  // sets the maximum voltage of the drive motors to 10 volts to make the robot
  // more consistant during auto this is because the batter will sag below 12
  // volts under load.
  config.voltageCompSaturation = str::drive_consts::MAX_DRIVE_VOLTAGE.to<double>();

  return config;
}

void str::SwerveModule::ConfigureDriveMotor() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig = ConfigureBaseMotorControllerSettings();

  // Configure base settings
  driveMotorController.ConfigAllSettings(baseConfig);

  // Enable voltage compensation to combat consistency from battery sag
  driveMotorController.EnableVoltageCompensation(true);

  // Invert IRL motors but not sim as the motors are flipped IRL
  if(frc::RobotBase::IsSimulation()) {
    driveMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
  } else {
    driveMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
  }

  // Set Neutral Mode to brake so we dont coast to a stop
  driveMotorController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void str::SwerveModule::ConfigureSteeringMotor() {
  steeringMotorController.RestoreFactoryDefaults();

  steeringMotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  steeringPIDController->SetFeedbackDevice(*steeringEncoder.get());
  steeringPIDController->SetFF(str::swerve_consts::STEER_KF);
  steeringPIDController->SetP(str::swerve_consts::STEER_KP);
  steeringPIDController->SetI(str::swerve_consts::STEER_KI);
  steeringPIDController->SetD(str::swerve_consts::STEER_KD);

  steeringMotorController.BurnFlash();
}

void str::SwerveModule::ResetEncoders() {
}

units::meter_t str::SwerveModule::ConvertDriveEncoderTicksToDistance(int ticks) {
  return str::Units::ConvertEncoderTicksToDistance(
    ticks,
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}

units::meters_per_second_t str::SwerveModule::ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms) {
  return str::Units::ConvertAngularVelocityToLinearVelocity(
    str::Units::ConvertTicksPer100MsToAngularVelocity(
      ticksPer100Ms,
      str::encoder_cprs::FALCON_CPR,
      str::physical_dims::DRIVEBASE_GEARBOX_RATIO
    ),
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}