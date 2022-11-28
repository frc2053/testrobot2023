#include "str/SwerveModule.h"
#include "Constants.h"
#include "constants/SwerveConstants.h"
#include "str/Units.h"
#include <frc/RobotBase.h>

str::SwerveModule::SwerveModule(int driveCanId, int rotationCanId) :
  driveMotorController(driveCanId), driveMotorSim(driveMotorController),
  steeringMotorController(rotationCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless) {
  steeringPIDController = std::make_unique<rev::SparkMaxPIDController>(steeringMotorController.GetPIDController());
  steeringEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(steeringMotorController.GetEncoder());
  ConfigureSteeringMotor();
  ConfigureDriveMotor();
  ffTimer.Reset();
  ffTimer.Start();
}

void str::SwerveModule::Periodic() {
}

void str::SwerveModule::SimulationPeriodic() {
  driveSim.SetInputVoltage(units::volt_t{driveMotorController.GetMotorOutputVoltage()});
  driveSim.Update(20_ms);

  steerSim.SetInputVoltage(units::volt_t{
    steeringMotorController.GetAppliedOutput() / steeringMotorController.GetVoltageCompensationNominalVoltage()});
  steerSim.Update(20_ms);

  units::meters_per_second_t currentLinearVel = str::Units::ConvertAngularVelocityToLinearVelocity(
    driveSim.GetAngularVelocity(),
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );

  driveTotalDistance = driveTotalDistance + (currentLinearVel * 20_ms);

  driveMotorSim.SetIntegratedSensorRawPosition(str::Units::ConvertDistanceToEncoderTicks(
    driveTotalDistance,
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  ));
  driveMotorSim.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
    driveSim.GetAngularVelocity(),
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  ));
  driveMotorSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
}

frc::SwerveModuleState str::SwerveModule::GetState() {
  frc::SwerveModuleState state;

  state.speed = ConvertDriveEncoderSpeedToVelocity(driveMotorController.GetSelectedSensorVelocity());
  state.angle = frc::Rotation2d(units::radian_t(steeringEncoder->GetPosition()));

  return state;
}

frc::SwerveModulePosition str::SwerveModule::GetPosition() {
  frc::SwerveModulePosition position;

  position.distance = ConvertDriveEncoderTicksToDistance(driveMotorController.GetSelectedSensorPosition());
  position.angle = frc::Rotation2d(units::radian_t(steeringEncoder->GetPosition()));

  return position;
}

void str::SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop) {
  const frc::SwerveModuleState state =
    frc::SwerveModuleState::Optimize(referenceState, units::radian_t(steeringEncoder->GetPosition()));

  units::volt_t driveFFResult = 12_V;
  units::second_t timeElapsed = ffTimer.Get();
  units::second_t dt = timeElapsed - prevTime;
  if(!openLoop) {
    driveFFResult = driveFF.Calculate(state.speed, (state.speed - prevModuleSpeed) / dt);
  }

  int falconSetpoint = str::Units::ConvertAngularVelocityToTicksPer100Ms(
    str::Units::ConvertLinearVelocityToAngularVelocity(
      state.speed,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  );

  driveMotorController.Set(
    ctre::phoenix::motorcontrol::ControlMode::Velocity,
    falconSetpoint,
    ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
    (driveFFResult / 12_V).to<double>()
  );

  steeringPIDController->SetReference(state.angle.Radians().to<double>(), rev::ControlType::kPosition);

  prevModuleSpeed = state.speed;
  prevTime = timeElapsed;
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
  config.slot0.kF = str::swerve_drive_consts::DRIVE_KF;
  config.slot0.kP = str::swerve_drive_consts::DRIVE_KP;
  config.slot0.kI = str::swerve_drive_consts::DRIVE_KI;
  config.slot0.kD = str::swerve_drive_consts::DRIVE_KD;

  // sets how often the falcon calculates the velocity. We want this as fast as
  // possible to minimize sensor delay
  config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
  config.velocityMeasurementWindow = 1;

  // sets the maximum voltage of the drive motors to 10 volts to make the robot
  // more consistant during auto this is because the batter will sag below 12
  // volts under load.
  config.voltageCompSaturation = str::swerve_drive_consts::MAX_DRIVE_VOLTAGE.to<double>();

  return config;
}

void str::SwerveModule::ConfigureDriveMotor() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig = ConfigureBaseMotorControllerSettings();

  // Configure base settings
  driveMotorController.ConfigAllSettings(baseConfig);

  // Enable voltage compensation to combat consistency from battery sag
  driveMotorController.EnableVoltageCompensation(true);

  driveMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);

  // Set Neutral Mode to brake so we dont coast to a stop
  driveMotorController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void str::SwerveModule::ConfigureSteeringMotor() {
  steeringMotorController.RestoreFactoryDefaults();

  steeringMotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  steeringPIDController->SetFeedbackDevice(*steeringEncoder.get());
  steeringPIDController->SetFF(str::swerve_drive_consts::STEER_KF);
  steeringPIDController->SetP(str::swerve_drive_consts::STEER_KP);
  steeringPIDController->SetI(str::swerve_drive_consts::STEER_KI);
  steeringPIDController->SetD(str::swerve_drive_consts::STEER_KD);

  steeringEncoder->SetPositionConversionFactor(2 * std::numbers::pi);

  steeringMotorController.BurnFlash();
}

void str::SwerveModule::ResetEncoders() {
  steeringEncoder->SetPosition(0);
  driveMotorController.SetSelectedSensorPosition(0);
  driveMotorSim.SetIntegratedSensorVelocity(0);
  driveMotorSim.SetIntegratedSensorRawPosition(0);
}

units::meter_t str::SwerveModule::ConvertDriveEncoderTicksToDistance(int ticks) {
  return str::Units::ConvertEncoderTicksToDistance(
    ticks,
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}

units::meters_per_second_t str::SwerveModule::ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms) {
  return str::Units::ConvertAngularVelocityToLinearVelocity(
    str::Units::ConvertTicksPer100MsToAngularVelocity(
      ticksPer100Ms,
      str::encoder_cprs::FALCON_CPR,
      str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
    ),
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}