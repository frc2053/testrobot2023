#include "str/DiffDrivebase.h"
#include "constants/Encoders.h"
#include "str/Field.h"
#include "str/Units.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

str::DiffDrivebase::DiffDrivebase() {
  ConfigureDriveMotors();
  frc::SmartDashboard::PutData("IMU", &imu);
  frc::SmartDashboard::PutData("Field", str::Field::GetInstance().GetField());
}

frc::Rotation2d str::DiffDrivebase::GetRobotYaw() {
  return imu.GetYaw();
}

frc::Pose2d str::DiffDrivebase::GetRobotPoseBasedOnOdometry() {
  return driveOdometry.GetPose();
}

void str::DiffDrivebase::Periodic() {
  driveOdometry.Update(
    imu.GetYaw(),
    ConvertDriveEncoderTicksToDistance(frontLeftController.GetSelectedSensorPosition()),
    ConvertDriveEncoderTicksToDistance(frontRightController.GetSelectedSensorPosition())
  );

  str::Field::GetInstance().SetRobotPosition(driveOdometry.GetPose());
}

void str::DiffDrivebase::SimulationPeriodic() {
  drivetrainSimulator.SetInputs(
    units::volt_t{frontLeftController.GetMotorOutputVoltage()},
    units::volt_t{frontRightController.GetMotorOutputVoltage()}
  );
  drivetrainSimulator.Update(20_ms);

  leftSideSim.SetIntegratedSensorRawPosition(str::Units::ConvertDistanceToEncoderTicks(
    drivetrainSimulator.GetLeftPosition(),
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  ));
  leftSideSim.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
    str::Units::ConvertLinearVelocityToAngularVelocity(
      drivetrainSimulator.GetLeftVelocity(),
      str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO
  ));
  rightSideSim.SetIntegratedSensorRawPosition(str::Units::ConvertDistanceToEncoderTicks(
    drivetrainSimulator.GetRightPosition(),
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  ));
  rightSideSim.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
    str::Units::ConvertLinearVelocityToAngularVelocity(
      drivetrainSimulator.GetRightVelocity(),
      str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO
  ));

  auto heading = drivetrainSimulator.GetHeading().Radians();
  frc::DataLogManager::Log("DT SIM HEADING: " + units::to_string(heading));
  imu.SetYaw(heading);

  leftSideSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  rightSideSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
}

void str::DiffDrivebase::ArcadeDrive(double fow, double rot) {
  drive.ArcadeDrive(fow, rot, true);
}

ctre::phoenix::motorcontrol::can::TalonFXConfiguration str::DiffDrivebase::ConfigureBaseMotorControllerSettings() {
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

void str::DiffDrivebase::ConfigureDriveMotors() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig = ConfigureBaseMotorControllerSettings();

  // Configure base settings
  frontLeftController.ConfigAllSettings(baseConfig);
  frontRightController.ConfigAllSettings(baseConfig);
  rearLeftController.ConfigAllSettings(baseConfig);
  rearRightController.ConfigAllSettings(baseConfig);

  // Enable voltage compensation to combat consistency from battery sag
  frontLeftController.EnableVoltageCompensation(true);
  frontRightController.EnableVoltageCompensation(true);
  rearLeftController.EnableVoltageCompensation(true);
  rearRightController.EnableVoltageCompensation(true);

  // Make the rear motors follow the front ones
  rearLeftController.Follow(frontLeftController);
  rearLeftController.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);

  rearRightController.Follow(frontRightController);
  rearRightController.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);

  // Invert IRL motors but not sim as the motors are flipped IRL
  if(frc::RobotBase::IsSimulation()) {
    frontLeftController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
    frontRightController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
  } else {
    frontLeftController.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
    frontRightController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
  }

  // Set Neutral Mode to brake so we dont coast to a stop
  frontLeftController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  frontRightController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  rearLeftController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  rearRightController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

units::meter_t str::DiffDrivebase::ConvertDriveEncoderTicksToDistance(int ticks) {
  return str::Units::ConvertEncoderTicksToDistance(
    ticks,
    str::encoder_cprs::FALCON_CPR,
    str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}

units::meters_per_second_t str::DiffDrivebase::ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms) {
  return str::Units::ConvertAngularVelocityToLinearVelocity(
    str::Units::ConvertTicksPer100MsToAngularVelocity(
      ticksPer100Ms,
      str::encoder_cprs::FALCON_CPR,
      str::physical_dims::DRIVEBASE_GEARBOX_RATIO
    ),
    str::physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}