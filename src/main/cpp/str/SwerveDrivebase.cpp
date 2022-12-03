#include "str/SwerveDrivebase.h"
#include "Constants.h"
#include "str/Field.h"
#include "str/Units.h"
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

str::SwerveDrivebase::SwerveDrivebase() {
  ResetPose();
  frc::SmartDashboard::PutData("IMU", &imu);
  frc::SmartDashboard::PutData("Field", str::Field::GetInstance().GetField());
}

frc::Rotation2d str::SwerveDrivebase::GetRobotYaw() {
  return imu.GetYaw();
}

frc::Pose2d str::SwerveDrivebase::GetRobotPoseBasedOnOdometry() {
  return frc::Pose2d();
}

void str::SwerveDrivebase::Drive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rotSpeed,
  bool fieldRelative,
  bool openLoopDrive,
  bool voltageComp
) {
  auto states = kinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, imu.GetYaw()) :
                    frc::ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
  );

  units::meters_per_second_t maxModuleSpeed = str::Units::ConvertAngularVelocityToLinearVelocity(
    str::motor_rpms::FALCON_MAX_RPM,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );

  kinematics.DesaturateWheelSpeeds(
    &states,
    frc::ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
    maxModuleSpeed,
    str::swerve_drive_consts::MAX_CHASSIS_SPEED,
    str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED
  );

  auto [fl, fr, bl, br] = states;

  desiredModuleDataForNT[0] = fl.angle.Radians().to<double>(),
  desiredModuleDataForNT[1] = fl.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[2] = fr.angle.Radians().to<double>();
  desiredModuleDataForNT[3] = fr.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[4] = bl.angle.Radians().to<double>();
  desiredModuleDataForNT[5] = bl.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[6] = br.angle.Radians().to<double>();
  desiredModuleDataForNT[7] = br.speed.convert<units::feet_per_second>().to<double>();

  frc::SmartDashboard::PutNumberArray("Desired Swerve Module Data", desiredModuleDataForNT);

  flModule.SetDesiredState(fl, openLoopDrive, voltageComp);
  frModule.SetDesiredState(fr, openLoopDrive, voltageComp);
  blModule.SetDesiredState(bl, openLoopDrive, voltageComp);
  brModule.SetDesiredState(br, openLoopDrive, voltageComp);
}

void str::SwerveDrivebase::Periodic() {
  frc::Rotation2d imuYaw = imu.GetYaw();
  std::array<frc::SwerveModulePosition, 4> modulePositions =
    {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(), brModule.GetPosition()};
  std::array<frc::SwerveModuleState, 4> moduleStates =
    {flModule.GetState(), frModule.GetState(), blModule.GetState(), brModule.GetState()};
  odometry.Update(imuYaw, {modulePositions[0], modulePositions[1], modulePositions[2], modulePositions[3]});
  estimator.Update(imuYaw, {modulePositions[0], modulePositions[1], modulePositions[2], modulePositions[3]});

  frc::Pose2d odomPose = odometry.GetPose();
  frc::Pose2d estimatorPose = estimator.GetEstimatedPosition();

  str::Field::GetInstance().SetRobotPosition(odomPose);
  str::Field::GetInstance().SetObjectPosition("Swerve Pose Estimation", estimatorPose);

  if(frc::RobotBase::IsSimulation()) {
    frc::Pose2d simPose = swerveSim.GetCurrentPose();
    str::Field::GetInstance().SetObjectPosition("Swerve Sim Position", simPose);
    str::Field::GetInstance().SetObjectPosition(
      "FL Module Pose",
      simPose.TransformBy(frc::Transform2d(flLocation, moduleStates[0].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "FR Module Pose",
      simPose.TransformBy(frc::Transform2d(frLocation, moduleStates[1].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BL Module Pose",
      simPose.TransformBy(frc::Transform2d(blLocation, moduleStates[2].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BR Module Pose",
      simPose.TransformBy(frc::Transform2d(brLocation, moduleStates[3].angle))
    );
  } else {
    str::Field::GetInstance().SetObjectPosition(
      "FL Module Pose",
      estimatorPose.TransformBy(frc::Transform2d(flLocation, moduleStates[0].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "FR Module Pose",
      estimatorPose.TransformBy(frc::Transform2d(frLocation, moduleStates[1].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BL Module Pose",
      estimatorPose.TransformBy(frc::Transform2d(blLocation, moduleStates[2].angle))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BR Module Pose",
      estimatorPose.TransformBy(frc::Transform2d(brLocation, moduleStates[3].angle))
    );
  }

  currentModuleDataForNT[0] = moduleStates[0].angle.Radians().to<double>();
  currentModuleDataForNT[1] = moduleStates[0].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[2] = moduleStates[1].angle.Radians().to<double>();
  currentModuleDataForNT[3] = moduleStates[1].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[4] = moduleStates[2].angle.Radians().to<double>();
  currentModuleDataForNT[5] = moduleStates[2].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[6] = moduleStates[3].angle.Radians().to<double>();
  currentModuleDataForNT[7] = moduleStates[3].speed.convert<units::feet_per_second>().to<double>();

  currentOdomPoseForNT[0] = odomPose.X().to<double>();
  currentOdomPoseForNT[1] = odomPose.Y().to<double>();
  currentOdomPoseForNT[2] = odomPose.Rotation().Radians().to<double>();

  currentEstimatorPoseForNT[0] = estimatorPose.X().to<double>();
  currentEstimatorPoseForNT[1] = estimatorPose.Y().to<double>();
  currentEstimatorPoseForNT[2] = estimatorPose.Rotation().Radians().to<double>();

  frc::SmartDashboard::PutNumberArray("Robot Odom Pose", currentOdomPoseForNT);
  frc::SmartDashboard::PutNumberArray("Robot Estimator Pose", currentEstimatorPoseForNT);
  frc::SmartDashboard::PutNumberArray("Current Swerve Module Data", currentModuleDataForNT);
}

void str::SwerveDrivebase::SimulationPeriodic() {
  flModule.SimulationPeriodic();
  frModule.SimulationPeriodic();
  blModule.SimulationPeriodic();
  brModule.SimulationPeriodic();

  std::array<frc::SwerveModuleSim, 4>& simModules = swerveSim.GetModules();

  simModules[0].SetInputVoltages(flModule.GetRotationAppliedVoltage(), flModule.GetDriveAppliedVoltage());
  simModules[1].SetInputVoltages(frModule.GetRotationAppliedVoltage(), frModule.GetDriveAppliedVoltage());
  simModules[2].SetInputVoltages(blModule.GetRotationAppliedVoltage(), blModule.GetDriveAppliedVoltage());
  simModules[3].SetInputVoltages(brModule.GetRotationAppliedVoltage(), brModule.GetDriveAppliedVoltage());

  for(int i = 0; i < 20; i++) {
    swerveSim.Update(0.001_s);
  }

  flModule.SetSimState(
    simModules[0].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[0].GetDriveEncoderPosition() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[0].GetDriveEncoderVelocity() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  );

  frModule.SetSimState(
    simModules[1].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[1].GetDriveEncoderPosition() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[1].GetDriveEncoderVelocity() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  );

  blModule.SetSimState(
    simModules[2].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[2].GetDriveEncoderPosition() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[2].GetDriveEncoderVelocity() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  );

  brModule.SetSimState(
    simModules[3].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[3].GetDriveEncoderPosition() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[3].GetDriveEncoderVelocity() / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  );

  imu.SetYaw(swerveSim.GetCurrentPose().Rotation().Radians());
}

void str::SwerveDrivebase::ResetPose(const frc::Pose2d& newPose) {
  swerveSim.ModelReset(newPose);
  imu.SetOffset(newPose.Rotation().Radians());
  imu.ZeroYaw();
  flModule.ResetEncoders();
  frModule.ResetEncoders();
  blModule.ResetEncoders();
  brModule.ResetEncoders();
  odometry.ResetPosition(
    imu.GetYaw(),
    {frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}}},
    newPose
  );
  estimator.ResetPosition(
    imu.GetYaw(),
    {frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
     frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}}},
    newPose
  );
}
