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
  bool openLoopDrive
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

  flModule.SetDesiredState(fl, openLoopDrive);
  frModule.SetDesiredState(fr, openLoopDrive);
  blModule.SetDesiredState(bl, openLoopDrive);
  brModule.SetDesiredState(br, openLoopDrive);
}

void str::SwerveDrivebase::Periodic() {
  odometry.Update(
    imu.GetYaw(),
    {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(), brModule.GetPosition()}
  );
  flModule.Periodic();
  frModule.Periodic();
  blModule.Periodic();
  brModule.Periodic();
  str::Field::GetInstance().SetRobotPosition(odometry.GetPose());
}

void str::SwerveDrivebase::SimulationPeriodic() {
  auto flPos = flModule.GetPosition();
  auto frPos = frModule.GetPosition();
  auto blPos = blModule.GetPosition();
  auto brPos = brModule.GetPosition();

  auto flPosDiff = (flPos.distance - prevflPos.distance) / 20_ms;
  auto frPosDiff = (frPos.distance - prevfrPos.distance) / 20_ms;
  auto blPosDiff = (blPos.distance - prevblPos.distance) / 20_ms;
  auto brPosDiff = (brPos.distance - prevbrPos.distance) / 20_ms;

  auto flStateDiff = frc::SwerveModuleState{flPosDiff, flModule.GetState().angle};
  auto frStateDiff = frc::SwerveModuleState{frPosDiff, frModule.GetState().angle};
  auto blStateDiff = frc::SwerveModuleState{blPosDiff, blModule.GetState().angle};
  auto brStateDiff = frc::SwerveModuleState{brPosDiff, brModule.GetState().angle};

  auto chassisSpeedDiff = kinematics.ToChassisSpeeds({flStateDiff, frStateDiff, blStateDiff, brStateDiff});

  imu.SetRate(chassisSpeedDiff.omega);

  flModule.SimulationPeriodic();
  frModule.SimulationPeriodic();
  blModule.SimulationPeriodic();
  brModule.SimulationPeriodic();
}

void str::SwerveDrivebase::ResetPose(const frc::Pose2d& newPose) {
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
}
