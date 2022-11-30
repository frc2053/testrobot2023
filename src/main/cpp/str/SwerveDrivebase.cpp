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
  if(frc::RobotBase::IsSimulation()) {
    frc::Pose2d simPose = swerveSim.GetCurrentPose();
    str::Field::GetInstance().SetRobotPosition(simPose);
    str::Field::GetInstance().SetObjectPosition(
      "FL Module Pose",
      simPose.TransformBy(frc::Transform2d(
        {str::swerve_physical_dims::WHEELBASE_WIDTH / 2, str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
        flModule.GetState().angle
      ))
    );
    str::Field::GetInstance().SetObjectPosition(
      "FR Module Pose",
      simPose.TransformBy(frc::Transform2d(
        {-str::swerve_physical_dims::WHEELBASE_WIDTH / 2, str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
        frModule.GetState().angle
      ))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BL Module Pose",
      simPose.TransformBy(frc::Transform2d(
        {str::swerve_physical_dims::WHEELBASE_WIDTH / 2, -str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
        blModule.GetState().angle
      ))
    );
    str::Field::GetInstance().SetObjectPosition(
      "BR Module Pose",
      simPose.TransformBy(frc::Transform2d(
        {-str::swerve_physical_dims::WHEELBASE_WIDTH / 2, -str::swerve_physical_dims::WHEELBASE_LENGTH / 2},
        brModule.GetState().angle
      ))
    );
  } else {
    str::Field::GetInstance().SetRobotPosition(odometry.GetPose());
  }
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

  frc::Pose2d prevRobotPose = swerveSim.GetCurrentPose();

  for(int i = 0; i < 20; i++) {
    swerveSim.Update(0.001_s);
  }

  flModule.SetSimState(
    simModules[0].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[0].GetDriveEncoderPosition(),
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[0].GetDriveEncoderVelocity()
  );

  frModule.SetSimState(
    simModules[1].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[1].GetDriveEncoderPosition(),
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[1].GetDriveEncoderVelocity()
  );

  blModule.SetSimState(
    simModules[2].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[2].GetDriveEncoderPosition(),
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[2].GetDriveEncoderVelocity()
  );

  brModule.SetSimState(
    simModules[3].GetSteerEncoderPosition(),
    str::Units::ConvertAngularDistanceToLinearDistance(
      simModules[3].GetDriveEncoderPosition(),
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
    ),
    simModules[3].GetDriveEncoderVelocity()
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
}
