#include "str/SwerveDrivebase.h"
#include "str/Field.h"
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

void str::SwerveDrivebase::Drive(double fow, double side, double rot) {
}

void str::SwerveDrivebase::Periodic() {
  flModule.Periodic();
  frModule.Periodic();
  blModule.Periodic();
  brModule.Periodic();
}

void str::SwerveDrivebase::SimulationPeriodic() {
  flModule.SimulationPeriodic();
  frModule.SimulationPeriodic();
  blModule.SimulationPeriodic();
  brModule.SimulationPeriodic();
}

void str::SwerveDrivebase::ResetPose(const frc::Pose2d& newPose) {
}
