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

void str::SwerveDrivebase::Periodic() {
}

void str::SwerveDrivebase::SimulationPeriodic() {
}

void str::SwerveDrivebase::ResetPose(const frc::Pose2d& newPose) {
}
