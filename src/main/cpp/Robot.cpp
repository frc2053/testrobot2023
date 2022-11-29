#include "Robot.h"
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Force2d.h>
#include <frc/ForceAtPose2d.h>
#include <frc/QuadSwerveSim.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <str/DiffDrivebase.h>

void Robot::RobotInit() {
  frc::QuadSwerveSim test{};

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  robot.ConfigureBindings();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::AutonomousInit() {
  autonomousCommand = robot.GetAutonomousCommand();

  if(autonomousCommand) {
    autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  if(autonomousCommand) {
    autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
}

void Robot::TestPeriodic() {
}

void Robot::SimulationInit() {
}

void Robot::SimulationPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
