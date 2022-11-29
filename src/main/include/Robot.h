#pragma once

#include "SwerveCommandRobot.h"
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <optional>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  SwerveCommandRobot robot;
  std::optional<frc2::CommandPtr> autonomousCommand;
};
