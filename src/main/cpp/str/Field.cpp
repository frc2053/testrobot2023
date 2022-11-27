#include "str/Field.h"

void str::Field::SetRobotPosition(const frc::Pose2d& newPosition) {
  field.SetRobotPose(newPosition);
}