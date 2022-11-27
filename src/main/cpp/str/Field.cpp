#include "str/Field.h"

void str::Field::SetRobotPosition(const frc::Pose2d& newPosition) {
  field.SetRobotPose(newPosition);
}

void str::Field::SetObjectPosition(std::string_view object_name, const frc::Pose2d& newPosition) {
  field.GetObject(object_name)->SetPose(newPosition);
}