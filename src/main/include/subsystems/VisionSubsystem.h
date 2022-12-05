#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonCamera.h>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();
  void Periodic() override;
 private:
  frc::AprilTagFieldLayout tagLayout;
  
};
