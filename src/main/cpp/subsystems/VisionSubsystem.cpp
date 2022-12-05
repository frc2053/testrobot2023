#include "subsystems/VisionSubsystem.h"
#include <frc/ComputerVisionUtil.h>
#include "Constants.h"
#include <photonlib/PhotonTrackedTarget.h>

VisionSubsystem::VisionSubsystem() : tagLayout{frc::filesystem::GetDeployDirectory() + str::vision::TAG_LAYOUT_FILENAME} {
}

void VisionSubsystem::Periodic() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  if(hasTargets) {
    wpi::ArrayRef<photonlib::PhotonTrackedTarget> targets = result.GetTargets();
    for(const auto& target : targets) {
      frc::Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      int aprilTagId = target.GetFiducialId();
      frc::Pose3d tagPose = tagLayout.GetTagPose(aprilTagId);
      frc::Pose2d estimatedRobotPose = frc::EstimateFieldToRobot(bestCamaraToTarget, tagPose, str::vision::CAMERA_TO_ROBOT);
      //add vision measurement here
    }
  }
}
