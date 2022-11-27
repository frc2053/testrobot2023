#include "constants/DriveConstants.h"
#include "constants/PhysicalDims.h"

namespace str {
  namespace drive_consts {
    const frc::DifferentialDriveKinematics DRIVE_KINEMATICS(physical_dims::WHEELBASE_WIDTH);

    const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT =
      frc::LinearSystemId::IdentifyDrivetrainSystem(KV, KA, KV_ANGULAR, KA_ANGULAR);
  }   // namespace drive_consts
}   // namespace str