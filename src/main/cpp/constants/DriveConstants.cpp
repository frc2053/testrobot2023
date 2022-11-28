#include "constants/DiffDriveConstants.h"
#include <frc/system/plant/LinearSystemId.h>

namespace str {
  namespace diff_drive_consts {
    const frc::DifferentialDriveKinematics DRIVE_KINEMATICS(diff_physical_dims::WHEELBASE_WIDTH);

    const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT =
      frc::LinearSystemId::IdentifyDrivetrainSystem(KV, KA, KV_ANGULAR, KA_ANGULAR);
  }   // namespace diff_drive_consts
}   // namespace str