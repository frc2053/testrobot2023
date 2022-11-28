#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/length.h>

namespace str {
  namespace swerve_can_ids {
    static constexpr int FRONT_LEFT_DRIVE_TALON_ID = 2;
    static constexpr int FRONT_LEFT_STEER_TALON_ID = 3;
    static constexpr int FRONT_RIGHT_DRIVE_TALON_ID = 4;
    static constexpr int FRONT_RIGHT_STEER_TALON_ID = 5;
    static constexpr int REAR_LEFT_DRIVE_TALON_ID = 6;
    static constexpr int REAR_LEFT_STEER_TALON_ID = 7;
    static constexpr int REAR_RIGHT_DRIVE_TALON_ID = 8;
    static constexpr int REAR_RIGHT_STEER_TALON_ID = 9;
  }   // namespace swerve_can_ids

  namespace swerve_drive_consts {
    static constexpr double STEER_KF = 0;
    static constexpr double STEER_KP = 0;
    static constexpr double STEER_KI = 0;
    static constexpr double STEER_KD = 0;

    static constexpr double DRIVE_KF = 0;
    static constexpr double DRIVE_KP = 0;
    static constexpr double DRIVE_KI = 0;
    static constexpr double DRIVE_KD = 0;

    static constexpr units::volt_t MAX_DRIVE_VOLTAGE = 10_V;
  }   // namespace swerve_drive_consts

  namespace swerve_physical_dims {
    static constexpr units::meter_t WHEELBASE_WIDTH = 27_in;
    static constexpr units::meter_t WHEELBASE_LENGTH = 27_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 3_in;
    static constexpr double STEER_GEARBOX_RATIO = (84 / 29) * (76 / 21) * (64 / 14);
    static constexpr double DRIVE_GEARBOX_RATIO = 4.71;
    static constexpr auto DRIVE_GEARBOX = frc::DCMotor::Falcon500(1);
    static constexpr auto STEER_GEARBOX = frc::DCMotor::NEO550(1);
  }   // namespace swerve_physical_dims
}   // namespace str