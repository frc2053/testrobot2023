#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/LinearSystem.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace str {
  namespace drive_consts {
    // Motor Velocity PID values
    static constexpr double kF = 0;
    static constexpr double kP = 0;
    static constexpr double kI = 0;
    static constexpr double kD = 0;

    // Diff drive path following constants from SysId
    static constexpr auto KS = 0.22_V;
    static constexpr auto KV = 1.98 * 1_V / 1_mps;
    static constexpr auto KA = 0.2 * 1_V / 1_mps_sq;
    static constexpr auto KV_ANGULAR = 1.5 * 1_V / 1_mps;
    static constexpr auto KA_ANGULAR = 0.3 * 1_V / 1_mps_sq;

    static constexpr units::volt_t MAX_DRIVE_VOLTAGE = 10_V;

    extern const frc::DifferentialDriveKinematics DRIVE_KINEMATICS;
    extern const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT;
  }   // namespace drive_consts

  namespace swerve_consts {
    static constexpr double STEER_KF = 0;
    static constexpr double STEER_KP = 0;
    static constexpr double STEER_KI = 0;
    static constexpr double STEER_KD = 0;

    static constexpr double DRIVE_KF = 0;
    static constexpr double DRIVE_KP = 0;
    static constexpr double DRIVE_KI = 0;
    static constexpr double DRIVE_KD = 0;
  }   // namespace swerve_consts
}   // namespace str