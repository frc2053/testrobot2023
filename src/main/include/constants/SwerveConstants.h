#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>

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
    static constexpr double STEER_KP = 1;
    static constexpr double STEER_KI = 0;
    static constexpr double STEER_KD = 0;

    static constexpr double DRIVE_KF = 0;
    static constexpr double DRIVE_KP = 0;
    static constexpr double DRIVE_KI = 0;
    static constexpr double DRIVE_KD = 0;

    static constexpr auto DRIVE_KS = 0.46814_V;
    static constexpr auto DRIVE_KV = 2.4253 * 1_V / 1_mps;
    static constexpr auto DRIVE_KA = 0.12445 * 1_V / 1_mps_sq;

    static constexpr auto GLOBAL_POSE_TRANS_KP = 10.0;
    static constexpr auto GLOBAL_POSE_ROT_KP = 5.0;

    static constexpr units::volt_t MAX_DRIVE_VOLTAGE = 10_V;

    static constexpr units::meters_per_second_t MAX_CHASSIS_SPEED = 17.5853_fps;
    static constexpr units::meters_per_second_t MAX_CHASSIS_SPEED_10_V = 14.78_fps;
    static constexpr units::radians_per_second_t MAX_CHASSIS_ROT_SPEED = 1080_deg_per_s;
    static constexpr units::radians_per_second_squared_t MAX_CHASSIS_ROT_ACCEL = 10000_deg_per_s_sq;

    extern const frc::TrapezoidProfile<units::radians>::Constraints GLOBAL_THETA_CONTROLLER_CONSTRAINTS;
  }   // namespace swerve_drive_consts

  namespace swerve_physical_dims {
    static constexpr units::meter_t WHEELBASE_WIDTH = 27_in;
    static constexpr units::meter_t WHEELBASE_LENGTH = 27_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 3_in;
    static constexpr units::scalar_t STEER_GEARBOX_RATIO = (84 / 29) * (76 / 21) * (64 / 14);
    static constexpr units::scalar_t DRIVE_GEARBOX_RATIO = 4.71;
    static constexpr units::scalar_t STEER_ENCODER_RATIO = 1.0;
    static constexpr units::scalar_t TREAD_STATIC_COEF_FRIC = 1;
    static constexpr units::scalar_t TREAD_KINETIC_COEF_FRIC = 1;
    static constexpr units::kilogram_t ROBOT_MASS = 61.235_kg;
    static constexpr units::kilogram_square_meter_t ROBOT_MOI = 5.808029_kg_sq_m;
    static constexpr units::kilogram_square_meter_t MODULE_MOI = 0.01_kg_sq_m;
    static constexpr auto DRIVE_GEARBOX = frc::DCMotor::Falcon500(1);
    static constexpr auto STEER_GEARBOX = frc::DCMotor::NEO550(1);
  }   // namespace swerve_physical_dims
}   // namespace str