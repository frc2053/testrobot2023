#pragma once

#include <units/angular_velocity.h>
#include <frc/geometry/Transform2d.h>

namespace str {
  namespace encoder_cprs {
    static constexpr int FALCON_CPR = 2048;
  }

  namespace motor_rpms {
    static constexpr units::revolutions_per_minute_t FALCON_MAX_RPM = 6380_rpm;
  }

  namespace oi {
    static constexpr int DRIVER_CONTROLLER = 0;
  }

  namespace vision {
    static constexpr frc::Transform2d CAMERA_TO_ROBOT{frc::Translation2d{5_in, 5_in}, frc::Rotation2d{0_deg}};
    static constexpr std::string_view TAG_LAYOUT_FILENAME{"2022-rapidreact.json"};
  }
}