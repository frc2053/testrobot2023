#pragma once

#include <units/angular_velocity.h>

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
}   // namespace str