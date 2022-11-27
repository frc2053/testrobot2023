#pragma once

#include <frc/system/plant/LinearSystemId.h>
#include <units/length.h>

namespace str {
  namespace physical_dims {
    static constexpr units::meter_t WHEELBASE_WIDTH = 27_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
    static constexpr double DRIVEBASE_GEARBOX_RATIO = 7.0;
    static constexpr auto DRIVEBASE_GEARBOX = frc::DCMotor::Falcon500(2);
  }   // namespace physical_dims
}   // namespace str