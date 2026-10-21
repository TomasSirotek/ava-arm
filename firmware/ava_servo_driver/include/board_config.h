#pragma once

#include <cstdint>

#include "pca9685_output/pca9685_config.h"

// Everything that depends on this board and its wiring, in one place.
namespace board {

constexpr uint32_t kSerialBaud = 115200;  // must match the ROS driver's baud_rate

// ESP32-C3-DevKitM-1 -> PCA9685. The C3 has no default I2C pins; GPIO0/GPIO1 are free and are
// not boot pins (those are GPIO2, 8, 9).
constexpr Pca9685Config kPca9685{
    /*sda_pin=*/0,
    /*scl_pin=*/1,
    /*address=*/0x40,
    /*frequency_hz=*/50.0f,
    // Nominally 25 MHz, but each chip differs by a few percent and every pulse width shifts by the
    // same factor. Calibrate once per board (README "Bring-up").
    /*oscillator_hz=*/25000000,
};

}  // namespace board
