#pragma once

#include <cstdint>

// How one PCA9685 is connected and set up. The values for this board are in include/board_config.h;
// the driver only receives them, so it works with any pins, address or second board.
struct Pca9685Config {
  int sda_pin;              // ESP32 GPIO for I2C data
  int scl_pin;              // ESP32 GPIO for I2C clock
  uint8_t address;          // I2C address, 0x40 when no solder jumper is closed
  float frequency_hz;       // pulse rate; 50 Hz = one pulse every 20 ms for hobby servos
  uint32_t oscillator_hz;   // the chip's real oscillator, nominally 25 MHz (calibrate per board)
};
