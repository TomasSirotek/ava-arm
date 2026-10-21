#pragma once

#include <cstdint>

namespace ava {

// What the firmware logic needs from the PWM hardware. The real implementation is the PCA9685
// (lib/pca9685_output); the unit tests use a fake. No Arduino or Adafruit types here, so the
// logic that uses it builds and runs on the PC.
class PwmOutput {
 public:
  virtual ~PwmOutput() = default;

  // Starts the hardware. Returns false when it does not answer (e.g. PCA9685 not on the I2C bus).
  virtual bool begin() = 0;

  // Puts a pulse of pulse_us microseconds on a channel, 50 times per second. 0 = channel off.
  virtual void writePulse(uint8_t channel, uint16_t pulse_us) = 0;
};

}  // namespace ava
