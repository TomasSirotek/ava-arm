#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "ava_firmware/pwm_output.h"
#include "pca9685_output/pca9685_config.h"

// PwmOutput on a PCA9685 16-channel PWM board over I2C.
class Pca9685Output : public ava::PwmOutput {
 public:
  Pca9685Output(TwoWire& wire, const Pca9685Config& config);

  bool begin() override;
  void writePulse(uint8_t channel, uint16_t pulse_us) override;

 private:
  TwoWire& wire_;
  const Pca9685Config config_;
  Adafruit_PWMServoDriver driver_;
};
