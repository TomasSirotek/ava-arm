#include "pca9685_output/pca9685_output.h"

namespace {
// Setting bit 12 of the OFF register ("full off") keeps the output low: no pulse, servo limp.
constexpr uint16_t kFullOff = 4096;
}  // namespace

Pca9685Output::Pca9685Output(TwoWire& wire, const Pca9685Config& config)
    : wire_(wire), config_(config), driver_(config.address, wire) {}

bool Pca9685Output::begin() {
  wire_.begin(config_.sda_pin, config_.scl_pin);

  // begin() asks the I2C bus whether a device answers at the address; false = PCA9685 missing.
  if (!driver_.begin()) {
    return false;
  }
  driver_.setOscillatorFrequency(config_.oscillator_hz);
  driver_.setPWMFreq(config_.frequency_hz);
  return true;
}

void Pca9685Output::writePulse(uint8_t channel, uint16_t pulse_us) {
  if (pulse_us == 0) {
    driver_.setPWM(channel, 0, kFullOff);
    return;
  }
  driver_.writeMicroseconds(channel, pulse_us);
}
