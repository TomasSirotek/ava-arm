#include "ava_firmware/servo_firmware.h"

#include <cstdio>

namespace ava {
namespace {
// Without the PWM hardware the firmware must not pretend to work: the ROS driver's handshake
// then fails with this message instead of sending pulses into nothing.
constexpr const char* kNoHardware = "ERR PCA9685 not found on I2C (wiring, address 0x40, VCC)";
}  // namespace

const char* ServoFirmware::begin() {
  hardware_ok_ = pwm_.begin();
  if (!hardware_ok_) {
    return kNoHardware;
  }
  allOff();
  std::snprintf(reply_, sizeof(reply_), "AVA_SERVO %d %d READY", kProtocolVersion, kChannelCount);
  return reply_;
}

const char* ServoFirmware::handleLine(const char* line) {
  const Command command = parseCommand(line);
  switch (command.type) {
    case CommandType::Empty:
      return nullptr;
    case CommandType::Invalid:
      std::snprintf(reply_, sizeof(reply_), "ERR %s", command.error);
      return reply_;
    default:
      break;
  }

  if (!hardware_ok_) {
    return kNoHardware;
  }
  switch (command.type) {
    case CommandType::Query:
      std::snprintf(reply_, sizeof(reply_), "AVA_SERVO %d %d", kProtocolVersion, kChannelCount);
      return reply_;
    case CommandType::SetPulses:
      for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
        pwm_.writePulse(channel, command.pulses_us[channel]);
      }
      return nullptr;
    case CommandType::Disable:
      allOff();
      return nullptr;
    default:
      return nullptr;
  }
}

const char* ServoFirmware::lineTooLong() {
  std::snprintf(reply_, sizeof(reply_), "ERR line too long (max %u characters)",
                static_cast<unsigned>(kMaxLineLength));
  return reply_;
}

void ServoFirmware::allOff() {
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    pwm_.writePulse(channel, 0);
  }
}

}  // namespace ava
