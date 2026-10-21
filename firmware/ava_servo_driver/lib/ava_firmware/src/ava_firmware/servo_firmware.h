#pragma once

#include "ava_protocol/parser.h"
#include "ava_firmware/pwm_output.h"

namespace ava {

// The whole firmware behaviour, without Arduino: it gets the PWM hardware injected and turns
// command lines into pulses and reply lines. main.cpp only moves bytes between Serial and this.
class ServoFirmware {
 public:
  explicit ServoFirmware(PwmOutput& pwm) : pwm_(pwm) {}

  // Starts the hardware and switches every channel off. Returns the boot line to print:
  // "AVA_SERVO 1 6 READY", or an "ERR ..." line if the PWM hardware does not answer.
  const char* begin();

  // Handles one complete line. Returns the reply to print, or nullptr when there is none
  // ("P" and "D" are silent when they succeed, empty lines are ignored).
  const char* handleLine(const char* line);

  // The reply for a line that was too long for the line buffer.
  const char* lineTooLong();

  bool hardwareOk() const { return hardware_ok_; }

 private:
  void allOff();

  PwmOutput& pwm_;
  bool hardware_ok_ = false;
  char reply_[96] = {};
};

}  // namespace ava
