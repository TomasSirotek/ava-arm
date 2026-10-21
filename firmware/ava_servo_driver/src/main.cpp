// Ava servo driver: USB serial protocol v1 -> PCA9685 pulses. Behaviour: README.md.
// This file only builds the parts and moves bytes; the logic is in lib/ (unit-tested on the PC).

#include <Arduino.h>
#include <Wire.h>

#include "ava_firmware/servo_firmware.h"
#include "ava_protocol/line_buffer.h"
#include "board_config.h"
#include "pca9685_output/pca9685_output.h"

namespace {
Pca9685Output pwm(Wire, board::kPca9685);
ava::ServoFirmware firmware(pwm);
ava::LineBuffer line_buffer;
}  // namespace

void setup() {
  Serial.begin(board::kSerialBaud);
  Serial.println(firmware.begin());  // "AVA_SERVO 1 6 READY" or "ERR PCA9685 not found ..."
}

void loop() {
  while (Serial.available() > 0) {
    if (!line_buffer.push(static_cast<char>(Serial.read()))) {
      continue;
    }
    const char* reply = line_buffer.overflowed() ? firmware.lineTooLong()
                                                 : firmware.handleLine(line_buffer.line());
    if (reply != nullptr) {
      Serial.println(reply);
    }
  }
}
