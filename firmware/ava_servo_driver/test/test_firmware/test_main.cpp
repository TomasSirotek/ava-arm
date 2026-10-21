// Unit tests for the firmware behaviour against a fake PWM output, run on the PC: pio test -e native
#include <unity.h>

#include <cstring>

#include "ava_firmware/servo_firmware.h"

using namespace ava;

namespace {

// Records what the firmware asks the hardware to do.
class FakePwm : public PwmOutput {
 public:
  bool begin_result = true;
  int begin_calls = 0;
  int writes = 0;
  int pulses[kChannelCount];

  FakePwm() {
    for (int& pulse : pulses) {
      pulse = -1;  // -1 = never written
    }
  }
  bool begin() override {
    ++begin_calls;
    return begin_result;
  }
  void writePulse(uint8_t channel, uint16_t pulse_us) override {
    TEST_ASSERT_TRUE_MESSAGE(channel < kChannelCount, "channel out of range");
    pulses[channel] = pulse_us;
    ++writes;
  }
};

}  // namespace

// ---------- firmware behaviour ----------

void test_boot_switches_all_channels_off_and_reports_ready() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  TEST_ASSERT_EQUAL_STRING("AVA_SERVO 1 6 READY", firmware.begin());
  TEST_ASSERT_EQUAL(1, pwm.begin_calls);
  for (int pulse : pwm.pulses) {
    TEST_ASSERT_EQUAL(0, pulse);
  }
}

void test_query_reply() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  firmware.begin();
  TEST_ASSERT_EQUAL_STRING("AVA_SERVO 1 6", firmware.handleLine("?"));
}

void test_pulses_reach_the_right_channels_silently() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  firmware.begin();
  TEST_ASSERT_NULL(firmware.handleLine("P 1000 1100 1200 1300 1400 3000"));
  const int expected[kChannelCount] = {1000, 1100, 1200, 1300, 1400, 2500};
  TEST_ASSERT_EQUAL_INT_ARRAY(expected, pwm.pulses, kChannelCount);
}

void test_disable_switches_everything_off() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  firmware.begin();
  firmware.handleLine("P 1500 1500 1500 1500 1500 1500");
  TEST_ASSERT_NULL(firmware.handleLine("D"));
  for (int pulse : pwm.pulses) {
    TEST_ASSERT_EQUAL(0, pulse);
  }
}

void test_invalid_line_replies_err_and_keeps_pulses() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  firmware.begin();
  firmware.handleLine("P 1500 1500 1500 1500 1500 1500");
  const int writes_before = pwm.writes;
  TEST_ASSERT_EQUAL_STRING("ERR P needs 6 values", firmware.handleLine("P 1000"));
  TEST_ASSERT_EQUAL(writes_before, pwm.writes);
  TEST_ASSERT_EQUAL(1500, pwm.pulses[0]);
}

void test_empty_line_is_ignored() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  firmware.begin();
  TEST_ASSERT_NULL(firmware.handleLine(""));
}

void test_missing_hardware_is_reported_not_hidden() {
  FakePwm pwm;
  pwm.begin_result = false;
  ServoFirmware firmware(pwm);
  const char* boot = firmware.begin();
  TEST_ASSERT_EQUAL_INT(0, std::strncmp(boot, "ERR PCA9685 not found", 21));
  TEST_ASSERT_FALSE(firmware.hardwareOk());
  // No "AVA_SERVO" reply, so the ROS driver's handshake fails instead of driving nothing.
  TEST_ASSERT_EQUAL_INT(0, std::strncmp(firmware.handleLine("?"), "ERR PCA9685", 11));
  TEST_ASSERT_EQUAL_INT(0, std::strncmp(firmware.handleLine("P 1500 0 0 0 0 0"), "ERR PCA9685", 11));
  TEST_ASSERT_EQUAL(0, pwm.writes);
}

void test_line_too_long_reply() {
  FakePwm pwm;
  ServoFirmware firmware(pwm);
  TEST_ASSERT_EQUAL_STRING("ERR line too long (max 64 characters)", firmware.lineTooLong());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_switches_all_channels_off_and_reports_ready);
  RUN_TEST(test_query_reply);
  RUN_TEST(test_pulses_reach_the_right_channels_silently);
  RUN_TEST(test_disable_switches_everything_off);
  RUN_TEST(test_invalid_line_replies_err_and_keeps_pulses);
  RUN_TEST(test_empty_line_is_ignored);
  RUN_TEST(test_missing_hardware_is_reported_not_hidden);
  RUN_TEST(test_line_too_long_reply);
  return UNITY_END();
}
