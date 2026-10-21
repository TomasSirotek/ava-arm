// Unit tests for the protocol module (parser + line buffer), run on the PC: pio test -e native
#include <unity.h>

#include <string>

#include "ava_protocol/line_buffer.h"
#include "ava_protocol/parser.h"

using namespace ava;

namespace {

std::string feed(LineBuffer& buffer, const char* text) {
  std::string completed;
  for (const char* c = text; *c != '\0'; ++c) {
    if (buffer.push(*c)) {
      completed = buffer.line();
    }
  }
  return completed;
}

}  // namespace

// ---------- protocol ----------

void test_clamp_keeps_zero_and_limits_range() {
  TEST_ASSERT_EQUAL_UINT16(0, clampPulse(0));
  TEST_ASSERT_EQUAL_UINT16(500, clampPulse(1));
  TEST_ASSERT_EQUAL_UINT16(500, clampPulse(500));
  TEST_ASSERT_EQUAL_UINT16(1500, clampPulse(1500));
  TEST_ASSERT_EQUAL_UINT16(2500, clampPulse(2500));
  TEST_ASSERT_EQUAL_UINT16(2500, clampPulse(99999));
}

void test_parse_query_and_disable() {
  TEST_ASSERT_TRUE(parseCommand("?").type == CommandType::Query);
  TEST_ASSERT_TRUE(parseCommand("  D  ").type == CommandType::Disable);
  TEST_ASSERT_TRUE(parseCommand("").type == CommandType::Empty);
  TEST_ASSERT_TRUE(parseCommand("   ").type == CommandType::Empty);
}

void test_parse_six_pulses_in_channel_order_and_clamped() {
  const Command command = parseCommand("P 1500 0 3000 100 2499 501");
  TEST_ASSERT_TRUE(command.type == CommandType::SetPulses);
  const uint16_t expected[kChannelCount] = {1500, 0, 2500, 500, 2499, 501};
  TEST_ASSERT_EQUAL_UINT16_ARRAY(expected, command.pulses_us, kChannelCount);
}

void test_parse_rejects_wrong_value_count() {
  TEST_ASSERT_TRUE(parseCommand("P 1500").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("P").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("P 1 2 3 4 5 6 7").type == CommandType::Invalid);
  TEST_ASSERT_EQUAL_STRING("P needs 6 values", parseCommand("P 1500 1500").error);
}

void test_parse_rejects_non_numbers_and_negatives() {
  TEST_ASSERT_TRUE(parseCommand("P 1500 abc 0 0 0 0").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("P 1500 -5 0 0 0 0").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("P 15x0 0 0 0 0 0").type == CommandType::Invalid);
}

void test_parse_rejects_unknown_and_glued_commands() {
  TEST_ASSERT_EQUAL_STRING("unknown command", parseCommand("hello").error);
  TEST_ASSERT_TRUE(parseCommand("P1500 0 0 0 0 0").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("? 1").type == CommandType::Invalid);
  TEST_ASSERT_TRUE(parseCommand("d").type == CommandType::Invalid);  // commands are case-sensitive
}

// ---------- line buffer ----------

void test_line_buffer_completes_on_newline_and_ignores_cr() {
  LineBuffer buffer;
  TEST_ASSERT_EQUAL_STRING("P 1 2", feed(buffer, "P 1 2\r\n").c_str());
  TEST_ASSERT_FALSE(buffer.overflowed());
  TEST_ASSERT_EQUAL_STRING("?", feed(buffer, "?\n").c_str());  // next line starts clean
}

void test_line_buffer_flags_overflow_then_recovers() {
  LineBuffer buffer;
  std::string long_line(kMaxLineLength + 10, 'x');
  long_line += '\n';
  feed(buffer, long_line.c_str());
  TEST_ASSERT_TRUE(buffer.overflowed());
  TEST_ASSERT_EQUAL_STRING("D", feed(buffer, "D\n").c_str());
  TEST_ASSERT_FALSE(buffer.overflowed());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_clamp_keeps_zero_and_limits_range);
  RUN_TEST(test_parse_query_and_disable);
  RUN_TEST(test_parse_six_pulses_in_channel_order_and_clamped);
  RUN_TEST(test_parse_rejects_wrong_value_count);
  RUN_TEST(test_parse_rejects_non_numbers_and_negatives);
  RUN_TEST(test_parse_rejects_unknown_and_glued_commands);
  RUN_TEST(test_line_buffer_completes_on_newline_and_ignores_cr);
  RUN_TEST(test_line_buffer_flags_overflow_then_recovers);
  return UNITY_END();
}
