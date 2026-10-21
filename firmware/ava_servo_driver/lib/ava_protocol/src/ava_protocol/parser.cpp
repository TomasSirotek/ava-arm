#include "ava_protocol/parser.h"

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace ava {
namespace {

const char* skipSpaces(const char* text) {
  while (*text == ' ' || *text == '\t') {
    ++text;
  }
  return text;
}

Command invalid(const char* reason) {
  Command command;
  command.type = CommandType::Invalid;
  command.error = reason;
  return command;
}

Command parsePulses(const char* text) {
  Command command;
  command.type = CommandType::SetPulses;
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    text = skipSpaces(text);
    if (*text == '\0') {
      return invalid("P needs 6 values");
    }
    if (!std::isdigit(static_cast<unsigned char>(*text))) {
      return invalid("P values must be whole numbers >= 0");
    }
    char* end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (errno == ERANGE || (*end != ' ' && *end != '\t' && *end != '\0')) {
      return invalid("P values must be whole numbers >= 0");
    }
    command.pulses_us[channel] = clampPulse(value);
    text = end;
  }
  if (*skipSpaces(text) != '\0') {
    return invalid("P needs 6 values");
  }
  return command;
}

}  // namespace

uint16_t clampPulse(long pulse_us) {
  if (pulse_us == 0) {
    return 0;
  }
  if (pulse_us < kMinPulseUs) {
    return kMinPulseUs;
  }
  if (pulse_us > kMaxPulseUs) {
    return kMaxPulseUs;
  }
  return static_cast<uint16_t>(pulse_us);
}

Command parseCommand(const char* line) {
  const char* text = skipSpaces(line);
  if (*text == '\0') {
    return Command{};  // Empty: ignored, no reply
  }

  // The command word ends at the first space or the end of the line.
  const std::size_t word_length = std::strcspn(text, " \t");
  const char* rest = skipSpaces(text + word_length);

  if (word_length == 1 && text[0] == '?') {
    if (*rest != '\0') {
      return invalid("? takes no values");
    }
    Command command;
    command.type = CommandType::Query;
    return command;
  }
  if (word_length == 1 && text[0] == 'D') {
    if (*rest != '\0') {
      return invalid("D takes no values");
    }
    Command command;
    command.type = CommandType::Disable;
    return command;
  }
  if (word_length == 1 && text[0] == 'P') {
    return parsePulses(rest);
  }
  return invalid("unknown command");
}

}  // namespace ava
