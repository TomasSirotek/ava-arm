#include "ava_hardware/ava_servo_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <thread>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace ava_hardware
{
namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using ParamMap = std::unordered_map<std::string, std::string>;

constexpr std::size_t kMaxLineLength = 96;   // protocol v1 line limit
constexpr int kDeviceMinUs = 500;            // the firmware clamps non-zero pulses to this range
constexpr int kDeviceMaxUs = 2500;
constexpr int kQueryIntervalMs = 250;        // handshake: resend "?" this often
constexpr int kWriteTimeoutMs = 50;          // give up on a line the UART will not take
constexpr int kThrottleMs = 2000;            // ERR log throttle
constexpr double kMaxParkDurationS = 4.0;    // ros2 launch escalates to SIGTERM ~5 s after Ctrl+C
constexpr std::chrono::milliseconds kParkStep{20};  // one P line per 20 ms (50 Hz, like write())

rclcpp::Logger logger()
{
  static const rclcpp::Logger instance = rclcpp::get_logger("AvaServoSystem");
  return instance;
}

std::string trim(const std::string & text)
{
  const auto first = std::find_if_not(
    text.begin(), text.end(), [](unsigned char c) {return std::isspace(c);});
  const auto last = std::find_if_not(
    text.rbegin(), text.rend(), [](unsigned char c) {return std::isspace(c);}).base();
  return first < last ? std::string(first, last) : std::string();
}

bool parse_double(const std::string & text, double & out)
{
  const std::string value = trim(text);
  try {
    std::size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(parsed)) {
      return false;
    }
    out = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_int(const std::string & text, int & out)
{
  const std::string value = trim(text);
  try {
    std::size_t used = 0;
    const long parsed = std::stol(value, &used);
    if (used != value.size() || parsed < INT_MIN || parsed > INT_MAX) {
      return false;
    }
    out = static_cast<int>(parsed);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool require_double(
  const std::string & joint, const ParamMap & params, const char * key, double & out)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    RCLCPP_ERROR(logger(), "Joint '%s': missing parameter '%s'.", joint.c_str(), key);
    return false;
  }
  if (!parse_double(it->second, out)) {
    RCLCPP_ERROR(
      logger(), "Joint '%s': parameter '%s' = '%s' is not a number.", joint.c_str(), key,
      it->second.c_str());
    return false;
  }
  return true;
}

bool require_int(
  const std::string & joint, const ParamMap & params, const char * key, int & out)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    RCLCPP_ERROR(logger(), "Joint '%s': missing parameter '%s'.", joint.c_str(), key);
    return false;
  }
  if (!parse_int(it->second, out)) {
    RCLCPP_ERROR(
      logger(), "Joint '%s': parameter '%s' = '%s' is not an integer.", joint.c_str(), key,
      it->second.c_str());
    return false;
  }
  return true;
}

bool baud_to_speed(int baud, speed_t & speed)
{
  switch (baud) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    case 460800: speed = B460800; return true;
    case 921600: speed = B921600; return true;
    default: return false;
  }
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & info) {return info.name == name;});
}

}  // namespace

AvaServoSystem::~AvaServoSystem()
{
  close_port();
}

double AvaServoSystem::position_to_us(const Joint & joint, double position)
{
  return joint.zero_us + joint.direction * position * joint.us_per_rad;
}

int AvaServoSystem::position_to_pulse(const Joint & joint, double position)
{
  const double us = std::round(position_to_us(joint, position));
  return static_cast<int>(
    std::clamp(us, static_cast<double>(joint.min_us), static_cast<double>(joint.max_us)));
}

CallbackReturn AvaServoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ---- <hardware> parameters ----
  const ParamMap & hw = info_.hardware_parameters;

  const auto port_it = hw.find("serial_port");
  serial_port_ = port_it == hw.end() ? std::string() : trim(port_it->second);
  if (serial_port_.empty()) {
    RCLCPP_ERROR(logger(), "Hardware parameter 'serial_port' is missing or empty.");
    return CallbackReturn::ERROR;
  }

  baud_rate_ = 115200;
  if (const auto it = hw.find("baud_rate"); it != hw.end()) {
    if (!parse_int(it->second, baud_rate_)) {
      RCLCPP_ERROR(logger(), "Hardware parameter 'baud_rate' = '%s' is not an integer.",
        it->second.c_str());
      return CallbackReturn::ERROR;
    }
  }
  speed_t speed;
  if (!baud_to_speed(baud_rate_, speed)) {
    RCLCPP_ERROR(
      logger(), "Unsupported baud_rate %d (use 9600..921600; the firmware runs at 115200).",
      baud_rate_);
    return CallbackReturn::ERROR;
  }

  handshake_timeout_ms_ = 4000;
  if (const auto it = hw.find("handshake_timeout_ms"); it != hw.end()) {
    if (!parse_int(it->second, handshake_timeout_ms_) || handshake_timeout_ms_ <= 0) {
      RCLCPP_ERROR(
        logger(), "Hardware parameter 'handshake_timeout_ms' = '%s' must be a positive integer.",
        it->second.c_str());
      return CallbackReturn::ERROR;
    }
  }

  release_on_deactivate_ = false;
  if (const auto it = hw.find("release_on_deactivate"); it != hw.end()) {
    std::string value = trim(it->second);
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (value == "true") {
      release_on_deactivate_ = true;
    } else if (value != "false") {
      RCLCPP_ERROR(
        logger(), "Hardware parameter 'release_on_deactivate' = '%s' must be 'true' or 'false'.",
        it->second.c_str());
      return CallbackReturn::ERROR;
    }
  }

  park_duration_s_ = 2.0;
  if (const auto it = hw.find("park_duration_s"); it != hw.end()) {
    if (!parse_double(it->second, park_duration_s_) || park_duration_s_ < 0.0 ||
      park_duration_s_ > kMaxParkDurationS)
    {
      RCLCPP_ERROR(
        logger(), "Hardware parameter 'park_duration_s' = '%s' must be 0..%.1f (0 = no parking).",
        it->second.c_str(), kMaxParkDurationS);
      return CallbackReturn::ERROR;
    }
  }

  // ---- joints ----
  if (info_.joints.empty() || info_.joints.size() > kChannelCount) {
    RCLCPP_ERROR(
      logger(), "Expected 1..%zu joints, got %zu.", kChannelCount, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  std::array<bool, kChannelCount> channel_used{};

  for (const hardware_interface::ComponentInfo & info : info_.joints) {
    Joint joint;
    joint.name = info.name;
    const char * name = joint.name.c_str();

    if (info.command_interfaces.size() != 1 ||
      info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger(), "Joint '%s': needs exactly one command interface, '%s'.", name,
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (info.state_interfaces.size() != 2 ||
      !has_interface(info.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(info.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        logger(),
        "Joint '%s': state interfaces must be exactly '%s' and '%s' (hobby servos have no "
        "effort feedback).", name, hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    const ParamMap & p = info.parameters;
    int channel = 0;
    double direction = 0.0;
    if (!require_int(joint.name, p, "channel", channel) ||
      !require_double(joint.name, p, "zero_us", joint.zero_us) ||
      !require_double(joint.name, p, "us_per_rad", joint.us_per_rad) ||
      !require_double(joint.name, p, "direction", direction) ||
      !require_int(joint.name, p, "min_us", joint.min_us) ||
      !require_int(joint.name, p, "max_us", joint.max_us) ||
      !require_double(joint.name, p, "max_velocity", joint.max_velocity) ||
      !require_double(joint.name, p, "min_position", joint.min_position) ||
      !require_double(joint.name, p, "max_position", joint.max_position) ||
      !require_double(joint.name, p, "rest_position", joint.rest_position))
    {
      return CallbackReturn::ERROR;
    }

    if (channel < 0 || channel >= static_cast<int>(kChannelCount)) {
      RCLCPP_ERROR(
        logger(), "Joint '%s': channel %d is outside 0..%zu.", name, channel, kChannelCount - 1);
      return CallbackReturn::ERROR;
    }
    if (channel_used[static_cast<std::size_t>(channel)]) {
      RCLCPP_ERROR(logger(), "Joint '%s': channel %d is already used.", name, channel);
      return CallbackReturn::ERROR;
    }
    channel_used[static_cast<std::size_t>(channel)] = true;
    joint.channel = static_cast<std::size_t>(channel);

    if (direction == 1.0) {
      joint.direction = 1;
    } else if (direction == -1.0) {
      joint.direction = -1;
    } else {
      RCLCPP_ERROR(logger(), "Joint '%s': direction must be 1 or -1.", name);
      return CallbackReturn::ERROR;
    }
    if (joint.us_per_rad <= 0.0) {
      RCLCPP_ERROR(logger(), "Joint '%s': us_per_rad must be > 0.", name);
      return CallbackReturn::ERROR;
    }
    if (joint.min_us < kDeviceMinUs || joint.max_us > kDeviceMaxUs ||
      joint.min_us >= joint.max_us)
    {
      RCLCPP_ERROR(
        logger(), "Joint '%s': need %d <= min_us < max_us <= %d, got min_us=%d max_us=%d.", name,
        kDeviceMinUs, kDeviceMaxUs, joint.min_us, joint.max_us);
      return CallbackReturn::ERROR;
    }
    if (joint.max_velocity <= 0.0) {
      RCLCPP_ERROR(logger(), "Joint '%s': max_velocity must be > 0.", name);
      return CallbackReturn::ERROR;
    }
    if (joint.min_position >= joint.max_position) {
      RCLCPP_ERROR(logger(), "Joint '%s': min_position must be < max_position.", name);
      return CallbackReturn::ERROR;
    }
    if (joint.rest_position < joint.min_position || joint.rest_position > joint.max_position) {
      RCLCPP_ERROR(
        logger(), "Joint '%s': rest_position %.3f rad is outside [%.3f, %.3f].", name,
        joint.rest_position, joint.min_position, joint.max_position);
      return CallbackReturn::ERROR;
    }

    const double us_at_min = position_to_us(joint, joint.min_position);
    const double us_at_max = position_to_us(joint, joint.max_position);
    const double us_low = std::min(us_at_min, us_at_max);
    const double us_high = std::max(us_at_min, us_at_max);
    // Pulses are sent as whole microseconds, so ignore rounding below half a microsecond.
    if (us_low < joint.min_us - 0.5 || us_high > joint.max_us + 0.5) {
      RCLCPP_WARN(
        logger(),
        "Joint '%s': position range [%.3f, %.3f] rad maps to [%.0f, %.0f] us, outside the pulse "
        "clamp [%d, %d] us. The servo stops at the clamp while the reported state keeps moving; "
        "check the calibration (center_us, direction, us_per_rad).",
        name, joint.min_position, joint.max_position, us_low, us_high, joint.min_us, joint.max_us);
    }

    joints_.push_back(joint);
  }

  RCLCPP_INFO(
    logger(), "Initialized %zu servo joints on %s @ %d baud.", joints_.size(),
    serial_port_.c_str(), baud_rate_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    for (Joint & joint : joints_) {
      joint.position_state =
        get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION);
      joint.velocity_state =
        get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
      joint.position_command =
        get_command_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Interface lookup failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (!open_port()) {
    return CallbackReturn::ERROR;
  }
  if (!handshake()) {
    close_port();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_port();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  active_ = false;
  close_port();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_error(const rclcpp_lifecycle::State &)
{
  active_ = false;
  close_port();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (fd_ < 0) {
    RCLCPP_ERROR(logger(), "Cannot activate: serial port %s is not open.", serial_port_.c_str());
    return CallbackReturn::ERROR;
  }

  // No feedback exists, so the plugin cannot start from where the arm happens to be: it commands
  // the rest pose and the servos go there at full speed. With the arm resting in that pose when
  // powered off (it was parked there), this first command barely moves it.
  for (Joint & joint : joints_) {
    joint.position = joint.rest_position;
    joint.prev_position = joint.rest_position;
    (void)set_state(joint.position_state, joint.rest_position, true);
    (void)set_state(joint.velocity_state, 0.0, true);
    (void)set_command(joint.position_command, joint.rest_position, true);
  }

  compute_pulses();
  if (!send_pulses()) {
    return CallbackReturn::ERROR;
  }
  active_ = true;
  RCLCPP_INFO(logger(), "Activated: all joints commanded to their rest pose.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AvaServoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (!park()) {
    return CallbackReturn::ERROR;
  }
  if (!release_on_deactivate_) {
    RCLCPP_INFO(logger(), "Deactivated: parked at the rest pose, servos keep holding it.");
    return CallbackReturn::SUCCESS;
  }
  if (!send_release()) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "Deactivated: parked at the rest pose, then all servos released (limp).");
  return CallbackReturn::SUCCESS;
}

bool AvaServoSystem::park()
{
  if (park_duration_s_ <= 0.0) {
    return true;
  }
  std::vector<double> start;
  start.reserve(joints_.size());
  bool at_rest = true;
  for (const Joint & joint : joints_) {
    start.push_back(joint.position);
    at_rest = at_rest && std::abs(joint.position - joint.rest_position) < 1e-4;
  }
  if (at_rest) {
    return true;
  }

  // Cosine profile: slow at both ends, all joints start and arrive together.
  const auto steps = std::max<long>(1, std::lround(park_duration_s_ * 1000.0 / kParkStep.count()));
  for (long step = 1; step <= steps; ++step) {
    const double k = 0.5 - 0.5 * std::cos(M_PI * static_cast<double>(step) / static_cast<double>(steps));
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      joints_[i].position = start[i] + (joints_[i].rest_position - start[i]) * k;
    }
    compute_pulses();
    if (!(sent_valid_ && pulses_ == sent_pulses_) && !send_pulses()) {
      return false;
    }
    std::this_thread::sleep_for(kParkStep);
  }
  for (Joint & joint : joints_) {
    joint.prev_position = joint.position;
    (void)set_state(joint.position_state, joint.position, false);
    (void)set_state(joint.velocity_state, 0.0, false);
  }
  return true;
}

return_type AvaServoSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!drain_input()) {
    return return_type::ERROR;
  }

  const double dt = period.seconds();
  for (Joint & joint : joints_) {
    const double velocity = dt > 0.0 ? (joint.position - joint.prev_position) / dt : 0.0;
    joint.prev_position = joint.position;
    (void)set_state(joint.position_state, joint.position, false);
    (void)set_state(joint.velocity_state, velocity, false);
  }
  return return_type::OK;
}

return_type AvaServoSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (Joint & joint : joints_) {
    double target = joint.position;
    if (!get_command(joint.position_command, target, false) || std::isnan(target)) {
      continue;  // no (valid) command this cycle: hold
    }
    target = std::clamp(target, joint.min_position, joint.max_position);
    const double max_step = dt > 0.0 ? joint.max_velocity * dt : 0.0;
    joint.position += std::clamp(target - joint.position, -max_step, max_step);
  }

  compute_pulses();
  if (sent_valid_ && pulses_ == sent_pulses_) {
    return return_type::OK;
  }
  return send_pulses() ? return_type::OK : return_type::ERROR;
}

bool AvaServoSystem::open_port()
{
  close_port();

  speed_t speed;
  if (!baud_to_speed(baud_rate_, speed)) {
    RCLCPP_ERROR(logger(), "Unsupported baud_rate %d.", baud_rate_);
    return false;
  }

  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      logger(), "Cannot open serial port %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    RCLCPP_ERROR(
      logger(), "tcgetattr on %s failed: %s (not a serial port?)", serial_port_.c_str(),
      std::strerror(errno));
    close_port();
    return false;
  }
  ::cfmakeraw(&tio);  // 8 data bits, no parity, no echo, no line processing
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | CRTSCTS);  // 1 stop bit, no flow control
  // With O_NONBLOCK, VMIN=1 makes read() return EAGAIN when idle and 0 only on hang-up.
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0 ||
    ::tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    RCLCPP_ERROR(
      logger(), "Configuring %s at %d baud failed: %s", serial_port_.c_str(), baud_rate_,
      std::strerror(errno));
    close_port();
    return false;
  }
  ::tcflush(fd_, TCIOFLUSH);

  rx_line_.clear();
  rx_line_.reserve(kMaxLineLength);
  rx_overflow_ = false;
  device_seen_ = false;
  device_channels_ = 0;
  sent_valid_ = false;
  return true;
}

void AvaServoSystem::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  device_seen_ = false;
  sent_valid_ = false;
}

bool AvaServoSystem::handshake()
{
  using SteadyClock = std::chrono::steady_clock;
  // The ESP32 resets when the port opens (DTR/RTS auto-reset), so keep asking until the
  // firmware has booted: it answers "?" with "AVA_SERVO 1 6" and prints
  // "AVA_SERVO 1 6 READY" once at boot; either line completes the handshake.
  const auto start = SteadyClock::now();
  const auto deadline = start + std::chrono::milliseconds(handshake_timeout_ms_);
  auto next_query = start;
  static constexpr char kQuery[] = "?\n";

  while (true) {
    auto now = SteadyClock::now();
    if (now >= next_query) {
      if (!write_line(kQuery, sizeof(kQuery) - 1)) {
        return false;
      }
      next_query = now + std::chrono::milliseconds(kQueryIntervalMs);
    }
    if (!drain_input()) {
      return false;
    }
    if (device_seen_) {
      break;
    }
    now = SteadyClock::now();
    if (now >= deadline) {
      RCLCPP_ERROR(
        logger(),
        "No 'AVA_SERVO 1' reply on %s within %d ms. Is the ESP32 connected to this port and "
        "flashed with the Ava servo firmware?", serial_port_.c_str(), handshake_timeout_ms_);
      return false;
    }
    const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::min(deadline, next_query) - now);
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, std::max<int>(1, static_cast<int>(wait.count()))) < 0 &&
      errno != EINTR)
    {
      RCLCPP_ERROR(logger(), "poll on %s failed: %s", serial_port_.c_str(), std::strerror(errno));
      return false;
    }
  }

  if (device_channels_ < static_cast<int>(kChannelCount)) {
    RCLCPP_ERROR(
      logger(), "Servo board on %s reports %d channels; %zu are required.",
      serial_port_.c_str(), device_channels_, kChannelCount);
    return false;
  }
  RCLCPP_INFO(
    logger(), "Servo board on %s answered (protocol 1, %d channels). Servos stay off until "
    "activation.", serial_port_.c_str(), device_channels_);
  return true;
}

bool AvaServoSystem::write_line(const char * line, std::size_t length)
{
  if (fd_ < 0) {
    return false;
  }
  using SteadyClock = std::chrono::steady_clock;
  const auto deadline = SteadyClock::now() + std::chrono::milliseconds(kWriteTimeoutMs);
  std::size_t done = 0;
  while (done < length) {
    const ssize_t n = ::write(fd_, line + done, length - done);
    if (n > 0) {
      done += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - SteadyClock::now()).count();
      if (left <= 0) {
        RCLCPP_ERROR(logger(), "Serial write to %s timed out.", serial_port_.c_str());
        return false;
      }
      pollfd pfd{fd_, POLLOUT, 0};
      ::poll(&pfd, 1, static_cast<int>(left));
      continue;
    }
    RCLCPP_ERROR(
      logger(), "Serial write to %s failed: %s", serial_port_.c_str(),
      n < 0 ? std::strerror(errno) : "no bytes written");
    return false;
  }
  return true;
}

void AvaServoSystem::compute_pulses()
{
  pulses_.fill(0);
  for (const Joint & joint : joints_) {
    pulses_[joint.channel] = position_to_pulse(joint, joint.position);
  }
}

bool AvaServoSystem::send_pulses()
{
  char line[64];
  const int length = std::snprintf(
    line, sizeof(line), "P %d %d %d %d %d %d\n", pulses_[0], pulses_[1], pulses_[2],
    pulses_[3], pulses_[4], pulses_[5]);
  if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(line)) {
    return false;
  }
  if (!write_line(line, static_cast<std::size_t>(length))) {
    return false;
  }
  sent_pulses_ = pulses_;
  sent_valid_ = true;
  return true;
}

bool AvaServoSystem::send_release()
{
  static constexpr char kRelease[] = "D\n";
  if (!write_line(kRelease, sizeof(kRelease) - 1)) {
    return false;
  }
  sent_valid_ = false;  // the next P must go out even if the pulses are unchanged
  return true;
}

bool AvaServoSystem::drain_input()
{
  if (fd_ < 0) {
    return false;
  }
  char buffer[256];
  while (true) {
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        const char c = buffer[i];
        if (c == '\n') {
          if (!rx_overflow_) {
            handle_line(rx_line_);
          }
          rx_line_.clear();
          rx_overflow_ = false;
        } else if (c == '\r') {
          continue;
        } else if (rx_line_.size() < kMaxLineLength) {
          rx_line_.push_back(c);
        } else {
          rx_overflow_ = true;  // drop over-long lines (boot ROM noise at another baud rate)
        }
      }
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    RCLCPP_ERROR(
      logger(), "Serial port %s: %s", serial_port_.c_str(),
      n == 0 ? "hung up (device unplugged?)" : std::strerror(errno));
    return false;
  }
}

void AvaServoSystem::handle_line(const std::string & line)
{
  if (line.empty()) {
    return;
  }
  if (line.rfind("ERR", 0) == 0) {
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock_, kThrottleMs, "Servo board on %s: %s", serial_port_.c_str(),
      line.c_str());
    return;
  }

  int version = 0;
  int channels = 0;
  if (std::sscanf(line.c_str(), "AVA_SERVO %d %d", &version, &channels) == 2) {
    if (version != 1) {
      RCLCPP_WARN_THROTTLE(
        logger(), steady_clock_, kThrottleMs,
        "Servo board on %s speaks protocol %d; this plugin needs 1: '%s'", serial_port_.c_str(),
        version, line.c_str());
      return;
    }
    const bool boot_banner =
      line.size() >= 5 && line.compare(line.size() - 5, 5, "READY") == 0;
    if (!device_seen_) {
      device_seen_ = true;
      device_channels_ = channels;
    } else if (boot_banner && active_) {
      // The board rebooted (USB glitch / brown-out) and switched every channel off.
      RCLCPP_WARN(
        logger(), "Servo board on %s restarted; re-sending the last pulses.",
        serial_port_.c_str());
      sent_valid_ = false;
    }
    return;
  }

  RCLCPP_DEBUG(logger(), "Ignoring serial line: '%s'", line.c_str());
}

}  // namespace ava_hardware

PLUGINLIB_EXPORT_CLASS(ava_hardware::AvaServoSystem, hardware_interface::SystemInterface)
