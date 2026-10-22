// ros2_control hardware plugin for the Ava arm: hobby PWM servos on a PCA9685 board driven by
// an ESP32 that talks to the PC over USB serial. Serial protocol v1 (ASCII, '\n'-terminated,
// 115200 8N1): "?" -> "AVA_SERVO 1 6"; "P u0 .. u5" sets the pulse (us, 0 = off) of channels
// 0..5; "D" turns all channels off; malformed lines -> "ERR <reason>"; boot -> "AVA_SERVO 1 6 READY".
// Hobby servos give no feedback, so the reported joint state is the last command sent. The arm
// therefore starts at its rest pose on activation and is parked there (smoothly) on deactivation.

#ifndef AVA_HARDWARE__AVA_SERVO_SYSTEM_HPP_
#define AVA_HARDWARE__AVA_SERVO_SYSTEM_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ava_hardware
{

class AvaServoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AvaServoSystem)

  /// Number of PCA9685 channels addressed by every `P` line.
  static constexpr std::size_t kChannelCount = 6;

  AvaServoSystem() = default;
  ~AvaServoSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::size_t channel = 0;
    double zero_us = 1500.0;
    double us_per_rad = 0.0;
    int direction = 1;
    int min_us = 500;
    int max_us = 2500;
    double max_velocity = 0.0;
    double min_position = 0.0;
    double max_position = 0.0;
    double rest_position = 0.0;  // start pose on activation, park pose on deactivation

    // Last position sent to the servo (= reported state) and its value at the previous read().
    double position = 0.0;
    double prev_position = 0.0;

    hardware_interface::StateInterface::SharedPtr position_state;
    hardware_interface::StateInterface::SharedPtr velocity_state;
    hardware_interface::CommandInterface::SharedPtr position_command;
  };

  /// Joint angle (rad) -> pulse width (us), not clamped.
  static double position_to_us(const Joint & joint, double position);
  /// Joint angle (rad) -> pulse width (us), rounded and clamped to [min_us, max_us].
  static int position_to_pulse(const Joint & joint, double position);

  bool open_port();
  void close_port();
  bool handshake();
  bool write_line(const char * line, std::size_t length);
  /// Fills pulses_ from the current joint positions (unused channels = 0 = off).
  void compute_pulses();
  /// Sends pulses_ as a `P` line and records them as sent.
  bool send_pulses();
  /// Sends `D` (all channels off).
  bool send_release();
  /// Reads every byte currently available without blocking and dispatches complete lines.
  bool drain_input();
  void handle_line(const std::string & line);
  /// Moves every joint smoothly to its rest position over park_duration_s_ (blocking).
  bool park();

  std::vector<Joint> joints_;

  std::string serial_port_;
  int baud_rate_ = 115200;
  int handshake_timeout_ms_ = 4000;
  bool release_on_deactivate_ = false;
  double park_duration_s_ = 2.0;

  int fd_ = -1;
  std::string rx_line_;
  bool rx_overflow_ = false;
  bool device_seen_ = false;
  int device_channels_ = 0;
  bool active_ = false;

  std::array<int, kChannelCount> pulses_{};
  std::array<int, kChannelCount> sent_pulses_{};
  bool sent_valid_ = false;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace ava_hardware

#endif  // AVA_HARDWARE__AVA_SERVO_SYSTEM_HPP_
