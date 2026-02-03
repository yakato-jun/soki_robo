#ifndef SOKI_HARDWARE__SOKI_SYSTEM_HPP_
#define SOKI_HARDWARE__SOKI_SYSTEM_HPP_

#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "soki_hardware/visibility_control.h"

#include <modbus/modbus.h>

namespace soki_hardware
{

class SokiSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SokiSystemHardware)

  SOKI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SOKI_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Modbus connection
  modbus_t * ctx_ = nullptr;
  std::string serial_port_;
  int baud_rate_ = 115200;
  int slave_id_ = 1;

  // Robot parameters
  double wheel_radius_ = 0.0335;
  int encoder_cpr_ = 440;

  // Previous encoder ticks for position tracking
  int32_t prev_encoder_[2] = {0, 0};
  bool first_read_ = true;

  // Accumulated wheel positions (rad)
  double hw_positions_[2] = {0.0, 0.0};

  // Unit conversion constants
  static constexpr double ACCEL_SCALE = 16.0 / 32768.0 * 9.80665;
  static constexpr double GYRO_SCALE  = 2000.0 / 32768.0 * M_PI / 180.0;

  // Modbus register addresses
  static constexpr int REG_START      = 0x00;
  static constexpr int REG_READ_COUNT = 0x36;  // 0x00..0x35 = 54 registers

  static constexpr int REG_ACCEL_X    = 0x10;
  static constexpr int REG_GYRO_X     = 0x13;
  static constexpr int REG_QUAT_W_LO  = 0x20;
  static constexpr int REG_ENC_L_HI   = 0x30;
  static constexpr int REG_SPEED_L    = 0x34;
  static constexpr int REG_CMD_SPEED_L = 0x40;

  // Helper: decode 2 registers as IEEE754 LE float
  static float regs_to_float(uint16_t lo, uint16_t hi);
  // Helper: decode 2 registers as int32 (HI<<16 | LO)
  static int32_t regs_to_int32(uint16_t hi, uint16_t lo);
};

}  // namespace soki_hardware

#endif  // SOKI_HARDWARE__SOKI_SYSTEM_HPP_
