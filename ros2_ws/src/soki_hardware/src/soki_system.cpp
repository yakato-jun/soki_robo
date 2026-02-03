#include "soki_hardware/soki_system.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace soki_hardware
{

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

float SokiSystemHardware::regs_to_float(uint16_t lo, uint16_t hi)
{
  // Modbus RTU transmits each register big-endian on the wire,
  // but libmodbus stores them as native uint16_t.
  // The MCU stores IEEE754 LE: LO register = low 16 bits, HI register = high 16 bits.
  uint32_t raw = (static_cast<uint32_t>(hi) << 16) | static_cast<uint32_t>(lo);
  float val;
  std::memcpy(&val, &raw, sizeof(float));
  return val;
}

int32_t SokiSystemHardware::regs_to_int32(uint16_t hi, uint16_t lo)
{
  uint32_t raw = (static_cast<uint32_t>(hi) << 16) | static_cast<uint32_t>(lo);
  return static_cast<int32_t>(raw);
}

// ------------------------------------------------------------
// Lifecycle: on_init
// ------------------------------------------------------------

hardware_interface::CallbackReturn SokiSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <param> tags
  serial_port_ = info_.hardware_parameters.at("serial_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  slave_id_ = std::stoi(info_.hardware_parameters.at("slave_id"));
  wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
  encoder_cpr_ = std::stoi(info_.hardware_parameters.at("encoder_cpr"));

  // Validate joint configuration
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
      "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
        "Joint '%s' must have exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
        "Joint '%s' must have exactly 2 state interfaces (position, velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Validate sensor configuration
  if (info_.sensors.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
      "Expected 1 sensor (IMU), got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------
// Lifecycle: on_configure — open Modbus connection
// ------------------------------------------------------------

hardware_interface::CallbackReturn SokiSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SokiSystemHardware"),
    "Configuring Modbus RTU: %s @ %d baud, slave %d",
    serial_port_.c_str(), baud_rate_, slave_id_);

  ctx_ = modbus_new_rtu(serial_port_.c_str(), baud_rate_, 'N', 8, 1);
  if (ctx_ == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
      "Failed to create Modbus context: %s", modbus_strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  modbus_set_slave(ctx_, slave_id_);

  // Response timeout: 500ms (Modbus cycle ~34ms, generous margin)
  modbus_set_response_timeout(ctx_, 0, 500000);

  if (modbus_connect(ctx_) == -1) {
    RCLCPP_FATAL(rclcpp::get_logger("SokiSystemHardware"),
      "Modbus connect failed: %s", modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("SokiSystemHardware"), "Modbus connected");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------
// Lifecycle: on_cleanup — close Modbus connection
// ------------------------------------------------------------

hardware_interface::CallbackReturn SokiSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (ctx_ != nullptr) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
  RCLCPP_INFO(rclcpp::get_logger("SokiSystemHardware"), "Modbus connection closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------
// Lifecycle: on_activate
// ------------------------------------------------------------

hardware_interface::CallbackReturn SokiSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset state
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;
  first_read_ = true;

  // Send zero velocity to ensure motors are stopped
  uint16_t zero[2] = {0, 0};
  modbus_write_registers(ctx_, REG_CMD_SPEED_L, 2, zero);

  RCLCPP_INFO(rclcpp::get_logger("SokiSystemHardware"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------
// Lifecycle: on_deactivate — stop motors
// ------------------------------------------------------------

hardware_interface::CallbackReturn SokiSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  uint16_t zero[2] = {0, 0};
  modbus_write_registers(ctx_, REG_CMD_SPEED_L, 2, zero);

  RCLCPP_INFO(rclcpp::get_logger("SokiSystemHardware"), "Deactivated — motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------
// export_state_interfaces
// ------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
SokiSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint states: position and velocity for left (0) and right (1)
  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  // IMU sensor states
  const auto & sensor_name = info_.sensors[0].name;
  state_interfaces.emplace_back(sensor_name, "orientation.x",          &imu_orientation_[0]);
  state_interfaces.emplace_back(sensor_name, "orientation.y",          &imu_orientation_[1]);
  state_interfaces.emplace_back(sensor_name, "orientation.z",          &imu_orientation_[2]);
  state_interfaces.emplace_back(sensor_name, "orientation.w",          &imu_orientation_[3]);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.x",    &imu_angular_velocity_[0]);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.y",    &imu_angular_velocity_[1]);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.z",    &imu_angular_velocity_[2]);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.x", &imu_linear_acceleration_[0]);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.y", &imu_linear_acceleration_[1]);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.z", &imu_linear_acceleration_[2]);

  return state_interfaces;
}

// ------------------------------------------------------------
// export_command_interfaces
// ------------------------------------------------------------

std::vector<hardware_interface::CommandInterface>
SokiSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

// ------------------------------------------------------------
// read() — FC03: bulk read registers 0x00..0x35
// ------------------------------------------------------------

hardware_interface::return_type SokiSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint16_t regs[REG_READ_COUNT];

  int rc = modbus_read_registers(ctx_, REG_START, REG_READ_COUNT, regs);
  if (rc == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("SokiSystemHardware"),
      "Modbus read failed: %s", modbus_strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  // --- IMU: accelerometer (0x10..0x12) ---
  imu_linear_acceleration_[0] = static_cast<int16_t>(regs[REG_ACCEL_X])     * ACCEL_SCALE;
  imu_linear_acceleration_[1] = static_cast<int16_t>(regs[REG_ACCEL_X + 1]) * ACCEL_SCALE;
  imu_linear_acceleration_[2] = static_cast<int16_t>(regs[REG_ACCEL_X + 2]) * ACCEL_SCALE;

  // --- IMU: gyroscope (0x13..0x15) ---
  imu_angular_velocity_[0] = static_cast<int16_t>(regs[REG_GYRO_X])     * GYRO_SCALE;
  imu_angular_velocity_[1] = static_cast<int16_t>(regs[REG_GYRO_X + 1]) * GYRO_SCALE;
  imu_angular_velocity_[2] = static_cast<int16_t>(regs[REG_GYRO_X + 2]) * GYRO_SCALE;

  // --- IMU: quaternion (0x20..0x27, 2 regs per float, LE word order) ---
  float qw = regs_to_float(regs[REG_QUAT_W_LO], regs[REG_QUAT_W_LO + 1]);
  float qx = regs_to_float(regs[REG_QUAT_W_LO + 2], regs[REG_QUAT_W_LO + 3]);
  float qy = regs_to_float(regs[REG_QUAT_W_LO + 4], regs[REG_QUAT_W_LO + 5]);
  float qz = regs_to_float(regs[REG_QUAT_W_LO + 6], regs[REG_QUAT_W_LO + 7]);
  imu_orientation_[0] = qx;
  imu_orientation_[1] = qy;
  imu_orientation_[2] = qz;
  imu_orientation_[3] = qw;

  // --- Encoders (0x30..0x33): 2 regs per encoder, HI<<16 | LO → int32 ---
  int32_t enc_l = regs_to_int32(regs[REG_ENC_L_HI], regs[REG_ENC_L_HI + 1]);
  int32_t enc_r = regs_to_int32(regs[REG_ENC_L_HI + 2], regs[REG_ENC_L_HI + 3]);

  if (first_read_) {
    prev_encoder_[0] = enc_l;
    prev_encoder_[1] = enc_r;
    first_read_ = false;
  }

  // Encoder ticks → radians
  const double rad_per_tick = (2.0 * M_PI) / static_cast<double>(encoder_cpr_);

  int32_t delta_l = enc_l - prev_encoder_[0];
  int32_t delta_r = enc_r - prev_encoder_[1];
  hw_positions_[0] += delta_l * rad_per_tick;
  hw_positions_[1] += delta_r * rad_per_tick;
  prev_encoder_[0] = enc_l;
  prev_encoder_[1] = enc_r;

  // --- Speed feedback (0x34..0x35): int16 in mm/s → rad/s ---
  int16_t speed_l = static_cast<int16_t>(regs[REG_SPEED_L]);
  int16_t speed_r = static_cast<int16_t>(regs[REG_SPEED_L + 1]);
  hw_velocities_[0] = (static_cast<double>(speed_l) / 1000.0) / wheel_radius_;
  hw_velocities_[1] = (static_cast<double>(speed_r) / 1000.0) / wheel_radius_;

  return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------
// write() — FC16: write 0x40..0x41 (speed command in mm/s)
// ------------------------------------------------------------

hardware_interface::return_type SokiSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert rad/s → mm/s: v_mm = omega * wheel_radius * 1000
  double vel_l_mm = hw_commands_[0] * wheel_radius_ * 1000.0;
  double vel_r_mm = hw_commands_[1] * wheel_radius_ * 1000.0;

  // Clamp to int16 range
  auto clamp_i16 = [](double v) -> int16_t {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::round(v));
  };

  int16_t cmd_l = clamp_i16(vel_l_mm);
  int16_t cmd_r = clamp_i16(vel_r_mm);

  uint16_t regs[2];
  regs[0] = static_cast<uint16_t>(cmd_l);
  regs[1] = static_cast<uint16_t>(cmd_r);

  int rc = modbus_write_registers(ctx_, REG_CMD_SPEED_L, 2, regs);
  if (rc == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("SokiSystemHardware"),
      "Modbus write failed: %s", modbus_strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace soki_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  soki_hardware::SokiSystemHardware,
  hardware_interface::SystemInterface)
