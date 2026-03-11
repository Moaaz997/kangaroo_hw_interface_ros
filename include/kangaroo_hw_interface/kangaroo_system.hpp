#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <string>

namespace kangaroo_hw_interface
{

class KangarooSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KangarooSystem)

  KangarooSystem();
  ~KangarooSystem();

  // Correct signatures
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // SERIAL
  std::string serial_port_;
  unsigned int baudrate_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;

  // JOINT STATES/COMMANDS
  double position_[2] = {0.0, 0.0};
  double velocity_[2] = {0.0, 0.0};
  double velocity_command_[2] = {0.0, 0.0};

  // HELPERS
  void send_command(int channel, double velocity);
  double read_velocity(int channel);
  double read_position(int channel);
  void write_kangaroo(const std::string &cmd);
  std::string read_kangaroo();
};

} // namespace kangaroo_hw_interface
