#include "kangaroo_hw_interface/kangaroo_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <chrono>
#include <thread>
#include <sstream>

namespace kangaroo_hw_interface
{

  constexpr double PI = 3.14159265358979323846;
  constexpr double GEAR_RATIO = 51.0;
  constexpr double PPR = 12.0; // Pulse Per Revolution
  constexpr double TPP = 4.0;  // Ticks Per Pulse


  KangarooSystem::KangarooSystem() : serial_(io_) {}

  KangarooSystem::~KangarooSystem()
  {
    if (serial_.is_open())
      serial_.close();
  }

  hardware_interface::CallbackReturn KangarooSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    serial_port_ = info.hardware_parameters.at("serial_port");
    baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));
    serial_.open(serial_port_);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));

    // Start both channels (1 and 2) on Kangaroo    
    write_kangaroo("1,start\r\n");
    write_kangaroo("2,start\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> KangarooSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("joint_wheel_fr", hardware_interface::HW_IF_POSITION, &position_[0]);
    state_interfaces.emplace_back("joint_wheel_fr", hardware_interface::HW_IF_VELOCITY, &velocity_[0]);
    state_interfaces.emplace_back("joint_wheel_fl", hardware_interface::HW_IF_POSITION, &position_[1]);
    state_interfaces.emplace_back("joint_wheel_fl", hardware_interface::HW_IF_VELOCITY, &velocity_[1]);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> KangarooSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("joint_wheel_fr", hardware_interface::HW_IF_VELOCITY, &velocity_command_[0]);
    command_interfaces.emplace_back("joint_wheel_fl", hardware_interface::HW_IF_VELOCITY, &velocity_command_[1]);
    return command_interfaces;
  }

  hardware_interface::CallbackReturn KangarooSystem::on_activate(const rclcpp_lifecycle::State &)
  {
    // ...
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KangarooSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // ...
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type KangarooSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // Get feedback from Kangaroo
    velocity_[0] = read_velocity(1);
    velocity_[1] = read_velocity(2);
    position_[0] = read_position(1);
    position_[1] = read_position(2);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type KangarooSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    send_command(1, velocity_command_[0]);
    send_command(2, velocity_command_[1]);
    return hardware_interface::return_type::OK;
  }

  void KangarooSystem::send_command(int channel, double velocity)
  {
    int kangaroo_speed = static_cast<int>(velocity * GEAR_RATIO * PPR  / 2 / PI); // scaling
    std::string cmd = std::to_string(channel) + ",s" + std::to_string(kangaroo_speed) + "\r\n";
    write_kangaroo(cmd);
  }

  double KangarooSystem::read_velocity(int channel)
  {
    std::string cmd = std::to_string(channel) + ",gets\r\n";
    write_kangaroo(cmd);
    std::string resp = read_kangaroo();
    // Parse response, e.g. "1,S1234"
    if (resp.size() > 3 && (resp[2] == 'S' || resp[2] == 's'))
    {
      try
      {
        return std::stoi(resp.substr(3)) / (GEAR_RATIO * PPR  / 2 / PI);
      }
      catch (...)
      {
      }
    }
    return 0.0;
  }

  double KangarooSystem::read_position(int channel)
  {
    std::string cmd = std::to_string(channel) + ",getp\r\n";
    write_kangaroo(cmd);
    std::string resp = read_kangaroo();
    // Parse response, e.g. "1,P12345"
    if (resp.size() > 3 && (resp[2] == 'P' || resp[2] == 'p'))
    {
      try
      {
        return std::stoi(resp.substr(3)) / (GEAR_RATIO * PPR  / 2 / PI);
      }
      catch (...)
      {
      }
    }
    return 0.0;
  }

  void KangarooSystem::write_kangaroo(const std::string &cmd)
  {
    boost::asio::write(serial_, boost::asio::buffer(cmd.c_str(), cmd.size()));
  }

  std::string KangarooSystem::read_kangaroo()
  {
    char c;
    std::string result;
    for (int i = 0; i < 32; ++i)
    {
      boost::asio::read(serial_, boost::asio::buffer(&c, 1));
      if (c == '\n')
        break;
      result += c;
    }
    return result;
  }

} // namespace kangaroo_hw_interface

PLUGINLIB_EXPORT_CLASS(kangaroo_hw_interface::KangarooSystem, hardware_interface::SystemInterface)