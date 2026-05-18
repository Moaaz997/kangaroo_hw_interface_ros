#include "kangaroo_hw_interface/kangaroo_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/system/error_code.hpp>
#include <chrono>
#include <istream>
#include <stdexcept>
#include <thread>

namespace kangaroo_hw_interface
{

  constexpr double PI = 3.14159265358979323846;
  constexpr double GEAR_RATIO = 51.0;
  constexpr double PPR = 12.0; // Pulse Per Revolution
  constexpr int READ_TIMEOUT_MS = 50;

  // Internal exception used to signal that a serial read timed out or was aborted.
  struct SerialFault : public std::runtime_error
  {
    using std::runtime_error::runtime_error;
  };

  KangarooSystem::KangarooSystem()
    : serial_(io_), deadline_timer_(io_) {}

  KangarooSystem::~KangarooSystem()
  {
    boost::system::error_code ec;
    if (serial_.is_open())
    {
      serial_.cancel(ec);
      const std::string s1 = "1,s0\r\n";
      const std::string s2 = "2,s0\r\n";
      boost::asio::write(serial_, boost::asio::buffer(s1), ec);
      boost::asio::write(serial_, boost::asio::buffer(s2), ec);
      serial_.close(ec);
    }
    deadline_timer_.cancel(ec);
  }

  hardware_interface::CallbackReturn KangarooSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    serial_port_ = info.hardware_parameters.at("serial_port");
    baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));

    auto it = info.hardware_parameters.find("watchdog_threshold");
    if (it != info.hardware_parameters.end())
      watchdog_threshold_ = std::stoi(it->second);

    try
    {
      serial_.open(serial_port_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    }
    catch (const boost::system::system_error &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("KangarooSystem"),
        "Failed to open serial port %s: %s", serial_port_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

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
    watchdog_cycles_since_write_ = 0;
    watchdog_active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn KangarooSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    try
    {
      write_kangaroo("1,s0\r\n");
      write_kangaroo("2,s0\r\n");
    }
    catch (const boost::system::system_error &e)
    {
      RCLCPP_WARN(rclcpp::get_logger("KangarooSystem"),
        "Failed to send stop on deactivate: %s", e.what());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type KangarooSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    if (watchdog_active_)
    {
      ++watchdog_cycles_since_write_;
      if (watchdog_cycles_since_write_ > watchdog_threshold_)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("KangarooSystem"),
          "Watchdog triggered: write() not called for %d cycles. Sending zero velocity.",
          watchdog_cycles_since_write_);
        write_kangaroo("1,s0\r\n");
        write_kangaroo("2,s0\r\n");
        watchdog_cycles_since_write_ = 0;
      }
    }

    try
    {
      velocity_[0] = read_velocity(1);
      velocity_[1] = read_velocity(2);
      position_[0] = read_position(1);
      position_[1] = read_position(2);
    }
    catch (const SerialFault &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("KangarooSystem"),
        "Serial fault during read: %s", e.what());
      velocity_command_[0] = 0.0;
      velocity_command_[1] = 0.0;
      ++serial_fault_count_;
      RCLCPP_WARN(rclcpp::get_logger("KangarooSystem"),
        "Returning ERROR — controller_manager will transition the hardware "
        "interface to ERROR state. serial_fault_count=%lu",
        static_cast<unsigned long>(serial_fault_count_));
      return hardware_interface::return_type::ERROR;
    }
    catch (const boost::system::system_error &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("KangarooSystem"),
        "Boost asio error during read: %s", e.what());
      velocity_command_[0] = 0.0;
      velocity_command_[1] = 0.0;
      ++serial_fault_count_;
      RCLCPP_WARN(rclcpp::get_logger("KangarooSystem"),
        "Returning ERROR — controller_manager will transition the hardware "
        "interface to ERROR state. serial_fault_count=%lu",
        static_cast<unsigned long>(serial_fault_count_));
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type KangarooSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    watchdog_cycles_since_write_ = 0;
    watchdog_active_ = true;
    try
    {
      send_command(1, velocity_command_[0]);
      send_command(2, velocity_command_[1]);
    }
    catch (const boost::system::system_error &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("KangarooSystem"),
        "Boost asio error during write: %s", e.what());
      velocity_command_[0] = 0.0;
      velocity_command_[1] = 0.0;
      ++serial_fault_count_;
      RCLCPP_WARN(rclcpp::get_logger("KangarooSystem"),
        "Returning ERROR — controller_manager will transition the hardware "
        "interface to ERROR state. serial_fault_count=%lu",
        static_cast<unsigned long>(serial_fault_count_));
      return hardware_interface::return_type::ERROR;
    }
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
    auto resp_opt = read_kangaroo();
    if (!resp_opt)
      throw SerialFault("read_velocity: timeout/aborted on channel " + std::to_string(channel));
    const std::string &resp = *resp_opt;
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
    auto resp_opt = read_kangaroo();
    if (!resp_opt)
      throw SerialFault("read_position: timeout/aborted on channel " + std::to_string(channel));
    const std::string &resp = *resp_opt;
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

  std::optional<std::string> KangarooSystem::read_kangaroo()
  {
    // Discard any leftover bytes from a prior aborted read so we line up on a fresh frame.
    read_buffer_.consume(read_buffer_.size());

    std::optional<std::string> result;

    deadline_timer_.expires_from_now(boost::posix_time::milliseconds(READ_TIMEOUT_MS));
    deadline_timer_.async_wait(
      [this](const boost::system::error_code &ec)
      {
        if (ec == boost::asio::error::operation_aborted) return;
        boost::system::error_code cancel_ec;
        serial_.cancel(cancel_ec);
      });

    boost::asio::async_read_until(serial_, read_buffer_, '\n',
      [this, &result](const boost::system::error_code &ec, std::size_t /*bytes*/)
      {
        boost::system::error_code timer_ec;
        deadline_timer_.cancel(timer_ec);
        if (ec) return;
        std::istream is(&read_buffer_);
        std::string line;
        std::getline(is, line);  // strips '\n'; '\r' (if present) remains in the line
        result = std::move(line);
      });

    io_.restart();
    io_.run();
    return result;
  }

} // namespace kangaroo_hw_interface

PLUGINLIB_EXPORT_CLASS(kangaroo_hw_interface::KangarooSystem, hardware_interface::SystemInterface)
