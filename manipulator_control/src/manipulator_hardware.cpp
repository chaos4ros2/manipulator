// 参考：https://github.com/rt-net/crane_plus/blob/use_new_ros2_control/crane_plus_control/src/crane_plus_hardware.cpp

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "manipulator_control/manipulator_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace manipulator_control
{

ManipulatorHardware::~ManipulatorHardware()
{
    driver_->close_port();
}

return_type ManipulatorHardware::configure(const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != return_type::OK) {
        return return_type::ERROR;
    }

    // Get parameters from URDF
    // Initialize member variables
    // info_の初期化はどこなにかは不明
    // Todo、urdfにパラメーターの用意 => port_nameの追加
    std::string port_name = info_.hardware_parameters["port_name"];
    const int frequency_hz = info_.hardware_parameters["frequency_hz"];
    const int address = info_.hardware_parameters["address"];
    timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);

    std::vector<uint8_t> servo_id_list;
    for (auto joint : info_.joints) {
        // Todo:urdfのパラメーターの用意：https://github.com/rt-net/crane_plus/blob/use_new_ros2_control/crane_plus_description/urdf/crane_plus.ros2_control.xacro#L44
        if (joint.parameters["servo_id"] != "") {
            servo_id_list.push_back(std::stoi(joint.parameters["servo_id"]));
        } else {
            RCLCPP_ERROR(
            rclcpp::get_logger("ManipulatorHardware"),
            "Joint '%s' does not have 'servo_id' parameter.",
            joint.name.c_str());
            return return_type::ERROR;
        }
    }


    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Open a driver
    // 疑問：型指定不要なのか？autoとか
    // port_name == deviceFile
    // 青木さんはここでbaudrateをも引数として渡し、open_port()メソッド中でsetBaudRate()を呼び出した。自分もset_feq()を、初期化に追加しよう。
    // open_port()を実装してないから
    // https://github.com/rt-net/crane_plus/blob/use_new_ros2_control/crane_plus_control/src/crane_plus_driver.cpp#L70
    driver_ = std::make_shared<ManipulatorDriver>(port_name, address, frequency_hz, servo_id_list);
    if (!driver_->open_port()) {
        RCLCPP_ERROR(rclcpp::get_logger("ManipulatorHardware"), driver_->get_last_error_log());
        return return_type::ERROR;
    }

    // Verify that the interface required by ManipulatorHardware is set in the URDF.
    // 意味不明
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ManipulatorHardware"),
          "Joint '%s' has %d command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return return_type::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ManipulatorHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
        return return_type::ERROR;
      }
    }

    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;    
}

std::vector<hardware_interface::StateInterface>ManipulatorHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>ManipulatorHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION,
              &hw_position_commands_[i])
        );
    }
  
    return command_interfaces;
}

return_type ManipulatorHardware::start()
{
    if (!driver_->torque_enable(false)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ManipulatorHardware"),
            driver_->get_last_error_log()); // Todo:未実装のはず
        return return_type::ERROR;
    }
    // Set current timestamp to disable the communication timeout.
    prev_comm_timestamp_ = rclcpp::Clock().now();
    timeout_has_printed_ = false;
  
    // Set current joint positions to hw_position_commands.
    read();
    for (uint i = 0; i < hw_position_commands_.size(); i++) {
        hw_position_commands_[i] = hw_position_states_[i];
    }
  
    status_ = hardware_interface::status::STARTED;
    return return_type::OK;
}

return_type ManipulatorHardware::stop()
{
    status_ = hardware_interface::status::STOPPED;
    return return_type::OK;
}

return_type ManipulatorHardware::read()
{
    if (communication_timeout()) {
        if (!timeout_has_printed_) {
            RCLCPP_ERROR(
            rclcpp::get_logger("ManipulatorHardware"), "Communication timeout!");
            timeout_has_printed_ = true;
        }
        return return_type::ERROR;
    }
  
    std::vector<double> joint_positions;
    if (!driver_->read_present_joint_positions(&joint_positions)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ManipulatorHardware"),
            driver_->get_last_error_log()); // Todo:未実装のはず
        return return_type::ERROR;
    } else {
        for (uint i = 0; i < hw_position_states_.size(); ++i) {
            hw_position_states_[i] = joint_positions[i];
        }
    }
  
    prev_comm_timestamp_ = rclcpp::Clock().now();
    return return_type::OK;
}

return_type ManipulatorHardware::write()
{
    if (communication_timeout()) {
        if (!timeout_has_printed_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ManipulatorHardware"), "Communication timeout!");
            timeout_has_printed_ = true;
        }
        return return_type::ERROR;
    }
  
    if (!driver_->write_goal_joint_positions(hw_position_commands_)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ManipulatorHardware"),
            driver_->get_last_error_log()); // Todo:未実装のはず
        return return_type::ERROR;
    }
  
    prev_comm_timestamp_ = rclcpp::Clock().now();
    return return_type::OK;
}

bool ManipulatorHardware::communication_timeout()
{
    if (rclcpp::Clock().now().seconds() - prev_comm_timestamp_.seconds() >= timeout_seconds_) {
        return true;
    } else {
        return false;
    }
}


} // namespace manipulator_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_control::ManipulatorHardware,
  hardware_interface::SystemInterface
)