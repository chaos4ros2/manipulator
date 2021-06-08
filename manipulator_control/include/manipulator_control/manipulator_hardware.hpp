#ifndef MANIPULATOR_CONTROL__MANIPULATOR_DRIVER_HPP_
#define MANIPULATOR_CONTROL__MANIPULATOR_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

// manipulator/manipulator_driver.hppだとmanipulatorパッケージのmanipulator_driver.hppになるから
// そもそもmanipulatorパッケージではなく、manipulator_controlパッケージだ
// 同じパッケージなので、コンパイル時に使うからまだそのパッケージができてないはず、manipulator_control/manipulator_driver.hppで試してもだめのはず
// 同じフォルダーなので、単に"manipulator_driver.hpp"だけでいいはず
#include "manipulator_control/manipulator_driver.hpp"
#include "manipulator_control/visibility_control.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// 意味不明 
// https://github.com/ros-controls/ros2_control/issues/151
using hardware_interface::return_type;

namespace manipulator_control
{
class ManipulatorHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ManipulatorHardware);

        // MANIPULATOR_CONTROL_PUBLIC
        // ~ManipulatorHardware();

        MANIPULATOR_CONTROL_PUBLIC
        return_type configure(const hardware_interface::HardwareInfo & info) override;

        MANIPULATOR_CONTROL_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        MANIPULATOR_CONTROL_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        MANIPULATOR_CONTROL_PUBLIC
        return_type start() override;

        MANIPULATOR_CONTROL_PUBLIC
        return_type stop() override;

        MANIPULATOR_CONTROL_PUBLIC
        return_type read() override;

        MANIPULATOR_CONTROL_PUBLIC
        return_type write() override;

    private:
        bool communication_timeout();
        
        std::shared_ptr<ManipulatorDriver> driver_;
        std::vector<double> hw_position_commands_;
        std::vector<double> hw_position_states_;

};   
} // namespace manipulator_control

#endif  // MANIPULATOR_CONTROL__MANIPULATOR_DRIVER_HPP_