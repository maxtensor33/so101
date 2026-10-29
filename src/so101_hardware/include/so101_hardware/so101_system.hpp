// Copyright 2024 The HuggingFace Inc. team. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SO101_HARDWARE__SO101_SYSTEM_HPP_
#define SO101_HARDWARE__SO101_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SCServo.h"

namespace so101_hardware
{

// Motor calibration data structure
struct MotorCalibration
{
  int id;
  int drive_mode;
  int range_min;
  int range_max;
  bool invert{false};
};

class SO101SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SO101SystemHardware)

  SO101SystemHardware();
  ~SO101SystemHardware();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string port_;
  std::string calibration_file_;
  std::string initial_positions_file_;

  // Joint state
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // C++ SCServo SDK
  SMS_STS servo_driver_;
  bool driver_initialized_;

  // Initial pose flag
  bool needs_initial_move_;
  int initial_move_cycles_remaining_;

  // 退出保护: 激活时的物理位姿 (on_deactivate 缓速回程目标) + 开关
  std::vector<double> start_pose_;
  bool return_to_start_{true};
  // park 进行中: read/write 主循环必须让路, 否则 write() 的旧命令 (speed 2400)
  // 会与 park 的低速插值命令打架, 表现为回程半路被拽停
  std::atomic<bool> parking_{false};

  // Motor ID mapping (joint name -> motor ID)
  std::map<std::string, int> motor_ids_;

  // Calibration data (motor ID -> calibration)
  std::map<int, MotorCalibration> motor_calibration_;
  // Per-joint calibration lookup (joint name -> calibration, 电机 ID 由此驱动)
  std::map<std::string, MotorCalibration> joint_calibration_;

  // Initial positions (joint name -> radians)
  std::map<std::string, double> initial_positions_;

  // Helper methods
  bool loadCalibration();
  bool loadInitialPositions();
  // 退出保护: 分步插值缓速回启动位姿 (禁止单步猛冲)
  void parkToStart();
  // rad↔raw 全行程线性映射, 限位按 joint 名索引 (D2: 本仓库 URDF 限位)
  double rawToRadians(int raw_position, const std::string& joint_name, const MotorCalibration& calib);
  int radiansToRaw(double radians, const std::string& joint_name, const MotorCalibration& calib);
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__SO101_SYSTEM_HPP_
