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

#include "so101_hardware/so101_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware {

SO101SystemHardware::SO101SystemHardware()
: driver_initialized_(false), needs_initial_move_(false), initial_move_cycles_remaining_(0) {}

SO101SystemHardware::~SO101SystemHardware() {
  if (driver_initialized_) {
    // 兜底失能 (正常路径 on_deactivate 已做; SIGTERM 优雅退出时析构仍会走到这里。
    // SIGKILL 强杀无法覆盖 — 故 park 上限压在 2s 内 + 提醒用户单次 Ctrl+C)
    for (const auto& pair : motor_ids_) {
      servo_driver_.EnableTorque(pair.second, 0);
    }
    servo_driver_.end();
  }
}

hardware_interface::CallbackReturn
SO101SystemHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get port parameter (default: /dev/ttyACM0)
  port_ = info_.hardware_parameters["port"];
  if (port_.empty()) {
    port_ = "/dev/ttyACM0";
  }

  // Get calibration file parameter
  calibration_file_ = info_.hardware_parameters["calibration_file"];

  // Get initial positions file parameter
  initial_positions_file_ = info_.hardware_parameters["initial_positions_file"];

  // 退出保护开关 (默认开): on_deactivate 缓速回启动位姿
  return_to_start_ = info_.hardware_parameters["return_to_start"] != "false";

  // Initialize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Load initial positions if specified
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints[i];
    if (joint.parameters.find("initial_position") != joint.parameters.end()) {
      hw_positions_[i] = std::stod(joint.parameters.at("initial_position"));
      hw_commands_[i] = hw_positions_[i];
      RCLCPP_INFO(
          rclcpp::get_logger("SO101SystemHardware"), "Joint '%s' initial position set to: %.3f",
          joint.name.c_str(), hw_positions_[i]);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SO101SystemHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Configuring...please wait...");

  // D4: 标定文件必填 (电机 ID / 量程真源), 缺失或加载失败一律 ERROR
  if (calibration_file_.empty()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "No calibration_file provided. Run scripts/setup_convert_lerobot_calibration.py first "
        "and pass its output via <param name=\"calibration_file\">.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!loadCalibration()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"), "Failed to load calibration from: %s",
        calibration_file_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 校验 6 个关节都拿到标定 (id + 量程)
  if (joint_calibration_.size() != info_.joints.size()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "Calibration covers %zu joints but URDF has %zu; check motor_calibration.yaml keys",
        joint_calibration_.size(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load initial positions
  if (!initial_positions_file_.empty() && !loadInitialPositions()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"), "Failed to load initial positions from: %s",
        initial_positions_file_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize servo driver
  if (!servo_driver_.begin(115200, port_.c_str())) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "Failed to connect to motor bus on port %s. Check connection and permissions.",
        port_.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("SO101SystemHardware"), "TROUBLESHOOTING:");
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "1. Check if robot is connected: ls /dev/ttyACM* /dev/ttyUSB*");
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "2. If robot is on different port, launch with: port:=/dev/ttyACMX");
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"),
        "3. Check permissions: sudo usermod -aG dialout $USER (then logout/login)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_initialized_ = true;

  // Initialize joint positions and commands
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0.0;
    }
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO101SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO101SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
SO101SystemHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Activating...please wait...");

  // Enable torque for all motors
  for (const auto& pair : motor_ids_) {
    int motor_id = pair.second;
    if (servo_driver_.EnableTorque(motor_id, 1) == -1) {
      RCLCPP_ERROR(
          rclcpp::get_logger("SO101SystemHardware"), "Failed to enable torque for motor %d",
          motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Read current positions
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Current motor positions:");
  size_t read_ok = 0;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::string& joint_name = info_.joints[i].name;
    int motor_id = motor_ids_[joint_name];

    if (servo_driver_.FeedBack(motor_id) != -1) {
      int raw_pos = servo_driver_.ReadPos(-1);
      hw_positions_[i] = rawToRadians(raw_pos, joint_name, joint_calibration_[joint_name]);

      hw_commands_[i] = hw_positions_[i];
      read_ok++;

      RCLCPP_INFO(
          rclcpp::get_logger("SO101SystemHardware"), "  %s: %.3f rad (motor: %d raw)",
          joint_name.c_str(), hw_positions_[i], raw_pos);
    } else {
      RCLCPP_ERROR(
          rclcpp::get_logger("SO101SystemHardware"),
          "Failed to read joint '%s' (motor %d): 总线无响应 — 检查 port 是否指向实机 "
          "(两个 ACM 设备时用 /dev/serial/by-id/ 区分, 或显式 port:=/dev/ttyACM1)",
          joint_name.c_str(), motor_id);
    }
  }
  // 读不全 = 不知道当前姿态, 宁可拒绝激活, 也不能拿 on_configure 的零位冒充
  // (2026-10-29 教训: 连错串口 → 6 关节全超时 → RViz 显示 home 假象)
  if (read_ok != info_.joints.size()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 记录激活时的物理位姿: on_deactivate 退出保护的回程目标
  start_pose_ = hw_positions_;

  // Prepare for initial position move (will be executed in first write() call)
  if (!initial_positions_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Preparing initial positions...");
    for (size_t i = 0; i < info_.joints.size(); i++) {
      const std::string& joint_name = info_.joints[i].name;

      // Use initial position if defined, otherwise keep current position
      if (initial_positions_.count(joint_name) > 0) {
        double init_pos = initial_positions_[joint_name];
        hw_commands_[i] = init_pos;
        // Update position state to match target so controllers don't fight the initial move
        hw_positions_[i] = init_pos;

        RCLCPP_INFO(
            rclcpp::get_logger("SO101SystemHardware"), "  %s: %.3f rad", joint_name.c_str(),
            init_pos);
      } else {
        // No initial position defined, keep current position
        hw_commands_[i] = hw_positions_[i];
      }
    }

    // Set flag to trigger initial move in first write() cycle
    needs_initial_move_ = true;
    // Lock position readings immediately to prevent controllers from reading actual positions
    initial_move_cycles_remaining_ = 1000;
    RCLCPP_INFO(
        rclcpp::get_logger("SO101SystemHardware"),
        "Position readings locked during initial move (10 seconds at 100Hz).");
  } else {
    RCLCPP_INFO(
        rclcpp::get_logger("SO101SystemHardware"),
        "No initial positions loaded, robot will stay at current position");
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SO101SystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Deactivating...please wait...");

  // 退出保护: 先缓速回启动位姿, 再失能力矩
  if (return_to_start_ && !start_pose_.empty()) {
    parking_ = true;
    parkToStart();
    parking_ = false;
  }

  // Disable torque for all motors
  for (const auto& pair : motor_ids_) {
    int motor_id = pair.second;
    if (servo_driver_.EnableTorque(motor_id, 0) == -1) {
      RCLCPP_ERROR(
          rclcpp::get_logger("SO101SystemHardware"), "Failed to disable torque for motor %d",
          motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SO101SystemHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // park 进行中: 冻结状态, 不与回程插值竞争
  if (parking_) {
    return hardware_interface::return_type::OK;
  }

  // If we're still in initial move period, skip reading actual positions
  // This keeps hw_positions_ locked to target values so controllers don't fight the move
  if (initial_move_cycles_remaining_ > 0) {
    initial_move_cycles_remaining_--;
    if (initial_move_cycles_remaining_ == 0) {
      RCLCPP_INFO(
          rclcpp::get_logger("SO101SystemHardware"),
          "Initial move complete. Resuming normal position readings.");
    }
    return hardware_interface::return_type::OK;
  }

  // Read positions from all motors
  static rclcpp::Clock read_clock;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::string& joint_name = info_.joints[i].name;
    int motor_id = motor_ids_[joint_name];

    if (servo_driver_.FeedBack(motor_id) != -1) {
      int raw_pos = servo_driver_.ReadPos(-1);

      hw_positions_[i] = rawToRadians(raw_pos, joint_name, joint_calibration_[joint_name]);
    } else {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("SO101SystemHardware"), read_clock, 2000,
          "Failed to read joint '%s' (motor %d), 位置保持上值 (总线间歇无响应?)",
          joint_name.c_str(), motor_id);
    }

    hw_velocities_[i] = 0.0;  // TODO: Read actual velocity if needed
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SO101SystemHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // park 进行中: parkToStart 自己发命令, 主循环不得写入 (否则旧命令 speed 2400
  // 与低速插值打架 → 回程半路被拽停, 2026-10-29 真机实测)
  if (parking_) {
    return hardware_interface::return_type::OK;
  }

  // Use slower speed for initial move
  u16 speed = needs_initial_move_ ? 1000 : 2400;

  if (needs_initial_move_) {
    RCLCPP_INFO(
        rclcpp::get_logger("SO101SystemHardware"),
        "Executing initial move to configured positions...");
  }

  // Prepare arrays for sync write
  std::vector<u8> motor_ids;
  std::vector<s16> positions;
  std::vector<u16> speeds;
  std::vector<u8> accelerations;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (!std::isnan(hw_commands_[i])) {
      const std::string& joint_name = info_.joints[i].name;
      int motor_id = motor_ids_[joint_name];

      int raw_position = radiansToRaw(hw_commands_[i], joint_name, joint_calibration_[joint_name]);

      motor_ids.push_back(static_cast<u8>(motor_id));
      positions.push_back(static_cast<s16>(raw_position));
      speeds.push_back(speed);
      accelerations.push_back(static_cast<u8>(50));
    }
  }

  // Write positions to all motors synchronously
  if (!motor_ids.empty()) {
    servo_driver_.SyncWritePosEx(
        motor_ids.data(), motor_ids.size(), positions.data(), speeds.data(), accelerations.data());
  }

  // Clear flag after first write
  if (needs_initial_move_) {
    needs_initial_move_ = false;
    RCLCPP_INFO(rclcpp::get_logger("SO101SystemHardware"), "Initial move command sent.");
  }

  return hardware_interface::return_type::OK;
}

void SO101SystemHardware::parkToStart() {
  // 缓速回启动位姿: 小步插值 + 低速 SyncWrite, 避免 Ctrl+C 时猛冲损坏。
  // 不走 MoveIt: 退出时 move_group 与 controller_manager 同时被杀, 轨迹会半路夭折,
  // 力矩在半途断掉比匀速回程更危险 (决策记录见 workflow/plan/setup_hardware.md)。
  const rclcpp::Logger kLog = rclcpp::get_logger("SO101SystemHardware");
  const u16 kParkSpeed = 800;  // 正常运行 2400, 回程限低速 (伺服自身也限速)
  const u8 kParkAcc = 30;
  const int kStepMs = 20;
  const double kMaxStepRad = 0.03;  // 每步最大角步长

  double max_dist = 0.0;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    max_dist = std::max(max_dist, std::fabs(hw_positions_[i] - start_pose_[i]));
  }
  // 上限 100 步 → 回程最长约 2s: 必须在 launch 的 SIGINT→SIGTERM 升级窗口内完成
  // (第二次 Ctrl+C 会立即强杀, park 被腰斩 — 提醒用户按一次 Ctrl+C 等它走完)
  int steps = static_cast<int>(std::ceil(max_dist / kMaxStepRad));
  steps = std::clamp(steps, 1, 100);

  std::vector<u8> ids;
  std::vector<u16> speeds;
  std::vector<u8> accs;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    ids.push_back(static_cast<u8>(motor_ids_[info_.joints[i].name]));
    speeds.push_back(kParkSpeed);
    accs.push_back(kParkAcc);
  }

  RCLCPP_INFO(
      kLog, "Returning to start pose: %.0f steps x %dms, speed %d", static_cast<double>(steps),
      kStepMs, kParkSpeed);
  std::vector<s16> positions;
  for (int s = 1; s <= steps; s++) {
    const double a = static_cast<double>(s) / steps;
    positions.clear();
    for (size_t i = 0; i < info_.joints.size(); i++) {
      const double rad = hw_positions_[i] + a * (start_pose_[i] - hw_positions_[i]);
      positions.push_back(static_cast<s16>(radiansToRaw(
          rad, info_.joints[i].name, joint_calibration_[info_.joints[i].name])));
    }
    servo_driver_.SyncWritePosEx(ids.data(), ids.size(), positions.data(), speeds.data(), accs.data());
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
  }

  // 低速下伺服物理运动滞后于插值命令: 命令发完 ≠ 到位。
  // 必须轮询实际位置等伺服真正到达, 否则立即失能会把臂松在半路 (2026-10-29 真机实测)。
  const double kArriveTolRad = 0.06;
  bool arrived = false;
  for (int wait = 0; wait < 100 && !arrived; wait++) {  // 最多约 5s
    arrived = true;
    for (size_t i = 0; i < info_.joints.size() && arrived; i++) {
      const std::string& joint_name = info_.joints[i].name;
      if (servo_driver_.FeedBack(motor_ids_[joint_name]) != -1) {
        const double cur =
            rawToRadians(servo_driver_.ReadPos(-1), joint_name, joint_calibration_[joint_name]);
        if (std::fabs(cur - start_pose_[i]) > kArriveTolRad) {
          arrived = false;
        }
      } else {
        arrived = false;  // 读失败视为未到位, 继续等
      }
    }
    if (!arrived) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  if (arrived) {
    RCLCPP_INFO(kLog, "Start pose reached, disabling torque.");
  } else {
    RCLCPP_WARN(
        kLog, "Start pose not confirmed within 5s (servo lag), disabling torque anyway.");
  }
}

bool SO101SystemHardware::loadCalibration() {
  if (calibration_file_.empty()) {
    RCLCPP_INFO(
        rclcpp::get_logger("SO101SystemHardware"),
        "No calibration file specified, using default values");
    return true;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("SO101SystemHardware"), "Loading calibration from: %s",
      calibration_file_.c_str());

  std::ifstream file(calibration_file_);
  if (!file.is_open()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"), "Failed to open calibration file: %s",
        calibration_file_.c_str());
    return false;
  }

  // Simple YAML parser for calibration file
  std::string line;
  std::string current_motor;
  MotorCalibration current_calib = {0, 0, 0, 0};

  while (std::getline(file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#')
      continue;

    // Check if this is a motor name line (no leading spaces, ends with :)
    if (line.find_first_not_of(" \t") == 0 && line.back() == ':') {
      // Save previous motor if it had an id
      if (!current_motor.empty() && current_calib.id != 0) {
        motor_ids_[current_motor] = current_calib.id;
        joint_calibration_[current_motor] = current_calib;
        motor_calibration_[current_calib.id] = current_calib;
      }

      // Start new motor
      current_motor = line.substr(0, line.length() - 1);
      current_calib = {0, 0, 0, 0};
      continue;
    }

    // Parse key-value pairs
    size_t colon_pos = line.find(':');
    if (colon_pos != std::string::npos) {
      std::string key = line.substr(0, colon_pos);
      std::string value = line.substr(colon_pos + 1);

      // Trim whitespace
      key.erase(0, key.find_first_not_of(" \t"));
      key.erase(key.find_last_not_of(" \t") + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      value.erase(value.find_last_not_of(" \t") + 1);

      if (key == "id")
        current_calib.id = std::stoi(value);
      else if (key == "drive_mode")
        current_calib.drive_mode = std::stoi(value);
      else if (key == "range_min")
        current_calib.range_min = std::stoi(value);
      else if (key == "range_max")
        current_calib.range_max = std::stoi(value);
      else if (key == "invert")
        current_calib.invert = (value == "true" || value == "True");
    }
  }

  // Save last motor
  if (!current_motor.empty() && current_calib.id != 0) {
    motor_ids_[current_motor] = current_calib.id;
    joint_calibration_[current_motor] = current_calib;
    motor_calibration_[current_calib.id] = current_calib;
  }

  file.close();

  RCLCPP_INFO(
      rclcpp::get_logger("SO101SystemHardware"), "Successfully loaded calibration for %zu motors",
      motor_calibration_.size());

  return true;
}

bool SO101SystemHardware::loadInitialPositions() {
  if (initial_positions_file_.empty()) {
    RCLCPP_INFO(
        rclcpp::get_logger("SO101SystemHardware"),
        "No initial positions file specified, will start from current positions");
    return true;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("SO101SystemHardware"), "Loading initial positions from: %s",
      initial_positions_file_.c_str());

  std::ifstream file(initial_positions_file_);
  if (!file.is_open()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SO101SystemHardware"), "Failed to open initial positions file: %s",
        initial_positions_file_.c_str());
    return false;
  }

  // Simple YAML parser for initial_positions section
  std::string line;
  bool in_initial_positions_section = false;

  while (std::getline(file, line)) {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#')
      continue;

    // Check if we're entering the initial_positions section
    if (line.find("initial_positions:") != std::string::npos) {
      in_initial_positions_section = true;
      continue;
    }

    // Exit if we encounter another top-level section
    if (in_initial_positions_section && line.find_first_not_of(" \t") == 0 && line.back() == ':') {
      break;
    }

    // Parse joint positions (indented lines with :)
    if (in_initial_positions_section) {
      size_t colon_pos = line.find(':');
      if (colon_pos != std::string::npos) {
        std::string joint_name = line.substr(0, colon_pos);
        std::string value = line.substr(colon_pos + 1);

        // Trim whitespace
        joint_name.erase(0, joint_name.find_first_not_of(" \t"));
        joint_name.erase(joint_name.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));

        // Remove comments from value
        size_t comment_pos = value.find('#');
        if (comment_pos != std::string::npos) {
          value = value.substr(0, comment_pos);
        }
        value.erase(value.find_last_not_of(" \t") + 1);

        // Check if this joint is valid
        if (motor_ids_.count(joint_name) > 0) {
          initial_positions_[joint_name] = std::stod(value);
          RCLCPP_INFO(
              rclcpp::get_logger("SO101SystemHardware"), "  %s: %.3f rad", joint_name.c_str(),
              initial_positions_[joint_name]);
        }
      }
    }
  }

  file.close();

  RCLCPP_INFO(
      rclcpp::get_logger("SO101SystemHardware"),
      "Successfully loaded initial positions for %zu joints", initial_positions_.size());

  return true;
}

// D2: 本仓库 src/so101_description/urdf/so101.urdf 限位 (rad), 按 joint 名索引。
// TODO: 后续可改为从 URDF 动态解析 (参考项目同款 TODO)
static bool so101JointLimits(const std::string& joint_name, double& lower, double& upper) {
  if (joint_name == "shoulder_pan") {
    lower = -1.91986;
    upper = 1.91986;
  } else if (joint_name == "shoulder_lift") {
    // 手动估计 ±1.91986 (对齐 models/so101_new_calib.xml 的手动放宽; 未经真机逐关节校准,
    // 见 workflow/todo/to_real_joint_calibration.md; 原导出值 ±1.74533)
    lower = -1.91986;
    upper = 1.91986;
  } else if (joint_name == "elbow_flex") {
    lower = -1.69;
    upper = 1.69;
  } else if (joint_name == "wrist_flex") {
    lower = -1.65806;
    upper = 1.65806;
  } else if (joint_name == "wrist_roll") {
    lower = -2.74385;
    upper = 2.84121;
  } else if (joint_name == "gripper") {
    lower = -0.174533;
    upper = 1.74533;
  } else {
    return false;
  }
  return true;
}

double SO101SystemHardware::rawToRadians(
    int raw_position, const std::string& joint_name, const MotorCalibration& calib) {
  // Clamp to calibration range (no homing offset applied)
  int clamped = std::max(calib.range_min, std::min(calib.range_max, raw_position));

  // Normalize to [0, 1] based on calibrated range
  double progress = static_cast<double>(clamped - calib.range_min) /
                    static_cast<double>(calib.range_max - calib.range_min);
  // invert: 电机安装方向与 URDF 约定相反时取反 (标定 YAML per-motor 可选字段)
  if (calib.invert) {
    progress = 1.0 - progress;
  }

  // D2: 本仓库 URDF 限位, 按 joint 名索引 (未知关节回退 ±pi)
  double urdf_lower, urdf_upper;
  if (!so101JointLimits(joint_name, urdf_lower, urdf_upper)) {
    urdf_lower = -M_PI;
    urdf_upper = M_PI;
  }

  // Scale to URDF limits (radians for ALL joints, including gripper)
  return progress * (urdf_upper - urdf_lower) + urdf_lower;
}

int SO101SystemHardware::radiansToRaw(
    double radians, const std::string& joint_name, const MotorCalibration& calib) {
  // D2: 本仓库 URDF 限位, 按 joint 名索引 (未知关节回退 ±pi)
  double urdf_lower, urdf_upper;
  if (!so101JointLimits(joint_name, urdf_lower, urdf_upper)) {
    urdf_lower = -M_PI;
    urdf_upper = M_PI;
  }

  // Clamp to URDF limits
  double clamped_radians = std::min(urdf_upper, std::max(urdf_lower, radians));

  // Normalize to [0, 1]
  double progress = (clamped_radians - urdf_lower) / (urdf_upper - urdf_lower);
  // invert: 与 rawToRadians 对称
  if (calib.invert) {
    progress = 1.0 - progress;
  }

  // Scale to motor range
  int raw_position =
      static_cast<int>(progress * (calib.range_max - calib.range_min) + calib.range_min);

  // Return raw position directly (no homing offset)
  return raw_position;
}

}  // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)
