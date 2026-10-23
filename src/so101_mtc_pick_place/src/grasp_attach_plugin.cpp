// Copyright 2026 maxtensor. Apache-2.0.
// Kinematic-attach grasp plugin for mujoco_ros2_control (workflow/plan/mtc.md Phase 2).
//
// Phase 0/1 spikes concluded that physical grasping with the single moving jaw
// and weld-equality approaches are unreliable (cube knocked away / constraint
// explosion). This plugin implements the standard "kinematic attach" instead:
// when the gripper command crosses the close threshold AND the cube is within
// reach of the jaw, the cube's freejoint qpos/qvel are overwritten each step to
// follow the moving jaw (relpose captured at attach time). On open command the
// cube is released back to physics.
//
// Loaded via mujoco_plugins parameter of the mujoco ros2_control node.

#include <atomic>
#include <array>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

namespace so101_mtc_pick_place
{

class GraspAttachPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data) override
  {
    node_ = node;
    model_ = model;

    // 插件参数是 **完整参数名** mujoco_plugins.<实例名>.<param> (与
    // FreeJointStatePublisherPlugin 等一致)。用相对名 "close_threshold" 声明
    // 会落在节点命名空间下, 永远读不到 YAML 里的值 (此前全部走默认值)。
    const std::string prefix = "mujoco_plugins." + node->get_sub_namespace() + ".";
    auto str_param = [&node, &prefix](const char * name, const char * def) {
      const std::string key = prefix + name;
      return node->has_parameter(key)
               ? node->get_parameter(key).as_string()
               : node->declare_parameter<std::string>(key, def);
    };
    auto dbl_param = [&node, &prefix](const char * name, double def) {
      const std::string key = prefix + name;
      return node->has_parameter(key) ? node->get_parameter(key).as_double()
                                      : node->declare_parameter<double>(key, def);
    };
    cube_body_ = str_param("cube_body", "cube");
    jaw_body_ = str_param("jaw_body", "moving_jaw_so101_v1");
    command_topic_ = str_param("command_topic", "/gripper_controller/commands");
    // 用 JTC 的 controller_state.reference 判断开合: /joint_states 的实际位置在
    // 压住 cube 时会回弹, 曾在闭合途中 0.77 → <0.3 造成瞬时误释放。
    controller_state_topic_ =
      str_param("controller_state_topic", "/gripper_controller/controller_state");
    gripper_joint_ = str_param("gripper_joint", "gripper");
    // 双阈值迟滞 (R3 纠正方向约定: SRDF closed=-0.02/open=1.0, JTC ref 闭合时
    // 从 1.0 降到 -0.02): attach 当 ref 降到 close_threshold 以下 (近全闭),
    // release 当 ref 升过 release_threshold (张开中)。旧约定 (ref>1.2 attach)
    // 对应的 SRDF closed=1.5 实为物理全开 —— 爪从未物理闭合。
    close_threshold_ = dbl_param("close_threshold", 0.1);
    release_threshold_ = dbl_param("release_threshold", 0.6);
    attach_radius_ = dbl_param("attach_radius", 0.15);
    // 规划时 cube 的初始位姿 (与 mtc_pick_place_node 的 cube_x/y/z 一致)
    cube_x_ = dbl_param("cube_x", 0.117);
    cube_y_ = dbl_param("cube_y", -0.077);
    cube_z_ = dbl_param("cube_z", 0.01);
    // R3 (workflow/plan/mtc_fix.md): R1 实测理想夹持位姿 (cube 相对动爪 body, 7 维
    // [x,y,z,qw,qx,qy,qz], scripts/probe_grasp_geometry.py calibrate 输出)。
    // 设置后 attach 用刚体跟随 (cube 随 jaw 旋转), 与抓/放两侧 IK 的 T_rel
    // 构造自洽; 不设则退回世界系平移偏移旧行为。
    const std::string rel_key = prefix + "ideal_rel_pose";
    ideal_rel_pose_ = node->has_parameter(rel_key)
      ? node->get_parameter(rel_key).as_double_array()
      : node->declare_parameter<std::vector<double>>(
          rel_key,
          {-0.019618, -0.048269, 0.0188, 0.707071, -0.707071, -0.007071,
           0.007071});
    ideal_mode_ = (ideal_rel_pose_.size() == 7);
    if (ideal_mode_) {
      std::copy(ideal_rel_pose_.begin(), ideal_rel_pose_.begin() + 3, rel_p_.begin());
      std::copy(ideal_rel_pose_.begin() + 3, ideal_rel_pose_.end(), rel_q_.begin());
      double qn = std::sqrt(rel_q_[0] * rel_q_[0] + rel_q_[1] * rel_q_[1] +
                            rel_q_[2] * rel_q_[2] + rel_q_[3] * rel_q_[3]);
      for (auto & v : rel_q_) { v /= qn; }
    }

    cube_bid_ = mj_name2id(model_, mjOBJ_BODY, cube_body_.c_str());
    jaw_bid_ = mj_name2id(model_, mjOBJ_BODY, jaw_body_.c_str());
    if (cube_bid_ < 0 || jaw_bid_ < 0) {
      RCLCPP_ERROR(node->get_logger(),
                   "GraspAttachPlugin: body '%s'(%d) or '%s'(%d) not found",
                   cube_body_.c_str(), cube_bid_, jaw_body_.c_str(), jaw_bid_);
      return false;
    }
    // MJCF 里 <freejoint/> 是 **匿名** 的: mj_name2id(mjOBJ_JOINT, "cube") 返回 -1,
    // 越界读取 jnt_qposadr[-1] 得到垃圾地址, 写 pose 时会覆盖机器人关节 qpos
    // (表现为 attach 瞬间机械臂跳到重力零位、cube 飞到 0.8m 外)。
    // 正确做法: 通过 body 的 body_jntadr 取它的 joints。
    if (model_->body_jntnum[cube_bid_] != 1) {
      RCLCPP_ERROR(node->get_logger(), "GraspAttachPlugin: body '%s' has %d joints, expected 1",
                   cube_body_.c_str(), model_->body_jntnum[cube_bid_]);
      return false;
    }
    cube_jid_ = model_->body_jntadr[cube_bid_];
    if (model_->jnt_type[cube_jid_] != mjJNT_FREE) {
      RCLCPP_ERROR(node->get_logger(), "GraspAttachPlugin: '%s' is not a freejoint (type=%d)",
                   cube_body_.c_str(), model_->jnt_type[cube_jid_]);
      return false;
    }
    qadr_ = model_->jnt_qposadr[cube_jid_];
    vadr_ = model_->jnt_dofadr[cube_jid_];
    RCLCPP_INFO(node->get_logger(),
                "GraspAttachPlugin: cube freejoint id=%d qpos_adr=%d dof_adr=%d",
                cube_jid_, qadr_, vadr_);

    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          last_grip_.store(msg->data[0]);
          cmd_seen_.store(true);
        }
      });

    // JointTrajectoryController **不发布** ~/commands topic (走 action), 因此
    // 单靠 command_topic 永远不会触发。以 controller_state 的 reference
    // (期望位置, 单调) 为准, 而不是实际 joint 位置 (受接触回弹影响)。
    state_sub_ = node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      controller_state_topic_, 10,
      [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr msg) {
        const auto & names = msg->joint_names;
        const auto & ref = msg->reference.positions;
        for (size_t i = 0; i < names.size() && i < ref.size(); ++i) {
          if (names[i] == gripper_joint_) {
            last_grip_.store(ref[i]);
            cmd_seen_.store(true);
            break;
          }
        }
      });

    RCLCPP_INFO(node->get_logger(),
                "GraspAttachPlugin ready: cube=%s jaw=%s joint=%s threshold=%.2f radius=%.2f",
                cube_body_.c_str(), jaw_body_.c_str(), gripper_joint_.c_str(),
                close_threshold_, attach_radius_);
    (void)data;
    return true;
  }

  void pre_step(mjData * data) override
  {
    if (attached_.load()) {
      write_attached_pose(data);
      // 释放: gripper 张开 (ref 升过 release_threshold; SRDF open=1.0)
      if (cmd_seen_ && last_grip_ > release_threshold_) {
        attached_.store(false);
        RCLCPP_INFO(node_->get_logger(), "GraspAttachPlugin: released cube");
      }
      return;
    }

    // 未附着: 检测闭合命令 (ref 降过 close_threshold) + cube 在爪附近 -> 抓取
    if (cmd_seen_ && last_grip_ < close_threshold_) {
      const double d = jaw_cube_dist(data);
      if (d < attach_radius_) {
        capture_relpose(data);
        attached_.store(true);
        RCLCPP_INFO(node_->get_logger(), "GraspAttachPlugin: attached cube (grip=%.2f)",
                    last_grip_.load());
      } else if (data->time - last_diag_time_ > 0.5) {
        last_diag_time_ = data->time;
        RCLCPP_WARN(node_->get_logger(),
                    "GraspAttachPlugin: closing (grip=%.2f) but cube is %.3f m from jaw "
                    "(attach_radius=%.2f) jaw=(%.3f, %.3f, %.3f) cube=(%.3f, %.3f, %.3f)",
                    last_grip_.load(), d, attach_radius_,
                    data->xpos[3 * jaw_bid_], data->xpos[3 * jaw_bid_ + 1],
                    data->xpos[3 * jaw_bid_ + 2], data->xpos[3 * cube_bid_],
                    data->xpos[3 * cube_bid_ + 1], data->xpos[3 * cube_bid_ + 2]);
      }
    }
  }

  void cleanup() override {}

private:
  double jaw_cube_dist(mjData * data) const
  {
    // mju_dist3 已经返回欧氏距离 (此前多套了一层 sqrt, 0.08m 被算成 0.28m)
    return mju_dist3(data->xpos + 3 * cube_bid_, data->xpos + 3 * jaw_bid_);
  }

  // 记录 cube 相对动爪的 **世界系平移偏移**。
  // - 用规划时的 cube 初始位姿 (不是被推走的当前位置), 否则放下点整体偏移;
  // - 只保持平移、不让 cube 随手腕旋转: grasp→place 的 wrist_roll 差 ~0.9rad,
  //   刚性旋转会让 cube 绕工具轴转出 5cm+ 的落点偏差, 而规划假设 cube
  //   始终在 EE 正下方 5cm。
  void capture_relpose(mjData * data)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (ideal_mode_) {
      // 理想位姿来自参数 (R1 实测), 无需 capture; 仅记录日志
      RCLCPP_INFO(node_->get_logger(),
                  "GraspAttachPlugin: attach with ideal_rel_pose (rigid follow)");
      return;
    }
    const mjtNum * pj = data->xpos + 3 * jaw_bid_;
    rel_w_[0] = cube_x_ - pj[0];
    rel_w_[1] = cube_y_ - pj[1];
    rel_w_[2] = cube_z_ - pj[2];
  }

  void write_attached_pose(mjData * data)
  {
    mjtNum pj[3];
    std::lock_guard<std::mutex> lock(pose_mutex_);
    std::memcpy(pj, data->xpos + 3 * jaw_bid_, sizeof(pj));

    mjtNum wp[3];
    mjtNum wq[4];
    if (ideal_mode_) {
      // 刚体跟随: wp = jaw_pos + R_jaw * rel_p, wq = q_jaw ⊗ rel_q
      const mjtNum * Rj = data->xmat + 9 * jaw_bid_;
      for (int i = 0; i < 3; ++i) {
        wp[i] = pj[i] + Rj[3 * i + 0] * rel_p_[0] + Rj[3 * i + 1] * rel_p_[1] +
                Rj[3 * i + 2] * rel_p_[2];
      }
      mju_mulQuat(wq, data->xquat + 4 * jaw_bid_, rel_q_.data());
    } else {
      for (int i = 0; i < 3; ++i) {
        wp[i] = pj[i] + rel_w_[i];
      }
      // cube 保持初始水平姿态 (不与手腕联动)
      wq[0] = 1.0; wq[1] = 0.0; wq[2] = 0.0; wq[3] = 0.0;
    }

    if (data->time - last_diag_time_ > 0.5) {
      last_diag_time_ = data->time;
      RCLCPP_INFO(node_->get_logger(),
                  "attached: jaw=(%.3f,%.3f,%.3f) rel=(%.3f,%.3f,%.3f) cube->(%.3f,%.3f,%.3f)",
                  pj[0], pj[1], pj[2], rel_w_[0], rel_w_[1], rel_w_[2], wp[0], wp[1], wp[2]);
    }

    // 覆盖 cube freejoint (pos + quat, 速度清零)
    data->qpos[qadr_ + 0] = wp[0];
    data->qpos[qadr_ + 1] = wp[1];
    data->qpos[qadr_ + 2] = wp[2];
    data->qpos[qadr_ + 3] = wq[0];
    data->qpos[qadr_ + 4] = wq[1];
    data->qpos[qadr_ + 5] = wq[2];
    data->qpos[qadr_ + 6] = wq[3];
    for (int i = 0; i < 6; ++i) {
      data->qvel[vadr_ + i] = 0.0;
    }
  }

  rclcpp::Node::SharedPtr node_;
  const mjModel * model_ = nullptr;
  std::string cube_body_, jaw_body_, command_topic_, controller_state_topic_, gripper_joint_;
  double close_threshold_ = 0.1;
  double release_threshold_ = 0.6;
  double attach_radius_ = 0.15;
  double cube_x_ = 0.117, cube_y_ = -0.077, cube_z_ = 0.01;
  bool ideal_mode_ = false;
  std::array<double, 3> rel_p_ {0.0, 0.0, 0.0};
  std::array<double, 4> rel_q_ {1.0, 0.0, 0.0, 0.0};
  std::vector<double> ideal_rel_pose_;
  double last_diag_time_ = -1.0;
  int cube_bid_ = -1, jaw_bid_ = -1, cube_jid_ = -1, qadr_ = -1, vadr_ = -1;
  std::atomic<double> last_grip_{0.0};
  std::atomic<bool> cmd_seen_{false};
  std::atomic<bool> attached_{false};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
  std::mutex pose_mutex_;
  mjtNum rel_w_[3] = {0, 0, 0};
};

}  // namespace so101_mtc_pick_place

PLUGINLIB_EXPORT_CLASS(so101_mtc_pick_place::GraspAttachPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
