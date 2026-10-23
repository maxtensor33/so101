// Copyright 2026 maxtensor. Apache-2.0.
// MTC pick and place for SO-ARM101 (workflow/plan/mtc.md Phase 2).
//
// 实际 stage 链 (扁平, 每个 stage 都是前向传播器, 无 SerialContainer/Connect):
//   CurrentState -> open gripper -> move to pick (OMPL 关节目标)
//   -> allow gripper-cube -> close gripper -> attach cube
//   -> lift cube (关节插值) -> move to place (OMPL) -> open gripper
//   -> detach cube -> retreat
//
// 为什么不用参考项目的 Connect + SerialContainer(FixedState/GeneratePose):
//   - Connect 之后的容器首 stage 必须是生成器/后向传播器才能通过 init, 但实测
//     place 侧的 Connect 在有 goal 状态时仍不触发 (0 解 0 failures);
//   - FixedState 手工 scene 不含 cube, attachObject 失败; 改从 /get_planning_scene
//     派生又引入 goal state 碰撞/ACM 的连锁问题。
//   扁平链完全绕开这些接口陷阱, 规划/执行在 mujoco 后端端到端可用。
//
// so101 特化:
//   - 5-dof 臂姿态受限: grasp/place 关节解来自 probe_pick_poses.py 的
//     "EE frame x 竖直" 族; grasp 解已按无自碰撞重新挑选 (旧解
//     gripper_link ↔ shoulder_link 自碰撞)。
//   - 抓取在 mujoco 中的物理效果由 grasp_attach_plugin (kinematic attach) 保证,
//     gz 侧为 DetachableJoint (物理夹持 spike 结论: 单动爪不可靠)。
//   - 所有 execution stage 都列全 arm_controller + gripper_controller, 避免
//     MoveIt 在执行某段时停用未列出的控制器。

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_task_constructor_msgs/msg/trajectory_execution_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <sstream>
#include <thread>

namespace mtc = moveit::task_constructor;

namespace so101_mtc_pick_place
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_node");

class MTCPickPlaceNode : public rclcpp::Node
{
public:
  MTCPickPlaceNode() : Node("mtc_pick_place_node")
  {
    arm_group_ = declare_parameter<std::string>("arm_group", "arm");
    gripper_group_ = declare_parameter<std::string>("gripper_group", "gripper");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "moving_jaw_so101_v1_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    cube_x_ = declare_parameter<double>("cube_x", 0.117);
    cube_y_ = declare_parameter<double>("cube_y", -0.077);
    // R2 (workflow/plan/mtc_fix.md): cube 4cm -> 2cm, 三处同步 (scene_pick.xml /
    // pick_place.sdf / 本节点 planning scene), 底面贴地 z=0.01
    cube_z_ = declare_parameter<double>("cube_z", 0.01);
    cube_size_ = declare_parameter<double>("cube_size", 0.02);
    // R2 (workflow/plan/mtc_fix.md) 重探测解 (pick=0.117,-0.077 / place=0.125,0.063,
    // cube 2cm): grasp_pose 由 R1 ideal_rel_pose 构造 (EE 带 standoff 偏移),
    // IK 解逐个过 mujoco 自碰撞校验 (probe_pick_poses.probe_pair)。
    // 旧 economy 解 [0.59347,...] 基于 "EE 正上方 cube" 目标, 是贴边根因之一。
    grasp_joints_ = declare_parameter<std::vector<double>>(
      "grasp_joints",
      {0.77386, -0.44185, 1.19005, 0.82260, 0.82253});
    // lift: grasp 位形沿抬起方向的 IK 解 (warm start grasp)
    lift_joints_ = declare_parameter<std::vector<double>>(
      "lift_joints",
      {0.77386, -0.64436, 1.07257, 1.14258, 0.82253});
    place_joints_ = declare_parameter<std::vector<double>>(
      "place_joints",
      {-0.63090, -0.47374, 1.21428, 0.83027, -0.58222});
    execute_ = declare_parameter<bool>("execute", true);
    auto_exec_delay_ = declare_parameter<double>("auto_exec_delay", 5.0);
    // RViz 模式: 任务完成后保持存活 (introspection 的 /description、/solution
    // 是 transient_local, 但随进程销毁——晚启动的面板需要进程活着才有数据)
    keep_running_ = declare_parameter<bool>("keep_running", false);
  }

  void spawnCollisionObjects()
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = world_frame_;
    cube.id = "cube";
    cube.primitives.resize(1);
    cube.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    cube.primitives[0].dimensions = {cube_size_, cube_size_, cube_size_};
    cube.primitive_poses.resize(1);
    cube.primitive_poses[0].position.x = cube_x_;
    cube.primitive_poses[0].position.y = cube_y_;
    cube.primitive_poses[0].position.z = cube_z_;
    cube.primitive_poses[0].orientation.w = 1.0;
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;
    objects.push_back(cube);

    // 注: 不加 ground box —— base_link 贴地, 与任何地面 box 天然重叠,
    // 会令所有 state 的碰撞检查失败 (FixedState/IK 全部 0 解)。
    // 防穿地由 mujoco/gz 物理负责。

    bool ok = psi.applyCollisionObjects(objects);
    RCLCPP_INFO(LOGGER, "applyCollisionObjects: %s; known objects: %s",
                ok ? "OK" : "FAILED",
                [&]() {
                  std::string s;
                  for (const auto & n : psi.getKnownObjectNames()) s += n + " ";
                  return s.empty() ? "(none)" : s;
                }().c_str());

    // ACM: 爪侧 link <-> cube 永久允许。注意 applyPlanningScene 的 ACM 是
    // **整体替换**——必须先取完整 ACM (含 SRDF disable_collisions) 再追加,
    // 否则相邻 link 的 disable 全被冲掉, 一切规划因自碰撞失败。
    {
      std::vector<std::string> gripper_links =
        task_.getRobotModel()->getJointModelGroup(gripper_group_)
          ->getLinkModelNamesWithCollisionGeometry();
      gripper_links.push_back("moving_jaw_so101_v1_link");

      auto gcli = shared_from_this();
      auto gclient = gcli->create_client<moveit_msgs::srv::GetPlanningScene>(
        "/get_planning_scene");
      moveit_msgs::srv::GetPlanningScene::Request greq;
      greq.components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
        moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
        moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;
      auto gfuture = gclient->async_send_request(std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>(greq));
      // main 的 executor 已在 spin 本节点, 这里等待即可 (禁止再 spin)
      if (gfuture.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(LOGGER, "get_planning_scene timeout");
        return;
      }
      moveit_msgs::msg::AllowedCollisionMatrix acm =
        gfuture.get()->scene.allowed_collision_matrix;

      // 追加新 link (moving_jaw 可能不在 SRDF ACM 里), 扩展现有行
      for (const auto & link : gripper_links) {
        if (std::find(acm.entry_names.begin(), acm.entry_names.end(), link)
            == acm.entry_names.end()) {
          acm.entry_names.push_back(link);
          for (auto & row : acm.entry_values) {
            row.enabled.push_back(false);
          }
        }
      }
      // cube 行/列扩展
      if (std::find(acm.entry_names.begin(), acm.entry_names.end(), "cube")
          == acm.entry_names.end()) {
        acm.entry_names.push_back("cube");
        for (auto & row : acm.entry_values) {
          row.enabled.push_back(false);
        }
      }
      const size_t n = acm.entry_names.size();
      acm.entry_values.resize(n);
      for (auto & row : acm.entry_values) {
        row.enabled.resize(n, false);
      }
      // 只放行 **爪侧 link** <-> cube。不要全放行: 全放行时 OMPL 会让整条臂
      // 直接穿过 cube, 物理仿真里把 cube 扫飞 (>0.8m), kinematic attach 就再也
      // 够不到 (attach_radius 0.15)。放行爪侧 link 是抓取/闭合语义所需的最小集,
      // 其余臂段仍会绕开 cube。
      for (size_t i = 0; i < n; ++i) {
        const bool i_cube = acm.entry_names[i] == "cube";
        for (size_t j = 0; j < n; ++j) {
          const bool j_cube = acm.entry_names[j] == "cube";
          if (i_cube == j_cube) continue;  // 跳过 cube-cube / 非 cube 对
          const std::string & other = acm.entry_names[i_cube ? j : i];
          if (std::find(gripper_links.begin(), gripper_links.end(), other)
              != gripper_links.end()) {
            acm.entry_values[i].enabled[j] = true;
            acm.entry_values[j].enabled[i] = true;
          }
        }
      }

      moveit_msgs::msg::PlanningScene diff;
      diff.is_diff = true;
      diff.robot_state.is_diff = true;
      diff.allowed_collision_matrix = acm;
      psi.applyPlanningScene(diff);
      RCLCPP_INFO(LOGGER, "ACM: gripper links <-> cube allowed (%zu links)",
                  gripper_links.size());
    }

    // 等 MTC 的 PlanningSceneMonitor 同步到 cube (attach stage 依赖它)
    for (int i = 0; i < 20; ++i) {
      if (!psi.getKnownObjectNames().empty()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    RCLCPP_INFO(LOGGER, "scene sync: known objects: %s",
                [&]() {
                  std::string s;
                  for (const auto & n : psi.getKnownObjectNames()) s += n + " ";
                  return s.empty() ? "(none)" : s;
                }().c_str());
  }

  bool run()
  {
    task_.stages()->setName("so101 pick place");
    task_.loadRobotModel(shared_from_this());
    // 必须在 init 之前开启: init() 会 publishTaskDescription (RViz MTC 面板),
    // 且只有 introspection 非空时 failure solutions 才会被保存 (诊断必需)。
    task_.enableIntrospection(true);
    // 顶层属性: 容器经 exposeTo/configureInitFrom 向下传播
    task_.setProperty("group", arm_group_);
    task_.setProperty("eef", gripper_group_);
    task_.setProperty("ik_frame", ee_frame_);
    spawnCollisionObjects();

    try {
      buildStages();
    } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR(LOGGER, "stage init failed: %s", e.what());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(LOGGER, "stage build failed: %s", e.what());
      return false;
    }

    try {
      task_.init();
    } catch (mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "task.init() failed: " << e);
      return false;
    }

    RCLCPP_INFO(LOGGER, "planning (max 5 solutions)...");
    task_.plan(5);
    const auto & sols = task_.solutions();
    if (sols.empty()) {
      RCLCPP_ERROR(LOGGER, "no solution found (per-stage solutions follow)");
      task_.stages()->traverseRecursively(
        [](const mtc::Stage & st, unsigned int) -> bool {
          RCLCPP_WARN(LOGGER, "  stage '%s': %zu solutions, %zu failures",
                      st.name().c_str(), st.solutions().size(), st.failures().size());
          for (const auto & f : st.failures())
            RCLCPP_WARN(LOGGER, "    failure: %s", f->comment().c_str());
          return true;
        });
      return false;
    }
    RCLCPP_INFO(LOGGER, "planning succeeded (%zu solutions)", sols.size());
    dumpSolution(*sols.front(), 0);
    task_.introspection().publishSolution(*sols.front());

    if (!execute_) {
      return true;
    }
    // 自动执行延迟: 给 RViz/Introspection 留出显示时间
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(auto_exec_delay_)));
    RCLCPP_INFO(LOGGER, "executing first solution via execute_task_solution...");
    return executeSolution(*task_.solutions().front());
  }

private:
  // 诊断: 逐个子解打印 creator / 轨迹点数 / 首末关节 (排查执行时的位形跳变)
  void dumpSolution(const mtc::SolutionBase & s, int depth) const
  {
    const std::string ind(depth * 2, ' ');
    if (const auto * seq = dynamic_cast<const mtc::SolutionSequence *>(&s)) {
      RCLCPP_INFO(LOGGER, "%s[%s]", ind.c_str(),
                  seq->creator() ? seq->creator()->name().c_str() : "?");
      for (const auto * sub : seq->solutions()) dumpSolution(*sub, depth + 1);
      return;
    }
    std::ostringstream oss;
    if (const auto * sub = dynamic_cast<const mtc::SubTrajectory *>(&s);
        sub && sub->trajectory()) {
      const auto & tr = *sub->trajectory();
      oss << tr.getWayPointCount() << " pts";
      auto fmt = [&oss](const moveit::core::RobotState & rs) {
        oss << " [";
        for (const char * j : {"shoulder_pan", "shoulder_lift", "elbow_flex",
                               "wrist_flex", "wrist_roll", "gripper"})
          oss << " " << rs.getVariablePosition(j);
        oss << " ]";
      };
      oss << " start";
      fmt(tr.getFirstWayPoint());
      oss << " end";
      fmt(tr.getLastWayPoint());
    } else {
      oss << "(no trajectory)";
    }
    RCLCPP_INFO(LOGGER, "%s- %s: %s", ind.c_str(),
                s.creator() ? s.creator()->name().c_str() : "?", oss.str().c_str());
  }

  // 每条子轨迹都列出 **两个** 控制器: MoveIt 的 TrajectoryExecutionManager 在
  // 执行时只激活所列控制器, 其余会被停用。若 close gripper 只列 gripper_controller,
  // arm_controller 会在该段被停用 → 机械臂失去力矩掉到重力姿态 (~1.3s), 期间
  // cube 被甩走。列全量控制器可避免切换。
  mtc::TrajectoryExecutionInfo allExecInfo() const
  {
    mtc::TrajectoryExecutionInfo info;
    info.set__controller_names({"arm_controller", "gripper_controller"});
    return info;
  }
  mtc::TrajectoryExecutionInfo armExecInfo() const { return allExecInfo(); }
  mtc::TrajectoryExecutionInfo gripperExecInfo() const { return allExecInfo(); }

  void buildStages()
  {
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    // 关节目标容差 1e-3 (MoveTo 用): 过宽会让末点偏差在后续插值段被放大
    sampling_planner->properties().set("goal_joint_tolerance", 1e-3);
    sampling_planner->properties().set("num_planning_attempts", 10u);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.005);

    // ── current state ───────────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
      task_.add(std::move(stage));
    }

    // ── open gripper ────────────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(gripper_group_);
      stage->setGoal("open");
      stage->properties().set("trajectory_execution_info", gripperExecInfo());
      task_.add(std::move(stage));
    }

    // ── move to pick ────────────────────────────────────────────────────────
    // 扁平 stage 链 (不用 SerialContainer/Connect): 每个 stage 都是前向传播器,
    // 接口解析简单可靠。MoveTo 用 PipelinePlanner (OMPL) 做关节空间规划 ——
    // JointInterpolationPlanner 的直线插值可能穿过自碰撞。
    // (容器 + Connect 的组合此前有两个坑: FixedState 生成器在 Connect 之后
    //  接口合法但 place Connect 不触发; 手工 scene 缺 cube 导致 attach 失败。
    //  两者都已定位, 扁平链直接绕开。)
    {
      const std::vector<std::string> ARMS = {"shoulder_pan", "shoulder_lift",
                                             "elbow_flex", "wrist_flex", "wrist_roll"};
      std::map<std::string, double> goals;
      for (size_t i = 0; i < ARMS.size(); ++i) goals[ARMS[i]] = grasp_joints_[i];
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to pick", sampling_planner);
      stage->setGroup(arm_group_);
      stage->setGoal(goals);
      stage->properties().set("trajectory_execution_info", armExecInfo());
      task_.add(std::move(stage));
    }
    // 注: 探测的 pre-grasp 关节解 OMPL 不可达(0 路径点), 而 grasp 位形可达。
    // 故 move to pick 直接以 grasp 位形为目标, 抓取接触语义由 close gripper +
    // attach + kinematic attach plugin 承担。

    // ── allow gripper <-> cube collision ────────────────────────────────────
    // grasp 位形下固定爪与动爪都与 cube 接触 (MuJoCo 实测), 闭合过程也必然接触。
    // 显式声明（不依赖 move_group 里那份全局 ACM）。
    {
      std::vector<std::string> links =
        task_.getRobotModel()->getJointModelGroup(gripper_group_)
          ->getLinkModelNamesWithCollisionGeometry();
      links.push_back("moving_jaw_so101_v1_link");
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-cube");
      stage->allowCollisions("cube", links, true);
      task_.add(std::move(stage));
    }

    // ── close gripper ───────────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(gripper_group_);
      stage->setGoal("closed");
      stage->properties().set("trajectory_execution_info", gripperExecInfo());
      task_.add(std::move(stage));
    }

    // ── attach cube (planning scene 语义, 供后续 planning) ─────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cube");
      stage->attachObject("cube", "gripper_frame_link");
      task_.add(std::move(stage));
    }

    // ── lift ────────────────────────────────────────────────────────────────
    // 用 MoveTo 到预求解的 lift 关节解, 不用 Cartesian MoveRelative: 5-dof 在
    // Cartesian IK 逐点 warm start 时仍会跳到另一冗余分支, 相邻 waypoint 关节
    // 突变 → 执行时机械臂甩动, 把未抓紧的 cube 扫飞 (>0.8m)。
    {
      const std::vector<std::string> ARMS = {"shoulder_pan", "shoulder_lift",
                                             "elbow_flex", "wrist_flex", "wrist_roll"};
      std::map<std::string, double> goals;
      for (size_t i = 0; i < ARMS.size(); ++i) goals[ARMS[i]] = lift_joints_[i];
      auto stage = std::make_unique<mtc::stages::MoveTo>("lift cube", interpolation_planner);
      stage->setGroup(arm_group_);
      stage->setGoal(goals);
      stage->properties().set("trajectory_execution_info", armExecInfo());
      task_.add(std::move(stage));
    }

    // ── move to place ───────────────────────────────────────────────────────
    {
      const std::vector<std::string> ARMS = {"shoulder_pan", "shoulder_lift",
                                             "elbow_flex", "wrist_flex", "wrist_roll"};
      std::map<std::string, double> goals;
      for (size_t i = 0; i < ARMS.size(); ++i) goals[ARMS[i]] = place_joints_[i];
      auto stage = std::make_unique<mtc::stages::MoveTo>("move to place", sampling_planner);
      stage->setGroup(arm_group_);
      stage->setGoal(goals);
      stage->properties().set("trajectory_execution_info", armExecInfo());
      task_.add(std::move(stage));
    }
    // 注: place_joints 探测时已是"放置释放位" (EE 在 place cube 中心上方
    // place_lift=5cm, cube 中心 z=0.02 恰好落地)。再向下 5cm 会把 cube
    // 压进地面, 故不再加 lower 段; 释放直接发生在 place 位形。

    // ── open gripper (place) ────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper (place)",
                                                         interpolation_planner);
      stage->setGroup(gripper_group_);
      stage->setGoal("open");
      stage->properties().set("trajectory_execution_info", gripperExecInfo());
      task_.add(std::move(stage));
    }

    // ── detach cube ─────────────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cube");
      stage->detachObject("cube", "gripper_frame_link");
      task_.add(std::move(stage));
    }

    // ── retreat ─────────────────────────────────────────────────────────────
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->setGroup(arm_group_);
      stage->setIKFrame("gripper_frame_link");
      stage->setMinMaxDistance(0.04, 0.06);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 0.05;
      stage->setDirection(vec);
      stage->properties().set("trajectory_execution_info", armExecInfo());
      task_.add(std::move(stage));
    }
  }

  bool executeSolution(const mtc::SolutionBase & solution)
  {
    using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    solution.toMsg(solution_msg, &task_.introspection());

    auto client_node = rclcpp::Node::make_shared("mtc_execute_client");
    auto client = rclcpp_action::create_client<ExecuteTaskSolution>(client_node,
                                                                    "execute_task_solution");
    if (!client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(LOGGER, "execute_task_solution action server not available "
                           "(move_group 需要 capabilities: "
                           "moveit_task_constructor_capabilities/ExecuteTaskSolutionCapability)");
      return false;
    }
    ExecuteTaskSolution::Goal goal;
    goal.solution = solution_msg;
    auto gh_future = client->async_send_goal(goal);
    // 必须 spin client_node: MTC 主节点已由 main 的 executor 线程 spin,
    // 再对它 spin_until_future_complete 会抛 "already added to an executor",
    // 且 client_node 的回调也只有 spin 它才会被处理。
    if (rclcpp::spin_until_future_complete(client_node, gh_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "execute goal send failed");
      return false;
    }
    auto gh = gh_future.get();
    if (!gh) {
      RCLCPP_ERROR(LOGGER, "execute goal rejected");
      return false;
    }
    auto result_future = client->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(client_node, result_future,
                                           std::chrono::seconds(300)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "execute result timeout");
      return false;
    }
    auto result = result_future.get();
    RCLCPP_INFO(LOGGER, "execution error_code=%d", result.result->error_code.val);
    return result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  std::string arm_group_, gripper_group_, ee_frame_, world_frame_;
  double cube_x_, cube_y_, cube_z_, cube_size_;
  double auto_exec_delay_;
  std::vector<double> grasp_joints_, lift_joints_, place_joints_;
  bool execute_;
  bool keep_running_ = false;

public:
  bool keepRunning() const { return keep_running_; }

private:
  mtc::Task task_;
};

}  // namespace so101_mtc_pick_place

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<so101_mtc_pick_place::MTCPickPlaceNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // 等待 move_group / planning scene 就绪
  rclcpp::sleep_for(std::chrono::seconds(3));
  bool ok = node->run();

  if (ok && node->keepRunning()) {
    RCLCPP_INFO(so101_mtc_pick_place::LOGGER,
                "task done; keeping node alive for RViz introspection (Ctrl+C to exit)");
    executor.spin();  // 阻塞直到 Ctrl+C / shutdown
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  RCLCPP_INFO(so101_mtc_pick_place::LOGGER, "mtc_pick_place_node exit (%s)",
              ok ? "SUCCESS" : "FAILED");
  return ok ? 0 : 1;
}
