#!/bin/bash
# 清理 SO-ARM101 demo 的 ROS2/仿真残留进程
# 用法: bash scripts/clean_ros.sh
pkill -f "ros2 launch so101_bringup" 2>/dev/null
pkill -f "ros2_control_node" 2>/dev/null
pkill -f "controller_manager" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "move_group" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "spawner" 2>/dev/null
sleep 1
LEFT=$(pgrep -f "ros2_control_node|gz sim|gzserver|rviz2|move_group" | wc -l)
if [ "$LEFT" -eq 0 ]; then
    echo "✓ 所有 demo 进程已清理"
else
    echo "⚠ 仍有 $LEFT 个残留进程:"; pgrep -af "ros2_control_node|gz sim|gzserver|rviz2|move_group"
fi
