#!/usr/bin/env bash
set -euo pipefail

pkill -INT -f "real_robot_bringup.launch.py" 2>/dev/null || true
pkill -INT -f "nav2_mppi_real.launch.py" 2>/dev/null || true
pkill -INT -f "continuous_navigator.py" 2>/dev/null || true
pkill -INT -f "elevation_mapping_node.py" 2>/dev/null || true
pkill -INT -f "livox_ros_driver2_node" 2>/dev/null || true
pkill -INT -f "fastlio_lidar_adapter" 2>/dev/null || true
pkill -INT -f "fastlio_imu_adapter" 2>/dev/null || true
pkill -INT -f "fastlio_odom_bridge" 2>/dev/null || true
pkill -INT -f "lio_node" 2>/dev/null || true

sleep 1

pkill -TERM -f "real_robot_bringup.launch.py" 2>/dev/null || true
pkill -TERM -f "nav2_mppi_real.launch.py" 2>/dev/null || true
pkill -TERM -f "continuous_navigator.py" 2>/dev/null || true
pkill -TERM -f "elevation_mapping_node.py" 2>/dev/null || true
pkill -TERM -f "livox_ros_driver2_node" 2>/dev/null || true
pkill -TERM -f "fastlio_lidar_adapter" 2>/dev/null || true
pkill -TERM -f "fastlio_imu_adapter" 2>/dev/null || true
pkill -TERM -f "fastlio_odom_bridge" 2>/dev/null || true
pkill -TERM -f "lio_node" 2>/dev/null || true

echo "[OK] Requested shutdown for NEXUS real-robot deployment processes."
