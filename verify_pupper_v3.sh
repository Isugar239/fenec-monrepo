#!/usr/bin/env bash
# Verification script for a complete pupper v3 project
# Usage: ./verify_pupper_v3.sh

# List of required files (relative to repository root)
REQUIRED_FILES=(
    # Core description
    "ros2_ws/src/pupper_v3_description/package.xml"
    "ros2_ws/src/pupper_v3_description/CMakeLists.txt"
    "ros2_ws/src/pupper_v3_description/description/pupper_v3.urdf.xacro"
    "ros2_ws/src/pupper_v3_description/description/pupper_v3.rviz"
    "ros2_ws/src/pupper_v3_description/launch/display_model.launch.py"
    "ros2_ws/src/pupper_v3_description/launch/robot_state_publisher.launch.py"

    # Mujoco simulation
    "ros2_ws/src/pupperv3_mujoco_sim/mujoco_hardware_interface.xml"
    "ros2_ws/src/pupperv3_mujoco_sim/launch/test_hw_interface.launch.py"
    "ros2_ws/src/pupperv3_mujoco_sim/lib/mujoco_hardware_interface.hpp"
    "ros2_ws/src/pupperv3_mujoco_sim/lib/mujoco_hardware_interface.cpp"

    # Neural controller
    "ros2_ws/src/neural_controller/package.xml"
    "ros2_ws/src/neural_controller/CMakeLists.txt"
    "ros2_ws/src/neural_controller/include/neural_controller/neural_controller.hpp"
    "ros2_ws/src/neural_controller/src/neural_controller.cpp"
    "ros2_ws/src/neural_controller/launch/launch.py"
    "ros2_ws/src/neural_controller/launch/policy_latest.json"

    # Real‑to‑Sim controller
    "ros2_ws/src/real2sim_controller/package.xml"
    "ros2_ws/src/real2sim_controller/include/real2sim_controller/real2sim_controller.hpp"
    "ros2_ws/src/real2sim_controller/launch/launch.py"
    "ros2_ws/src/real2sim_controller/launch/config.yaml"

    # IMU‑to‑TF
    "ros2_ws/src/imu_to_tf/package.xml"
    "ros2_ws/src/imu_to_tf/launch/imu_to_tf.launch.py"
    "ros2_ws/src/imu_to_tf/src/imu_to_tf_node.cpp"

    # Hailo perception stack
    "ros2_ws/src/hailo/package.xml"
    "ros2_ws/src/hailo/hailo/hailo_detection.py"
    "ros2_ws/src/hailo/hailo/hailo_inference.py"
    "ros2_ws/src/hailo/launch/hailo_detection_launch.py"

    # LLM WebSocket server
    "ros2_ws/src/llm_websocket_server/package.xml"
    "ros2_ws/src/llm_websocket_server/llm_websocket_server/websocket_server.py"
    # "ros2_ws/src/llm_websocket_server/launch/websocket_server_launch.py"  # No launch file provided; optional

    # Pupper feelings
    "ros2_ws/src/pupper_feelings/package.xml"
    "ros2_ws/src/pupper_feelings/pupper_feelings/face_control.py"
    "ros2_ws/src/pupper_feelings/pupper_feelings/ear_control.py"

    # Command‑velocity multiplexer
    "ros2_ws/src/cmd_vel_mux/package.xml"
    "ros2_ws/src/cmd_vel_mux/src/cmd_vel_mux_node.cpp"

    # Joy utilities
    "ros2_ws/src/joy_utils/package.xml"
    "ros2_ws/src/joy_utils/src/estop_controller.cpp"

    # Bag recorder
    "ros2_ws/src/bag_recorder/package.xml"
    "ros2_ws/src/bag_recorder/launch/bag_recorder.launch.py"

    # Top‑level scripts & metadata
    "robot/start_ui.sh"
    ".gitignore"
    "LICENSE"
    "README.md"
)

echo "Starting verification of required pupper v3 files..."
missing=0

for file in "${REQUIRED_FILES[@]}"; do
    if [[ ! -e "$file" ]]; then
        echo "MISSING: $file"
        ((missing++))
    fi
done

if [[ $missing -eq 0 ]]; then
    echo "All required files are present."
else
    echo "Verification completed with $missing missing file(s)."
    exit 1
fi