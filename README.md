# KRBAI SAUVC 2026

## Project Structure

- `ros2_ws/`: ROS2 Workspace
  - `src/eggplant`: Main ROS2 package for the robot
    - `eggplant`: Python source code
- `arduino_firmware/`: Arduino sketches
  - `main_controller/`: Main Arduino code for low-level control
- `scripts/`: Utility scripts
- `docs/`: Documentation

## Getting Started

### ROS2 - Ground Station

1. Navigate to the workspace:
   ```bash
   cd ros2_ws
   ```
2. Build the package:
   ```bash
   colcon build
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```
4. Run the main node:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch eggplant_bringup eggplant.launch.py port:=/dev/ttyACM0 mission:=mission.yaml
   ```

### Arduino

1. Open `arduino_firmware/main_controller/main_controller.ino` in Arduino IDE.
2. Upload to your Arduino board.