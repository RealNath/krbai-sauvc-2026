# KRBAI SAUVC 2026

## Project Structure

```
.
├── esp32_firmware (esp32 firmware for microcontroller)
│   └── main_controller
├── ros2_ws (ROS2 package for our robot)
│   └── src
│       ├── eggplant
│       ├── eggplant_bringup
│       ├── eggplant_controller
│       ├── eggplant_waypoint
│       └── thrusters_8
└── scripts (Utility scripts)
```

## Getting Started

### ESP32
1. Open `esp32_firmware/main_controller/main_controller.ino` in Arduino IDE.
2. Upload to your ESP32 board.
3. Alternatively, we can use esptool to upload firmware from ESP

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


### ROS2 - Raspberry Pi
1. SSH into the Rasberry Pi (make sure the proper network is used):  
   ```bash
   ssh ristek@ristek.local
   ```
   When prompted for password, enter `ristek`.

2. Make sure all the dependency exist in the Raspberry Pi OS:
   ```bash
   sudo apt update && sudo apt install \
      git \
      curl \
      picamera2 \
      minicom \
      python3 \
      python3-pip \
      python3-flask \
      python3-opencv \
      python3-numpy \
      python3-esptool && \
      curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh && rm get-docker.sh
   ```

3. Clone the repository (if not yet exist), navigate into the project root repository, and add the necessary permission,
   ```bash
   git clone https://github.com/RealNath/krbai-sauvc-2026
   cd krbai-sauvc-2026
   chmod +x run.sh build.sh attach.sh
   ```

4. Run the camera service in the background,
   ```bash
   python3 scripts/camera_service.py --stream --log &
   ```
   You may omit the `--stream` flag if no live video stream to ground station is needed (recommended when actual test in water is done) and the `--log` if no video logging is needed (not recommended)

5. Build and run the ROS2 docker image (make sure to modify the parameter `mission:=mission.yaml` inside the `raspi-entrypoint.sh` into the correct waypoint mission file),
   ```bash
   ./build.sh && ./run.sh &
   ```
   Now all ROS2 functionality should start and robot will start running autonomously

6. (Optional) ssh into the Raspberry Pi in another terminal then attach to the ROS2 docker container,
   ```bash
   cd krbai-sauvc-2026
   ./attach.sh
   ```
   Now you can do regular ROS2 command to monitor the eggplant activity, for example,
   ```bash
   ros2 pkg list
   ros2 topic list
   ros2 topic echo /sauvc/motors_vel > motor.log &
   ```

7. (Optional) ssh into the Raspberry Pi in another terminal and monitor the ESP32 serial monitor using `minicom`,
   ```bash
   minicom -D /dev/ttyACM0
   ```
   Press `CTRL + A` then `q` to exit minicom.



**Note 1:** video stream into ground station web browser may not be possible if robot is underwater due to weak wifi signal, but logging should be fine

**Note 2**: the motor parameters in `ros2_ws/eggplant_controller/kinematics.yaml` may need adjustment depending on the situation

**Note 3**: make sure the mission file is formatted correctly and placed in `ros2_ws/eggplant_waypoint/config/` (you can find example mission file in that directory)