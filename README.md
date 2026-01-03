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
4. Run the main node (choose the proper mission file):
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
      make \
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
   ```

4. Run the ROS2 docker container,
   ```bash
   make
   ```
   All the modifiable Makefile flag is as follow,
   ```bash
   ESP32_SRC               : esp32 firmware source code location (default: ./esp32_firmware)
   ESP32_PORT              : esp32 port connected to Raspberry Pi (default: /dev/ttyACM0)
   ESP32_FQBN              : esp32 board official name (default: esp32:esp32:esp32s3)
   ESP32_BAUDRATE          : esp32 board baud rate (default: 115200)
   ROS2_WS                 : ros2 workspace location (default: ./ros2_ws)
   ROS2_CONTAINER_TAG      : ros2 docker container tag (default: ros-humble-core-dev)
   MISSION_FILE            : ros2 waypoint mission file (default: mission.yaml)
   SCRIPT_SRC              : utility script source code location (default: ./scripts)
   CAMERA_SERVICE_OPTION   : python flask camera service option (default: --log)
   ```


5. (Optional) Run the Raspberry Pi camera web server,
   ```bash
   make webserver
   ```
   Set the option of python camera service action using flag `--stream` to enable stream, and `--log` to enable logging (default: logging enabled and stream disabled). For example:
   ```bash
   make webserver CAMERA_SERVICE_OPTION="--stream --log"
   make webserver CAMERA_SERVICE_OPTION="--stream"
   ```


6. (Optional) ssh into the Raspberry Pi in another terminal then attach to the ROS2 docker container,
   ```bash
   make attach
   ```
   Now you can do regular ROS2 command to monitor the eggplant activity, for example,
   ```bash
   ros2 pkg list
   ros2 topic list
   ros2 topic echo /sauvc/motors_vel > motor.log &
   ```

7. (Optional) compile and upload new ESP32 firmware (make sure the .ino file location is correct)
   ```bash
   make esp32-build
   ```


8. (Optional) view the output of the esp32 serial monitor,
   ```bash
   make esp32-monitor
   ```
   Press `CTRL + C` to exit.


9. (Optional) hard reset the esp32 to boot from bootloader,
   ```bash
   make esp32-reset
   ```



**Note 1:** video stream into ground station web browser may not be possible if robot is underwater due to weak wifi signal, but logging should be fine

**Note 2**: the motor parameters in `ros2_ws/eggplant_controller/kinematics.yaml` may need adjustment depending on the situation

**Note 3**: make sure the mission file is formatted correctly and placed in `ros2_ws/eggplant_waypoint/config/` (you can find example mission file in that directory)