# Eggplant Controller - KRBAI SAUVC 2026

Sistem kontrol robot bawah air yang terintegrasi untuk kompetisi SAUVC 2026. Package ini menangani seluruh pipeline kontrol dari input joystick hingga komunikasi dengan mikrokontroler.

## Arsitektur Sistem

Package `eggplant_controller` terdiri dari tiga node utama yang bekerja secara berurutan:

```
Joystick → joy_to_twist_node → thrusters_8_kinematics → serial_manager → Mikrokontroler
```

### Node 1: joy_to_twist_node

Mengonversi sinyal joystick (`sensor_msgs/msg/Joy`) menjadi perintah kecepatan (`geometry_msgs/msg/Twist`).

**Topik Input:**
- `/joy` (sensor_msgs/msg/Joy)

**Topik Output:**
- `/cmd_vel` (geometry_msgs/msg/Twist)

**Pemetaan Kontrol:**
- Joystick Kiri (Vertikal): Maju/Mundur → `linear.x`
- Joystick Kiri (Horizontal): Geser Kiri/Kanan → `linear.y`
- Joystick Kanan (Horizontal): Rotasi Yaw → `angular.z`
- Tombol LB + Joystick Kanan (Vertikal): Naik/Turun → `linear.z`

### Node 2: thrusters_8_kinematics

Melakukan kalkulasi kinematika untuk 8 thruster berdasarkan input Twist. Node ini menghitung kecepatan individual setiap motor dengan mempertimbangkan pergerakan translasi (x, y, z) dan rotasi (roll, pitch, yaw).

**Topik Input:**
- `/cmd_vel` (geometry_msgs/msg/Twist)

**Topik Output:**
- `motors_vel` (std_msgs/msg/Float32MultiArray)

**Konstanta Gain:**
- K_X, K_Y, K_Z = 500.0
- K_ROLL, K_PITCH, K_YAW = 500.0

**Algoritma Kinematika:**

Motor horizontal (1-4):
- Motor 1 (Front Right): `x - y - yaw`
- Motor 2 (Front Left): `x + y + yaw`
- Motor 3 (Back Right): `x + y - yaw`
- Motor 4 (Back Left): `x - y + yaw`

Motor vertikal (5-8):
- Motor 5 (Up Front Right): `-z + roll + pitch`
- Motor 6 (Up Front Left): `-z - roll + pitch`
- Motor 7 (Up Back Right): `-z + roll - pitch`
- Motor 8 (Up Back Left): `-z - roll - pitch`

Output nilai PWM dijaga dalam rentang 1000-2000 dengan titik netral 1500.

### Node 3: serial_manager

Mengelola komunikasi serial dua arah dengan mikrokontroler:
- Mengirim perintah motor ke mikrokontroler
- Menerima data sensor dari mikrokontroler (status, kedalaman, IMU)

**Topik Input:**
- `motors_vel` (std_msgs/msg/Float32MultiArray)

**Topik Output:**
- `motors_status` (std_msgs/msg/Bool)
- `depth` (std_msgs/msg/Float32)
- `imu` (sensor_msgs/msg/Imu)

**Parameter:**
- `port`: Alamat port serial (default: `/dev/ttyUSB0`)

**Protokol Serial:**

Transmit (ke mikrokontroler):
```
CMD,v1,v2,v3,v4,v5,v6,v7,v8\n
```

Receive (dari mikrokontroler):
```
FB,status,depth,ax,ay,az,gx,gy,gz\n
```
- status: 0/1 (motor status)
- depth: kedalaman dalam meter
- ax,ay,az: akselerasi linear (m/s²)
- gx,gy,gz: kecepatan angular (rad/s)

**Spesifikasi Komunikasi:**
- Baud rate: 115200
- Mode: Non-blocking read
- Frekuensi pembacaan: 100 Hz (10 ms)

## Struktur File

```
eggplant_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── joystick.yaml
├── include/
│   └── eggplant_controller/
│       ├── serial_manager.hpp
│       └── thrusters_8_kinematics.hpp
├── launch/
│   ├── eggplant_controller.launch.py
│   └── joystick.launch.py
└── src/
    ├── joy_to_twist_node.cpp
    ├── serial_manager.cpp
    └── thrusters_8_kinematics.cpp
```

## Instalasi

### Prasyarat

- ROS2 Humble
- Dependensi package: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `joy`
- Hardware: Joystick USB, Mikrokontroler (ESP32/Arduino) terhubung via serial

### Instalasi Package Joy

```bash
sudo apt install ros-humble-joy
```

### Kompilasi Package

```bash
cd ~/ros2_ws
colcon build --packages-select eggplant_controller
source install/setup.bash
```

## Penggunaan

### Konfigurasi Port Serial

Pastikan user memiliki izin akses ke port serial:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Untuk akses permanen, tambahkan user ke grup `dialout`:

```bash
sudo usermod -a -G dialout $USER
```

Logout dan login kembali agar perubahan berlaku.

### Verifikasi Joystick

Cek apakah joystick terdeteksi:

```bash
ls /dev/input/js*
```

Test joystick:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

### Menjalankan Sistem Lengkap

#### Opsi 1: Launch Semua Node

Untuk menjalankan seluruh sistem (joystick + kinematika + serial):

```bash
# Terminal 1: Joystick dan konversi
ros2 launch eggplant_controller joystick.launch.py

# Terminal 2: Kinematika dan serial manager
ros2 launch eggplant_controller eggplant_controller.launch.py
```

#### Opsi 2: Menjalankan Node Individual

```bash
# Terminal 1: Joystick driver
ros2 run joy joy_node

# Terminal 2: Konversi joy ke twist
ros2 run eggplant_controller joy_to_twist_node

# Terminal 3: Kinematika thruster
ros2 run eggplant_controller thrusters_8_kinematics

# Terminal 4: Serial manager
ros2 run eggplant_controller serial_manager --ros-args -p port:=/dev/ttyUSB0
```

### Monitoring dan Debugging

Monitor topik Twist:
```bash
ros2 topic echo /cmd_vel
```

Monitor kecepatan motor:
```bash
ros2 topic echo /motors_vel
```

Monitor data sensor:
```bash
ros2 topic echo /depth
ros2 topic echo /imu
ros2 topic echo /motors_status
```

Visualisasi graph node:
```bash
rqt_graph
```

## Konfigurasi

### File config/joystick.yaml

Parameter joystick dapat disesuaikan:

```yaml
joystick:
  ros__parameters:
    device_id: 0              # ID device joystick
    deadzone: 0.5             # Deadzone untuk mencegah drift
    autorepeat_rate: 20.0     # Frekuensi publikasi (Hz)
    coalesce_interval_ms: 1   # Interval penggabungan pesan
```

### Modifikasi Port Serial

Edit launch file atau gunakan parameter runtime:

```bash
ros2 run eggplant_controller serial_manager --ros-args -p port:=/dev/ttyACM0
```

## Troubleshooting

### Joystick tidak terdeteksi
```bash
# Cek device
ls -l /dev/input/js*

# Test manual
jstest /dev/input/js0
```

### Port serial tidak dapat diakses
```bash
# Cek koneksi
ls -l /dev/ttyUSB*

# Berikan permission
sudo chmod 666 /dev/ttyUSB0

# Atau tambahkan user ke grup dialout
sudo usermod -a -G dialout $USER
```

### Motor tidak bergerak
1. Verifikasi pesan terkirim: `ros2 topic echo /motors_vel`
2. Cek koneksi serial dengan mikrokontroler
3. Monitor output serial di Arduino IDE atau minicom
4. Pastikan ESC sudah dikalibrasi

### Data sensor tidak muncul
1. Cek format protokol serial dari mikrokontroler
2. Verifikasi baud rate (harus 9600)
3. Monitor raw serial data:
```bash
minicom -D /dev/ttyUSB0 -b 9600
```

## Spesifikasi Teknis

### Batasan Sistem
- Rentang PWM: 1000-2000 (netral: 1500)
- Maksimum offset: ±500 dari netral
- Frekuensi kontrol: 20 Hz (dari joystick)
- Frekuensi pembacaan sensor: 100 Hz

### Format Data

**Float32MultiArray (motors_vel):**
```
[m1, m2, m3, m4, m5, m6, m7, m8]
```
Setiap nilai dalam rentang 1000.0 - 2000.0

## Lisensi

Apache License 2.0

## Maintainer

Karol - karolyangqian14@gmail.com
