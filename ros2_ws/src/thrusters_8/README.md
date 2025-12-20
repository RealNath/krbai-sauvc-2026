# Thruster Control System - KRBAI SAUVC 2026

Sistem ini dirancang untuk mengendalikan robot bawah air dengan konfigurasi 8 motor (thruster). Sistem bekerja dengan memproses input dari joystick, mengubahnya menjadi koordinat pergerakan (Twist), dan mengirimkan perintah kecepatan motor ke mikrokontroler melalui komunikasi serial.

## Arsitektur Sistem

Sistem terdiri dari tiga lapisan utama:

1. **Input Layer (`joy_to_twist_node`)**: Mengonversi sinyal dari `sensor_msgs/msg/Joy` menjadi `geometry_msgs/msg/Twist`.
2. **Processing Layer (`thrusters_8_node`)**: Menghitung kinematika untuk 8 motor berdasarkan input Twist dan mengirimkan paket data serial.
3. **Hardware Layer (ESP32/Arduino)**: Menerima perintah serial dan mengatur sinyal PWM ke ESC motor.

## Konfigurasi Hardware

### Pemetaan Motor

Robot menggunakan 8 motor dengan pembagian tugas sebagai berikut:

| No | Nama Thruster | Posisi/Fungsi |
| --- | --- | --- |
| 1 | Front Right | Horizontal (Maju, Mundur, Geser, Yaw) |
| 2 | Front Left | Horizontal (Maju, Mundur, Geser, Yaw) |
| 3 | Back Right | Horizontal (Maju, Mundur, Geser, Yaw) |
| 4 | Back Left | Horizontal (Maju, Mundur, Geser, Yaw) |
| 5 | Up Front Right | Vertikal (Naik, Turun, Pitch, Roll) |
| 6 | Up Front Left | Vertikal (Naik, Turun, Pitch, Roll) |
| 7 | Up Back Right | Vertikal (Naik, Turun, Pitch, Roll) |
| 8 | Up Back Left | Vertikal (Naik, Turun, Pitch, Roll) |

### Spesifikasi Elektronik

* **Baud Rate**: 115200.
* **Rentang RPM**: -500 hingga 500.
* **Protokol Serial**: `CMD,v1,v2,v3,v4,v5,v6,v7,v8\n`.

## Logika Kendali Joystick

Pemetaan tombol pada node `joy_to_twist_node` adalah sebagai berikut:

* **Joystick Kiri (Sumbu Vertikal)**: Maju dan Mundur (`linear.x`).
* **Joystick Kiri (Sumbu Horizontal)**: Geser Kiri dan Kanan (`linear.y`).
* **Joystick Kanan (Sumbu Horizontal)**: Rotasi Yaw (`angular.z`).
* **Tombol LB (Hold) + Joystick Kanan (Sumbu Vertikal)**: Naik dan Turun (`linear.z`).

## Instalasi dan Persiapan

### Prasyarat

* ROS2 Humble.
* Paket dependensi: `rclcpp`, `geometry_msgs`, `sensor_msgs`.

### Kompilasi

Lakukan proses build pada workspace ROS2 Anda:

```bash
colcon build --packages-select thrusters_8
source install/setup.bash
```

## Penggunaan

### Konfigurasi Parameter

Parameter dapat diubah melalui file `config/thrusters_8.yaml`:

* `serial_port`: Alamat port serial mikrokontroler (contoh: `/dev/ttyUSB0`).
* `topic_name`: Nama topik untuk menerima pesan Twist (default: `cmd_vel`).

### Menjalankan Program

Gunakan file launch untuk menjalankan seluruh sistem secara bersamaan:

```bash
ros2 launch thrusters_8 thrusters_8.launch.py
```

## Detail Protokol Komunikasi

Setiap paket data yang dikirim ke mikrokontroler harus diawali dengan header `CMD` untuk validasi. Node akan secara otomatis melakukan kalkulasi kinematics dan membatasi (*constrain*) nilai output agar tetap berada dalam batas operasional ESC (1000-2000) sebelum dikirim melalui serial.

Mikrokontroler akan memisahkan pesan berdasarkan tanda koma (CSV format) dan memperbarui nilai PWM setiap motor secara real-time.

---

**Catatan**: Pastikan user memiliki izin akses ke port serial dengan menjalankan perintah `sudo chmod 666 /dev/ttyUSB0` sebelum menjalankan node.
