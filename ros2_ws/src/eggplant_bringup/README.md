# Eggplant Bringup Package

Paket ini merupakan pusat kendali (master launch) untuk sistem robot bawah air Eggplant. `eggplant_bringup` mengintegrasikan kontrol pergerakan, manajemen hardware, dan sistem navigasi berbasis waypoint ke dalam satu namespace terpadu yaitu `/sauvc`.

## Arsitektur Sistem

Sistem dijalankan dalam namespace `/sauvc` untuk memastikan modularitas dan menghindari konflik data. Berikut adalah komponen utama yang dijalankan:

### eggplant_controller

- **serial_manager**: Mengelola komunikasi serial dengan hardware (STM32/ESP32) untuk mengirim perintah motor dan menerima data IMU/Depth.
- **thrusters_8_kinematics**: Melakukan kalkulasi mixing untuk 8 thruster (4 vertikal, 4 horizontal) dan memiliki fitur Auto-Balancing berbasis PID menggunakan data IMU.

### eggplant_waypoint

- **waypoint_sequencer**: Membaca file misi (YAML) dan mengirimkan perintah kecepatan (`cmd_vel`) secara otomatis berdasarkan durasi atau trigger tertentu.

## Persiapan Awal

Pastikan semua dependensi telah terinstal. Dari root workspace, jalankan:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Perintah ini akan menginstal semua dependensi yang didefinisikan dalam `package.xml` masing-masing paket.

## Cara Menjalankan Sistem Utama

Untuk menjalankan seluruh sistem kontrol dan autonomus pada robot (onboard computer):

```bash
ros2 launch eggplant_bringup eggplant.launch.py
```

### Argumen Launch

Anda dapat menyesuaikan jalannya sistem dengan argumen berikut:

- **port**: Alamat port serial (default: `/dev/ttyUSB0`)
- **balancing**: Mengaktifkan fitur self-leveling (default: `false`)
- **mission**: Nama file YAML di folder config `eggplant_waypoint` (default: `mission.yaml`)

Contoh menjalankan dengan balancing aktif:

```bash
ros2 launch eggplant_bringup eggplant.launch.py balancing:=true mission:=gate_task.yaml
```

## Visualisasi dan Monitoring

Sistem visualisasi dipisahkan dari sistem utama agar deployment pada robot tetap ringan. Visualisasi biasanya dijalankan di laptop monitoring (Ground Station).

### 1. Menjalankan Visualizer & RViz2

Gunakan perintah ini untuk memantau pergerakan robot secara real-time:

```bash
ros2 launch eggplant_waypoint visualizer.launch.py
```

Apa yang terjadi di balik layar?

- **displacement_visualizer**: Menghitung estimasi perpindahan robot (dead reckoning) berdasarkan input `/sauvc/cmd_vel`
- **Static TF Publisher**: Menyediakan transformasi koordinat agar frame `odom` dikenali oleh RViz2
- **RViz2**: Menampilkan jalur (Path) berwarna dan posisi robot (Odometry) dalam ruang 3D

### 2. Konfigurasi RViz2

Jika menjalankan RViz secara manual, pastikan pengaturan berikut benar:

- **Fixed Frame**: Ubah menjadi `odom` atau `world`
- **Add Path**: Masukkan topik `/sauvc/visual/path`
- **Add Odometry**: Masukkan topik `/sauvc/visual/odom`

### Interaksi dengan Waypoint

Jika misi Anda menggunakan tipe `wait_for_trigger`, robot akan berhenti dan menunggu perintah sebelum lanjut ke waypoint berikutnya. Jalankan perintah ini untuk memberikan trigger:

```bash
ros2 service call /sauvc/waypoint_sequencer/next std_srvs/srv/Trigger {}
```

## Struktur Topik Penting

| Topik | Tipe | Deskripsi |
|-------|------|-----------|
| `/sauvc/cmd_vel` | `geometry_msgs/msg/Twist` | Perintah kecepatan (Linear X, Y, Z dan Yaw) |
| `/sauvc/imu` | `sensor_msgs/msg/Imu` | Data orientasi dari sensor IMU untuk balancing |
| `/sauvc/visual/path` | `nav_msgs/msg/Path` | Jejak lintasan robot yang divisualisasikan di RViz2 |
