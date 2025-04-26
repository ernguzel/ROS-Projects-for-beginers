# Map Bot Projesi

## Proje Açıklaması
Bu ROS projesi, Gazebo simülasyon ortamında çalışan bir haritalama (mapping) robotudur. Proje, robot tanımlamaları, simülasyon dünyaları ve kontrol mekanizmaları içerir.

## Özellikler
- Gazebo simülasyonu entegrasyonu
- Özel robot URDF (Unified Robot Description Format) tanımlaması
- Lidar ve kamera sensörleri
- Robot kontrol sistemleri
- Haritalama senaryoları

## Gereksinimler
- ROS2 Humble
- Gazebo Simulator
- Colcon build sistemi
- Python 3.8+
- Ubuntu 22.04 LTS

## Kurulum Adımları
1. ROS2 ve Gazebo kurulumunu yapın
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
```

2. Çalışma alanını hazırlayın
```bash
mkdir -p ~/map_bot_ws/src
cd ~/map_bot_ws
git clone [repository_url] src/kasva_sekli
```

3. Bağımlılıkları yükleyin
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Projeyi derleyin
```bash
colcon build
source install/setup.bash
```

## Çalıştırma
1. Simülasyonu başlatın
```bash
ros2 launch kasva_sekli launch_sim.launch.py
```

2. Haritalama senaryosunu çalıştırın
```bash
ros2 launch kasva_sekli map_sekli.launch.py
```

## Proje Yapısı
- `config/`: Kontrol ve köprü konfigürasyonları
- `description/`: Robot URDF tanımlamaları
- `launch/`: ROS2 launch dosyaları
- `meshes/`: Robot 3D model dosyaları
- `worlds/`: Gazebo dünya dosyaları

## Sensörler
- Lidar sensörü
- Kamera
- Gazebo kontrol sistemleri

## Notlar
- Proje ROS2 Humble için geliştirilmiştir
- Gazebo simülasyonu gerektirir
- Haritalama senaryoları için Navigation2 stack önerilir

## Sorun Giderme
- ROS2 ve Gazebo kurulumlarını kontrol edin
- Gerekli tüm bağımlılıkların yüklü olduğundan emin olun
- Launch dosyalarında herhangi bir yol veya parametre hatası olmadığını doğrulayın
