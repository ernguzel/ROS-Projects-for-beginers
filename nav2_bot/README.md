# Nav2 Bot Projesi

## Proje Açıklaması
Bu ROS2 projesi, Navigation2 (Nav2) stack kullanarak otonom navigasyon yetenekleri olan bir mobil robot projesidir. Proje, önceden oluşturulmuş bir harita üzerinde gezinme ve navigasyon senaryolarını içerir.

## Özellikler
- Navigation2 stack entegrasyonu
- Gazebo simülasyon ortamı
- Lidar tabanlı navigasyon
- Dinamik engel algılama
- Rota planlama
- Harita üzerinde otonom hareket

## Gereksinimler
- ROS2 Humble
- Gazebo Simulator
- Navigation2 stack
- Colcon build sistemi
- Python 3.8+
- Ubuntu 22.04 LTS

## Harita Bilgisi
Projede `first_my_map.pgm` ve `first_my_map.yaml` dosyaları ile önceden oluşturulmuş bir harita bulunmaktadır. Bu harita, robotun navigasyon için kullanacağı temel referans haritasıdır.

## Kurulum Adımları
1. ROS2 ve gerekli paketleri kurun
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs
```

2. Çalışma alanını hazırlayın
```bash
mkdir -p ~/nav2_bot_ws/src
cd ~/nav2_bot_ws
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
1. Gazebo simülasyonunu başlatın
```bash
ros2 launch kasva_sekli launch_sim.launch.py
```

2. Navigation2 düğümlerini çalıştırın
```bash
ros2 launch kasva_sekli nav2_sekli.launch.py
```

## Proje Yapısı
- `config/`: Kontrol ve köprü konfigürasyonları
- `description/`: Robot URDF tanımlamaları
- `launch/`: ROS2 launch dosyaları
  - `nav2_sekli.launch.py`: Navigation2 başlatma dosyası
- `meshes/`: Robot 3D model dosyaları
- `worlds/`: Gazebo dünya dosyaları

## Sensörler ve Donanım
- Lidar sensörü
- Kamera
- Gazebo kontrol sistemleri
- Navigasyon için gerekli sensör füzyonu

## Notlar
- Proje ROS2 Humble için geliştirilmiştir
- Gazebo simülasyonu gerektirir
- Gerçek robotlarda kullanım için sensör kalibrasyonu gerekebilir

## Sorun Giderme
- ROS2 ve Navigation2 kurulumlarını kontrol edin
- Harita dosyalarının doğru yüklendiğinden emin olun
- Launch dosyalarındaki yol ve parametre ayarlarını doğrulayın
- Gerekirse Navigation2 parametrelerini ayarlayın

## Gelişmiş Kullanım
- Kendi haritanızı oluşturmak için SLAM (Simultaneous Localization and Mapping) araçlarını kullanabilirsiniz
- Navigasyon parametrelerini `config/` dizinindeki dosyalardan özelleştirebilirsiniz
