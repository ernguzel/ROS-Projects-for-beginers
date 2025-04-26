# Hunt Workspace (hunt_ws)

## Proje Açıklaması
Bu ROS projesi, bir simülasyon ortamında kaplumbağa avlama senaryosunu içerir. İki ana bileşenden oluşur:
- `bringup`: Projenin başlatma ve yapılandırma ayarları
- `turtle_hunter`: Kaplumbağaları avlama mantığını içeren ana paket

## Gereksinimler
- ROS Noetic veya ROS2 Foxy
- Ubuntu 20.04 LTS
- Python 3.8+
- Colcon build sistemi
- turtlesim paketi

## Kurulum Adımları
1. ROS ortamınızı kurun
2. Çalışma alanını hazırlayın
```bash
mkdir -p ~/hunt_ws/src
cd ~/hunt_ws
```

3. Bağımlılıkları yükleyin
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Çalışma alanını derleyin
```bash
colcon build
```

5. Ortam değişkenlerini kaydedin
```bash
source install/setup.bash
```

## Çalıştırma
1. Turtlesim simülasyonunu başlatın
```bash
rosrun turtlesim turtlesim_node
```

2. Avlama düğümünü çalıştırın
```bash
ros2 launch bringup hunt.launch.py
```

## Proje Yapısı
- `bringup/`: Başlatma dosyaları ve yapılandırma
- `turtle_hunter/`: Avlama mantığını içeren paket
  - `hunting.py`: Ana avlama algoritması
  - `__init__.py`: Paket tanımlamaları

## Notlar
- Proje ROS2 için geliştirilmiştir
- Simülasyon ortamında çalışır
- Kaplumbağaları takip etme ve avlama senaryosu içerir

## Sorun Giderme
- Eğer launch dosyaları çalışmazsa, ROS ortam değişkenlerini kontrol edin
- Python bağımlılıklarını güncel tutun
- Gerekirse `colcon build --packages-select turtle_hunter` ile spesifik paketi derleyin
