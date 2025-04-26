# YOLO Nesne Tanıma Robot Projesi

## Proje Açıklaması
Bu ROS2 projesi, YOLOv8 (You Only Look Once) nesne tanıma algoritması kullanan bir mobil robot yazılımını içerir. Proje, gerçek zamanlı nesne tespiti ve takibi yeteneklerini sergiler.

## Özellikler
- YOLOv8 nesne tanıma entegrasyonu
- Gerçek zamanlı nesne tespiti
- Çoklu nesne sınıflandırması
- Nesne takip mekanizması
- Gazebo simülasyon ortamı
- ROS2 düğüm mimarisi

## Gereksinimler
- ROS2 Humble
- Python 3.8+
- Ultralytics YOLOv8
- OpenCV
- NumPy
- Gazebo Simulator
- Colcon build sistemi
- Ubuntu 22.04 LTS

## Desteklenen Nesne Sınıfları
- İnsan
- Araç
- Hayvan
- Mobilya
- Elektronik cihazlar
- Diğer günlük nesneler

## Kurulum Adımları
1. Gerekli ROS2 paketlerini kurun
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-pip
```

2. Python bağımlılıklarını yükleyin
```bash
pip3 install ultralytics
pip3 install opencv-python
pip3 install numpy
```

3. Çalışma alanını hazırlayın
```bash
mkdir -p ~/yolo_bot_ws/src
cd ~/yolo_bot_ws
git clone [repository_url] src/
```

4. Bağımlılıkları yükleyin
```bash
rosdep install --from-paths src --ignore-src -r -y
```

5. Projeyi derleyin
```bash
colcon build
source install/setup.bash
```

## Çalıştırma
1. Gazebo simülasyonunu başlatın
```bash
ros2 launch kasva_sekli launch_sim.launch.py
```

2. YOLO nesne tanıma düğümünü çalıştırın
```bash
ros2 run yolo_bot detect_node
```

## Proje Yapısı
- `yolov8n.pt`: Ön eğitimli YOLOv8 Nano modeli
- `launch/`: ROS2 launch dosyaları
- `config/`: Nesne tanıma konfigürasyonları

## Nesne Tanıma Özellikleri
- Yüksek doğruluk oranı
- Düşük gecikme süresi
- Gerçek zamanlı performans
- Esnek sınıflandırma
- Çoklu nesne tespiti

## Sensörler ve Donanım
- Kamera
- Lidar sensörü
- GPU (performans için önerilir)
- Gazebo kontrol sistemleri

## Notlar
- Proje ROS2 Humble için geliştirilmiştir
- Gazebo simülasyonu gerektirir
- Gerçek performans donanıma bağlıdır

## Sorun Giderme
- CUDA ve GPU sürücülerini kontrol edin
- OpenCV ve NumPy bağımlılıklarını doğrulayın
- Model ağırlıklarının doğru yüklendiğinden emin olun
- ROS2 düğüm bağlantılarını kontrol edin

## Gelişmiş Kullanım
- Özel eğitimli modeller yüklenebilir
- Farklı nesne sınıfları eklenebilir
- Performans için GPU hızlandırma yapılandırılabilir
- Makine öğrenmesi ile nesne tanıma geliştirilebilir
