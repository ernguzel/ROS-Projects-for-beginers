# Otonom Şarj Robotu Projesi

## Proje Açıklaması
Bu ROS2 projesi, otonom olarak şarj istasyonunu bulabilen ve kendini şarj edebilen bir mobil robot yazılımını içerir. Proje, Navigation2 ve özel şarj yönetimi algoritmaları kullanarak robotun bağımsız şarj sürecini gerçekleştirir.

## Özellikler
- Otonom şarj istasyonu bulma
- Navigasyon2 stack entegrasyonu
- Pil seviyesi takibi
- Otomatik şarj süreci
- Gazebo simülasyon ortamı
- Lidar ve kamera sensörleri ile çevre algılama

## Gereksinimler
- ROS2 Humble
- Gazebo Simulator
- Navigation2 stack
- Colcon build sistemi
- Python 3.8+
- Ubuntu 22.04 LTS

## Harita Bilgisi
Projede `first_my_map.pgm` ve `first_my_map.yaml` dosyaları ile önceden oluşturulmuş bir harita bulunmaktadır. Bu harita, robotun navigasyon ve şarj istasyonu bulma için kullanacağı temel referans haritasıdır.

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
mkdir -p ~/otonom_sarj_ws/src
cd ~/otonom_sarj_ws
git clone [repository_url] src/
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

2. Otonom şarj düğümlerini çalıştırın
```bash
ros2 launch otonom_sarj otonom_sarj_launch.py
```

## Proje Yapısı
- `config/`: Kontrol ve köprü konfigürasyonları
- `description/`: Robot URDF tanımlamaları
- `launch/`: ROS2 launch dosyaları
- `meshes/`: Robot 3D model dosyaları
- `otonom_sarj/`: Ana şarj yönetimi paketi
  - `otonom_sarj.py`: Şarj algoritması ve yönetimi

## Sensörler ve Donanım
- Lidar sensörü
- Kamera
- Pil seviye sensörü
- Şarj bağlantı algılama sensörleri
- Gazebo kontrol sistemleri

## Şarj Süreci
1. Pil seviyesi düşük olduğunda şarj istasyonu arama
2. Navigasyon ile şarj istasyonuna gitme
3. Şarj bağlantısını kurma
4. Tam şarj olana kadar bekleme
5. Şarj tamamlandığında göreve devam etme

## Notlar
- Proje ROS2 Humble için geliştirilmiştir
- Gazebo simülasyonu gerektirir
- Gerçek robotlarda kullanım için sensör kalibrasyonu gerekebilir

## Sorun Giderme
- ROS2 ve Navigation2 kurulumlarını kontrol edin
- Harita dosyalarının doğru yüklendiğinden emin olun
- Şarj istasyonu parametrelerini doğrulayın
- Pil yönetimi ayarlarını kontrol edin

## Gelişmiş Kullanım
- Şarj stratejilerini `otonom_sarj.py` içinden özelleştirebilirsiniz
- Farklı pil tipleri ve şarj istasyonları için destek eklenebilir
