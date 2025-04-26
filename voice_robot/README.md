# Ses Kontrollü Robot Projesi

## Proje Açıklaması
Bu ROS2 projesi, ses komutları ile kontrol edilebilen bir mobil robot yazılımını içerir. Proje, ses tanıma teknolojilerini kullanarak robotun hareket ve görevlerini sesli komutlarla yönetmeyi amaçlar.

## Özellikler
- Ses tabanlı robot kontrolü
- Ses tanıma entegrasyonu
- Çoklu komut desteği
- ROS2 düğüm mimarisi
- Gazebo simülasyon ortamı

## Gereksinimler
- ROS2 Humble
- Python 3.8+
- SpeechRecognition kütüphanesi
- PyAudio
- Gazebo Simulator
- Colcon build sistemi
- Ubuntu 22.04 LTS

## Desteklenen Ses Komutları
- İleri git
- Geri git
- Sağa dön
- Sola dön
- Dur
- Başlangıç konumuna dön

## Kurulum Adımları
1. Gerekli ROS2 paketlerini kurun
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-pip
```

2. Python bağımlılıklarını yükleyin
```bash
pip3 install SpeechRecognition
pip3 install pyaudio
```

3. Çalışma alanını hazırlayın
```bash
mkdir -p ~/voice_robot_ws/src
cd ~/voice_robot_ws
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

2. Ses kontrol düğümünü çalıştırın
```bash
ros2 run voice_control voice_control_node
```

## Proje Yapısı
- `voice_control/`: Ana ses kontrol paketi
  - `voice_control.py`: Ses tanıma ve robot kontrol mantığı
- `launch/`: ROS2 launch dosyaları
- `config/`: Ses tanıma ve kontrol konfigürasyonları

## Ses Tanıma Özellikleri
- Türkçe ses komutları desteği
- Düşük gürültü ortamları için optimize edilmiş
- Komut doğrulama mekanizması
- Hata toleransı yüksek tanıma algoritması

## Sensörler ve Donanım
- Mikrofon
- Lidar sensörü
- Kamera
- Gazebo kontrol sistemleri

## Notlar
- Proje ROS2 Humble için geliştirilmiştir
- Gazebo simülasyonu gerektirir
- Gerçek ses tanıma performansı donanıma bağlıdır

## Sorun Giderme
- Mikrofon izinlerini kontrol edin
- Ses tanıma kütüphanesi bağımlılıklarını doğrulayın
- Ses seviyesi ve gürültü ayarlarını kontrol edin
- ROS2 düğüm bağlantılarını doğrulayın

## Gelişmiş Kullanım
- Özel ses komutları eklenebilir
- Farklı dil desteği geliştirilebilir
- Makine öğrenmesi ile ses tanıma geliştirilebilir
