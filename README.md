# ROS Projeler Koleksiyonu

## Proje Listesi

### 1. Hunt Workspace (hunt_ws)
Kaplumbağa avlama simülasyonu içeren ROS2 projesi.
- **Özellikler**: Turtlesim entegrasyonu, avlama senaryosu
- **Teknolojiler**: ROS2, Python

### 2. Map Bot
Harita oluşturma ve haritalama yetenekleri olan robot
- **Özellikler**: Gazebo simülasyonu, robot tanımlama
- **Teknolojiler**: ROS2, URDF, Gazebo

### 3. Nav2 Bot
Navigation2 stack kullanan otonom navigasyon robotu
- **Özellikler**: Harita üzerinde gezinme, engel algılama
- **Teknolojiler**: ROS2 Navigation2, Gazebo

### 4. Otonom Şarj Robotu
Kendini şarj edebilen otonom mobil robot
- **Özellikler**: Şarj istasyonu bulma, otonom şarj
- **Teknolojiler**: ROS2, Navigation2, Pil yönetimi

### 5. Ses Kontrollü Robot
Ses komutları ile kontrol edilen mobil robot
- **Özellikler**: Ses tanıma, sesli komut kontrolü
- **Teknolojiler**: ROS2, SpeechRecognition, PyAudio

### 6. YOLO Nesne Tanıma Robotu
YOLOv8 nesne tanıma algoritması kullanan robot
- **Özellikler**: Gerçek zamanlı nesne tespiti, çoklu sınıflandırma
- **Teknolojiler**: ROS2, YOLOv8, OpenCV

## Genel Gereksinimler
- ROS2 Humble
- Ubuntu 22.04 LTS
- Python 3.8+
- Colcon build sistemi
- Gazebo Simulator

## Kurulum
1. ROS2 Humble'ı kurun
2. Her proje için kendi README dosyasındaki kurulum adımlarını izleyin

## Proje Yapısı
Her proje kendi çalışma alanında (workspace) organize edilmiştir:
- `src/`: Kaynak kodlar
- `launch/`: ROS2 launch dosyaları
- `config/`: Yapılandırma dosyaları
- `description/`: Robot tanımlamaları
- `meshes/`: 3D model dosyaları

## Geliştirme ve Katkı
- Her proje bağımsız olarak geliştirilebilir
- Kendi README dosyalarındaki detaylı bilgileri inceleyin
- Hata bildirimleri ve geliştirme önerileri için GitHub Issues kullanılabilir



## İletişim
Proje geliştiricisi: [Eren GÜZEL]
