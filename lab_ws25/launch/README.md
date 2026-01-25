# F1Tenth Full System Launch

## Quick Start - Single Command 🚀

Tüm sistemi tek komutla başlatmak için:

```bash
cd ~/repositories/f1tenth_AutonomousBlocking/lab_ws25
source install/setup.bash
ros2 launch launch/full_system.launch.py
```

## Başlatılan Bileşenler

`full_system.launch.py` aşağıdaki tüm bileşenleri otomatik olarak başlatır:

1. **State Machine** - Dinamik raceline seçimi (rakip tespitine göre)
2. **Pure Pursuit** - Yol takip kontrolcüsü
3. **Particle Filter** - Lokalizasyon sistemi (harita üzerinde konum belirleme)
4. **ZED Camera** - Kamera görüntüsü yayınlayıcı
5. **YOLO Detector** - Rakip araç tespit sistemi

## Alternatif: Bileşenleri Ayrı Ayrı Başlatma

Gerekirse her bileşeni ayrı terminallerde de başlatabilirsiniz:

```bash
# Terminal 1: State Machine
ros2 launch state_machine state_machine.launch.py

# Terminal 2: Pure Pursuit
ros2 launch pure_pursuit pure_pursuit_launch.py

# Terminal 3: Particle Filter
ros2 launch particle_filter localize_launch.py

# Terminal 4: Camera
ros2 launch sensors camera.launch.py

# Terminal 5: Opponent Detection
ros2 launch opponent_detection opponent_detector.launch.py
```

## Sistemi Durdurma

```bash
Ctrl+C  # Full system launch'ı durdurmak için
```

## Notlar

- **Keyboard Control:** Eğer state machine'i keyboard ile kontrol etmek istiyorsanız (raceline_mode=2), keyboard_listener'ı ayrı bir terminalde çalıştırmalısınız:
  ```bash
  ros2 run state_machine keyboard_listener
  ```

- **Environment Variable:** Launch dosyası otomatik olarak `LD_PRELOAD` environment variable'ını ayarlıyor (ZED camera için gerekli).

- **RViz:** Pure pursuit launch dosyası otomatik olarak RViz'i başlatır. Harita, raceline'lar ve araç konumunu görselleştirebilirsiniz.

## Troubleshooting

### Kamera bulunamadı hatası
```bash
# Kamera bağlı mı kontrol edin:
ls /dev/video*

# Kamera erişim yetkisi:
sudo usermod -a -G video $USER
```

### Model dosyası bulunamadı
```bash
# Model dosyasının varlığını kontrol edin:
ls ~/repositories/f1tenth_AutonomousBlocking/lab_ws25/src/opponent_detection/models/best.pt
```

### Build hatası
```bash
# Workspace'i yeniden build edin:
cd ~/repositories/f1tenth_AutonomousBlocking/lab_ws25
colcon build
source install/setup.bash
```
