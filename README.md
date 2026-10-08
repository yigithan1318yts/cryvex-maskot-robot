# Cryvex – kafe maskot robotu

Robot otonom geziyor, engellere çarpmıyor, HOME'dan (barmen) masaya gidip sipariş bekliyor ve HOME'a geri dönüyor.
Bu depo, robotun üzerinde çalışan kodun **8 Ekim 2026** itibarıyla anlık kopyasıdır.
Kaldığımız yerden devam etmek için önce bu dosyayı, sonra `docs/GUNLUK.md`'yi oku.

---

## 1. Donanım (kısa)

| Parça | Detay |
|---|---|
| Bilgisayar | Raspberry Pi 5, ROS 2 Jazzy, kullanıcı `main`, hostname `cryvex-robot` (IP DHCP ile değişiyor, en son `192.168.1.8`) |
| Motor kontrol | STM32 Nucleo (firmware 1.3.4, **kaynak kodu elimizde yok**), USART3 ↔ Pi GPIO 8/10. Metin protokolü `V vx wz`, 200 ms bekçi |
| Tekerler | 2× DM860H sürücü + NEMA34 step motor. **Enkoder yok**: odometri gönderilen adımdan geliyor, adım kaçırınca robot bunu bilmiyor |
| Lidar | YDLidar (360°, 7 Hz), yerden **36 cm** yükseklikte, gövde merkezinden x +0,125 m. Bu düzlemin altını göremez |
| Kamera | Logitech Brio 100, NEMA23 taret üstünde (DMA860H, 6400 adım). **Asla 360° dönmez**: ±180°, geldiği yoldan geri döner (kablo) |
| Gövde | 59 cm geniş (yazılımda yarıçap R = 0,295 m). **Güç kablosuyla çalışıyor**, kablo dolanmasın |
| Yok / sipariş edilecek | Sonar fiziksel olarak takılı değil. IMU yok. Liste: `docs/robot_alisveris_listesi.md` (2× AS5600 enkoder, ICM-20948 IMU, 3× VL53L1X, TCA9548A, tampon + e-stop) |

## 2. Klasörler

```
robot/
  cryvex_araclar/          -> Pi'de ~/cryvex_araclar  (bizim sürüş/algılama/görev kodu)
    devriye.py             ANA SÜRÜŞ: gezinti durum makinesi + GÜVENLİK KAPISI (surt -> guvenlik)
    algilama.py            kamera (YOLO) + lidar -> nesne hitbox'ları, canlı yayın :8081, kamera tareti
    basit_surus.py         temel sınıf (odom, lidar, kamera_bak, donus_guvenli ...)
    guvenlik.json          TÜM GÜVENLİK PAYLARI (dur 35 cm, yavaş 70 cm, dönüş 40 cm, sınıf payları, belirsizlik)
    kamera_ayar.json       kamera FOV/eğim/merkez kalibrasyonu
    govde_maske.json       kameranın gördüğü kendi gövdesi
    gorev/                 YENİ görev sistemi (mevcut koda dokunmaz, onu içeri alır)
      masa_testi.py        HOME -> Masa 1 -> HOME görevi (2 dk haritalama dahil)
      home_hizala.py       HOME hizalama: arkası duvara düz, lidarla kapalı döngü, kamera 0°
      haritala.py          otonom haritalama + HOME kaydı + yardımcılar
      poz_kaydet.py        hareketsiz poz fotoğrafı (lidar + duvarlar)
    (eski araçlar: gezgin.py, otonom_gezgin.py, kesif.py, baslangica_don.py, kalibre_donus.py ...)
  cryvex_hw_ws/src/
    cryvex_bringup/        ROS paketi: stm32_bridge, launch (hardware_bringup, mapping, otonom),
                           config (ekf, nav2_otonom), urdf, cafe_ui_server (telefon uygulaması API :8080)
    ydlidar_ros2_driver/   lidar sürücüsü (kopya)
  cryvex_veri/             -> Pi'de ~/cryvex_veri
    waypoints.yaml         HOME ve MASA_1 (harita koordinatı)
    pozlar/home_referans.json   HOME tanımı (aşağıda)
    haritalar/kafe_20261008_1236.*   BAŞARILI testin haritası (slam_toolbox posegraph)
docs/
  GUNLUK.md                gün gün ne yapıldı, ne öğrenildi  <- MUTLAKA OKU
  mimari_v1.md             onaylanan görev mimarisi (durum makinesi, modüller, 12 aşamalı plan)
  robot_alisveris_listesi.md
  resimler/                başarılı harita, bozulan harita, masa ayağı çarpma fotoğrafı
```

**Bu depoda olmayanlar:**
- `cryvex_araclar/modeller/`: YOLO modeli, 38 MB. Pi'de `yolo11n.pt`'den yeniden üretilir: `yolo export model=yolo11n.pt format=ncnn imgsz=416` → `modeller/yolo11n_416_ncnn_model`. Python ortamı: `~/cryvex_ai` (ultralytics).
- `web/cryvex.apk`: telefon uygulaması, 52 MB.
- Dış ROS paketleri, kendi depolarından `cryvex_hw_ws/src` içine klonlanmalı:
  - https://github.com/MAPIRlab/rf2o_laser_odometry
  - https://github.com/mertgulerx/frontier_exploration_ros2
- Yedek dosyalar (`*.yedek-*`) ve loglar. Bunlar robotta duruyor.

> ⚠️ Telefon uygulamasının operatör şifresi (`1234`) kodda açıkça yazılı (`cafe_ui_server.py`, `bekci.py`, `gezgin.py`). Depoyu gizli tut.

## 3. Çalıştırma

```bash
ssh main@<robot-ip>                      # sadece anahtarla
sudo -n systemctl restart cryvex-bringup # donanım + lidar + EKF + rf2o (yeniden başlatma odometriyi sıfırlar)
source /opt/ros/jazzy/setup.bash; source ~/cryvex_hw_ws/install/setup.bash

# algılama (kamera + taret + canlı yayın http://<robot-ip>:8081/)
nohup ~/cryvex_ai/bin/python -u ~/cryvex_araclar/algilama.py --fov 38 --yuk 0.32 > /tmp/algilama.log 2>&1 &

# SLAM + sade Nav2 (uygulama API'si):
curl -X POST localhost:8080/api/start_mapping -d '{"password":"1234"}' -H 'Content-Type: application/json'

cd ~/cryvex_araclar
python3 -u devriye.py 60                 # 60 sn serbest gezinti   (--dene: hareketsiz, ne gördüğünü yazar)
cd gorev
python3 -u masa_testi.py --dene          # HAREKETSİZ kontrol (her zaman önce bunu çalıştır)
python3 -u masa_testi.py 120             # robot HOME'dayken: hizala -> Masa1 kaydet -> 2 dk harita -> HOME -> Masa1 (10 sn) -> HOME
python3 -u masa_testi.py --devam         # haritalama yapılmışsa: HOME'a dön -> Masa1 -> HOME
python3 -u masa_testi.py 120 --devam     # önce 2 dk gez (SLAM sıfırlanmaz), sonra HOME -> Masa1 -> HOME
python3 -u masa_testi.py --devam --sadece-home
python3 -u home_hizala.py --dene         # HOME hizalaması hareketsiz kontrol
```

⚠️ **pkill dikkat:** `pkill -f` desenini SSH komut satırının içine yazarsan kendi oturumunu da öldürür. Süreci PID ile durdur ya da betik dosyasından çalıştır. `pkill -f /tmp/` asla kullanma; ROS düğümlerinin argümanlarında `/tmp/launch_params` geçiyor.

## 4. Test kuralları (sahibinin kuralları, çok önemli)

1. **Sahibi "başla" demeden hiçbir hareket testi başlamaz.** Önce `--dene` ile hareketsiz kontrol yapılır.
2. **"Dur" denince** önce robotta çalışan programa bakılır (`ps -eo pid,args | grep '[d]evriye\|[m]asa_testi'`), PID ile kapatılır ve sıfır `/cmd_vel` gönderilir. Yerel komut iptal olsa bile `nohup` ile başlamış süreç robotta çalışmaya devam edebilir.
3. **Kablo:** Robot tam tur atmaz. 45°'den büyük dönüşlerden önce sorulur ve 10 sn beklenir. Kamera ±180°'yi geçmez.
4. **Basit ve öngörülebilir davranış:** Her seferinde tek değişiklik yapılır. Salınım yapan "akıllı" düzeltme, yalpalama, dur-kalk, yerinde fırıl fırıl dönme olmaz.
5. **Geometri önce gelir** (`mimari` ve `guvenlik.json`): Her fiziksel cisim, tanınmasa bile ("BİLİNMEYEN") engeldir. Nesnenin sınıfı güvenlik payını sadece **büyütebilir**, asla küçültemez. "Görülmedi" demek "boş" demek değildir. Kameradan tek başına mesafe tahmini yapmak yasak; mesafe lidardan gelir.

## 5. 8 Ekim durumu – nerede kaldık

### ✅ Çalışan
- **HOME → Masa 1 → HOME** (12:36–12:48): Ayrıntılar `docs/GUNLUK.md` "8 Ekim" bölümünde.
  - HOME'da hizalanma: arka duvara +0,15°.
  - 2 dk otonom haritalama.
  - Nav2 + lidarla HOME'a yerleşme: −0,18°.
  - Masa 1'e gidiş: masanın **tam 35 cm** önünde durdu.
  - 10 sn sipariş bekleme.
  - Dümdüz geri dönüş: duvara −0,02°, **0,444 m**.
- **HOME tanımı** (`cryvex_veri/pozlar/home_referans.json`):
  - Robot sahibi tarafından elle konuldu.
  - Arkası duvara düz, gövde merkezi ile duvar arası lidara göre 0,44 m (metreyle ölçülen lidar–duvar mesafesi 57,5 cm).
  - Kamera tam karşıya bakıyor.
  - Masa 1 HOME'un tam karşısında. Bu yüzden gidiş dümdüz ileri, dönüş dümdüz geri; yerinde dönüş yok.
- **Güvenlik kapısı** (`devriye.py` → `guvenlik()`, her sürüş komutu buradan geçer):
  - Gidiş yolunda gövdeye **35 cm'den yakın** bir şey varsa **DUR**.
  - **70 cm** içinde hız en fazla 0,08 m/s.
  - Yerinde dönüş: **40 cm** içinde bir şey varsa dönmez.
  - Önü kapanırsa ve arkası boşsa **dümdüz 30 cm geri** çekilir, orayı 30 sn çıkmaz sayar ve başka bir yöne gider. Test edildi, çalışıyor.
- **Bilinmeyen alan:** İki yakın lidar ölçümü arasında yansıma dönmeyen ışınlar dolu sayılıyor.
- **Algılama:**
  - İnsan etiketi sadece kamera insan derse veriliyor.
  - Hitbox = fiziksel kutu + sınıf payı + ölçüm belirsizliği. Ekranda "en 42±4 cm" şeklinde yazılıyor.
  - Tanınmayan cisimler "BİLİNMEYEN" olarak gösteriliyor.

### ❌ Açık sorunlar (sıradaki işler)
1. **Harita bozuluyor.** Robot masaya takılınca ya da tekerleri boşa dönünce SLAM yerini kaybediyor ve oda iki kopya, ~60° dönmüş halde üst üste biniyor (`docs/resimler/harita_bozuk_1336.png`). Böyle olunca HOME'a otomatik dönüş **güvenli değil**: robotu elle HOME'a koyup temiz tur başlat. **Kalıcı çözüm: AS5600 enkoder + IMU** (EKF'ye). Dururken bile SLAM yönü dakikada birkaç derece kayıyor.
2. **Nav2 global costmap 12×12 m kayan pencere.** 6 m'den uzak hedefi reddediyor (GOAL_OUTSIDE_MAP). `masa_testi.py` artık 4,5 m'lik ara noktalarla gidiyor, ama bu henüz robotta denenmedi.
3. **Nav2 son yönü çeviremiyor** (yerinde dönüş kapalı). Yön, harita (`don_haritayla`) ve lidar (`home_hizala`) ile düzeltiliyor.
4. **Nav2 sürerken bizim güvenlik kapımız devrede değil.** Nav2 kendi collision_monitor'ünü kullanıyor. Mimari plan: Nav2 sadece planlasın, yolu bizim sürücümüz izlesin.
5. **Lidar düzleminin altı kör** (36 cm). Eğik masa ayağı ucu, yerdeki çerçeve ve kablolar görülmüyor; şimdilik sadece paylarla korunuyor. VL53L1X sensörleri gelince ToF katmanı eklenecek.
6. **Uzak ve parlak cisimler:** Lidar 6 m'deki beyaz kutuyu göremedi, 4,4 m'de gördü.
7. `save_map` (pgm) hata veriyor; `serialize_map` çalışıyor.
8. **Birden fazla masa:** Düz çizgide olmayan masalar, masa seçme ekranı ve masa sırası (mimari aşama 9–12).

## 6. Önerilen sıradaki adım
1. Robotu elle HOME'a koy, `masa_testi.py --dene`, sonra `masa_testi.py 120` ile temiz tur at. Yeni güvenlik kapısıyla tam görevi doğrula.
2. Sensörler gelince: bağlantı krokisi → TCA9548A okuyucu → AS5600 + IMU'yu EKF'ye ekle → takılma tespiti → ToF ile alçak engel.
3. Nav2'yi sadece planlayıcı yap, yolu `devriye.py`'nin güvenlik kapısından geçen kendi takipçimiz izlesin.
