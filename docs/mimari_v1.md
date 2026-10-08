# Cryvex Kafe Maskot Robotu — Sistem Mimarisi v1 (onay için)

**Tarih:** 8 Ekim 2026. **Durum:** Taslak. Onaylanmadan kod yazılmayacak.
**Öncelik sırası:** GÜVENLİK → KONUM DOĞRULUĞU → GÜVENİLİR NAVİGASYON → GÖREV → HIZ

Bu doküman sıfırdan bir sistem önermiyor. Robotta **çalışan ve test edilmiş parçaların** üstüne kurulur. Değişecek yerler açıkça işaretlendi (⚠️).

---

## 0. Bugünkü gerçek durum (mimarinin dayandığı noktalar)

| Parça | Şu an | Not |
|---|---|---|
| ROS 2 Jazzy, Raspberry Pi 5 | Çalışıyor | |
| Lidar YDLidar (360°, 7 Hz, 36 cm yükseklik) | Çalışıyor | Alçak engelleri göremez |
| SLAM: **slam_toolbox** | Çalışıyor | Canlı harita, döngü kapatma |
| Odometri: **EKF (robot_localization)** = teker vx + **rf2o** (lidar odometrisi) dönüş hızı | Çalışıyor | rf2o yavaş harekette hız ölçemiyor (7 Ekim bulgusu) |
| Nav2 (sade: planner, controller RPP+RotationShim, BT, smoother, collision_monitor) | Çalışıyor | `otonom.launch.py` |
| Yerel sürüş: **`devriye.py`** (tünel, huni, kosinüs teoremi, insan süzülme, geri kurtarma, sıfır dönüş) | Çalışıyor | 8 Ekim: 2 dk akıcı test |
| Algılama: **`algilama.py`** (YOLO11n, lidar füzyonu, kamera tareti, gövde maskesi, kablo) | Çalışıyor | Kamera sadece sınıflandırır, mesafe lidardan |
| Uygulama: **`cafe_ui_server.py`** (telefon arayüzü, harita, kurulum ekranı, `waypoints.json`) | Çalışıyor | Eski devriye modülü `patrol.py` |
| STM32 (firmware 1.3.4), metin protokolü `V <vx_mm/s> <wz_mrad/s>` ↓ / `S ...` ↑ | Çalışıyor | ⚠️ **Firmware kaynak kodu elimizde yok.** CRC yok. Teker "odometrisi" gerçek dönüş değil, gönderilen adımdan geliyor. |
| E-stop / tampon girişleri (STM32) | GND'ye köprülü | Devre dışı |

**Karar gerektiren konu (bkz. §G):** STM32 firmware'inin kaynak kodu (STM32CubeIDE projesi) sende var mı?
- **Varsa:** Enkoderleri ve hız denetimini STM32'ye taşırız (doğru yer orası).
- **Yoksa:** Enkoderleri Pi'den okuruz (§D). Güvenli ve çalışır, ama ideal değil.

---

## A. Sistem mimarisi

```
┌──────────────────────────── RASPBERRY PI 5 (ROS 2 Jazzy) ────────────────────────────┐
│                                                                                       │
│  SENSÖR KATMANI                      KONUM KATMANI                 NAVİGASYON          │
│  ydlidar_driver ──/scan───────┬────► slam_toolbox ──map→odom──┐   Nav2 global planlayıcı│
│  sensor_hub (YENİ) ───────────┤      (haritalama / konum)      │   (uzun yol, A*)       │
│   ├ AS5600 ×2 → /wheel/enc    │                                │        │ /plan         │
│   ├ ICM-20948 → /imu/data     ├────► EKF (robot_localization)──┘        ▼              │
│   └ VL53L1X ×3 → /tof/*       │      odom→base_footprint        yerel_surucu (devriye.py│
│  stm32_bridge ──/wheel/odom──►┘                                  mantığının modüler hali)│
│  algilama.py ──/nesneler, kamera                                 tünel / huni / süzülme │
│                                                                   │ /cmd_vel (ham)      │
│  GÖREV KATMANI                         GÜVENLİK KATMANI           ▼                     │
│  mission_manager (durum makinesi) ───► safety_monitor (YENİ) ─► /cmd_vel (güvenli)      │
│  waypoint_manager (HOME, TABLE_n)      e-stop, ToF, ölümcül bölge, takılma, zaman aşımı │
│  home_align (duvar hizalama)                                      │                     │
└───────────────────────────────────────────────────────────────────┼─────────────────────┘
                                                        UART 115200 │
┌──────────────────────────── STM32 (gerçek zamanlı) ───────────────▼─────────────────────┐
│  hız komutu → ters kinematik → step pals üretimi (DM860H ×2)                              │
│  200 ms bekçi: komut gelmezse DUR · e-stop / tampon girişi: yazılımdan bağımsız DUR       │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

**Temel ilkeler:**
1. **Tek komut yolu:** Bütün hareket komutları `safety_monitor`'dan geçer. Görev ve sürüş katmanı güvenliği atlayamaz. E-stop hiçbir yazılım komutuyla geçersiz kılınamaz (§16).
2. **Uzun yol ve yerel davranış ayrı:**
   - Nav2 **sadece yol planlar**; A'dan B'ye hangi koridordan gidileceğini söyler.
   - Yolu **bizim yerel sürücümüz** izler. Bugünkü `devriye.py` davranışları (tünel, huni, sıfır dönüş, insan süzülme, yumuşak fren) bu sürücüde korunur.
   - Böylece Nav2'nin dar yerlerdeki dur-kalk ve yalpalama sorunu geri gelmez.
3. **Her düğüm tek iş yapar ve ayrı test edilir.** Dev bir Python dosyası olmaz.

---

## B. Durum makinesi

```
           ┌──────────┐ sistemler hazır ┌──────────────────┐ harita tamam ┌────────────────┐
  açılış ─►│  INIT    │────────────────►│  SCAN_MAPPING    │─────────────►│  HOME_ALIGN    │
           └────┬─────┘  (harita varsa  └──────────────────┘              └───────┬────────┘
                │         doğrudan ↓)                                      hizalandı│
                ▼                                                                    ▼
           ┌──────────┐  görev: "masa 1" ┌──────────────────┐ yaklaşma  ┌──────────────────┐
           │  IDLE    │◄────────────────│  HOME (hazır)     │◄─────────│  HOME_SAVED       │
           └────┬─────┘                  └────────▲─────────┘          └──────────────────┘
                │ GO                              │ hizalandı
                ▼                                 │
     ┌─────────────────────┐ noktaya 0,6 m ┌──────┴──────────┐
     │ GO_TO_TABLE_n       │──────────────►│ TABLE_n_ALIGN    │
     └─────────────────────┘               └──────┬──────────┘
                                                 │ hizalandı
                                                 ▼
     ┌─────────────────────┐   10 sn bitti ┌─────────────────┐
     │ RETURN_HOME          │◄─────────────│ TABLE_n_WAIT     │ (v = 0, ω = 0, kamera öne)
     └─────────┬───────────┘               └─────────────────┘
               │ HOME'a 0,8 m
               ▼
         HOME_ALIGN ──► HOME

  HER DURUMDAN:  ciddi hata ──► ERROR ──(kurtarılabilir: bekle/yeniden dene)──► önceki durum
                                      └─(kritik: e-stop, lidar yok, iletişim yok)──► DUR + uygulamaya uyarı
```

**Geçiş kuralları:**
- Her durumun **giriş koşulu, çıkış koşulu ve zaman aşımı** var. Örnek: GO_TO_TABLE 120 sn içinde varamazsa → ERROR (kurtarılabilir: yeniden planla, 2 deneme).
- Seyir sırasında yerel durumlar (NORMAL, HUNİ, TÜNEL, GERİ_KAÇIŞ, KİLİTLENME) **yerel sürücünün iç durumlarıdır.** Görev durum makinesi onları görmez, sadece "ilerliyor / takıldı / vardı" bilgisini alır.

---

## C. Navigasyon mimarisi

| Katman | Araç | Görev |
|---|---|---|
| **Haritalama** | slam_toolbox (mapping modu) | Otonom gezinti sırasında doluluk ızgarası çıkarır. Bitince haritayı **serialize edip kaydeder** (konum grafiği + ızgara). |
| **Otonom keşif** | `frontier_exploration_ros2` (robotta kurulu) + yerel sürücü | Bilinmeyen alan sınırlarına sırayla gider. Keşfedecek alan kalmayınca biter. |
| **Konum bulma** | slam_toolbox **localization** modu (kayıtlı harita üstünde). AMCL yedek. | Haritaya göre x, y, yaw. Kafede eşya yer değiştirdiği için slam_toolbox'ın uzun ömürlü haritası AMCL'den daha uygun. |
| **Uzun yol planlama** | Nav2 planner (NavFn, A*; ileride Smac 2D) | HOME → TABLE yolu. Dolu hücreler + robot yarıçapı + güvenlik payı. |
| **Yerel izleme ve kaçınma** | **yerel_surucu** (`devriye.py`'nin modüler hali) | Planın 1–1,5 m ilerisindeki noktayı hedef alır (pure pursuit). Engel olunca bugünkü kurallarla davranır. |
| **Dar geçit** | Kosinüs teoremi + **HUNİ** + **TÜNEL** (korunuyor) | Plan bir dar geçitten geçiyorsa yerel sürücü huni ve tünel moduna girer. Tünelde ω ≤ 0,05 rad/s, sabit hız, sol duvar takibi. |
| **Son güvenlik** | safety_monitor | Ölümcül bölge (lidar + ToF), e-stop, takılma, iletişim. |

**Korunan davranışlar (değişmiyor):**
- Dinamik tünel: genişlik ≥ robot + 10 cm, kosinüs teoremi.
- HUNİ (STATE_FUNNEL_ALIGN): 40 cm önce dur, eksene hizalan.
- Tünelde ω ≤ 0,05 rad/s, 2 cm ölü bant.
- Yavaşlama 0,25 m/s²; acil fren ani ve bağımsız.
- Sol duvar takibi, panel ayağı için 15 cm sanal pay.
- Sıfır dönüş: dar yerde yerinde dönme yok.
- İnsan: yolundaysa süzül, iki yan kapalıysa yol ver.
- Hayalet engel temizleme, takılma hafızası.

⚠️ **Değişecek tek şey:** Bugün `devriye.py` hedefsiz geziyor. Yeni yapıda **Nav2 planından hedef alacak.** Davranış kodu aynı kalır, sadece hedef kaynağı değişir.

---

## D. Sensör füzyonu

| Sensör | Ölçtüğü | Kullanıldığı yer | Sıklık |
|---|---|---|---|
| **AS5600 ×2** (tekerlerde) | Tekerin **gerçek** dönüşü | 1) EKF'ye gerçek teker hızı (vx, ω). 2) **Adım kaçırma:** gönderilen adım ↔ ölçülen dönüş. | 50 Hz |
| **ICM-20948** (IMU) | Dönüş hızı (jiroskop), yön (manyetometre güvenilmez: çelik masa ayakları), ivme | EKF'ye ω ve yön | 100 Hz |
| **Lidar** | 360° mesafe | SLAM, konum düzeltmesi, kaçınma, rf2o | 7 Hz |
| **VL53L1X ×3** (ön, yerden 5–10 cm) | Alçak engel mesafesi | safety_monitor: gidiş yönünde 15 cm'den yakın alçak engel → yumuşak dur; 8 cm → acil | 30 Hz |
| Kamera (YOLO) | Ne olduğu (insan, masa) | Davranış seçimi (insan → süzül/yol ver). **Mesafe için kullanılmaz.** | 4 Hz |

**EKF girişleri (robot_localization):**
- `odom0` = AS5600 teker odometrisi (vx, ω)
- `imu0` = ICM-20948 (ω, yön)
- `odom1` = rf2o (ω; yedek)
- Çıktı: `odom → base_footprint`.
- slam_toolbox `map → odom` düzeltmesini verir.

**Tutarsızlık tespiti (7 Ekim'deki yanlış alarmın dersi):**

| Durum | Koşul | Tepki |
|---|---|---|
| **Adım kaçırma** | STM32'ye gönderilen adım sayısı ile AS5600'ün ölçtüğü dönüş arasındaki fark > %30, **2 sn boyunca** | Step sıfırlama (250 ms bobin gevşetme, 0,06 m/s ile 0,5 sn geri), sonra yeniden dene |
| **Teker kayması** | AS5600 tekerin döndüğünü gösteriyor; IMU ivmesi ve lidar konumu 2 sn'de 2 cm'den az hareket gösteriyor | Hızı düşür, kayma noktasını işaretle, başka yoldan dene |
| **Mekanik engel** | Teker dönmüyor (AS5600 ≈ 0) ama komut var, 2 sn | Dur, geri çekil, ERROR (kurtarılabilir) |
| **Konum kaybı** | SLAM eşleşme skoru düşük ya da EKF ile SLAM arasında > 0,3 m sıçrama | Dur, yerinde bekle, lidar ile yeniden eşleştir; olmazsa ERROR |

Tek bir anlık ölçüme göre karar verilmez; her koşul **zaman penceresiyle** (2 sn) değerlendirilir.

---

## E. HOME hizalama (barmen noktası)

**Amaç:** Robot her seferinde aynı yere ve aynı açıyla (±2 cm, ±1°) dönebilsin.

**Yöntem: arkadaki düz duvar referansı**
1. **Duvar bul:** Lidarın arka sektöründe (150°–210°), robot kenarından 1,5 m içinde, ≥ 60 cm uzunluğunda düz bir yüzey aranır. Doğruya uydurma, aykırı noktaları atarak yapılır (bugünkü `duvar_cizgisi`).
2. **Uygunluk:** Uydurulan doğru düz olmalı (artık hata < 1 cm) ve ≥ 60 cm uzun olmalı. Değilse "uygun duvar yok" → B yöntemi.
3. **Açı hizalama:** Robot, duvar sırtına **dik** olana kadar yavaşça (0,2 rad/s) yerinde döner. HOME açık alanda olduğu için yerinde dönüş serbest; teker süpürme kontrolü yine yapılır. Hata < 1° olunca durur.
4. **Mesafe:** ω = 0 ile 0,05 m/s'de dümdüz geri gider; robot kenarı duvara **25 cm** kalınca durur.
5. **İnce ayar:** Açı bir kez daha ölçülür, < 0,5° düzeltilir.
6. **Kayıt:**
   - Haritadaki x, y, yaw.
   - Duvar referansı: duvar açısı, duvara mesafe.
   - Lidar parmak izi (360° tarama, `baslangic.npz` gibi).
   - Hepsi `waypoints.yaml`'a yazılır.

**Dönüşte:**
1. Nav2 ile HOME'un **0,8 m önüne** gidilir.
2. Aynı 1–5 adımları tekrarlanır.
3. Son kontrol: kayıtlı lidar parmak izi ile yeni tarama ICP ile eşleştirilir (araçları `baslangica_don.py`'de var). Fark > 3 cm ya da > 2° ise ince düzeltme yapılır.

**B yöntemi (arkada duvar yoksa):** Sadece ICP parmak izi + SLAM konumu. Doğruluk ±3–5 cm.

---

## F. TABLE_1 noktası (haritadan elle seçme)

**Arayüz:** Mevcut uygulamanın **kurulum ekranı** genişletilir (telefon ya da bilgisayar tarayıcısı):
1. Kayıtlı harita gösterilir; HOME ve robot üstünde işaretlidir.
2. Kullanıcı nesneye (örn. beyaz yazıcı kutusu) dokunur → **hedef nokta**.
3. Sistem otomatik bir **yaklaşma pozu** önerir:
   - Nesneden boş alana doğru, robot kenarı nesneye **35 cm** kalacak şekilde.
   - Yönü nesneye bakacak şekilde.
4. Kullanıcı öneriyi sürükleyip döndürerek düzeltebilir, sonra kaydeder.
5. Kayıtta hem hedef hem yaklaşma pozu tutulur: `TABLE_1: {hedef: [x,y], yaklasma: [x,y,yaw]}`.

**TABLE_1_ALIGN:**
1. Yaklaşma pozunun 0,6 m önünde hız 0,08 m/s'ye düşer.
2. Son 20 cm'de nesneye olan mesafe lidar (ve ToF) ile ölçülerek durulur; harita koordinatına körü körüne güvenilmez.
3. Yön hizalanır (yerinde dönüş, sadece yer varsa).

**Kayıt formatı (genişletilebilir):** `~/cryvex_veri/waypoints.yaml`
```yaml
harita: kafe_2026_10_08
HOME:    {poz: [0.00, 0.00, 0.0], duvar: {aci: 179.6, mesafe: 0.25}, parmak_izi: home.npz}
TABLE_1: {hedef: [3.20, 1.10], yaklasma: [2.75, 1.10, 0.0], bekle_sn: 10}
# ileride: TABLE_2..n, KITCHEN, CHARGER ...
```

---

## G. Raspberry Pi ↔ STM32 iletişimi

**Bugün:** Metin protokolü, `V vx wz\n` ↓ 10 Hz, `S ...` ↑ 20 Hz. CRC yok. STM32'de 200 ms bekçi var.

**Önerilen (firmware kaynağı varsa): ikili paket, CRC-16**
```
| 0xAA | 0x55 | MSG_ID (1) | SEQ (1) | LEN (1) | PAYLOAD (LEN) | CRC16-CCITT (2) |
```

| ID | Yön | İçerik | Sıklık |
|---|---|---|---|
| 0x01 CMD_VEL | Pi → STM32 | vx (int16, mm/s), wz (int16, mrad/s), bayraklar (motor_enable, stop) | 50 Hz |
| 0x02 HEARTBEAT | Pi → STM32 | görev durumu (uint8) | 10 Hz |
| 0x81 STATUS | STM32 → Pi | sol/sağ gönderilen adım (int32), sol/sağ enkoder (int32, firmware okuyorsa), gerçek hız, motor durumu, e-stop, tampon, hata kodu | 50 Hz |
| 0x82 EVENT | STM32 → Pi | e-stop basıldı, sürücü hatası, bekçi tetiklendi | olayda |

**Davranış:**
- **Bekçi:** STM32, 200 ms paket alamazsa hızı rampayla 0'a çeker ve EVENT gönderir.
- **Pi tarafı:** 300 ms STATUS gelmezse ERROR (kritik).
- **Hatalı paket:** CRC hatalı paket atılır ve sayılır. Saniyede 5'ten fazla hata olursa uyarı verilir.
- **E-stop:** STM32 girişinde. Basılıyken CMD_VEL yok sayılır; bırakılınca bile Pi açıkça "yeniden başla" demeden hareket olmaz.

**Firmware kaynağı yoksa:** Metin protokolü kalır. Pi tarafında komut sıklığı 20 Hz'e çıkarılır, zaman aşımı 0,3 sn'ye indirilir. Enkoder ve IMU **Pi'den** okunur. Adım kaçırma karşılaştırması Pi'de yapılır (gönderilen hız × süre ↔ AS5600). STM32'ye yazılım tarafında dokunulmaz.

**Acil stop donanımı:** Mantar buton iki işi birden yapar:
1. **NC kontak** STM32 e-stop girişinde (yazılım durur).
2. **İkinci kontak** sürücülerin **ENA** hattını keser (DM860H ENA: aktifken motor serbest kalır), **yazılımdan bağımsız.**

Bu "hiçbir yazılım komutu e-stop'u aşamaz" şartını donanımla garanti eder.

---

## H. Yazılım yapısı

Yeni ROS 2 paketi `cryvex_gorev`, mevcut `cryvex_bringup` yanında:

```
cryvex_hw_ws/src/
├── cryvex_bringup/                 (MEVCUT: sürücü, köprü, launch, Nav2 ayarları, uygulama)
└── cryvex_gorev/                   (YENİ)
    ├── config/
    │   ├── robot.yaml              ölçüler, hızlar, ivmeler, eşikler (tek yer)
    │   ├── guvenlik.yaml           ölümcül bölge, ToF eşikleri, zaman aşımları
    │   └── ekf_sensorlu.yaml       AS5600 + IMU + rf2o füzyonu
    ├── cryvex_gorev/
    │   ├── sensor_hub/             TCA9548A + AS5600 + VL53L1X + ICM-20948 okuyucu (thread-safe, 50–100 Hz)
    │   ├── guvenlik/               safety_monitor: tek komut kapısı, e-stop, ToF, takılma, iletişim
    │   ├── yerel_surucu/           devriye.py'nin modülleri: gecit.py (kosinüs, huni), tunel.py,
    │   │                           kacinma.py (süzülme, taraf koruma), kurtarma.py (geri kaçış, kilitlenme),
    │   │                           duvar_takibi.py, hiz_profili.py (rampa, orantılı hız)
    │   ├── gorev/                  mission_manager: durum makinesi (§B), hata yönetimi
    │   ├── noktalar/               waypoint_manager: YAML okuma/yazma, yaklaşma pozu hesabı
    │   ├── home_hizalama/          duvar bulma, hizalama, ICP parmak izi
    │   ├── haritalama/             keşif başlat/bitir, haritayı serialize et, konum moduna geç
    │   └── ortak/                  geometri (doğru uydurma, kosinüs, yol), loglama
    ├── launch/gorev.launch.py
    └── test/                       her modül için birim testleri + kayıtlı lidar verisiyle tekrar testleri
```

`algilama.py` ve `kamera_tarete.py` olduğu gibi kalır (çalışıyorlar). İleride paketin içine taşınabilirler.

---

## I. Gereken yazılımlar

| Yazılım | Durum | Neden |
|---|---|---|
| ROS 2 Jazzy | Kurulu | Temel |
| slam_toolbox | Kurulu | Haritalama + konum (localization modu) |
| Nav2 (planner, BT, collision_monitor) | Kurulu | Uzun yol planlama |
| robot_localization (EKF) | Kurulu | Sensör füzyonu |
| rf2o_laser_odometry | Kurulu | Lidar odometrisi (yedek) |
| frontier_exploration_ros2 | Kurulu | Otonom keşif |
| **python3-smbus2** | Kurulacak | I2C okuma |
| **VL53L1X Python sürücüsü** (`vl53l1x` / Pololu) | Kurulacak | ToF |
| **ICM-20948 sürücüsü** (`icm20948`, Pimoroni ya da SparkFun) | Kurulacak | IMU |
| AS5600 | Kendi küçük sürücümüz (smbus2 ile 2 kayıt okuma) | Basit; harici kütüphane gerekmez |
| **imu_filter_madgwick** (ROS paketi) | Kurulacak | Jiroskop ve ivmeden yön (manyetometresiz) |
| numpy, opencv, ultralytics/ncnn | Kurulu | Algılama |
| pytest | Kurulacak | Testler |
| **STM32CubeIDE + HAL** | Sadece firmware kaynağı varsa | Yeni protokol, enkoder, CRC |

C++ ancak gerçekten gerekirse kullanılır: önce Python ile ölçeriz, Pi 5'te 50–100 Hz I2C okuma Python'da yeterli.

---

## J. Uygulama planı (her aşama ayrı test edilir, onayınla ilerler)

| Aşama | İş | Test / kabul ölçütü |
|---|---|---|
| **1** | Sensör montajı + kablolama (kroki verilecek) | Bütün cihazlar I2C taramasında görünüyor |
| **2** | `sensor_hub`: AS5600, ToF, IMU okuma | Robot elle itilince enkoder ve IMU doğru yönde; ToF 10–100 cm ±1 cm |
| **3** | Enkoder doğrulama | 1 m düz gidiş: enkoder ↔ şerit metre farkı < 2 cm. Tümsekte adım kaçırma yakalanıyor, yanlış alarm yok (10 dk sürüşte 0). |
| **4** | IMU + EKF füzyonu | 10 tam tur yerinde dönüş (kablo izin verirse): yön hatası < 2° |
| **5** | `safety_monitor` | ToF önüne kutu: yumuşak dur. E-stop: anında dur, yazılım geçersiz kılamıyor. |
| **6** | Otonom haritalama + kayıt | Oda tek seferde, döngü kapatarak haritalanıyor |
| **7** | Konum bulma (kayıtlı harita) | Robot elle 2 m taşınınca 10 sn içinde yeniden konum buluyor |
| **8** | HOME hizalama | 10 deneme: ±2 cm, ±1° |
| **9** | Nokta sistemi + TABLE_1 seçme ekranı | Seçilen yaklaşma pozu haritada doğru |
| **10** | HOME → TABLE_1 (plan + yerel sürücü) | 10 gidişte 10 varış, temas yok |
| **11** | 10 sn bekleme + HOME'a dönüş + hizalama | 10 tur: HOME'a ±3 cm dönüş |
| **12** | Hata ve kurtarma testleri | Yol kapatma, insan, kablo, e-stop, lidar kablosunu çekme: her biri doğru duruma gidiyor |

---

## Onayını istediğim kararlar

1. **STM32 firmware kaynağı sende var mı?** Bu, enkoderlerin STM32'ye mi Pi'ye mi bağlanacağını belirler (§G).
2. **Yerel sürüş mimarisi:** Nav2 sadece planlasın, yolu bizim yerel sürücümüz (bugünkü davranışlar) izlesin. Kabul mü?
3. **Konum bulma:** slam_toolbox localization modu, AMCL yedek. Kabul mü?
4. **E-stop:** Butonun ikinci kontağı sürücülerin ENA hattını kessin (yazılımdan bağımsız). Bunun için **2 kontaklı** mantar buton gerekir.
5. **HOME:** Arkada düz bir duvar olacak şekilde mi seçilecek? Barmen noktasının arkasında duvar var mı?
6. **Yaklaşma mesafesi:** Masaya 35 cm, HOME'da duvara 25 cm. Uygun mu?

Parçalar gelene kadar yapılabilecekler (donanım gerektirmez): **Aşama 6–9** (haritalama, konum, HOME hizalama, nokta sistemi). Onay verirsen buradan başlayabiliriz.
