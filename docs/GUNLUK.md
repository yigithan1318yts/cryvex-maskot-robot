# Cryvex – Çalışma Günlüğü

Her çalışma gününün sonunda buraya eklenir. Robotta: `~/cryvex_araclar/GUNLUK.md`

---

## 5 Ekim 2026 (Pazartesi)

### Donanım
- Sol teker göbeği mile kama ile sabitlendi; "zayıf sol teker" sorunu buydu.
- Pi ↔ Nucleo seri hattı USART3'e bağlandı (CN7 pin 1/2 ↔ Pi pin 10/8, GND pin 6).
- Acil stop (A1) ve tampon (A0) girişleri **geçici olarak GND'ye köprülü**; güvenlik devre dışı, düzeltilecek.
- Robot genişliği ölçüldü: **59 cm** (tekerden tekere). Yazılımın her yerinde yarıçap 0,30 m.
- Logitech **Brio 100** kamera lidarın hemen altına, öne bakacak şekilde takıldı. Lidar 360° görmeye devam ediyor (kontrol edildi).

### Yazılım – yapılanlar ve durumu
| Konu | Durum |
|---|---|
| Dönüş kalibrasyonu yeniden yapıldı (ODOM_WHEEL_BASE_M 0,549, ANGULAR_CMD_GAIN 1,02) | ✅ Ölçüldü, dönüş %100 uyumlu |
| Lidar odometrisi (rf2o) kuruldu, EKF yönü lidardan alıyor | ✅ Sapma 1-2° |
| SLAM ayarları (±20° arama), harita kaymıyor | ✅ |
| Nav2 otonom yığını: MPPI denetleyici, Smac planlayıcı, Collision Monitor, Spin yok | ✅ Açılıyor · ⚠️ uzun mesafede "ilerleme yok" hataları |
| Keşif paketi frontier_exploration_ros2 | ✅ Kurulu |
| `otonom_gezgin.py` – keşif / devriye / taşınma algılama / dar yer kurtarma | ✅ Çalışıyor · ⚠️ devriye hedeflerinin bir kısmı başarısız |
| `koridor_cik.py` – iki masa/palet arasından düz ileri-geri çıkış, yerleşip ortalanma, masa girişini kaydetme | ✅ 70 cm aralıktan çıktı (2-3 cm kenar payı) |
| `kamera_engel.py` – kamera nesneleri tanıyor, lidar uzaklığını ölçüyor, Nav2 haritasına yazıyor | ✅ ~3 kare/sn · ⚠️ paleti "sandalye" sanıyor |
| Telefon sayfaları: `:8080/otonom` (Keşfet / Devriye / DUR), `:8081` (canlı kamera + lidar) | ✅ |
| Bekçi (1 dk hareketsiz kalırsa durdurur, ilk 2,5 dk açılış payı) | ✅ |

### Öğrenilenler
- Paletlerin alt kısmı lidarın göremediği yerde içeri taşıyor (lidar 84-88 cm görüyor, gerçekte 64-70 cm). **Lidar tek düzlemde tarıyor**; masa tablası, yayvan ayak tabanı ve çanta gibi şeyleri göremez.
- 59 cm'lik robot için güvenilir en dar geçit yaklaşık **70 cm**. 60 cm'lik aralıklar ancak yan sensör ve yumuşak tamponla mümkün olur.
- Robot koridor içindeyken **asla yerinde dönmemeli**; sadece düz ileri ya da geri gitmeli. Bu kural koda eklendi.
- Nav2 ayar dosyası YAML takma adı (&id) içerirse bütün Nav2 parçaları çöküyor; dosya takma adsız yazılmalı.

### Yarıda kalan / denenmemiş
- Açılışa eklenen kamera ve 12 maddelik kontrol listesi: **yarın ilk iş**.
- 0,45 m/sn hız, masaya dönüşün yeni yaklaşımı (girişin 80 cm önü, sonra koridora gir), kameranın görüş açısı (48° varsayıldı).

### Yarının planı (6 Ekim)
1. Kontrol listesi: robot hareket etmeyecek.
2. Masa testi: paletlerden çık → 2 dk devriye → masaya dön ve gir (kullanıcı "başla" deyince).
3. **Yapay zekâ ekleme** – seçenekler:
   - **Hailo yapay zekâ kartı (Raspberry Pi AI HAT+)**: Pi 5'in üstüne takılıyor; YOLO ile saniyede 30+ kare nesne tanıma. İşlemciyi Nav2'ye bırakır, tanıma çok daha doğru olur (palet ≠ sandalye).
   - **Daha iyi model** (YOLO serisi): masa, sandalye, insan, çanta vb. daha doğru tanınır.
   - **Tek kameradan derinlik tahmini** (Depth Anything gibi): lidarın göremediği masa tablası ve alçak engellerin uzaklığını tahmin eder. Yapay zekâ kartıyla gerçek zamanlı çalışabilir.
4. Sırası gelince: acil stop ve tamponu gerçek düğmelere bağlama; ToF sensörleri gelince yan ve alt algılama.

---

## 6 Ekim 2026 (Salı)

### Yapılanlar
| Konu | Durum |
|---|---|
| Sabah kontrol listesi (12 madde, hareketsiz) | ✅ Hepsi tamam |
| **Yapay zekâ: YOLO11n** (NCNN, 416 px), ayrı Python ortamı `~/cryvex_ai` | ✅ ~6 kare/sn, 80 nesne türü |
| **algilama.py** – lidar kümeleri + YOLO + zemin üstü engel → hitbox'lar, en/boy ölçümü, insan kırmızı kutu, geçit (aralık) hesabı | ✅ Açılışta otomatik başlıyor |
| Titreme önleme: cisimler dünya koordinatında takip, 5 karenin 3'ünde görülme şartı, yumuşatma | ✅ Kutular sabit |
| Engel hafızası: sabit cisimler 30 sn, zemin engeli 5 sn, insan 1,5 sn | ✅ |
| Kamera kaydı: saniyede 1 kare, son 15 dk (`~/cryvex_araclar/kayit/`) | ✅ |
| Canlı ekran `:8081` – kendini yenileyen sayfa (donmuyor) | ✅ |
| Nav2: kamera katmanı + Collision Monitor kamerayı da görüyor; genel costmap canlı (SLAM izi yok); BT bekleme 500 ms; planlayıcı NavFn | ✅ |
| MPPI yerine **Rotation Shim + Regulated Pure Pursuit** (önce hızlı dön, sonra akıcı git, geri gitme kapalı) | ⚠️ Kuruldu, henüz denenmedi |
| Geçit görevi: önü açıksa dönmeden dümdüz geç; önüne biri çıkarsa geldiği izden geri çık | ⚠️ Kısmen denendi |
| Video inceleme: telefondan video → robotta ffmpeg (kareler) + Whisper (konuşma yazıya) | ✅ |
| **Kamera taretesi** (NEMA 23 + CWD556), `kamera_tarete.py`, ±180° kesin sınır | ⚠️ Yazılımda ±15° testi çalıştı, hareketi gözle doğrulanmadı |

### Sorunlar / öğrenilenler
- **Gri duvar ve siyah tahta (mat koyu yüzey) lidarda neredeyse görünmüyor.** Geri geri çıkarken sol teker gri duvara sürttü. Düzeltme: geri çıkışta gelinen iz takip ediliyor + engel hafızası.
- MPPI dolu ortamda neredeyse kıpırdamıyordu (ort. 2 cm/s) → robot ufak ufak dönüp titriyordu. Denetleyici değiştirildi.
- Videoda **"teker boşta dönüyor"** dendi → sol tekerin kaması kontrol edilmeli.
- Sahte zemin engelleri (duvar dibi gölgesi) hâlâ zaman zaman çıkıyor.

### Kamera motoru bağlantısı
- Sürücü: B−, B+, A−, A+ = motor (gri), +Vdc = kırmızı → Omron +V, GND = mavi → Omron −V (boş vida). Sarı/beyaz yalıtılmış.
- DIP: SW1-3 ON, SW4 OFF, SW5 ON, SW6 OFF, SW7 ON, SW8 ON (en düşük akım, 1600 adım/tur).
- Pi: pin 11 → PUL+, pin 13 → DIR+, pin 9 → PUL− + DIR− (köprü). ENA boş.

### Yarın
1. Robota bağlan (en son SSH bağlantıyı reddediyordu; robotu yeniden başlat).
2. Kamera motoru: ±30° testi, yön doğru mu (+ = sola), adım kaçırıyor mu.
3. **Kendini görme filtresi:** kamera sağa/arkaya dönünce lidar direği ve güç kaynağını engel saymasın (gövde içi filtresi + bir kerelik öğrenme taraması).
4. Tareteyi algılamaya bağla (açıya göre izdüşüm, dönerken kare işleme yok, hareket yönüne bakma).
5. Yeni denetleyiciyle (RPP) geçit + devriye testi.
6. Sol teker kaması kontrolü; tampon bağlantısı.

---

## 7 Ekim 2026

### Kamera motoru (taret)
- Yeni sürücü **Leadshine DMA860H**:
  - Güç: 24 V iki AC ucuna.
  - Motor: A+ siyah, A− yeşil, B+ kırmızı, B− mavi; sarı/beyaz bantlı.
  - Pi: pin 11 PUL+, pin 13 DIR+, pin 9 PUL−/DIR−.
- DIP: **4 ve 7 OFF, geri kalanı ON** (en düşük akım, **6400 adım/tur**). 1600'de titriyordu.
- Yön: + = sola (`YON_TERS = True`).
- **Ortalama:** Duvardaki + işaretiyle ölçüldü; sıfır −9,3° kaymıştı, düzeltildi. Gerçek yatay görüş açısı **~38°** (`kamera_ayar.json`).
- Hız 90°/s, ivme 300°/s².
- Motor süreci 3. çekirdekte, gerçek zamanlı öncelikle (`chrt`). Yapay zekâ 0–2. çekirdeklerde.
- Kablo kuralı: ±180°, tam tur yok. Elektrik hareket ortasında kesilirse kilitli açılır (kamerayı öne çevir + `--sifirla`).
- Kamera **aşağı eğildi**. Gövde maskesi yeniden tarandı (`govde_maske.json`; −120…−150° kör, lidar direği).

### Algılama (`algilama.py`)
- Kamera motoru ayrı süreç. Dönerken çekilen kareler kullanılmıyor.
- **Kamera SADECE sınıflandırıyor; mesafe her zaman LİDAR'dan** (pikselden mesafe yasak).
- **Zemin algılama KAPALI** (parlak zemin, koyu panel ve gölgeyi engel sanıyordu).
- Gövde içi lidar noktaları (r < 30 cm = robotun kendi güç kablosu/fişi) atılıyor.
- Tek başına duran ince direk (vantilatör) → etrafı 32 cm dolu (tabanı lidarın altında).
- Ayaklar birleşiyor: 75 cm içindeki ince kümeler tek mobilya; 3–4 ayak dikdörtgen köşesindeyse tek masa.
- İnsan payı (+23 cm) sadece kararlarda; Nav2'ye paysız kutu gider.
- Kamera bakışı sakin: dururken öne bakıyor, sadece yeni beliren yakın şeye kısa bakış.

### Sürüş: `devriye.py` (yeni, tek durum makinesi; Nav2 sürmüyor)
- **NORMAL:** Dümdüz; önü kapanınca bir taraf seç ve koru (taraf değiştirme cezası, 250 ms dönüş histerezisi).
- **İnsan:** Yolundaysa durmadan karşı tarafından süzül (yan pay 15 cm); iki yan kapalıysa dur, 3 sn yol ver.
- **İki engel arası geçit (kosinüs teoremi):** Ön ±45°, 69–120 cm, eksen ±45°, arkası açık.
- **HUNİ:** Giriş noktasına git, eksene dön, çaprazlar eşit → TÜNEL.
- **TÜNEL:**
  - Sadece iki yan duvar varsa ve eksene ≤15° iken.
  - Orta çizgi kilitli, ölü bant 2 cm, dönüş en fazla 0,05 rad/s.
  - Hız 0,12 m/s, mesafeyle orantılı.
  - Panel varsa o yana 15 cm sanal pay (tahta ayağı).
  - Tek duvar takibi.
- **TÜNELDE ÖNÜ KAPANIRSA:** Arka açıksa ω = 0 dümdüz geri. Değilse kilitlenme bekleme: ön ya da arka 1,5 sn açık kalırsa çık; 3 sn'de hafıza temizleme.
- **Sıfır dönüş:** Dar yerde yerinde dönme yok; tekerlerin süpüreceği yay kontrol ediliyor.
- **Hız:** Hızlanma **0,04 m/s²** (step adım kaçırmasın), yavaşlama 0,25 m/s². Acil fren ani.
- **Hayalet engel:** Lidar boş görünce çıkmaz kaydı siliniyor (30 sn ömür). Takılma kayıtları ayrı (60 sn, silinmez).
- **Tahta koridoru tanıma:** 0,9–2 m düz panel + paralel duvar, 80–125 cm.

### Sistem
- **Nav2 sadeleştirildi:** `otonom.launch.py` sadece planner, controller, behavior, bt_navigator, velocity_smoother, collision_monitor. İşlemci yükü %229'dan %179'a indi.
- BT'den kör BackUp kaldırıldı; RotationShim eşiği 1,3 rad; collision_monitor min_points 4.
- `bekci.py`: gezgin bilerek beklerken (`gezgin_bekliyor`) testi kesmiyor.
- Pi–STM32 kontrolü:
  - Komut 10 Hz (20'ye çıkarılabilir).
  - STM32'de 200 ms bekçi var.
  - Paket doğrulaması (checksum) yok.
  - **IMU takılı değil, sonarlar fiziksel olarak takılı değil.**

### En önemli bulgu: adım kaçırma
- Tekerler step motorlu; **teker odometrisi gönderilen adımdan hesaplanıyor**, gerçek dönüşten değil.
- Kaygan ya da hafif eğimli zeminde motor adım kaçırınca yazılım bunu göremiyor ("tekerler dönüyor ama robot gitmiyor").
- Lidar odometrisi (rf2o) yavaş hızda hareketi ölçemiyor → takılma tespiti **yanlış alarm** veriyordu. Son testte robot 60 cm ilerlerken "ADIM KAÇIRMA" dedi.
- **Sonuç:** Gerçek teker enkoderi + IMU + alçak ToF şart.
- Alışveriş listesi: `robot_alisveris_listesi.md`.

### Kullanıcının istediği davranış (özet)
- Engelleri gör ve çarpma; sığdığı aralıktan geç, sığmadığına girme.
- Önüne biri çıkarsa: arka boşsa geldiği gibi dümdüz geri; önü-arkası kapalıysa bekle; hangi taraf açılırsa oradan çık.
- Yerinde fırıl fırıl dönme yok, yalpalama yok, dur-kalk yok, kendini sıkışmış sanma yok.
- Kamera ±180°, asla tam tur.
- Her test öncesi kullanıcı "başla" der.

### Sıradaki adımlar (parçalar gelince)
1. Bağlantı krokisi:
   - Pi I2C → TCA9548A.
   - Kanal 0–1: AS5600 (tekerler); kanal 2–4: VL53L1X (ön).
   - IMU.
2. Çoklayıcılı okuma kodu (thread-safe, 20–50 Hz).
3. `devriye.py`:
   - **Takılma tespiti:** AS5600 gerçek dönüşü + IMU ile (rf2o yerine).
   - **Alçak engel:** ToF.
4. Tampon ve acil stop: NC, STM32 girişlerine.
5. Tekerlere poliüretan/silikon kaplama (sert plastik kayıyor); ağırlığı aksa yaklaştır.
6. İsteğe bağlı: STM32 komut sıklığını 20 Hz'e çıkar, Pi komut zaman aşımını 0,3 sn'ye indir.

---

## 8 Ekim 2026 — İLK TAM BAŞARI: HOME → Masa 1 → HOME ⭐

> Kullanıcı: "mükemmel bir şekilde çalıştı, 5 günde yaptığımız en iyi şey buydu — genel olarak zaten bunu yapması lazım: masa masa gidip sipariş almak."

Arşiv: `Documents\Cryvex\basari_20261008_masa1\` (kod, loglar, harita resmi, kamera kareleri, robot tarafı `.tgz`).

### Sonuç (12:36–12:48)
| Adım | Sonuç |
|---|---|
| HOME hizalama | arka duvara **+0,15°**, 0,44 m (lidar–duvar 57,5 cm), kamera 0° |
| Masa 1 kaydı | tam karşıdaki kutu, ön yüzden 4,30 m |
| 2 dk otonom haritalama | oda düzgün, çift duvar yok |
| HOME'a dönüş | Nav2 HOME'un 0,8 m önüne → harita yönüyle dönüş → lidarla ince ayar → dümdüz geri → **−0,18°, 0,44 m** |
| Masa 1'e gidiş | yolda kablo (yavaş, dümdüz geçti), önüne biri çıktı (bekledi), masaya **tam 35 cm** |
| Sipariş bekleme | 10 sn |
| HOME'a dönüş | dümdüz geri → **−0,02°, 0,444 m** (kullanıcının elle koyduğu pozla aynı) |

### Neden çalıştı
- **Basit:** Masa HOME'un tam karşısında → gidiş dümdüz ileri, dönüş dümdüz geri; yerinde dönüş yok (kablo).
- **Lidarla kapalı döngü:** Dönüş açısı teker odometrisinden değil, her taramada duvara bakarak ölçülüyor (step adım kaçırdığı için odometri ±%100 hata yapıyordu: −5,7° komut → ~12° dönüş).
- **HOME = kullanıcının elle koyduğu poz:** arkası duvara düz, merkez–duvar 0,44 m (lidar ölçümü), kamera tam karşı (`~/cryvex_veri/pozlar/home_referans.json`).
- **Yanlış duvara dönmeme:** HOME'a dönüşte duvar haritadan bilinen yönde (±35°) aranıyor.

### Yeni dosyalar (`~/cryvex_araclar/gorev/`, mevcut kod değişmedi)
- `masa_testi.py`: tüm görev. Kullanım: `[gezinti_sn]`, `--devam` (haritadan sonrası), `--dene` (hareketsiz kontrol).
- `home_hizala.py`: arkası en yakın düz duvara paralel, referans mesafeye geri, kamera 0°.
- `haritala.py`: otonom haritalama + HOME kaydı. `poz_kaydet.py`: hareketsiz poz fotoğrafı.
- `~/cryvex_veri/waypoints.yaml`: HOME, MASA_1.
- `algilama.py` küçük düzeltme: 10 sn kıpırdamayan ve kameranın insan demediği nesne "insan" etiketini kaybeder (kutunun yanında durulunca kutu kırmızı/insan kalıyordu).

### Öğrenilenler / açık işler
- Nav2 son yönü çeviremiyor (yerinde dönüş kapalı) → konum 25 cm içindeyse yönü biz düzeltiyoruz.
- Robot dururken SLAM yönü dakikada birkaç derece kayıyor → enkoder + IMU gelince.
- Lidar 6 m'deki beyaz kutuyu göremedi (zayıf yansıma / hafif eğiklik); 4,4 m'de gördü.
- `save_map` (pgm) hata veriyor; konum grafiği (serialize) kaydediliyor.
- Sıradaki: birden fazla masa (düz çizgide olmayan), masa seçme, masa sırası.

---

## 8 Ekim 2026 (öğleden sonra / akşam) — güvenlik, GitHub, panel, uygulama

### 1. Masa ayağına çarpma → evrensel nesne kuralı ve güvenlik kapısı
- **Olay (13:00, 2. tur):** Haritalama gezintisinde X ayaklı masanın ayağına sağ ön tekerle çarptı ve takıldı.
  - **Sebep 1:** Kamera insan kutusunun 0,9 m gerisindeki kümelere de "insan" diyordu.
  - **Sebep 2:** "Hareket eden ince cisim = insan" kuralı vardı. Adım kaçırınca duran ayak da "hareketli" göründü.
  - **Sebep 3:** "İnsanın yanından durmadan süzül" kuralı 10 cm'de bile hızı en az %70'te tutuyordu.
- **Kullanıcının kuralı (evrensel nesne algılama):**
  - Geometri önce gelir. Sınıf güvenliği **sadece artırabilir**.
  - Bilinmeyen alan boş sayılmaz.
  - Paylar ayarlanabilir olmalı. Ölçüler ± belirsizlikle verilmeli.
- **Yapılanlar:**
  - **`guvenlik.json`:** Tüm paylar tek dosyada.
  - **`devriye.py` → `guvenlik()` tek kapı.** Her `surt()` buradan geçer:
    - gidiş şeridinde 35 cm'de DUR, 70 cm'de 0,08 m/s;
    - yerinde dönüş için 40 cm yarıçapta engel olmamalı;
    - hitbox sadece nesneye doğru giderken sayılır; robotun gövdesine binen kutu güvenilmez sayılır;
    - gövde çevresindeki 2 cm (kendi kablosu) yok sayılır.
  - **Ön kapanırsa:** Arka boşsa dümdüz 30 cm geri çekilir, o yeri 30 sn çıkmaz sayar. Geri giderken sadece gerçek lidar noktalarına bakılır, çünkü uydurulan masa kutuları arkayı kilitliyordu.
  - **`lidar()`:** İki yakın ölçüm arasında yansıma dönmeyen ışınlar dolu sayılıyor.
  - **`algilama.py`:**
    - insan etiketi sadece kamera "insan" derse ve sadece en yakın, insan boyundaki kümeye;
    - hitbox = fiziksel ölçü + sınıf payı + belirsizlik;
    - ekranda "BİLİNMEYEN", "en 42±4 cm".
- **Test:** Masaya yaklaşınca durdu, dümdüz geri çekildi, başka yöne gitti. ✔

### 2. HOME'u kaybettiğinde bulma: `gorev/home_bul.py`
- **Sorun:** Takılma sonrası SLAM haritası ~60° dönmüş ikinci bir kopya ile bozuldu (`docs/resimler/harita_bozuk_1336.png`).
- **Çözüm:** Haritayı kullanmadan, şu anki lidar taramasını HOME'daki kayıtla (`kullanici_koydu.npy`) her yön ve konumda eşleştiriyor (mesafe alanı + ince arama).
  - Sonuç: "HOME'un 1,5 m önünde, 150° ters".
  - Sonra SLAM sıfırlanır → harita yönüyle dönüş (10 sn kablo beklemesi) → lidarla hizalama → dümdüz geri.
- **Test:** HOME'a döndü. Doğrulama: ileri-geri 0 cm, yanda 5 cm, yön 0°. ✔
- **Nav2 notu:** Global costmap 12×12 m kayan pencere; 6 m'den uzak hedefi reddediyor. `masa_testi.py` 4,5 m'lik ara noktalarla gidiyor (bu yol robotta henüz denenmedi).

### 3. GitHub ve arkadaşla ortak çalışma
- **Depo:** https://github.com/yigithan1318yts/cryvex-maskot-robot (açık)
  - Robotun kodu, ROS paketleri, YOLO modeli, belgeler, README (devir notu).
- **Gönderme:** Bilgisayara Git + GitHub CLI kuruldu. `git push` için izin kuralı eklendi. Kural: sadece önemli/büyük değişiklikler gönderilir.
- **Robotu aynı anda iki Claude kullanıyor.** Arkadaş robottaki dosyaları doğrudan değiştiriyor (patrol takılma algılama, şifre dosyası `~/.config/cryvex/operator_pin`, `konum_bulucu.py`). **Robottaki bir dosyanın üzerine yazmadan önce mutlaka karşılaştır.**
- **Asıl uygulamanın kaynağı bulundu:** https://github.com/imre-robotics/cryvex_project_maskot (`cryvex_app`, paket `com.cryvex.cryvex_control`). İçinde `stm32_firmware` de var.

### 4. Cryvex Görev Paneli: `http://<robot>:8090/`
- **Yer:** `cryvex_araclar/panel/` (`panel_sunucu.py` + `panel.html`). Robot açılışında crontab `@reboot` ile kendiliğinden başlar.
- **İçerik:**
  - canlı kamera + harita;
  - görevler: HOME→Masa1→HOME, HOME'a dön, kısa devriye, hareketsiz kontrol;
  - şifre + "kablo / etraf boş" onayı;
  - hareket kilidi (`~/HAREKET_KILIDI`);
  - şifresiz büyük DUR;
  - canlı log;
  - güvenlik ayarları (kaydırma çubukları).
- **Görünüm:** CryvexTech kimliği (cryvextech.com.tr): X logosu, `#26ccea` / `#0d7f96`, Space Grotesk / Inter. Telefona eklenebilir.
- **Uygulamaya bağlantı:** Operatör menüsünde (`index.html`) 🧭 Görev Paneli düğmesi.

### 5. Telefon uygulaması: Cryvex Kontrol 1.2.0
- **Kurulum:** Flutter 3.47.6 (`C:\flutter`), Android SDK, JDK 17.
- **Yenilikler:**
  - Görev Paneli ekranı + ⛔ ACİL DUR;
  - joystick yerine **yön tuşları** (basılı tut = git, bırak = dur, Yavaş/Normal);
  - **harita kurulumu sürükle-bırak şekiller**: dikdörtgen / daire / duvar; engel / boş alan; taşı / boyut / döndür / cm gir / kopyala;
  - **uygulama içi güncelleme** (`:8090/api/surum`);
  - CryvexTech logo, renkler, simge.
- **Dağıtım:** `http://<robot>:8090/cryvex.apk`. Telefondaki eski sürüm başka bilgisayarda imzalandığı için **bir kerelik kaldır → kur** gerekiyor.
- **Kod:** `uygulama/cryvex_control/`.

### 6. Robot arkadaki insanı ezdi → lidar güvenlik kapısı (Nav2 dışı sürüş)
- **Sebep:** Uygulamanın devriyesi (`patrol.py`) masadan ayrılırken 1 m dümdüz geri çekiliyor, kurtulma manevrasında da geri gidiyor. Bunlar `/cmd_vel`'e doğrudan yazıyor ve arkayı **ultrasonikle** kontrol ediyordu (sensör takılı değil → hep "boş").
- **Çözüm:** `cryvex_bringup/lidar_guvenlik.py` (aynı kurallar: 35 cm dur, 70 cm yavaş, 40 cm dönüş, lidar yoksa hareket yok).
  - `patrol.drive_raw` ve `cafe_ui_server.publish_cmd` (telefondan elle sürüş) buradan geçiyor.
  - Geri çekilme ve kurtulma yönü lidara bakıyor.
- Arkadaşın robottaki değişiklikleri korundu. Servis yeniden başlatıldı. Kapı arkayı görüyor (hareketsiz test). ✔

### Yapılacaklar (sıradaki)
1. **Arka güvenlik testi (hareketli):** Arkaya kutu ya da sandalye koy. Uygulamanın devriyesinde masadan ayrılırken 35 cm'de durmalı.
2. **Telefona 1.2.0 kurulumu:** Eski uygulamayı kaldır → `:8090/cryvex.apk`. Sırasıyla dene:
   - yön tuşlarını (Yavaş modda),
   - şekil aracını (Haritaya Uygula'dan sonra haritayı kontrol et),
   - Görev Paneli'ni,
   - Güncelle kartını.
3. **Temiz tur:** Robot elle HOME'a → `masa_testi.py --dene` → `masa_testi.py 120` (yeni güvenlik kapısıyla tam görev).
4. **Ara noktalı HOME dönüşü:** Uzaktan dönüşü (`home_a_nav2`) robotta dene.
5. **Nav2 kendi sürüşü:** Hâlâ bizim kapıdan geçmiyor, sadece collision_monitor var. Plan: Nav2 sadece planlasın, yolu bizim kapılı sürücümüz izlesin.
6. **Birden fazla masa:** Düz çizgide olmayan masalar, masa seçme ekranı, masa sırası (mimari aşama 9–12).
7. **Sensörler gelince:**
   - AS5600 + IMU (EKF, takılma ve harita kayması);
   - VL53L1X (lidar altı: kablo, ayak ucu, yerdeki çerçeve);
   - tampon switch + e-stop (NC).
8. `save_map` (pgm) hatası.
9. imre-robotics deposuna değişiklikleri vermek isteniyorsa yazma izni ya da pull request.
10. **İmza anahtarı:** Kalıcı bir anahtar oluşturup sakla (`key.properties`). Uygulamanın her bilgisayardan aynı imzayla güncellenebilmesi için gerekli.
