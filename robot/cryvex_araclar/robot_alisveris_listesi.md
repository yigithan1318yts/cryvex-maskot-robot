# Cryvex Kafe Robotu - Sensör Alışveriş Listesi (düzeltilmiş, 7 Ekim 2026)

Bütçe: en fazla **3.500 TL**. Fiyatlar tahminidir; sipariş öncesi kontrol edin.

Amaç: 7 Ekim testlerinde görülen üç sorunu çözmek:
1. **Adım kaçırma / teker kayması:** Step motor tümsekte senkron kaybediyor. Teker odometrisi gerçek dönüşü değil, *gönderilen adımı* sayıyor, bu yüzden yazılım bunu göremiyor.
2. **Yön kayması:** IMU yok.
3. **Lidarın altındaki alçak engeller:** Vantilatör tabanı, masa/sandalye/tahta ayağı, kablo.

Bütün sensörler **Raspberry Pi'nin I2C hattına** bağlanacak (STM32 firmware kaynağı elimizde yok).

---

## 1. Sensörler (önem sırasıyla)

| # | Parça | Adet | Yaklaşık fiyat | Not |
|---|---|---|---|---|
| 1 | **AS5600 manyetik enkoder modülü** | 2 | ~300 TL | Tekerin gerçekten dönüp dönmediğini ölçer → adım kaçırma ve göbek kayması anında görülür. |
| 1a | **ÇAPRAZ (diametrik) mıknatıslı neodimyum mıknatıs, 6×2,5 mm** | 4 (2 yedek) | ~80 TL | **ŞART.** Normal (eksenel) mıknatısla AS5600 ÇALIŞMAZ. Ürün açıklamasında "diametrically magnetized" yazmalı. |
| 2 | **IMU: ICM-20948 (tercihen tek başına kart) ya da BNO085** | 1 | ~450–750 TL | Gerçek dönüş açısı. Pico'ya özel modül de olur ama montajı zor. **MPU9250 almayın** (üretimi bitti). |
| 3 | **VL53L1X ToF mesafe sensörü (TOF400C)** | 3 | ~1.440 TL | Sol ön / orta / sağ ön, yerden **5–10 cm** yükseğe. Lidarın göremediği alçak engeller. |
| 4 | **TCA9548A I2C 8 kanallı çoklayıcı** | 1 | ~60 TL | Aynı adresli sensörler (2 AS5600 + 3 VL53L1X) için. Robotun ortasına. |

## 2. Güvenlik

| # | Parça | Adet | Yaklaşık fiyat | Not |
|---|---|---|---|---|
| 5 | Mikro sınır anahtarı (tampon için) | 4 | ~250 TL (5 ve 6 birlikte) | STM32'nin mevcut tampon girişine **NC (normalde kapalı)** seri bağlanacak (şu an GND'ye köprülü). |
| 6 | Mantar acil stop butonu | 1 | | STM32 e-stop girişine NC. Motor elektriğini doğrudan kesmesi istenirse, sürücü güç hattına uygun akımlı buton gerekir. |

## 3. Bağlantı ve montaj

| # | Parça | Fiyat | Not |
|---|---|---|---|
| 7 | 4 damarlı ince kablo (tercihen burgulu/blendajlı), 2–3 m | ~80 TL | I2C uzun düz kabloda hata yapar. Tekerlere ve ön sensörlere **40–60 cm**. 20 cm jumper YETMEZ. |
| 8 | JST-XH konnektör seti ya da lehimli bağlantı | ~80 TL | Titreşimde jumper uçları gevşer. |
| 9 | Dişi-dişi + dişi-erkek jumper (40'lık) | ~100 TL | Sadece masa üstü deneme için. |
| 10 | Enkoder tutucu (3D baskı ya da küçük alüminyum L parça) | ~50 TL | AS5600 mıknatısın **1–3 mm** karşısında ve tam merkezinde durmalı. |
| 11 | Hızlı yapıştırıcı (mıknatıs için) | ~60 TL | |
| 12 | Köpük bant, cırt kelepçe, vida | ~150 TL | |
| 13 | Havya + lehim (yoksa) | ~200 TL | Sensör pinlerini lehimlemek için. |

**Toplam:** yaklaşık **3.200–3.500 TL**.

---

## Montaj öncesi kontrol (önemli)

- **Mıknatıs nereye?** NEMA34 (86HS156) motorların **arkasında mil çıkışı var mı** bakın.
  - Yoksa mıknatıs **teker aksının dış ucuna** yapıştırılır. Bu daha iyidir: hem adım kaçırmayı hem göbekte kaymayı görür.
- **ToF yüksekliği:** Yerden 5–10 cm, hafif öne eğik. Zemine değil, alçak engel yüksekliğine bakmalı.
- **IMU:** Robotun merkezine yakın, titreşimden uzak, düz monte edilmeli.

## Sırada (parçalar gelince)

1. Bağlantı krokisi: Pi I2C (pin 3/5), TCA9548A kanalları (0-1 enkoder, 2-4 ToF), IMU.
2. Çoklayıcılı okuma kodu: thread-safe, bloklamayan, 20-50 Hz.
3. `devriye.py` entegrasyonu:
   - **Takılma tespiti:** Gerçek teker dönüşü (AS5600) ile lidar odometrisini karşılaştıracak.
   - **Yön:** IMU ile.
   - **Alçak engel:** ToF ile; gidiş yönünde 15 cm'den yakın alçak engelde dur.
