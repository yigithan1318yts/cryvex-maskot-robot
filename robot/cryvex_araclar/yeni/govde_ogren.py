"""GOVDE KALIBRASYONU: kamerayi -180..+180 arasi her 15 dereceye gonderir (calisan algilama uzerinden, /tarete/istek),
her acida cizimsiz bir kare (/ham.jpg) kaydeder ve bir pano yapar. Kablo kurali: hep -180..180 icinde, sira
0 -> +180 -> 0 -> -180 -> 0 (arkadan gecis yok). Sonunda kamera serbest birakilir (normal bakisa doner)."""
import json, time, urllib.request, os, numpy as np, cv2, rclpy
from rclpy.node import Node
from std_msgs.msg import String
cikti = os.path.expanduser('~/cryvex_araclar/govde'); os.makedirs(cikti, exist_ok=True)
rclpy.init(); n = Node('govde_ogren'); durum = {}
n.create_subscription(String, 'tarete/durum', lambda m: durum.update(json.loads(m.data)), 10)
pub = n.create_publisher(String, 'tarete/istek', 10)
def bekle(s):
    t = time.time()
    while time.time() - t < s: rclpy.spin_once(n, timeout_sec=0.05)
bekle(1.0)
sira = list(range(0, 181, 15)) + list(range(165, -1, -15))[1:] + list(range(-15, -181, -15)) + list(range(-165, 1, 15))
kareler = {}
for i, a in enumerate(sira):
    kid = f'kal-{i}'; t = time.time(); ok = False
    while time.time() - t < 8:
        pub.publish(String(data=f'{kid};{a};10')); bekle(0.2)
        if durum.get('istek_id') == kid and not durum.get('hareket') and abs(durum.get('aci', 999) - a) < 0.6: ok = True; break
    if not ok: print('aci', a, 'ulasilamadi', durum, flush=True); continue
    if a in kareler: continue
    bekle(1.0)
    r = urllib.request.urlopen('http://127.0.0.1:8081/ham.jpg', timeout=3)
    if abs(float(r.headers.get('X-Kamera-Aci', 999)) - a) > 0.6: bekle(0.8); r = urllib.request.urlopen('http://127.0.0.1:8081/ham.jpg', timeout=3)
    img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
    cv2.imwrite(f'{cikti}/ham_{a:+04d}.jpg', img); kareler[a] = img; print('aci', a, flush=True)
pub.publish(String(data='serbest')); bekle(0.5)
anahtarlar = sorted(kareler)
kucuk = []
for k in anahtarlar:
    im = cv2.resize(kareler[k], (320, 240))
    for x in range(0, 320, 32): cv2.line(im, (x, 0), (x, 239), (0, 255, 0), 1)       # 0.1'lik izgara (maske icin)
    for y in range(0, 240, 24): cv2.line(im, (0, y), (319, y), (0, 255, 0), 1)
    cv2.putText(im, f'{k:+d}', (5, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2); kucuk.append(im)
while len(kucuk) % 5: kucuk.append(np.zeros((240, 320, 3), np.uint8))
pano = np.vstack([np.hstack(kucuk[i:i + 5]) for i in range(0, len(kucuk), 5)])
cv2.imwrite(f'{cikti}/ham_pano.jpg', pano, [cv2.IMWRITE_JPEG_QUALITY, 70]); print('bitti', anahtarlar)
