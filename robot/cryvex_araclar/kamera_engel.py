"""KAMERA + LIDAR birlesimi (Cryvex, 2026-10-05).
Kamera (Logitech Brio 100, lidarin hemen altinda, one bakiyor) nesneleri TANIR (MobileNet-SSD:
masa, sandalye, insan, koltuk, saksi, kopek, kedi), lidar o yondeki UZAKLIGI olcer. Lidar
goremiyorsa (ornegin sadece masa tablasi gorunuyor) uzaklik nesnenin goruntude yere bastigi yerden
hesaplanir. Taninan nesne GERCEK BOYUTUYLA (masa: tablasiyla ~90 cm) Nav2 engel haritasina yazilir:
  /kamera_engeller  (PointCloud2, base_footprint) -> costmap 'kamera_layer' isaretler
  /kamera_bos       (PointCloud2)                 -> kameranin bos gordugu yonleri temizler
  /kamera_tespit    (String, JSON)                -> kayit / diger programlar icin
Canli ekran (kutular + uzakliklar + lidar noktalari): http://<robot>:8081/   tek kare: /kare.jpg
Kullanim: python3 -u kamera_engel.py [--fov 48] [--yuk 0.32] [--egim 0]"""
import sys, os, glob, json, math, time, threading, numpy as np, cv2, rclpy
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Header

def arg(ad, v): return type(v)(sys.argv[sys.argv.index(ad) + 1]) if ad in sys.argv else v
FOV, YUK, EGIM, PORT = math.radians(arg('--fov', 48.0)), arg('--yuk', 0.32), math.radians(arg('--egim', 0.0)), arg('--port', 8081)
LIDAR_Z, LX, LYY, LY = 0.36, 0.125, -0.045, math.radians(-90)
CAM_X, CAM_Y = LX, LYY                     # kamera lidarin hemen altinda
MODEL = os.path.expanduser('~/cryvex_araclar/modeller/MobileNetSSD_deploy')
SINIF = ['arka', 'ucak', 'bisiklet', 'kus', 'tekne', 'sise', 'otobus', 'araba', 'kedi', 'sandalye', 'inek',
         'masa', 'kopek', 'at', 'motor', 'insan', 'saksi', 'koyun', 'koltuk', 'tren', 'tv']
YARICAP = {'masa': 0.45, 'sandalye': 0.30, 'insan': 0.50, 'koltuk': 0.60, 'saksi': 0.25, 'kopek': 0.35, 'kedi': 0.30}
RENK = {'masa': (0, 200, 255), 'sandalye': (255, 200, 0), 'insan': (0, 0, 255)}
ORTAK = {'jpg': None, 'n': 0, 'P': np.zeros((0, 2))}

class KameraEngel(Node):
    def __init__(self):
        super().__init__('kamera_engel')
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.pub_isaret = self.create_publisher(PointCloud2, 'kamera_engeller', 5)
        self.pub_bos = self.create_publisher(PointCloud2, 'kamera_bos', 5)
        self.pub_tespit = self.create_publisher(String, 'kamera_tespit', 5)

    def _scan(self, m):
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.12) & (r < 8)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
        ORTAK['P'] = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)

    def yayinla(self, isaret, bos, tespit):
        h = Header(); h.frame_id = 'base_footprint'; h.stamp = self.get_clock().now().to_msg()
        self.pub_isaret.publish(point_cloud2.create_cloud_xyz32(h, isaret or [(0.0, 0.0, -5.0)]))   # -5: yuksekliktan elenir
        self.pub_bos.publish(point_cloud2.create_cloud_xyz32(h, bos))
        self.pub_tespit.publish(String(data=json.dumps(tespit, ensure_ascii=False)))

def kamera_ac():
    d = (sorted(glob.glob('/dev/v4l/by-id/*video-index0')) or ['/dev/video0'])[0]
    cap = cv2.VideoCapture(d, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return d, cap

def ana_dongu(node):
    net = cv2.dnn.readNetFromCaffe(MODEL + '.prototxt', MODEL + '.caffemodel')
    d, cap = kamera_ac(); t_son = 0.0
    while rclpy.ok():
        ok, img = cap.read()
        if not ok:
            time.sleep(0.5); cap.release(); d, cap = kamera_ac(); continue
        if time.time() - t_son < 0.33: continue                       # ~3 kare/sn islenir
        t_son = time.time(); h, w = img.shape[:2]; f = (w / 2) / math.tan(FOV / 2)
        net.setInput(cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)), 0.007843, (300, 300), 127.5))
        out = net.forward()[0, 0]
        P = ORTAK['P']; Pc = P - [CAM_X, CAM_Y]                        # kameradan bakis
        yon = np.arctan2(Pc[:, 1], Pc[:, 0]); uz = np.hypot(Pc[:, 0], Pc[:, 1])
        isaret, tespit, dolu = [], [], []
        for det in out:
            guven, sinif = float(det[2]), SINIF[int(det[1])]
            if guven < 0.5 or sinif not in YARICAP: continue
            x1, y1, x2, y2 = (det[3:7] * [w, h, w, h]).astype(int)
            x1, x2 = max(0, x1), min(w - 1, x2); y1, y2 = max(0, y1), min(h - 1, y2)
            b_sol, b_sag = math.atan((w / 2 - x1) / f), math.atan((w / 2 - x2) / f)   # sol + / sag -
            sec = (yon <= b_sol) & (yon >= b_sag) & (uz < 6.0)
            mesafe, kaynak = None, ''
            if sec.sum() >= 3:
                mesafe, kaynak = float(np.percentile(uz[sec], 20)), 'lidar'
            elif y2 < h - 5:                                            # yere bastigi yer gorunuyor
                alfa = math.atan((y2 - h / 2) / f) - EGIM
                if alfa > 0.03: mesafe, kaynak = YUK / math.tan(alfa), 'zemin'
            if mesafe is None or mesafe > 4.5: continue
            R = YARICAP[sinif]; b_orta = (b_sol + b_sag) / 2
            merkez = mesafe + (R * 0.6 if sinif == 'masa' else R * 0.3)   # masa: tabla merkezi biraz geride
            cx, cy = CAM_X + merkez * math.cos(b_orta), CAM_Y + merkez * math.sin(b_orta)
            for gx in np.arange(-R, R + 0.01, 0.05):
                for gy in np.arange(-R, R + 0.01, 0.05):
                    if gx * gx + gy * gy <= R * R: isaret.append((float(cx + gx), float(cy + gy), 0.5))
            dolu.append((b_sag - 0.05, b_sol + 0.05))
            tespit.append({'sinif': sinif, 'guven': round(guven, 2), 'mesafe': round(mesafe, 2), 'kaynak': kaynak,
                           'yon_deg': round(math.degrees(b_orta)), 'x': round(cx, 2), 'y': round(cy, 2)})
            renk = RENK.get(sinif, (0, 255, 0))
            cv2.rectangle(img, (x1, y1), (x2, y2), renk, 2)
            cv2.putText(img, f'{sinif} {mesafe:.1f}m ({kaynak})', (x1 + 3, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2)
        bos = []                                                         # kameranin bos gordugu yonler: temizle
        for b in np.linspace(-FOV / 2, FOV / 2, 25):
            if any(lo <= b <= hi for lo, hi in dolu): continue
            bos.append((float(CAM_X + 3.0 * math.cos(b)), float(CAM_Y + 3.0 * math.sin(b)), 0.5))
        node.yayinla(isaret, bos, tespit)
        if len(P):                                                       # lidar noktalari (ekran icin)
            ileri = Pc[:, 0]; on = ileri > 0.05
            u = w / 2 - f * Pc[on, 1] / ileri[on]; v = h / 2 - f * (LIDAR_Z - YUK) / ileri[on]
            for ui, vi, di in zip(u, v, uz[on]):
                if 0 <= ui < w and 0 <= vi < h:
                    cv2.circle(img, (int(ui), int(vi)), 2, (0, 0, 255) if di < 1 else (0, 165, 255) if di < 2 else (0, 255, 0), -1)
        cv2.putText(img, time.strftime('%H:%M:%S') + f'  taninan: {len(tespit)}', (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ORTAK['jpg'] = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes(); ORTAK['n'] += 1

SAYFA = b'''<!DOCTYPE html><html lang="tr"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cryvex Kamera</title><style>body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif;padding:12px}
img{width:100%;max-width:960px;display:block;margin:0 auto;border-radius:8px}h1{font-size:1.1rem;text-align:center}</style></head>
<body><h1>Cryvex kamera + lidar (taninan nesneler)</h1><img src="/akis" alt="canli goruntu"></body></html>'''

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith('/akis'):
            self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=kare'); self.end_headers()
            n0 = -1
            try:
                while True:
                    if ORTAK['jpg'] is not None and ORTAK['n'] != n0:
                        n0 = ORTAK['n']; j = ORTAK['jpg']
                        self.wfile.write(b'--kare\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                    time.sleep(0.05)
            except Exception:
                return
        elif self.path.startswith('/kare.jpg'):
            j = ORTAK['jpg'] or b''
            self.send_response(200 if j else 503); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(j))); self.end_headers(); self.wfile.write(j)
        else:
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(SAYFA))); self.end_headers(); self.wfile.write(SAYFA)

def main():
    rclpy.init(); node = KameraEngel()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=lambda: ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever(), daemon=True).start()
    print(f'kamera+lidar calisiyor - ekran http://0.0.0.0:{PORT}/', flush=True)
    try:
        ana_dongu(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
