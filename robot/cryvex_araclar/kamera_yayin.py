"""Canli kamera ekrani: Logitech kameranin goruntusu + ustunde lidar noktalari.
Tarayicida:  http://<robot>:8081/        (canli, ~5 kare/sn)
Tek kare:    http://<robot>:8081/kare.jpg
Kullanim: python3 kamera_yayin.py [--yuk 0.32] [--egim 0] [--fov 70] [--port 8081]"""
import sys, glob, math, time, threading, numpy as np, cv2
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

def arg(ad, v):
    return type(v)(sys.argv[sys.argv.index(ad) + 1]) if ad in sys.argv else v
YUK, EGIM, FOV, PORT = arg('--yuk', 0.32), math.radians(arg('--egim', 0.0)), math.radians(arg('--fov', 70.0)), arg('--port', 8081)
LIDAR_Z, LX, LYY, LY = 0.36, 0.125, -0.045, math.radians(-90)
SON = {'jpg': None, 'P': np.zeros((0, 2)), 'n': 0}

def lidar_dinle():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init(); n = rclpy.create_node('kamera_yayin')
    def cb(m):
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.15) & (r < 8)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
        SON['P'] = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)
    n.create_subscription(LaserScan, 'scan', cb, qos_profile_sensor_data)
    rclpy.spin(n)

def izdusum(X, Y, Z, w, h):
    f = (w / 2) / math.tan(FOV / 2)
    dx, dy, dz = X - LX, Y - LYY, Z - YUK
    ileri = dx * math.cos(EGIM) + dz * math.sin(EGIM); yukari = -dx * math.sin(EGIM) + dz * math.cos(EGIM)
    on = ileri > 0.05; b = np.where(on, ileri, 1)
    return w / 2 - f * dy / b, h / 2 - f * yukari / b, on

def kamera_dongu():
    d = (sorted(glob.glob('/dev/v4l/by-id/*video-index0')) or ['/dev/video0'])[0]
    cap = cv2.VideoCapture(d, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    t_son = 0.0
    while True:
        ok, img = cap.read()
        if not ok:
            time.sleep(0.5); cap.release(); cap = cv2.VideoCapture(d, cv2.CAP_V4L2); continue
        if time.time() - t_son < 0.2: continue                # ~5 kare/sn islenir (Pi'yi yormasin)
        t_son = time.time(); h, w = img.shape[:2]; P = SON['P']
        if len(P):
            u, v, on = izdusum(P[:, 0], P[:, 1], np.full(len(P), LIDAR_Z), w, h); dist = np.hypot(P[:, 0], P[:, 1])
            for ui, vi, oi, di in zip(u, v, on, dist):
                if oi and 0 <= ui < w and 0 <= vi < h:
                    cv2.circle(img, (int(ui), int(vi)), 3, (0, 0, 255) if di < 1 else (0, 165, 255) if di < 2 else (0, 255, 0), -1)
        u, v, on = izdusum(np.array([1.0, 1.0]), np.array([0.6, -0.6]), np.array([YUK, YUK]), w, h)
        cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (255, 255, 0), 1)
        cv2.putText(img, time.strftime('%H:%M:%S') + '  lidar: kirmizi<1m turuncu<2m yesil', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        SON['jpg'] = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes(); SON['n'] += 1

SAYFA = b'''<!DOCTYPE html><html lang="tr"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cryvex Kamera</title><style>body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif;padding:12px}
img{width:100%;max-width:960px;display:block;margin:0 auto;border-radius:8px}h1{font-size:1.1rem;text-align:center}</style></head>
<body><h1>Cryvex canli kamera + lidar</h1><img src="/akis" alt="canli goruntu"></body></html>'''

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith('/akis'):
            self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=kare'); self.end_headers()
            n0 = -1
            try:
                while True:
                    if SON['jpg'] is not None and SON['n'] != n0:
                        n0 = SON['n']; j = SON['jpg']
                        self.wfile.write(b'--kare\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                    time.sleep(0.05)
            except Exception:
                return
        elif self.path.startswith('/kare.jpg'):
            j = SON['jpg'] or b''
            self.send_response(200 if j else 503); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(j))); self.end_headers(); self.wfile.write(j)
        else:
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(SAYFA))); self.end_headers(); self.wfile.write(SAYFA)

threading.Thread(target=lidar_dinle, daemon=True).start()
threading.Thread(target=kamera_dongu, daemon=True).start()
print(f'canli kamera: http://0.0.0.0:{PORT}/', flush=True)
ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
