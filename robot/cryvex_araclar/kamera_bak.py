"""Kamera ayari icin anlik kare: Logitech kameradan 1 kare alir, ustune lidar noktalarini cizer.
Kullanim: python3 kamera_bak.py [--yuk 0.45] [--egim 0] [--fov 70] [--cikti /tmp/kamera.jpg]
  --yuk : kameranin yerden yuksekligi (m)   --egim: yukari + / asagi - (derece)
  --fov : yatay gorus acisi (derece)        kamera lidarin ustunde, ayni dikeyde, ONE bakiyor varsayilir"""
import sys, glob, math, time, numpy as np, cv2

def arg(ad, vars):
    return type(vars)(sys.argv[sys.argv.index(ad) + 1]) if ad in sys.argv else vars
YUK, EGIM, FOV = arg('--yuk', 0.45), math.radians(arg('--egim', 0.0)), math.radians(arg('--fov', 70.0))
CIKTI = arg('--cikti', '/tmp/kamera.jpg')
LIDAR_Z, LX, LYY, LY = 0.36, 0.125, -0.045, math.radians(-90)

def kamera_ac():
    adaylar = sorted(glob.glob('/dev/v4l/by-id/*video-index0')) + [f'/dev/video{i}' for i in range(0, 10)]
    for d in adaylar:
        cap = cv2.VideoCapture(d, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            for _ in range(40): ok, img = cap.read()     # pozlama otursun (~1.5 sn)
            cap.release()
            if ok and img is not None: return d, img
    return None, None

def lidar_noktalari():
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except Exception:
        return np.zeros((0, 2))
    rclpy.init(); n = rclpy.create_node('kamera_bak'); M = []
    n.create_subscription(LaserScan, 'scan', lambda m: M.append(m), qos_profile_sensor_data)
    t = time.time()
    while len(M) < 2 and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()
    if not M: return np.zeros((0, 2))
    m = M[-1]; a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
    ok = np.isfinite(r) & (r > 0.15) & (r < 8)
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
    return np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)   # robot cercevesi

def izdusum(X, Y, Z, w, h):
    """robot cercevesindeki (X ileri, Y sol, Z yukari) noktayi goruntuye; kamera lidarin ustunde"""
    f = (w / 2) / math.tan(FOV / 2)
    dx, dy, dz = X - LX, Y - LYY, Z - YUK
    ileri = dx * math.cos(EGIM) + dz * math.sin(EGIM); yukari = -dx * math.sin(EGIM) + dz * math.cos(EGIM)
    on = ileri > 0.05
    u = w / 2 - f * dy / np.where(on, ileri, 1); v = h / 2 - f * yukari / np.where(on, ileri, 1)
    return u, v, on

d, img = kamera_ac()
if img is None:
    print('KAMERA BULUNAMADI'); sys.exit(1)
h, w = img.shape[:2]; P = lidar_noktalari()
if len(P):
    u, v, on = izdusum(P[:, 0], P[:, 1], np.full(len(P), LIDAR_Z), w, h)
    dist = np.hypot(P[:, 0], P[:, 1])
    for ui, vi, oi, di in zip(u, v, on, dist):
        if oi and 0 <= ui < w and 0 <= vi < h:
            renk = (0, 0, 255) if di < 1.0 else (0, 165, 255) if di < 2.0 else (0, 255, 0)
            cv2.circle(img, (int(ui), int(vi)), 3, renk, -1)
# ufuk (kamera yuksekligi) ve 1 m ilerideki 75 cm masa tablasi
for z, renk, yazi in ((YUK, (255, 255, 0), 'ufuk'), (0.75, (255, 0, 255), '1 m: 75 cm masa'), (0.0, (200, 200, 200), '1 m: zemin')):
    xs = np.array([1.0, 1.0]); ys = np.array([0.6, -0.6]); u, v, on = izdusum(xs, ys, np.array([z, z]), w, h)
    if on.all() and -h < v[0] < 2 * h:
        cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), renk, 2)
        cv2.putText(img, yazi, (int(u[1]) + 4, int(v[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 1)
cv2.putText(img, f'{d}  yuk {YUK:.2f} m  egim {math.degrees(EGIM):+.0f}  lidar: kirmizi<1m turuncu<2m yesil', (6, 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
cv2.imwrite(CIKTI, img); print('KAYDEDILDI', CIKTI, d, f'{w}x{h}', 'lidar nokta', len(P))
