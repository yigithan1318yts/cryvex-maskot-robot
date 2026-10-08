"""HOME BUL (Cryvex gorev sistemi - 2026-10-08)

Harita bozuldugunda (SLAM yerini kaybetti) robot HOME'a gore NEREDE? Haritayi KULLANMAZ: su anki lidar taramasini,
HOME'da kaydedilen taramaya (~/cryvex_veri/pozlar/kullanici_koydu.npy, robot HOME'da elle konmusken) her yon ve konumda
dener (mesafe alani), en iyi oturani secer, sonra ince ayar (ICP). Oda dikdortgen oldugu icin 180 derece TERS cozum da
iyi oturabilir -> en iyi iki FARKLI cozumun puani yazilir; fark azsa 'belirsiz' der, HAREKET ETMEZ.

  --dene : hareketsiz; konumu yazar, ust uste bindirme resmi /tmp/home_bul.png
  (git)  : SLAM'i sifirlar (temiz harita), HOME'u yeni haritaya yerlestirir, Nav2 (ara noktalarla) HOME'un 0.8 m onune,
           harita yonuyle don + lidarla duvara hizala + dumduz geri yerles (masa_testi.home_yerles)
Kullanim: python3 -u home_bul.py --dene   |   python3 -u home_bul.py"""
import os, sys, time, math
sys.path.insert(0, os.path.expanduser('~/cryvex_araclar')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, cv2, rclpy, tf2_ros
import devriye as D
from basit_surus import log
from haritala import servis, slam_acik, harita_pozu

REF = os.path.expanduser('~/cryvex_veri/pozlar/kullanici_koydu.npy')
RES = 0.05

def alan(P, pay=1.0):
    """referans noktalarindan mesafe alani (m): (dist, x0, y0)"""
    x0, y0 = P.min(0) - pay; x1, y1 = P.max(0) + pay
    w, h = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
    img = np.full((h, w), 255, np.uint8)
    ij = ((P - [x0, y0]) / RES).astype(int); img[ij[:, 1], ij[:, 0]] = 0
    return cv2.distanceTransform(img, cv2.DIST_L2, 5) * RES, x0, y0

def puanla(dist, x0, y0, Q, kes=0.30):
    """Q: (k, n, 2) aday noktalar -> (k,) ortalama kirpilmis mesafe (kucuk = iyi)"""
    h, w = dist.shape; ij = np.floor((Q - [x0, y0]) / RES).astype(int)
    ic = (ij[..., 0] >= 0) & (ij[..., 0] < w) & (ij[..., 1] >= 0) & (ij[..., 1] < h)
    d = np.full(ij.shape[:2], kes); d[ic] = np.minimum(dist[ij[..., 1][ic], ij[..., 0][ic]], kes)
    return d.mean(1)

def bul(H, C):
    """C (robot cercevesi) -> HOME cercevesindeki robot pozu (x, y, yaw), puan; ve en iyi FARKLI ikinci cozum"""
    dist, x0, y0 = alan(H)
    C = C[np.random.default_rng(0).choice(len(C), min(len(C), 200), replace=False)]
    xs = np.arange(H[:, 0].min(), H[:, 0].max(), 0.25); ys = np.arange(H[:, 1].min(), H[:, 1].max(), 0.25)
    TX, TY = np.meshgrid(xs, ys); T = np.stack([TX.ravel(), TY.ravel()], 1)
    adaylar = []
    for yaw in np.radians(np.arange(0, 360, 4)):
        c, s = math.cos(yaw), math.sin(yaw); Cr = C @ np.array([[c, s], [-s, c]])
        p = puanla(dist, x0, y0, Cr[None] + T[:, None]); i = np.argsort(p)[:3]
        adaylar += [(float(p[j]), float(T[j, 0]), float(T[j, 1]), float(yaw)) for j in i]
    adaylar.sort()
    ince = []
    for p0, tx, ty, yaw in adaylar[:12]:                                    # ince arama (0.05 m, 0.5 derece)
        for dyaw in np.radians(np.arange(-4, 4.01, 0.5)):
            c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw); Cr = C @ np.array([[c, s], [-s, c]])
            g = np.arange(-0.25, 0.2501, 0.05); GX, GY = np.meshgrid(tx + g, ty + g); T2 = np.stack([GX.ravel(), GY.ravel()], 1)
            p = puanla(dist, x0, y0, Cr[None] + T2[:, None], kes=0.15); j = int(np.argmin(p))
            ince.append((float(p[j]), float(T2[j, 0]), float(T2[j, 1]), float((yaw + dyaw) % (2 * math.pi))))
    ince.sort(); en = ince[0]
    ikinci = next((a for a in ince if math.hypot(a[1] - en[1], a[2] - en[2]) > 1.0
                   or abs(math.atan2(math.sin(a[3] - en[3]), math.cos(a[3] - en[3]))) > math.radians(25)), None)
    return en, ikinci

def resim(H, C, poz, yol='/tmp/home_bul.png'):
    x, y, yaw = poz; c, s = math.cos(yaw), math.sin(yaw); W = C @ np.array([[c, s], [-s, c]]) + [x, y]
    A = np.vstack([H, W]); x0, y0 = A.min(0) - 0.5; k = 60
    w, h = int((A[:, 0].max() - x0 + 0.5) * k), int((A[:, 1].max() - y0 + 0.5) * k)
    img = np.full((h, w, 3), 255, np.uint8)
    def px(p): return int((p[0] - x0) * k), h - int((p[1] - y0) * k)
    for p in H: cv2.circle(img, px(p), 2, (0, 0, 0), -1)                 # siyah: HOME taramasi
    for p in W: cv2.circle(img, px(p), 2, (0, 0, 255), -1)               # kirmizi: su anki tarama
    cv2.circle(img, px((0, 0)), 8, (0, 160, 0), -1); cv2.line(img, px((0, 0)), px((0.6, 0)), (0, 160, 0), 3)   # HOME
    cv2.circle(img, px((x, y)), 8, (255, 0, 0), -1); cv2.line(img, px((x, y)), px((x + 0.6 * c, y + 0.6 * s)), (255, 0, 0), 3)   # robot
    cv2.imwrite(yol, img)

def main():
    dene = '--dene' in sys.argv
    rclpy.init(); n = D.Devriye(); tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n); n._tfb = tfb
    try:
        t = time.time()
        while time.time() - t < 3 or n.scan is None: rclpy.spin_once(n, timeout_sec=0.1)
        H = np.load(REF); C = n.lidar(ham=True)
        t0 = time.time(); (p, x, y, yaw), ik = bul(H, C)
        log(f'HOME BUL ({time.time() - t0:.1f} sn): robot HOME a gore x {x:+.2f} m (HOME un onu +), y {y:+.2f} m (solu +), '
            f'yon {math.degrees(yaw):+.0f} derece | uyum {p * 100:.1f} cm' + (f' | 2. cozum uyum {ik[0] * 100:.1f} cm '
            f'(x {ik[1]:+.2f}, y {ik[2]:+.2f}, {math.degrees(ik[3]):+.0f} derece)' if ik else ''))
        resim(H, C, (x, y, yaw))
        net = p < 0.10 and (ik is None or ik[0] - p > 0.015)                # (ortalama, HOME taramasinda olmayan yeni alanlari da icerir)
        log(f'  sonuc: {"NET" if net else "BELIRSIZ - hareket etmiyorum"} (resim: /tmp/home_bul.png)')
        if dene or not net: return
        import masa_testi as M
        if slam_acik():
            log('  SLAM sifirlaniyor (bozuk harita yerine temiz)'); servis('/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'); time.sleep(5)
        P = harita_pozu(n, tfb, 30.0)
        if P is None: log('  HATA: harita konumu yok'); return
        # HOME'un robot cercevesindeki yeri = ters donusum; sonra haritaya
        c, s = math.cos(yaw), math.sin(yaw); hx, hy = -(c * x + s * y), -(-s * x + c * y); hyaw = -yaw
        cp, sp = math.cos(P[2]), math.sin(P[2])
        home = (P[0] + cp * hx - sp * hy, P[1] + sp * hx + cp * hy, math.atan2(math.sin(P[2] + hyaw), math.cos(P[2] + hyaw)))
        log(f'  HOME yeni haritada: x {home[0]:+.2f}, y {home[1]:+.2f}, yon {math.degrees(home[2]):+.0f} derece')
        if 0.4 < x < 3.0 and abs(y) < 0.30:                                  # zaten HOME ekseninin uzerindeyim: Nav2 gereksiz
            log(f'  HOME ekseni uzerindeyim ({x:.2f} m onunde) - Nav2 yok: don, hizalan, dumduz geri')
        elif not M.yakinda(n, home, 0.8) and not M.home_a_nav2(n, home): log('  HATA: HOME onune gidemedim - durdum'); return
        log('  10 sn sonra basliyorum (kablo)'); time.sleep(10)
        if not M.home_yerles(n, home): log('  HATA: HOME a yerlesemedim - durdum'); return
        log('  HOME DAYIM (hizali).')
    except KeyboardInterrupt:
        pass
    finally:
        n.dur(yumusak=False); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
