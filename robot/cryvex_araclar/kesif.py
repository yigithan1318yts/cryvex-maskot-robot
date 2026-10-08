"""Kabloya uygun otomatik kesif + haritalama v2 (robot supurge mantigi).
- engeller: anlik lidar + SLAM haritasi (gorulen engelleri ezberler) + takildigi yerler (sanal engel)
- donmeden once govde etrafinda bosluk arar, donerken takilirsa (lidar donmedigini gosterir) durur, kurtulur
- arkadaki kabloyu tutan insan (|aci| > 135 deg) guvenlik kontrollerinde sayilmaz
- bitince haritayi kaydeder, Barmen/Masa 1 noktalarini yazar, baslangica (B) doner ve bekler
Kullanim: servisi B noktasinda yeniden baslat, sonra: python3 -u kesif.py [--sure 600] [--donme]"""
import sys, time, math, json, random, subprocess, urllib.request, numpy as np, rclpy
from concurrent.futures import ThreadPoolExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from ortak import Robot, icp

SURE = int(sys.argv[sys.argv.index('--sure') + 1]) if '--sure' in sys.argv else 600
GOVDE = 0.30                  # merkezden en dis nokta: govde 52 cm daire, tekerler 54 cm
KORIDOR = GOVDE + 0.08        # ileri giderken yanlarda bu kadar bos koridor
DUR_ON = 0.80                 # onde 0.80 m (govde onunden ~50 cm) engel -> dur, yon degistir
GENIS = GOVDE + 0.50          # sag/sol: govde kenarindan 50 cm bos koridor (dar yerde KORIDOR'a duser)
DON_MIN = GOVDE + 0.14        # donmeye baslamak icin her yanda (arka haric) bu kadar bosluk
DON_ACIL = GOVDE + 0.06       # donerken bundan yakina bir sey gelirse dur
ARKA = math.radians(160)      # bundan buyuk acilar = tam arka (takip eden insan) - sayilmaz
if '--yalniz' in sys.argv: ARKA = math.radians(181)   # kimse takip etmiyor: her yon kontrol edilir
KABLO_LIMIT = math.radians(100000)   # kullanici: kabloyu bosver
BEKLE_ACI = math.radians(100000)     # kullanici: donmeden once bekleme yok
HIZ, DONUS_HIZ = 0.15, 0.35

def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def api(path, body=None):
    data = json.dumps(dict(body or {}, password='1234')).encode()
    req = urllib.request.Request('http://127.0.0.1:8080' + path, data=data, headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=30).read())
def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)

rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
tfb = tf2_ros.Buffer(); tfl = tf2_ros.TransformListener(tfb, r, spin_thread=False)
DUR = {'v': False}
def on_cmd(m):
    if m.data.strip() == 'stop' or m.data.startswith('teleop') or m.data.startswith('rescue'): DUR['v'] = True
r.create_subscription(String, 'patrol_command', on_cmd, 10)

# --- haritadan ezber ---
HARITA = {'P': None}
def on_map(m):
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = g >= 65; iy, ix = np.nonzero(occ); res = m.info.resolution; o = m.info.origin.position
    gen = occ.copy()                        # eslesme kontrolu icin 2 hucre (10 cm) genisletilmis
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            gen |= np.roll(np.roll(occ, dy, 0), dx, 1)
    HARITA.update(P=np.stack([o.x + (ix + 0.5) * res, o.y + (iy + 0.5) * res], 1), gen=gen, ox=o.x, oy=o.y, res=res)
r.create_subscription(OccupancyGrid, 'map', on_map,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
HARITA['uyari'] = 0.0
def harita_pts(S):
    """haritadaki engeller (robot cercevesinde) - sadece canli lidar haritayla ortusuyorsa (konum dogruysa)"""
    P = HARITA['P']
    if P is None or not len(P) or not len(S): return np.zeros((0, 2))
    try:
        t = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except Exception:
        return np.zeros((0, 2))
    q = t.transform.rotation; yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z); c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = t.transform.translation.x, t.transform.translation.y
    Sy = S[np.hypot(S[:, 0], S[:, 1]) < 4.0][::3]
    if len(Sy) > 20:
        gen = HARITA['gen']; res = HARITA['res']
        ix = ((c * Sy[:, 0] - s * Sy[:, 1] + tx - HARITA['ox']) / res).astype(int)
        iy = ((s * Sy[:, 0] + c * Sy[:, 1] + ty - HARITA['oy']) / res).astype(int)
        ok = (ix >= 0) & (iy >= 0) & (ix < gen.shape[1]) & (iy < gen.shape[0])
        uyum = float(gen[iy[ok], ix[ok]].sum()) / len(Sy)
        if uyum < 0.5:
            if time.time() - HARITA['uyari'] > 10: log(f'  (harita ortusmuyor %{uyum*100:.0f} - simdilik sadece lidar)'); HARITA['uyari'] = time.time()
            return np.zeros((0, 2))
    d = P - [tx, ty]
    d = d[np.hypot(d[:, 0], d[:, 1]) < 2.5]
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)

# --- takildigi yerler (odom cercevesinde sanal engel) ---
ZIHIN = []
def zihin_ekle(pts_base):
    x, y, th = r.odo(); c, s = math.cos(th), math.sin(th)
    for px, py in pts_base: ZIHIN.append((x + c * px - s * py, y + s * px + c * py, time.time()))
def zihin_pts():
    ZIHIN[:] = [z for z in ZIHIN if time.time() - z[2] < 120]   # 2 dk sonra unut (sikismasin)
    if not ZIHIN: return np.zeros((0, 2))
    x, y, th = r.odo(); d = np.array(ZIHIN)[:, :2] - [x, y]; c, s = math.cos(th), math.sin(th)
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)

def engeller():
    S = r.points(1)
    H = harita_pts(S) if '--harita-engel' in sys.argv else np.zeros((0, 2))   # canli SLAM haritasi kayik olabiliyor
    return S, np.concatenate([S, H, zihin_pts()])
def koridor(P, ang, w=KORIDOR):
    c, s = math.cos(ang), math.sin(ang); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
    sel = (px > 0) & (np.abs(py) < w)
    return float(px[sel].min()) if sel.any() else 9.0
def etraf(P):
    a = np.arctan2(P[:, 1], P[:, 0]); sel = np.abs(a) < ARKA
    return float(np.hypot(P[sel, 0], P[sel, 1]).min()) if sel.any() else 9.0
def arka_bos(S):
    a = np.arctan2(S[:, 1], S[:, 0]); sel = np.abs(a) > math.radians(145)
    return float(np.hypot(S[sel, 0], S[sel, 1]).min()) if sel.any() else 9.0
def on_mesafe(S):
    a = np.arctan2(S[:, 1], S[:, 0]); sel = np.abs(a) < math.radians(8)
    return float(np.median(np.hypot(S[sel, 0], S[sel, 1]))) if sel.sum() > 3 else 99.0

# --- gezdigi yerler (odom, 0.5 m hucre): gitmedigi yonleri tercih etsin ---
GEZDI = set(); HUC = 0.5
def gezdi_isle():
    x, y, _ = r.odo()
    for dx in (-0.3, 0.0, 0.3):
        for dy in (-0.3, 0.0, 0.3): GEZDI.add((round((x + dx) / HUC), round((y + dy) / HUC)))
def yeni_hucre(ang, mesafe):
    x, y, th = r.odo(); n = 0; seen = set()
    for k in np.arange(0.5, min(mesafe, 4.0), 0.25):
        h = (round((x + k * math.cos(th + ang)) / HUC), round((y + k * math.sin(th + ang)) / HUC))
        if h not in seen: seen.add(h); n += h not in GEZDI
    return n

def stop(): r.stop(); r.wait(0.3)
cum = 0.0; prev_yaw = None
def upd_cum():
    global cum, prev_yaw
    y = r.odo()[2]
    if prev_yaw is not None: cum += wrap(y - prev_yaw)
    prev_yaw = y

# --- takilma (stall) algilama: tekerler donuyor ama lidar hareket gormuyor ---
EX = ThreadPoolExecutor(1)
def _olc(P, P0, g):
    a = icp(P, P0, (0.0, 0.0, 0.0)); b = icp(P, P0, g)
    return (a if a[3] > b[3] + 0.1 else b), g
class Takilma:
    def __init__(s, donus): s.donus = donus; s.ref = None; s.fut = None; s.say = 0
    def kontrol(s, S):
        now = time.time(); o = r.odo()
        if s.fut is not None and s.fut.done():
            (x, y, th, fit), g = s.fut.result(); s.fut = None
            if s.donus:
                takili = fit > 0.45 and abs(g[2]) > math.radians(10) and abs(th) < 0.35 * abs(g[2])
                s.say = s.say + 1 if takili else 0
        if not s.donus and s.ref is not None and now - s.ref[2] > 1.2:
            od = math.hypot(o[0] - s.ref[1][0], o[1] - s.ref[1][1]); f0, f1 = s.ref[3], on_mesafe(S)
            if od > 0.10 and f0 < 4.0 and f1 < 4.0:
                s.say = s.say + 1 if (f0 - f1) < 0.3 * od else 0
            s.ref = None
        if s.donus and s.fut is None and (s.ref is None or now - s.ref[2] > 0.8):
            if s.ref is not None:
                P0, o0 = s.ref[0], s.ref[1]; c, sn = math.cos(o0[2]), math.sin(o0[2]); dx, dy = o[0] - o0[0], o[1] - o0[1]
                s.fut = EX.submit(_olc, S.copy(), P0, (c * dx + sn * dy, -sn * dx + c * dy, wrap(o[2] - o0[2])))
            s.ref = (S.copy(), o, now, 0.0)
        if not s.donus and s.ref is None:
            s.ref = (None, o, now, on_mesafe(S))
        return s.say >= (1 if s.donus else 3)   # donerken hemen: odometri yonu kaymasin

def surus(v, mesafe, kontrol_arka=True):
    """kisa duz hareket (kurtulma icin), yavas"""
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = float(v)
    try:
        while math.hypot(r.odo()[0] - x0, r.odo()[1] - y0) < mesafe and not DUR['v']:
            S, P = engeller()
            if v > 0 and koridor(P, 0.0) < GOVDE + 0.15: break
            if v < 0 and kontrol_arka and arka_bos(S) < 0.55: break
            r.pub.publish(tw); r.wait(0.06); upd_cum()
    finally:
        stop()

def kurtul(neden):
    log(f'  TAKILDIM ({neden}) - buraya sanal engel koyup geri cekiliyorum')
    zihin_ekle([(GOVDE + 0.05, y) for y in np.linspace(-0.3, 0.3, 7)])
    for _ in range(50):
        S, _P = engeller()
        if arka_bos(S) >= 0.55 or DUR['v']: break
        if _ == 0: log('  arkam dolu (kabloyu tutan?) - 5 sn bekliyorum, biraz geri cekil')
        r.wait(0.1)
    surus(-0.10, 0.20)

def forward(maxdist=2.5, w=GENIS):
    """duz ilerle; onunde veya on-yanlarinda govdeye 50 cm'den yakin bir sey gorurse dur"""
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = HIZ; tk = Takilma(False)
    try:
        while math.hypot(r.odo()[0] - x0, r.odo()[1] - y0) < maxdist:
            if DUR['v']: return 'durduruldu'
            S, P = engeller()
            if koridor(P, 0.0, w) < DUR_ON:
                return f'onde/yanda engel: lidar {koridor(S, 0.0, w):.2f} m, hepsi {koridor(P, 0.0, w):.2f} m, gitti {math.hypot(r.odo()[0]-x0, r.odo()[1]-y0):.2f} m'
            if etraf(S) < DON_ACIL: return 'yanda engel'
            if tk.kontrol(S): stop(); kurtul('ileri giderken'); return 'takildi'
            r.pub.publish(tw); r.wait(0.06); upd_cum(); gezdi_isle()
        return 'mesafe doldu'
    finally:
        stop()

def turn(rad):
    # once govde etrafinda yer var mi?
    for deneme in range(3):
        S, P = engeller(); e = etraf(S)
        if e >= DON_MIN: break
        log(f'  donmek icin yer dar ({e*100:.0f} cm) - yer aciyorum')
        if koridor(P, 0.0) > 0.70: surus(0.10, 0.15)
        elif arka_bos(S) > 0.70: surus(-0.10, 0.15)
        else: r.wait(3.0)
    else:
        log('  donecek yer yok - bu yonden vazgectim'); return False
    if abs(rad) > BEKLE_ACI:
        log(f'  {math.degrees(rad):+.0f} deg donulecek - kablo icin 10 sn bekleniyor'); r.wait(10.0)
    start = cum; tw = Twist(); tw.angular.z = DONUS_HIZ if rad > 0 else -DONUS_HIZ; tk = Takilma(True)
    try:
        while abs(cum - start) < abs(rad) - math.radians(2):
            if DUR['v']: return False
            S, P = engeller()
            if etraf(S) < DON_ACIL:
                stop(); log('  donerken yakin engel - 3 sn bekliyorum')
                t = time.time()
                while time.time() - t < 3.0 and etraf(engeller()[0]) < DON_ACIL: r.wait(0.1)
                if etraf(engeller()[0]) < DON_ACIL: log('  hala yakin - donusten vazgectim'); return False
                tk = Takilma(True); continue
            if tk.kontrol(S):
                stop(); log('  TAKILDIM (donerken) - geri donup aciliyorum')
                tw2 = Twist(); tw2.angular.z = -tw.angular.z; t = time.time()
                while time.time() - t < 0.6: r.pub.publish(tw2); r.wait(0.06); upd_cum()
                stop(); S, P = engeller()
                if koridor(P, 0.0) > 0.7: surus(0.10, 0.15)
                else: surus(-0.10, 0.15)
                return False
            r.pub.publish(tw); r.wait(0.06); upd_cum()
        return True
    finally:
        stop(); upd_cum()

# ================= ana program =================
log('haritalama baslatiliyor: ' + str(api('/api/start_mapping')))
r.wait(8.0); upd_cum(); gezdi_isle()
t0 = time.time(); adim = 0; kotu = []; w = GENIS
while time.time() - t0 < SURE and not DUR['v']:
    adim += 1
    why = forward(w=w); log(f'adim {adim}: ileri bitti ({why}) | toplam donus {math.degrees(cum):+.0f} deg')
    if DUR['v']: break
    S, P = engeller(); cands = []
    for k in range(-11, 12):
        ang = math.radians(15 * k)
        if abs(ang) < math.radians(30) or abs(cum + ang) > KABLO_LIMIT: continue
        if any(abs(wrap(cum + ang - b)) < math.radians(20) for b in kotu[-3:]): continue
        fd, cw = koridor(P, ang, GENIS), GENIS        # once yanlarinda da 50 cm bosluk olan yonler
        if fd < 1.2: fd, cw = koridor(P, ang), KORIDOR   # yoksa dar gecit (ceza puanli)
        if fd < 1.2: continue
        score = (1.0 * yeni_hucre(ang, fd) + 0.4 * min(fd, 4.0) - 1.0 * abs(ang) / math.pi
                 + random.uniform(0, 0.5) - (0 if cw == GENIS else 1.0))
        cands.append((score, ang, fd, cw))
    on_cands = [c for c in cands if abs(c[1]) <= math.radians(120)]   # geri donmek son care
    if on_cands: cands = on_cands
    if not cands:
        log('  acik yon yok - biraz geri cekiliyorum'); surus(-0.10, 0.25); kotu.clear(); continue
    score, ang, fd, w = max(cands)
    log(f'  en iyi yon {math.degrees(ang):+.0f} deg ({fd:.1f} m acik, {"genis" if w == GENIS else "dar gecit"})')
    if turn(ang): kotu.clear()
    else: kotu.append(cum + ang)
stop()
log('gezinti bitti' + (' (DURDURULDU)' if DUR['v'] else '') + f' | toplam donus {math.degrees(cum):+.0f} deg | sanal engel {len(ZIHIN)}')
log('harita kaydediliyor: ' + str(api('/api/finish_mapping')))
r.wait(25.0)
wp = {'barista': {'isim': 'Barmen', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'door': None,
      'tables': [{'isim': 'Masa 1', 'x': 2.98, 'y': -0.09, 'yaw': 0.0}]}
if '--nokta-yok' not in sys.argv: log('noktalar: ' + str(api('/api/waypoints', wp)))
EX.shutdown(); r.destroy_node(); rclpy.shutdown()
if not DUR['v'] and '--donme' not in sys.argv:
    log('baslangica (B) donuyorum')
    subprocess.run(['python3', '-u', 'baslangica_don.py'])
log('BITTI - bekliyorum')
