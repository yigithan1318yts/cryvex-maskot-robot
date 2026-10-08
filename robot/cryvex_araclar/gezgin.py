"""GEZGIN - anlik haritalayan, kendi yolunu bulan otonom gezinti (2026-10-05).
- Nereye konursa konsun SIFIRDAN haritalar (SLAM hep acik, harita sabit degil) ve gezer.
- Onunde / on-yanlarinda govdeye 50 cm'den yakin bir sey varsa durur, bos yone doner.
- Once gitmedigi yerlere gider (kesif), hepsini gezdikten sonra da dolasmaya devam eder (devriye).
- Haritada OLMAYAN yeni bir engel yolunu keserse (biri onunden gecerse): beklemeden, en kucuk
  donusle sagindan/solundan gecer; tamamen kapaliysa baska bos yone gider.
- Gorulen engeller SLAM haritasina islenir (ezber); takildigi yerlere 2 dk sanal engel koyar.
- Robot alinip BASKA bir yere konursa (lidar kayitli haritayla ortusmez): eski haritayi
  arsivler, yeni yerde sifirdan haritalamaya baslar.
- Harita her 2 dk'da ~/cryvex_araclar/haritalar/ altina kaydedilir.
Kullanim: python3 -u gezgin.py [--sure SN] [--takip]   (--takip: arkada kabloyu tutan biri var)
Durdurma: uygulamadaki Durdur ya da pkill -f gezgin.py"""
import os, sys, time, math, json, random, subprocess, urllib.request, numpy as np, rclpy
from concurrent.futures import ThreadPoolExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from ortak import Robot, icp

SURE = int(sys.argv[sys.argv.index('--sure') + 1]) if '--sure' in sys.argv else 10 ** 9
GOVDE = 0.30                  # merkezden en dis nokta: govde 52 cm daire, tekerler 54 cm
KORIDOR = GOVDE + 0.03        # carpma geometrisi: govde yaricapi + 3 cm (yanindan gecilen cisimler engel sayilmaz)
DON_MIN = GOVDE + 0.06        # yerinde donmek icin gereken bosluk
DON_ACIL = GOVDE + 0.03       # bundan yakina bir sey gelirse hemen dur
ARKA = math.radians(160 if '--takip' in sys.argv else 181)   # --takip: tam arkadaki insan sayilmaz
HIZ, DAR_HIZ, DONUS_HIZ = 0.10, 0.07, 0.25   # yeni ortamda yavas yavas
if '--hizli' in sys.argv: HIZ, DAR_HIZ, DONUS_HIZ = 0.15, 0.10, 0.30
RASTGELE = '--rastgele' in sys.argv          # gezilmemis yer tercihi yok, rastgele yon
KAYIT_DIR = os.path.expanduser('~/cryvex_araclar/haritalar'); os.makedirs(KAYIT_DIR, exist_ok=True)

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
class YeniYer(Exception): pass

# ---------------- canli SLAM haritasi (ezber) ----------------
HARITA = {'P': None, 'n': 0, 'kotu': 0, 'son': 0.0}
def on_map(m):
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = g >= 65; iy, ix = np.nonzero(occ); res = m.info.resolution; o = m.info.origin.position
    gen = occ.copy()                        # 2 hucre (10 cm) genisletilmis - eslesme kontrolu icin
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            gen |= np.roll(np.roll(occ, dy, 0), dx, 1)
    HARITA.update(P=np.stack([o.x + (ix + 0.5) * res, o.y + (iy + 0.5) * res], 1), gen=gen,
                  ox=o.x, oy=o.y, res=res, n=int(occ.sum()))
r.create_subscription(OccupancyGrid, 'map', on_map,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
def harita_poz():
    try:
        t = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except Exception:
        return None
    q = t.transform.rotation
    return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
def haritada_mi(Pb, poz):
    """robot cercevesindeki noktalarin haritada dolu (bilinen engel) olup olmadigi"""
    x, y, th = poz; c, s = math.cos(th), math.sin(th); gen = HARITA['gen']; res = HARITA['res']
    ix = ((c * Pb[:, 0] - s * Pb[:, 1] + x - HARITA['ox']) / res).astype(int)
    iy = ((s * Pb[:, 0] + c * Pb[:, 1] + y - HARITA['oy']) / res).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < gen.shape[1]) & (iy < gen.shape[0])
    out = np.zeros(len(Pb), bool); out[ok] = gen[iy[ok], ix[ok]]
    return out
def harita_uyum(S):
    if HARITA['P'] is None or HARITA['n'] < 300: return None
    poz = harita_poz()
    if poz is None: return None
    Sy = S[np.hypot(S[:, 0], S[:, 1]) < 4.0][::3]
    return float(haritada_mi(Sy, poz).mean()) if len(Sy) > 20 else None
def harita_pts(S):
    """haritadaki engeller (robot cercevesinde) - sadece konum dogruysa (lidar haritayla ortusuyor)"""
    u = harita_uyum(S); poz = harita_poz()
    if u is None or u < 0.5 or poz is None: return np.zeros((0, 2))
    x, y, th = poz; c, s = math.cos(th), math.sin(th)
    d = HARITA['P'] - [x, y]; d = d[np.hypot(d[:, 0], d[:, 1]) < 2.5]
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)
# --- tasinma algilama 1: lidarin gordugu hareket tekerlerinkinden cok fazla (robot kaldirildi) ---
from nav_msgs.msg import Odometry
LOD = {'m': None}; IZ = []
r.create_subscription(Odometry, 'odom_rf2o', lambda m: LOD.__setitem__('m', m), 10)
def _poz(m):
    q = m.pose.pose.orientation
    return m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
def tasinma_kontrol():
    if LOD['m'] is None: return
    now = time.time()
    if IZ and now - IZ[-1][0] < 0.5: return
    IZ.append((now, r.odo(), _poz(LOD['m'])))
    while IZ and now - IZ[0][0] > 3.0: IZ.pop(0)
    if now - IZ[0][0] < 2.5: return
    (_, w0, l0), (_, w1, l1) = IZ[0], IZ[-1]
    dw = math.hypot(w1[0] - w0[0], w1[1] - w0[1]); dl = math.hypot(l1[0] - l0[0], l1[1] - l0[1])
    aw = abs(wrap(w1[2] - w0[2])); al = abs(wrap(l1[2] - l0[2]))
    if dl > dw + 1.0:   # sadece YER DEGISTIRME (donus sapmasi yanlis alarm veriyordu); 2 kez ust uste
        LOD['say'] = LOD.get('say', 0) + 1
    else:
        LOD['say'] = 0
    if LOD.get('say', 0) >= 2:
        LOD['say'] = 0; IZ.clear(); raise YeniYer(f'tekerler {dw:.2f} m/{math.degrees(aw):.0f} deg, lidar {dl:.2f} m/{math.degrees(al):.0f} deg - kaldirildim')

def yer_kontrol(S):
    """robot baska yere tasindi mi? (1) kaldirilip tasindi (2) lidar, haritayla 8+ sn ortusmuyor"""
    tasinma_kontrol()
    if time.time() - HARITA['son'] < 2.0: return
    HARITA['son'] = time.time(); u = harita_uyum(S)
    if u is None: return
    HARITA['kotu'] = HARITA['kotu'] + 1 if u < 0.25 else 0
    if HARITA['kotu'] >= 4: raise YeniYer(f'harita uyumu %{u*100:.0f}')

# ---------------- harita arsivi ----------------
ODA = {'ad': None, 'son_kayit': 0.0}
def harita_kaydet(bekle=False):
    if ODA['ad'] is None or HARITA['n'] < 300: return
    f = os.path.join(KAYIT_DIR, ODA['ad'])
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', '/map', '-f', f,
           '--ros-args', '-p', 'save_map_timeout:=5.0', '-p', 'use_sim_time:=false']
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if bekle: p.wait(timeout=20)
    ODA['son_kayit'] = time.time()
def yeni_harita():
    ODA['ad'] = 'oda-' + time.strftime('%Y%m%d-%H%M%S')
    log(f'YENI HARITA: {ODA["ad"]} - ' + str(api('/api/start_mapping'))); r.wait(5.0)
    # haritalama zaten aciksa uygulama komutu yok sayiyor -> SLAM'i sifirla (eski harita silinsin)
    rs = subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'],
                        capture_output=True, text=True, timeout=20)
    log('  SLAM sifirlandi' if 'result=0' in rs.stdout.replace(' ', '') else f'  SLAM sifirlanamadi: {rs.stdout[-120:]}')
    HARITA.update(P=None, n=0, kotu=0); GEZDI.clear(); ZIHIN.clear()
    r.wait(8.0)

# ---------------- takildigi yerler (odom cercevesinde, 2 dk) ----------------
ZIHIN = []
def zihin_ekle(pts_base):
    x, y, th = r.odo(); c, s = math.cos(th), math.sin(th)
    for px, py in pts_base: ZIHIN.append((x + c * px - s * py, y + s * px + c * py, time.time()))
def zihin_pts():
    ZIHIN[:] = [z for z in ZIHIN if time.time() - z[2] < 120]
    if not ZIHIN: return np.zeros((0, 2))
    x, y, th = r.odo(); d = np.array(ZIHIN)[:, :2] - [x, y]; c, s = math.cos(th), math.sin(th)
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)

def engeller():
    S = r.points(1); yer_kontrol(S)
    return S, np.concatenate([S, harita_pts(S), zihin_pts()])
def koridor_sec(P, ang, w):
    c, s = math.cos(ang), math.sin(ang); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
    return (px > 0) & (np.abs(py) < w), px
def koridor(P, ang, w=KORIDOR):
    """o yone duz giderse (yuvarlak govde) bir seye DEGMEDEN once gidebilecegi yol (m)"""
    c, s = math.cos(ang), math.sin(ang); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
    sel = (px > 0) & (np.abs(py) < w)
    if not sel.any(): return 9.0
    t = px[sel] - np.sqrt(w * w - py[sel] ** 2)
    return float(max(0.0, t.min()))
def etraf(P):
    a = np.arctan2(P[:, 1], P[:, 0]); sel = np.abs(a) < ARKA
    return float(np.hypot(P[sel, 0], P[sel, 1]).min()) if sel.any() else 9.0
def yan_engel(S, w):
    """tam sagda/solda (govde hizasinda, onde-arkada 30 cm) w'den yakin bir sey var mi"""
    sel = (np.abs(S[:, 0]) < 0.30) & (np.abs(S[:, 1]) < w)
    return bool(sel.any())
def arka_bos(S):
    a = np.arctan2(S[:, 1], S[:, 0]); sel = np.abs(a) > math.radians(145)
    return float(np.hypot(S[sel, 0], S[sel, 1]).min()) if sel.any() else 9.0
def on_mesafe(S):
    a = np.arctan2(S[:, 1], S[:, 0]); sel = np.abs(a) < math.radians(8)
    return float(np.median(np.hypot(S[sel, 0], S[sel, 1]))) if sel.sum() > 3 else 99.0
def yeni_engel_mi(S, w, mesafe):
    """onu kesen (lidar) noktalar haritada yoksa -> sonradan konmus engel"""
    sel, px = koridor_sec(S, 0.0, w); sel &= px < mesafe + GOVDE + 0.05
    if sel.sum() < 3: return False
    u = harita_uyum(S); poz = harita_poz()
    if u is None or u < 0.5 or poz is None: return False      # harita/konum guvenilir degil
    return haritada_mi(S[sel], poz).mean() < 0.4

# ---------------- gezdigi yerler (odom, 0.5 m hucre) ----------------
GEZDI = set(); HUC = 0.5
def gezdi_isle():
    x, y, _ = r.odo()
    for dx in (-0.3, 0.0, 0.3):
        for dy in (-0.3, 0.0, 0.3): GEZDI.add((round((x + dx) / HUC), round((y + dy) / HUC)))
def yeni_hucre(ang, mesafe):
    x, y, th = r.odo(); n = 0; seen = set()
    for k in np.arange(0.5, min(mesafe, 6.0), 0.25):
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

# ---------------- takilma algilama (tekerler donuyor, lidar hareket gormuyor) ----------------
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
        return s.say >= (1 if s.donus else 3)

# ---------------- hareket: mesafeler sabit degil, hiza ve cismin hareketine gore hesaplanir ----------------
A_FREN, T_TEPKI = 0.30, 0.40     # m/s^2 yavaslama, s tepki suresi
def dur_mesafesi(v, v_yaklasan=0.0):
    """degmeden once birakilacak yol: 15 cm pay + tepki + fren mesafesi (karsidan gelenin hizi da eklenir)"""
    vt = max(0.0, v) + max(0.0, v_yaklasan)
    return 0.15 + vt * T_TEPKI + vt * vt / (2 * A_FREN)
def arka_mesafe(P):
    return koridor(P, math.pi, KORIDOR)

def surus(v, mesafe):
    """kisa, yavas duz hareket (kurtulma)"""
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = float(v)
    try:
        while math.hypot(r.odo()[0] - x0, r.odo()[1] - y0) < mesafe and not DUR['v']:
            S, P = engeller()
            if v > 0 and koridor(P, 0.0) < 0.12: break
            if v < 0 and arka_mesafe(S) < 0.12: break
            r.pub.publish(tw); r.wait(0.06); upd_cum()
    finally:
        stop()

def kurtul(neden):
    log(f'  TAKILDIM ({neden}) - sanal engel koyup geri cekiliyorum')
    zihin_ekle([(GOVDE + 0.05, y) for y in np.linspace(-0.3, 0.3, 7)])
    surus(-0.10, 0.25)

def yan_acik_var(S, P):
    """donebilecegi ve yana/ileri acik bir yol gorebilecegi yerde mi?"""
    if etraf(S) < DON_MIN + 0.03: return False
    for k in range(-8, 9):
        ang = math.radians(15 * k)
        if abs(ang) < math.radians(45): continue
        if koridor(P, ang) >= 0.7: return True
    return False

def geri_cik(neden, maks=2.5):
    """dar yerden / karsidan gelenden geri geri cik: donecek ve yeni yol gorecek yere kadar"""
    log(f'  GERI GERI CIKIYORUM ({neden})')
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = -0.10
    try:
        while not DUR['v']:
            gitti = math.hypot(r.odo()[0] - x0, r.odo()[1] - y0)
            S, P = engeller()
            if arka_mesafe(S) < 0.12:
                log(f'  arkam da dolu ({gitti:.2f} m geri gittim)'); return gitti > 0.1
            if gitti > 0.25 and yan_acik_var(S, P) and (neden != 'karsidan gelen' or koridor(S, 0.0) > 0.7):
                log(f'  {gitti:.2f} m geri ciktim - yol var'); return True
            if gitti >= maks:
                log(f'  {gitti:.2f} m geri gittim'); return True
            r.pub.publish(tw); r.wait(0.06); upd_cum(); gezdi_isle()
        return False
    finally:
        stop()

def kacis(neden, maks=3.0):
    """KACIS - geri vites: dar yolda iki araba karsilasinca biri geri gider. Donmeden DUZ geri gider;
    onu acilinca 'temiz' (ayni yone ileri devam), engel kalkmazsa koridordan cikinca 'cikti' doner."""
    log(f'  KACIS ({neden}) - donmeden geri geri gidiyorum')
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = -0.12; temiz_t = None
    try:
        while not DUR['v']:
            gitti = math.hypot(r.odo()[0] - x0, r.odo()[1] - y0)
            S, P = engeller(); on = koridor(S, 0.0)
            if on >= 0.8:                                # onu acildi: 1.5 sn acik kalirsa devam
                temiz_t = temiz_t or time.time(); stop()
                if time.time() - temiz_t > 1.5:
                    log(f'  KACIS bitti: {gitti:.2f} m geri gittim, onum acildi - ayni yone devam'); return 'temiz'
                r.wait(0.1); continue
            temiz_t = None
            if arka_mesafe(S) < 0.12:
                log(f'  KACIS: arkam dolu ({gitti:.2f} m) - bekliyorum'); stop(); r.wait(0.5)
                if arka_mesafe(engeller()[0]) < 0.12: return 'arka dolu'
                continue
            if gitti >= maks or (gitti > 0.4 and etraf(S) >= DON_MIN + 0.05 and yan_acik_var(S, P)):
                log(f'  KACIS: {gitti:.2f} m geri gittim, genis yere ciktim'); return 'cikti'
            r.pub.publish(tw); r.wait(0.06); upd_cum(); gezdi_isle()
        return 'durduruldu'
    finally:
        stop()

def dar_yer(S):
    """etrafinda donecek yer yoksa (masa arasi / koridor)"""
    return etraf(S) < DON_MIN + 0.05

SON = {'mesafe': []}   # son ileri gidislerin mesafeleri (sikisma algilama)
def forward(maxdist=3.0):
    """duz ilerle; hiz engele gore ayarlanir, durma mesafesi hiza ve cismin yaklasmasina gore hesaplanir"""
    x0, y0, _ = r.odo(); tw = Twist(); tk = Takilma(False)
    v = 0.0; onceki = None; v_yak = 0.0; n_yak = 0
    try:
        while math.hypot(r.odo()[0] - x0, r.odo()[1] - y0) < maxdist:
            if DUR['v']: return 'durduruldu'
            S, P = engeller(); f = koridor(P, 0.0); fs = koridor(S, 0.0); now = time.time()
            if onceki is not None and fs < 3.0 and onceki[1] < 3.0 and now - onceki[0] > 0.05:
                yak = (onceki[1] - fs) / (now - onceki[0]) - v      # cismin kendi yaklasma hizi
                if abs(yak) < 1.5:                                  # sicrama (one yeni cisim girdi) degil
                    v_yak = 0.6 * v_yak + 0.4 * yak; n_yak = n_yak + 1 if yak > 0.08 else 0
            onceki = (now, fs)
            ds = dur_mesafesi(v, v_yak if n_yak >= 3 else 0.0)
            if f < ds:
                if v_yak > 0.12 and n_yak >= 3 and fs < 2.0:
                    log(f'  KARSIDAN GELEN VAR ({fs:.2f} m, {v_yak:.2f} m/s yaklasiyor)'); return 'karsidan gelen'
                if yeni_engel_mi(S, KORIDOR, ds):
                    log(f'  YENI ENGEL ({fs:.2f} m) - haritada yok, etrafindan yol ariyorum'); return 'yeni engel'
                return f'engel {f:.2f} m (gitti {math.hypot(r.odo()[0]-x0, r.odo()[1]-y0):.2f} m)'
            if etraf(S) < DON_ACIL: return 'cok yakin engel'
            if tk.kontrol(S): stop(); kurtul('ileri giderken'); return 'takildi'
            v = max(0.04, min(HIZ, 0.5 * (f - ds), v + 0.03))     # engele yaklastikca yavasla, yumusak hizlan
            tw.linear.x = v; r.pub.publish(tw); r.wait(0.06); upd_cum(); gezdi_isle()
        return 'mesafe doldu'
    finally:
        stop(); SON['mesafe'] = (SON['mesafe'] + [math.hypot(r.odo()[0] - x0, r.odo()[1] - y0)])[-5:]

def turn(rad):
    """'ok' | 'yer yok' (donecek alan yok -> geri cikilmali) | 'engel' | 'takildi'"""
    S, P = engeller(); dar = False; acil = DON_ACIL; hiz = DONUS_HIZ
    if etraf(S) < DON_MIN:
        # dar yer (koridorda enine kalmis gibi): once on/arka bosluklari esitle, sonra cok yavas don
        f, b = koridor(S, 0.0), arka_mesafe(S); d = (f - b) / 2
        if abs(d) > 0.02:
            log(f'  dar yer: ortalaniyorum ({d*100:+.0f} cm)')
            x0, y0, _ = r.odo(); tw0 = Twist(); tw0.linear.x = 0.05 if d > 0 else -0.05
            while math.hypot(r.odo()[0] - x0, r.odo()[1] - y0) < min(abs(d), 0.15) and not DUR['v']:
                S0 = engeller()[0]
                if (koridor(S0, 0.0) if d > 0 else arka_mesafe(S0)) < 0.03: break
                r.pub.publish(tw0); r.wait(0.06)
            stop(); S, P = engeller()
        if etraf(S) < GOVDE + 0.02:
            log(f'  donecek yer yok ({etraf(S)*100:.0f} cm)'); return 'yer yok'
        dar = True; acil = GOVDE + 0.01; hiz = 0.15
        log(f'  dar yerde yavas donuyorum (etraf {etraf(S)*100:.0f} cm)')
    start = cum; tw = Twist(); tw.angular.z = hiz if rad > 0 else -hiz; tk = Takilma(True)
    t_bas = time.time()
    try:
        while abs(cum - start) < abs(rad) - math.radians(2):
            if DUR['v']: return 'engel'
            if time.time() - t_bas > 3 * abs(rad) / hiz + 3: log('  donus cok uzun surdu - birakiyorum'); return 'engel'
            S, P = engeller()
            if etraf(S) < acil:
                stop(); log(f'  donerken yakin engel ({etraf(S)*100:.0f} cm)'); return 'yer yok'
            if tk.kontrol(S):
                stop(); log('  TAKILDIM (donerken) - geri donup aciliyorum')
                tw2 = Twist(); tw2.angular.z = -tw.angular.z; t = time.time()
                while time.time() - t < 0.6: r.pub.publish(tw2); r.wait(0.06); upd_cum()
                stop(); return 'takildi'
            r.pub.publish(tw); r.wait(0.06); upd_cum()
        return 'ok'
    finally:
        stop(); upd_cum()

def sikistim():
    return len(SON['mesafe']) >= 5 and sum(SON['mesafe']) < 1.0

def yon_sec(etrafindan_gec):
    kacis = sikistim()
    S, P = engeller(); cands = []
    esik = 0.4 if kacis else max(0.6, dur_mesafesi(HIZ) + 0.4)
    for k in range(-11, 12):
        ang = math.radians(15 * k)
        if abs(ang) < math.radians(30): continue
        if any(abs(wrap(cum + ang - b)) < math.radians(20) for b in kotu[-3:]): continue
        fd = koridor(P, ang)
        if fd < esik: continue
        aci_cezasi = 3.0 if etrafindan_gec else 1.0          # engelin etrafindan: en kucuk donus
        if kacis:                                            # en uzun acik yol
            score = fd + random.uniform(0, 0.3)
        elif RASTGELE and not etrafindan_gec:
            score = 0.3 * min(fd, 4.0) + random.uniform(0, 3.0)
        else:
            score = (1.0 * yeni_hucre(ang, fd) + 0.6 * min(fd, 6.0) - aci_cezasi * abs(ang) / math.pi
                     + random.uniform(0, 0.5))
        cands.append((score, ang, fd, KORIDOR))
    on = [] if kacis else [c for c in cands if abs(c[1]) <= math.radians(120)]
    return max(on or cands) if (on or cands) else None
# ================= ana program =================
t0 = time.time(); adim = 0; kotu = []; KAC = {'yer': None}
def konumum_belli():
    t = time.time()
    while time.time() - t < 5.0:
        if harita_poz() is not None and HARITA['n'] >= 300: return True
        r.wait(0.2)
    return False
if '--devam' in sys.argv and konumum_belli():   # acik haritada konumu biliniyor -> devam
    ODA['ad'] = 'oda-onceki-' + time.strftime('%Y%m%d-%H%M%S'); log('mevcut haritayla devam ediyorum'); r.wait(3.0)
else:
    if '--devam' in sys.argv: log('haritada nerede oldugumu bilmiyorum -> yeni bir yerdeyim, sifirdan haritaliyorum')
    yeni_harita()
upd_cum(); gezdi_isle()
while time.time() - t0 < SURE and not DUR['v']:
    try:
        adim += 1
        why = forward()
        log(f'adim {adim}: ileri bitti ({why}) | gezilen {len(GEZDI)} hucre | harita {HARITA["n"]} engel hucresi')
        if DUR['v']: break
        if time.time() - ODA['son_kayit'] > 120: harita_kaydet()
        if why in ('yeni engel', 'karsidan gelen') or (why.startswith('engel') and dar_yer(engeller()[0])):
            xo, yo, _ = r.odo()
            kalici = KAC['yer'] is not None and math.hypot(xo - KAC['yer'][0], yo - KAC['yer'][1]) < 0.25
            if kalici:                                   # ayni yerde yine ayni engel -> hareket etmiyor
                log('  ayni engel hala orada (kalici) - genis yere kadar geri cikiyorum')
                KAC['yer'] = None; geri_cik('kalici engel', maks=3.0)
            else:
                KAC['yer'] = (xo, yo)
                k = kacis(why.split(' (')[0])            # once geri vites, donme yok
                if k == 'temiz': continue                # onu acildi -> ayni yone ileri
        else:
            KAC['yer'] = None
        sec = yon_sec(why in ('yeni engel', 'karsidan gelen'))
        if sec is None:
            geri_cik('acik yon yok'); kotu.clear(); continue
        score, ang, fd, _w = sec
        log(f'  yon {math.degrees(ang):+.0f} deg ({fd:.1f} m acik)' + ('  [SIKISMA - en uzun yol]' if sikistim() else ''))
        sonuc = turn(ang)
        if sonuc == 'ok': kotu.clear()
        elif sonuc == 'yer yok': geri_cik('donecek yer yok')
        else: kotu.append(cum + ang); (geri_cik('takildim') if sonuc == 'takildi' else None)
    except YeniYer as e:
        stop(); log(f'BASKA BIR YERE TASINDIM ({e}) - eski haritayi kaydedip burada sifirdan basliyorum')
        harita_kaydet(bekle=True)
        log('  yerime oturmam icin 10 sn bekliyorum'); r.wait(10.0)
        yeni_harita(); kotu.clear()
stop()
log('gezinti bitti' + (' (DURDURULDU)' if DUR['v'] else ''))
harita_kaydet(bekle=True); log(f'harita kaydedildi: {KAYIT_DIR}/{ODA["ad"]}.pgm')
EX.shutdown(); r.destroy_node(); rclpy.shutdown()
