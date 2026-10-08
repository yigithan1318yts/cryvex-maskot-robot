"""ALGILAMA - kamera (YOLO11n yapay zeka) + lidar birlesimi, hitbox'lar (Cryvex, 2026-10-06).

LIDAR: tarama noktalari cisimlere ayrilir (kume). Her kume = bir cisim; eni/derinligi olculur
       (yonlu sinir kutusu). Taninsin taninmasin her cisim engeldir -> carpmama guvencesi.
YOLO : kamerada 80 tur nesne (insan, masa, sandalye, koltuk, canta, valiz, saksi, kopek...).
       Kutunun yonu lidar kumeleriyle eslestirilir -> cismin ADI + gercek UZAKLIGI.
       Boyut: en = kutu genisligi(px)/odak * uzaklik, boy = kutu yuksekligi(px)/odak * uzaklik.
HITBOX: cismin gercek olculu kutusu (robot cercevesi). Masa: lidar sadece ayaklari gorur ->
       kameranin olctugu tabla eniyle buyutulur. Insan: KIRMIZI, etrafina +25 cm sosyal pay.
GECIT: onundeki komsu hitbox'lar arasi bosluk; robot 59 cm + 2x4 cm pay >= ise GECILEBILIR.
TARETE (2026-10-07): kamera NEMA 23 uzerinde doner (kamera_tarete.py, ayri surec). Her karede kameranin acisi
       bilinir; kamera donerken cekilen kareler kullanilmaz. Kamera ile lidar is birligi:
       - kamera -> lidar: taninan/olculen cisimler hitbox olarak Nav2'ye ve /nesneler'e (lidarin goremedigi alcak/
         ince cisimler de); lidar kumelerine ad verilir
       - lidar -> kamera: lidarin gordugu ama kameranin tanimadigi / yakindan bakmadigi cisimlere kamera doner
       BAKIS: robot nereye gidecekse kamera ONCE oraya bakar (Nav2 plani, hiz komutu); geri giderken arkaya, yerinde
       donerken donus yonune; duz giderken arada yanlara goz atar; dururken etrafi tarar. /tarete/istek ile
       otonom_gezgin bir yone baktirabilir ("id;aci;sure", aci derece ya da 'arka'; "serbest").
       Kendini gorme: robot govdesinin (59 cm) icine dusen kamera tespitleri yok sayilir.
Cikti: /kamera_engeller (PointCloud2, hitbox'lar dolu) + /kamera_bos (temizleme) -> Nav2 'kamera_layer'
       /nesneler (String JSON: cisimler + gecitler), ~/cryvex_araclar/nesneler.json (hafiza), /tarete/durum
Ekran: http://<robot>:8081/ (kamera + ustten hitbox haritasi)   tek kare: /kare.jpg
Kullanim: ~/cryvex_ai/bin/python -u algilama.py [--fov 48] [--yuk 0.32] [--model yolo11n_ncnn_model] [--tarete-yok]"""
import sys, os, glob, json, math, time, threading, multiprocessing as mp, numpy as np, cv2, rclpy
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Header
import tf2_ros
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kamera_tarete

def arg(ad, v): return type(v)(sys.argv[sys.argv.index(ad) + 1]) if ad in sys.argv else v
FOV, YUK, EGIM, PORT = math.radians(arg('--fov', 48.0)), arg('--yuk', 0.32), math.radians(arg('--egim', 0.0)), arg('--port', 8081)
try:   # olculmus kamera ayari komut satirindan ONCELIKLI (2026-10-07 duvardaki isaretle olculdu: yatay gorus ~38 derece)
    _ka = json.load(open(os.path.expanduser('~/cryvex_araclar/kamera_ayar.json')))
    FOV = math.radians(float(_ka.get('fov', math.degrees(FOV)))); EGIM = math.radians(float(_ka.get('egim', math.degrees(EGIM))))
except Exception:
    pass
MODEL = os.path.expanduser(arg('--model', '~/cryvex_araclar/modeller/yolo11n_416_ncnn_model'))
IMGSZ = arg('--imgsz', 416)
LIDAR_Z, LX, LYY, LY = 0.36, 0.125, -0.045, math.radians(-90)
CAM_X, CAM_Y = LX, LYY
ROBOT_R, GECIT_PAY = 0.295, 0.04                     # robot 59 cm; gecitte iki yanda 4 cm pay
TR = {'person': 'insan', 'chair': 'sandalye', 'dining table': 'masa', 'couch': 'koltuk', 'bench': 'bank',
      'potted plant': 'saksi', 'backpack': 'canta', 'handbag': 'canta', 'suitcase': 'valiz', 'dog': 'kopek',
      'cat': 'kedi', 'bed': 'yatak', 'tv': 'tv', 'laptop': 'laptop', 'bottle': 'sise', 'cup': 'bardak',
      'refrigerator': 'buzdolabi', 'toilet': 'klozet', 'bicycle': 'bisiklet', 'umbrella': 'semsiye'}
YERDE = {'insan', 'sandalye', 'masa', 'koltuk', 'bank', 'saksi', 'canta', 'valiz', 'kopek', 'kedi', 'yatak',
         'buzdolabi', 'bisiklet', 'semsiye', 'klozet'}      # yerde duran (engel) siniflar
RENK = {'insan': (0, 0, 255), 'masa': (0, 200, 255), 'sandalye': (255, 200, 0), 'cisim': (160, 160, 160), 'engel': (0, 255, 255), 'mobilya': (255, 120, 200), 'direk': (0, 140, 255)}
ORTAK = {'jpg': None, 'n': 0, 'P': np.zeros((0, 2)), 'A': np.zeros(0), 'odom': (0.0, 0.0, 0.0)}
TARETE = {}            # kamera motoru sureciyle paylasilan degerler (main'de dolar); bossa kamera sabit, onde
BAKIS = {'log': np.zeros(72), 'son_kare': 0.0, 'dikkat': [], 'sebep': 'baslangic', 'gorev': None, 'goz_t': 0.0,
         'goz_yan': 1.0, 'tarama_i': 0, 'son_hareket': 0.0, 'hedef': 0.0}

def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def kutu_no(a): return int((math.degrees(wrap(a)) + 180.0) // 5) % 72
def bakildi(a): return BAKIS['log'][kutu_no(a)]       # o yone en son ne zaman gecerli kareyle bakildi

def kamera_durumu():
    """(gecerli, pan_rad): kamera donuyorsa / yeni durduysa / konumu belirsizse kare KULLANILMAZ"""
    if not TARETE: return True, 0.0
    pan = math.radians(TARETE['aci'].value)
    if TARETE['durum'].value != 0 or TARETE['hareket'].value: return False, pan
    if time.time() - TARETE['durus'].value < 0.25: return False, pan   # bulanik / tampondaki eski kare
    if kor_aci(TARETE['aci'].value): return False, pan                 # bu acida kamerayi kendi govdesi kapatiyor
    return True, pan

class Dugum(Node):
    def __init__(self):
        super().__init__('algilama')
        self.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.create_subscription(Twist, 'cmd_vel_smoothed', lambda m: self._cmd(m, 'niyet'), 10)   # gitmek istedigi
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._cmd(m, 'cikis'), 10)
        self.create_subscription(Path, 'plan', lambda m: ORTAK.__setitem__('plan', (time.time(), m)), 5)
        self.create_subscription(String, 'tarete/istek', self._istek, 10)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.pub_isaret = self.create_publisher(PointCloud2, 'kamera_engeller', 5)
        self.pub_bos = self.create_publisher(PointCloud2, 'kamera_bos', 5)
        self.pub_nesne = self.create_publisher(String, 'nesneler', 5)
        self.pub_tarete = self.create_publisher(String, 'tarete/durum', 10)

    def _cmd(self, m, ad):
        ORTAK['cmd_' + ad] = (time.time(), m.linear.x, m.angular.z)

    def _istek(self, m):
        s = m.data.strip()
        if s == 'serbest': ORTAK['istek'] = None; return
        if s == 'sifirla':                                               # kamera su an tam onde: burasi yeni 0 (ortalama)
            if TARETE and not TARETE['hareket'].value:
                ORTAK['istek'] = None; BAKIS['hedef'] = 0.0                     # once istegi birak ki eski aciya geri gitmesin
                TARETE['sifirla'].value = 1; log('tarete: sifirlama istendi')
            return
        try:
            kid, aci, sure = s.split(';'); aci = aci if aci == 'arka' else max(-180.0, min(180.0, float(aci)))
            ORTAK['istek'] = {'id': kid, 'aci': aci, 'bitis': time.time() + float(sure), 'bakti': False}
        except Exception:
            log(f'tarete istegi anlasilmadi: {s!r}')

    def _scan(self, m):
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.12) & (r < 8)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
        P = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)
        P = P[np.hypot(P[:, 0], P[:, 1]) > 0.30]                          # govdenin ICI = kendi guc kablosu/fisi: cisim degil
        ORTAK['P'] = P; ORTAK['A'] = np.arctan2(P[:, 1], P[:, 0])

    def _odom(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        ORTAK['odom'] = (p.x, p.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def yayinla(self, isaret, bos, ozet):
        h = Header(); h.frame_id = 'base_footprint'; h.stamp = self.get_clock().now().to_msg()
        self.pub_isaret.publish(point_cloud2.create_cloud_xyz32(h, isaret or [(0.0, 0.0, -5.0)]))
        self.pub_bos.publish(point_cloud2.create_cloud_xyz32(h, bos))
        self.pub_nesne.publish(String(data=json.dumps(ozet, ensure_ascii=False, default=float)))

    def plan_acisi(self):
        """Nav2 planinda robotun ~1.3 m ilerisindeki noktanin yonu (robot cercevesi, kameradan); yoksa None"""
        p = ORTAK.get('plan')
        if not p or time.time() - p[0] > 3.0 or not p[1].poses: return None
        m = p[1]
        try: T = self.tfb.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
        except Exception: return None
        tr, q = T.transform.translation, T.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); c, s = math.cos(yaw), math.sin(yaw)
        xy = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in m.poses[:400]])
        R = np.stack([c * xy[:, 0] - s * xy[:, 1] + tr.x, s * xy[:, 0] + c * xy[:, 1] + tr.y], 1)
        i0 = int(np.argmin(np.hypot(R[:, 0], R[:, 1]))); R = R[i0:]
        yay = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(R, axis=0).T))]) if len(R) > 1 else np.zeros(1)
        j = int(np.searchsorted(yay, 1.3)); pt = R[min(j, len(R) - 1)]
        if math.hypot(*pt) < 0.35: return None                           # hedefe vardi
        return math.atan2(pt[1] - CAM_Y, pt[0] - CAM_X)

# ---------------- bakis: kamera nereye baksin ----------------
def ongoru(v, w, T=2.0):
    """bu hiz komutuyla T sn sonra varilacak noktanin yonu (robot cercevesi); cok yakinsa None"""
    if abs(w) < 1e-3: x, y = v * T, 0.0
    else: x, y = v / w * math.sin(w * T), v / w * (1 - math.cos(w * T))
    if math.hypot(x, y) < 0.15: return None
    return math.atan2(y, x)                                              # govde merkezinden: duz gidiste tam 0

def gorev_bitti(g, simdi):
    """bir yone 'bakma gorevi' tamam mi: oraya vardi, vardiktan sonra gecerli kare islendi, bekleme suresi doldu"""
    if simdi - g['bas'] > 5.0: return True                               # zaman asimi
    if TARETE['hareket'].value or abs(TARETE['aci'].value - g['aci']) > 1.0: return False
    vardi = max(TARETE['durus'].value, g['bas'])
    return BAKIS['son_kare'] > vardi + 0.25 and simdi - vardi >= g['dwell']

def bakis_sec(node):
    """(hedef_derece, sebep, gorev_mu). Oncelik: istek > geri > donus/plan > goz atma > dururken tarama"""
    simdi = time.time(); su_an = TARETE['aci'].value
    ist = ORTAK.get('istek')
    if ist and simdi < ist['bitis']:
        a = ist['aci']; kal = ist['id'].startswith('kal')               # kalibrasyon: govdeyi ogrenmek icin kor acilara da bakar
        if a == 'arka': a = 180.0 if not kor_aci(180.0) else -180.0     # arkaya: govdenin kapatmadigi yandan
        if not kal: a = gorulebilir_aci(a)
        g = BAKIS.get('ist_g')
        if not g or g['id'] != ist['id']: g = BAKIS['ist_g'] = {'id': ist['id'], 'aci': a, 'bas': simdi, 'dwell': 0.0}
        if not ist['bakti'] and gorev_bitti(g, simdi): ist['bakti'] = True
        return a, ('kalibrasyon' if kal else 'istek'), False
    cmd = None
    for ad in ('cmd_niyet', 'cmd_cikis'):
        c = ORTAK.get(ad)
        if c and simdi - c[0] < 0.6 and (abs(c[1]) > 0.02 or abs(c[2]) > 0.10): cmd = c; break
    if cmd:
        BAKIS['son_hareket'] = simdi; BAKIS['gorev'] = None; BAKIS['tarama_i'] = 0
        _t, v, w = cmd
        if v < -0.02:                                                    # GERI: arkaya (gidecegi tarafa) bak
            y = ongoru(v, w, 3.0); d = 180.0 if y is None else math.degrees(y)
            if abs(d) > 175: d = 180.0 if not kor_aci(180.0) else -180.0
            return gorulebilir_aci(d), 'geri', False
        pa = node.plan_acisi()
        if abs(v) <= 0.03:                                               # YERINDE DONUS: donecegi yere once bak
            d = math.degrees(pa) if pa is not None else 75.0 * math.copysign(1, w)
            return gorulebilir_aci(max(-120.0, min(120.0, d))), 'donus', False
        y = pa if pa is not None else ongoru(v, w)
        d = 0.0 if y is None else max(-110.0, min(110.0, math.degrees(y) * (1.0 if pa is not None else 1.3)))
        g = BAKIS.get('goz')                                             # duz giderken arada yanlara goz at
        if g:
            if gorev_bitti(g, simdi): BAKIS['goz'] = None; BAKIS['goz_t'] = simdi
            else: return g['aci'], 'goz atma', True
        if abs(d) < 35 and simdi - BAKIS['goz_t'] > 6.0:                 # yanlara en sik 6 sn'de bir goz at
            hedefler = [math.degrees(a) for _u, a in BAKIS['dikkat'] if abs(math.degrees(a)) < 130]
            ga = hedefler[0] if hedefler else en_eski_yon(su_an, sinir=100.0, haric=d)   # yanlarda en eski bakilan yer
            ga = gorulebilir_aci(ga)
            BAKIS['goz'] = {'aci': ga, 'bas': simdi, 'dwell': 0.3}
            return ga, 'goz atma', True
        return gorulebilir_aci(d), 'gidis yonu', False
    BAKIS['goz'] = None
    if simdi - BAKIS['son_hareket'] < 1.5: return BAKIS['hedef'], 'bekliyor', False
    # DURURKEN SAKIN (2026-10-07 kullanici: "guvercin gibi bakiyor"): ONE bak ve dur. Sadece 1.5 m icinde yeni bir sey
    # belirirse bir kez oraya bakip one don; en sik 10 sn'de bir. Surekli tarama YOK.
    g = BAKIS['gorev']
    if g and not gorev_bitti(g, simdi): return g['aci'], g['sebep'], True
    yakin_dikkat = BAKIS.get('yeni_yakin', [])                          # SADECE yeni beliren (son 4 sn) ve 1.5 m icinde
    if yakin_dikkat and simdi - BAKIS.get('son_dikkat', 0) > 10.0:
        BAKIS['son_dikkat'] = simdi
        a = gorulebilir_aci(math.degrees(yakin_dikkat[0][1]))
        BAKIS['gorev'] = {'aci': a, 'bas': simdi, 'dwell': 1.5, 'sebep': 'yakinda bir sey'}
        return a, 'yakinda bir sey', True
    BAKIS['gorev'] = None
    return 0.0, 'one bakiyor', False

def en_eski_yon(su_an, sinir=165.0, haric=None):
    """SABIT ACI YOK: en uzun suredir bakilmayan yonu sec (5 derecelik her yon aday), yakin olani biraz tercih et.
    Kablo kurali: adaylar hep -sinir..+sinir icinde, git() da mutlak aciyla gider (tam tur yok)."""
    simdi = time.time(); en_iyi, puan = 0.0, -1e9
    for d in np.arange(-sinir, sinir + 0.1, 5.0):
        if haric is not None and abs(d - haric) < math.degrees(FOV) * 0.6: continue   # zaten gorulen yer
        if kor_aci(d): continue                                          # kendi govdesi kapatiyor: bakmanin anlami yok
        # kameranin gorus alani (FOV) boyunca ortalama ne kadar zamandir bakilmadi
        yas = float(np.mean([min(30.0, simdi - bakildi(math.radians(d + b))) for b in range(-20, 21, 5)]))
        p = yas - 0.03 * abs(d - su_an)
        if p > puan: en_iyi, puan = float(d), p
    return en_iyi

def bakis_dongusu(node):
    """10 Hz: hedef aciyi kamera motoru surecine yazar, /tarete/durum yayinlar"""
    while rclpy.ok():
        try:
            if TARETE:
                d, sebep, gorev = bakis_sec(node)
                d = max(-180.0, min(180.0, float(d)))
                simdi = time.time()
                if sebep in ('gidis yonu', 'donus', 'geri'):              # surekli degisen yon: yumusat (titreme olmasin)
                    yd = BAKIS.get('yumusak', d); yd = yd + 0.3 * (d - yd) if abs(d - yd) < 90 else d
                    BAKIS['yumusak'] = d = yd
                    degis = abs(d - BAKIS['hedef']) > 15.0 and simdi - BAKIS.get('son_komut', 0) > 0.5
                else:
                    BAKIS['yumusak'] = d
                    degis = gorev or sebep in ('istek', 'kalibrasyon') or abs(d - BAKIS['hedef']) > 8.0
                if degis and abs(d - BAKIS['hedef']) > 0.5:
                    BAKIS['hedef'] = d; TARETE['hedef'].value = d; BAKIS['son_komut'] = simdi
                BAKIS['sebep'] = sebep
                ist = ORTAK.get('istek')
                node.pub_tarete.publish(String(data=json.dumps({
                    'aci': round(TARETE['aci'].value, 1), 'hedef': round(BAKIS['hedef'], 1), 'hareket': bool(TARETE['hareket'].value),
                    'durum': ['calisiyor', 'gpio bekleniyor', 'KILITLI'][TARETE['durum'].value], 'sebep': sebep,
                    'istek_id': ist['id'] if ist else None, 'bakti': bool(ist and ist['bakti'])})))
        except Exception as e:
            log('bakis hatasi: ' + repr(e))
        time.sleep(0.1)

# ---------------- lidar: kumeleme + yonlu sinir kutusu ----------------
def kumele(P, menzil=6.0):
    if len(P) < 3: return []
    sira = np.argsort(np.arctan2(P[:, 1], P[:, 0])); Q = P[sira]
    Q = Q[np.hypot(Q[:, 0], Q[:, 1]) < menzil]
    if len(Q) < 3: return []
    d = np.hypot(*(Q[1:] - Q[:-1]).T); r = np.hypot(Q[:-1, 0], Q[:-1, 1])
    kes = np.nonzero(d > 0.06 + 0.04 * r)[0] + 1
    # ince direk (vantilator) uzaktan 1-2 nokta verir: 2 nokta da kume sayilir (3 m icinde); gurultuyu takip (5 karenin 3'u) eler
    parcalar = [p for p in np.split(Q, kes) if len(p) >= 4 or (len(p) >= 2 and np.hypot(*p.mean(0)) < 3.0)]
    if len(parcalar) > 1 and np.hypot(*(parcalar[0][0] - parcalar[-1][-1])) < 0.15:   # -180/+180 sinirinda birlesim
        parcalar[0] = np.vstack([parcalar[-1], parcalar[0]]); parcalar.pop()
    return parcalar

def kutu(pts, pay=0.0):
    """yonlu sinir kutusu: (merkez_x, merkez_y, en, derinlik, aci, 4 kose)"""
    (cx, cy), (w, h), a = cv2.minAreaRect((pts * 100).astype(np.float32))
    w, h = max(w / 100, 0.05) + 2 * pay, max(h / 100, 0.05) + 2 * pay
    rect = ((cx / 100, cy / 100), (w, h), a)
    return cx / 100, cy / 100, w, h, math.radians(a), cv2.boxPoints(((cx, cy), (w * 100, h * 100), a)) / 100, rect

def doldur(koseler, adim=0.05):
    xmin, ymin = koseler.min(0); xmax, ymax = koseler.max(0)
    gx, gy = np.meshgrid(np.arange(xmin, xmax + 1e-6, adim), np.arange(ymin, ymax + 1e-6, adim))
    G = np.stack([gx.ravel(), gy.ravel()], 1).astype(np.float32)
    kont = koseler.astype(np.float32).reshape(-1, 1, 2)
    icinde = np.array([cv2.pointPolygonTest(kont, (float(x), float(y)), False) >= 0 for x, y in G])
    return [(float(x), float(y), 0.5) for x, y in G[icinde]]

def poligon_arasi(a, b):
    """iki dortgen arasi en kisa mesafe (kenarlardan ornekleme)"""
    def ornek(k): return np.vstack([np.linspace(k[i], k[(i + 1) % 4], 12) for i in range(4)])
    A, B = ornek(a), ornek(b)
    D = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1]); i, j = np.unravel_index(D.argmin(), D.shape)
    return float(D[i, j]), A[i], B[j]

GOVDE_DOSYA = os.path.expanduser('~/cryvex_araclar/govde_maske.json')
GOVDE_VARSAYILAN = {str(a): 'tam' for a in (-150, -135, -120, -105)}   # gecici: sag arkada lidar diregi + guc kaynagi
GOVDE = {'maske': dict(GOVDE_VARSAYILAN), 't': 0.0}
def govde_maskesi(pan_deg):
    """kamera bu acidayken KENDI GOVDESININ (lidar diregi, guc kaynagi, kablolar) kapladigi yer:
    None (temiz) | 'tam' (goruntunun tamami govde: kamera bu acida kor) | [x1, y1, x2, y2] (0..1 oranli kutu).
    ~/cryvex_araclar/govde_maske.json'dan okunur (15 derecelik acilar), 10 sn'de bir tazelenir."""
    if time.time() - GOVDE['t'] > 10:
        GOVDE['t'] = time.time()
        try: GOVDE['maske'] = json.load(open(GOVDE_DOSYA))
        except Exception: GOVDE['maske'] = dict(GOVDE_VARSAYILAN)
    k = str(int(round(pan_deg / 15.0)) * 15)
    if k == '-180': k = '180' if '180' not in GOVDE['maske'] and '-180' not in GOVDE['maske'] else ('-180' if '-180' in GOVDE['maske'] else '180')
    return GOVDE['maske'].get(k)
def kor_aci(d): return govde_maskesi(d) == 'tam'
def gorulebilir_aci(d):
    """kamera bu acida kor ise en yakin gorebilen aciyi ver (kablo kurali: -180..180 icinde)"""
    if not kor_aci(d): return d
    for f in range(5, 361, 5):
        for a in (d + f, d - f):
            if -180 <= a <= 180 and not kor_aci(a): return float(a)
    return d
def kutular_(maske):                                                    # tek kutu [x1,y1,x2,y2] ya da kutu listesi
    return [maske] if maske and not isinstance(maske[0], (list, tuple)) else (maske or [])
def govdeye_dusuyor(maske, u, v, w, h):
    if maske is None: return False
    if maske == 'tam': return True
    return any(x1 * w <= u <= x2 * w and y1 * h <= v <= y2 * h for x1, y1, x2, y2 in kutular_(maske))

def kablo_bul(img):
    """YERDEKI KABLO (2026-10-08): acik renk zeminde INCE, UZUN, KOYU cizgi; sadece goruntunun alt %45'i
    yakini. Lidar yerdeki kabloyu goremez. Donus: (var_mi, en_uzun_px, satir_orani) ya da (False,0,0)"""
    h, w = img.shape[:2]; y0 = int(h * 0.68)                              # alt %32: robotun ~1 m onu (uzaktaki kablo onemsiz)
    g = cv2.cvtColor(img[y0:], cv2.COLOR_BGR2GRAY); g = cv2.GaussianBlur(g, (5, 5), 0)
    zemin = float(np.median(g))
    if zemin < 110: return False, 0, 0.0                                 # zemin acik renk degil: guvenilmez, bakma
    koyu = ((g < min(70, zemin - 70))).astype(np.uint8) * 255
    koyu = cv2.morphologyEx(koyu, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    sayi, etk, ist, _ = cv2.connectedComponentsWithStats(koyu, 8)
    en = 0; satir = 0.0
    for i in range(1, sayi):
        x, y, bw, bh, alan = ist[i]
        uzun = math.hypot(bw, bh)
        if uzun < w * 0.30 or alan > uzun * 18: continue                 # kisa ya da kalin (kablo degil: golge, esya)
        if uzun > en: en = int(uzun); satir = (y0 + y + bh / 2) / h
    return en > 0, en, satir

def govdede(x, y, pay=0.10):
    """robot cercevesindeki nokta robotun kendi govdesinin (59 cm daire + pay) icinde mi: kamera yana/arkaya
    donunce lidar diregini, guc kaynagini, kasayi gorur -> bunlar engel SAYILMAZ"""
    return math.hypot(x, y) < ROBOT_R + pay

# ---------------- zemin ustu engel (yapay zekanin tanimadigi, lidarin goremedigi) ----------------
ZEMIN = {'ort': None, 'sap': None}
def zemin_engelleri(img, P, f, pan=0.0):
    """robotun onundeki zemini ornek alir; zemine benzemeyen ve yukari uzanan ilk seyin yere bastigi
    noktanin uzakligini (kamera yuksekliginden) hesaplar. Donus: [(yon_rad robot cercevesi, uzaklik_m, u, v)]
    Zemin ornegi sadece kamera one bakarken (|pan| < 9 derece) ve robotun onu bosken alinir."""
    h, w = img.shape[:2]; k = 4                                         # 160x120'de calis
    kucuk = cv2.resize(img, (w // k, h // k)); hsv = cv2.cvtColor(kucuk, cv2.COLOR_BGR2HSV).astype(np.float32)
    kh, kw = hsv.shape[:2]; ufuk = int(kh / 2 + (EGIM * f / k)) + 2
    on_bos = abs(pan) < 0.15
    if len(P) and on_bos:                                               # ornek bolge (onde ~0.6-1 m) bos mu?
        on = (P[:, 0] > 0) & (P[:, 0] < 1.2) & (np.abs(P[:, 1]) < 0.35); on_bos = not on.any()
    ornek = hsv[kh - 12:kh - 2, kw // 2 - 18:kw // 2 + 18].reshape(-1, 3)
    if on_bos:
        o, s = ornek.mean(0), ornek.std(0) + [4, 6, 8]
        ZEMIN['ort'] = o if ZEMIN['ort'] is None else 0.8 * ZEMIN['ort'] + 0.2 * o
        ZEMIN['sap'] = s if ZEMIN['sap'] is None else 0.8 * ZEMIN['sap'] + 0.2 * s
    if ZEMIN['ort'] is None: return []
    o, s = ZEMIN['ort'], ZEMIN['sap']
    fark = np.abs(hsv[..., 1] - o[1]) / s[1] * 0.6 + np.abs(hsv[..., 2] - o[2]) / s[2]   # doygunluk + parlaklik
    dH = np.minimum(np.abs(hsv[..., 0] - o[0]), 180 - np.abs(hsv[..., 0] - o[0]))
    fark += np.where(hsv[..., 1] > 40, dH / 8.0, 0)                    # renkli yuzeyde renk farki da sayilir
    degil = (fark > 4.0).astype(np.uint8)
    degil = cv2.morphologyEx(degil, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    sonuc = []
    for u in range(2, kw - 2, 2):
        kol = degil[ufuk:kh, u][::-1]                                   # alttan yukari
        bas = None; say = 0
        for i, v in enumerate(kol):
            say = say + 1 if v else 0
            if say >= 3:                                                # 3 hucre = ~12 px yukari uzaniyor
                bas = i - 2; break
        if bas is None: continue
        vy = (kh - 1 - bas) * k                                         # yere bastigi satir (tam boyut)
        alfa = math.atan((vy - h / 2) / f) - EGIM
        if alfa < 0.06: continue                                        # ~3 m'den uzak: guvenilmez
        uz = YUK / math.tan(alfa); yon = wrap(math.atan((w / 2 - u * k) / f) + pan)
        if govdede(CAM_X + uz * math.cos(yon), CAM_Y + uz * math.sin(yon)): continue   # kendi govdesi
        sonuc.append((yon, uz, u * k, vy))
    return sonuc

# ---------------- takip (odom cercevesinde) ----------------
class Takip:
    """cisimleri DUNYA (odom) cercevesinde takip eder; titremeyi onlemek icin:
    - son 5 karenin en az 3'unde gorulen cisim 'kararli' sayilir (tek karelik hayaletler cikmaz)
    - merkez, en, derinlik ve aci kareler boyunca yumusatilir
    - kararli cisim gorulmese de 1.5 sn daha yerinde tutulur"""
    def __init__(self): self.n = 0; self.liste = {}
    @staticmethod
    def _param(K):
        K = np.asarray(K, float); m = K.mean(0); e1, e2 = K[1] - K[0], K[2] - K[1]
        return m, float(np.hypot(*e1)), float(np.hypot(*e2)), math.atan2(e1[1], e1[0])
    def guncelle(self, nesneler):
        ox, oy, oth = ORTAK['odom']; c, s = math.cos(oth), math.sin(oth); now = time.time()
        for t in self.liste.values(): t['gor'] = (t['gor'] + [0])[-5:]
        for nes in nesneler:
            W = np.array([(ox + c * x - s * y, oy + s * x + c * y) for x, y in nes['koseler']])
            m, en, der, aci = self._param(W)
            en_iyi, mes = None, 0.5
            for k, t in self.liste.items():
                if t['gor'][-1]: continue                                # bu karede zaten eslesti
                dd = math.hypot(t['m'][0] - m[0], t['m'][1] - m[1])
                if dd < mes: en_iyi, mes = k, dd
            if en_iyi is None:
                self.n += 1; en_iyi = self.n
                self.liste[en_iyi] = {'m': m, 'en': en, 'der': der, 'aci': aci, 'gor': [0, 0, 0, 0, 0], 'oy': {}, 'son': now, 'boy': None, 'ilk': now}
            t = self.liste[en_iyi]
            t['ham'] = nes.get('ham', True)                              # True: lidar kutusu, insan payi henuz eklenmedi
            g = t.setdefault('gecmis', []); g.append((now, float(m[0]), float(m[1])))
            while len(g) > 2 and now - g[0][0] > 1.5: g.pop(0)
            t['hiz'] = (math.hypot(g[-1][1] - g[0][1], g[-1][2] - g[0][2]) / (g[-1][0] - g[0][0])) if g[-1][0] - g[0][0] >= 0.9 else 0.0
            fark = (aci - t['aci'] + math.pi / 2) % math.pi - math.pi / 2  # dikdortgen 180 derece simetrik
            if abs(fark) > math.pi / 4:                                  # en/derinlik yer degistirmis
                en, der = der, en; fark = (fark + math.pi) % math.pi - math.pi / 2
            a = 0.3                                                      # yumusatma
            t['m'] = (1 - a) * np.asarray(t['m']) + a * m; t['en'] = (1 - a) * t['en'] + a * en
            t['der'] = (1 - a) * t['der'] + a * der; t['aci'] = t['aci'] + a * fark
            t['gor'][-1] = 1; t['son'] = now; t['boy'] = nes.get('boy') or t['boy']
            t['oy'][nes['sinif']] = t['oy'].get(nes['sinif'], 0) + (3 if nes['sinif'] not in ('cisim', 'engel') else 1)
            if nes['sinif'] == 'cisim' and t['hiz'] > 0.20 and max(t['en'], t['der']) < 0.8:
                t['oy']['insan'] = t['oy'].get('insan', 0) + 2                # yuruyen insan boyutunda cisim: insan
            if t['oy'].get('insan', 0) >= 6: t['oy']['insan'] = max(t['oy']['insan'], max(t['oy'].values()))   # insan olarak taninan, yaklasinca (sadece bacak gorunce) de insan kalir
            t['kararli'] = t.get('kararli', False) or sum(t['gor']) >= 3
        self.liste = {k: t for k, t in self.liste.items() if now - t['son'] < 30}
        cikti = []
        for k, t in self.liste.items():                                  # kararli + son 1.5 sn icinde gorulmus
            sinif = max(t['oy'], key=t['oy'].get)
            omur = 1.0 if sinif == 'insan' else (5.0 if sinif == 'engel' else 30.0)   # sabit cisimler arkada kalsa da hatirlanir
            if not t.get('kararli') or now - t['son'] > omur: continue
            ek = 0.46 if sinif == 'insan' and t.get('ham', True) else 0.0    # insana +23 cm sosyal pay (iki yana)
            ca, sa = math.cos(t['aci']), math.sin(t['aci']); u = np.array([ca, sa]) * (t['en'] + ek) / 2; v = np.array([-sa, ca]) * (t['der'] + ek) / 2
            W = np.array([t['m'] - u - v, t['m'] + u - v, t['m'] + u + v, t['m'] - u + v])
            d = W - [ox, oy]; R = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)   # robot cercevesine
            u0, v0 = np.array([ca, sa]) * t['en'] / 2, np.array([-sa, ca]) * t['der'] / 2   # paysiz kutu: Nav2 icin
            d0 = np.array([t['m'] - u0 - v0, t['m'] + u0 - v0, t['m'] + u0 + v0, t['m'] - u0 + v0]) - [ox, oy]
            R0 = np.stack([c * d0[:, 0] + s * d0[:, 1], -s * d0[:, 0] + c * d0[:, 1]], 1)
            mr = R.mean(0)
            cikti.append({'id': k, 'sinif': sinif, 'x': float(mr[0]), 'y': float(mr[1]), 'hiz': t.get('hiz', 0.0),
                          'en': t['en'] + ek, 'derinlik': t['der'] + ek, 'boy': t['boy'], 'koseler': R, 'koseler_nav': R0, 'yas': now - t['ilk'],
                          'uz': float(np.min(np.hypot(R[:, 0] - CAM_X, R[:, 1] - CAM_Y)))})
        return cikti

    def kaydet(self):
        try:
            json.dump([{'id': k, 'sinif': max(t['oy'], key=t['oy'].get), 'x': round(float(t['m'][0]), 2), 'y': round(float(t['m'][1]), 2),
                        'en': round(t['en'], 2), 'derinlik': round(t['der'], 2), 'boy': t.get('boy'),
                        'gorulme': time.strftime('%H:%M:%S', time.localtime(t['son']))} for k, t in self.liste.items() if t.get('kararli')],
                      open(os.path.expanduser('~/cryvex_araclar/nesneler.json'), 'w'), ensure_ascii=False, default=float)
        except Exception:
            pass

# ---------------- ana dongu ----------------
def kamera_ac():
    d = (sorted(glob.glob('/dev/v4l/by-id/*video-index0')) or ['/dev/video0'])[0]
    cap = cv2.VideoCapture(d, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap

def kus_bakisi(P, nesneler, gecitler, pan=None, gecerli=True, boyut=480, olcek=100):
    """ustten hitbox haritasi: robot ortada (arkasi da gorunsun), ileri yukari; 1 m = 100 px"""
    img = np.full((boyut, boyut, 3), 30, np.uint8)
    def px(x, y): return int(boyut / 2 - y * olcek), int(boyut / 2 + 40 - x * olcek)
    for m in range(1, 5): cv2.circle(img, px(0, 0), m * olcek, (60, 60, 60), 1)
    if pan is not None:                                                  # kameranin baktigi dilim
        uc = [px(CAM_X, CAM_Y)] + [px(CAM_X + 3.0 * math.cos(pan + b), CAM_Y + 3.0 * math.sin(pan + b))
                                   for b in np.linspace(-FOV / 2, FOV / 2, 9)]
        kat = img.copy(); cv2.fillPoly(kat, [np.array(uc, np.int32)], (70, 90, 40) if gecerli else (40, 40, 90))
        img = cv2.addWeighted(kat, 0.5, img, 0.5, 0)
    for _u, a in BAKIS['dikkat']:                                        # lidarin kameraya gosterdigi yonler
        cv2.line(img, px(CAM_X, CAM_Y), px(CAM_X + 1.5 * math.cos(a), CAM_Y + 1.5 * math.sin(a)), (255, 0, 255), 1)
    for x, y in P[::2]:
        u, v = px(x, y)
        if 0 <= u < boyut and 0 <= v < boyut: img[v, u] = (200, 200, 200)
    for n in nesneler:
        k = np.array([px(x, y) for x, y in n['koseler']], np.int32)
        cv2.polylines(img, [k], True, RENK.get(n['sinif'], (0, 255, 0)), 2)
        cv2.putText(img, f"{n['id']} {n['sinif']}", tuple(k[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, RENK.get(n['sinif'], (0, 255, 0)), 1)
    for g in gecitler:
        renk = (0, 220, 0) if g['gecilir'] else ((0, 0, 255) if g.get('durum') != 'kamera bakmadi' else (0, 220, 255))
        cv2.line(img, px(*g['a']), px(*g['b']), renk, 2)
        cv2.putText(img, f"{g['bosluk']*100:.0f}cm", px((g['a'][0] + g['b'][0]) / 2, (g['a'][1] + g['b'][1]) / 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, renk, 1)
    cv2.circle(img, px(0, 0), int(ROBOT_R * olcek), (255, 255, 255), 2)
    cv2.line(img, px(0, 0), px(0.45, 0), (255, 255, 255), 2)
    cv2.putText(img, 'hitbox haritasi (1 halka = 1 m)', (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img

def ana_dongu(node):
    from ultralytics import YOLO
    model = YOLO(MODEL, task='detect'); log(f'yapay zeka modeli yuklendi: {MODEL}')
    hafiza = []   # (zaman, ad, dunya_x, dunya_y, r, boy) - lidarin goremedigi, kameranin gordugu cisimler
    cap = kamera_ac(); takip = Takip(); t_son = 0.0; t_kayit = 0.0
    while rclpy.ok():
        ok, img = cap.read()
        if not ok:
            time.sleep(0.5); cap.release(); cap = kamera_ac(); continue
        if time.time() - t_son < 0.25: continue                       # en fazla ~4 kare/sn (Nav2'ye islemci kalsin)
        t_son = time.time(); h, w = img.shape[:2]; f = (w / 2) / math.tan(FOV / 2)
        P = ORTAK['P']
        gecerli, pan = kamera_durumu()                                   # kamera donuyorsa bu kare yapay zekaya gitmez
        maske = govde_maskesi(math.degrees(pan)) if TARETE else None     # kendi govdesinin goruntude kapladigi yer
        if TARETE and not TARETE['hareket'].value and time.time() - TARETE['durus'].value > 0.4 \
                and time.time() - ORTAK.get('ham_t', 0) > 0.5:            # govde kalibrasyonu icin ham kare (cizimsiz)
            ORTAK['ham'] = (cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes(), round(TARETE['aci'].value, 1))
            ORTAK['ham_t'] = time.time()
        sonuc = model.predict(img, imgsz=IMGSZ, conf=0.30, verbose=False)[0] if gecerli else None
        kumeler = kumele(P)
        kbilgi = []
        for kp in kumeler:
            cxy = kp.mean(0) - [CAM_X, CAM_Y]
            kbilgi.append({'pts': kp, 'yon': math.atan2(cxy[1], cxy[0]), 'uz': float(np.min(np.hypot(kp[:, 0] - CAM_X, kp[:, 1] - CAM_Y))), 'sinif': 'cisim', 'kam': None})
        kamera_nes = []
        kutular = zip(sonuc.boxes.xyxy.cpu().numpy(), sonuc.boxes.cls.cpu().numpy(), sonuc.boxes.conf.cpu().numpy()) if sonuc is not None else []
        for b, c, s in kutular:
            ad = TR.get(sonuc.names[int(c)], sonuc.names[int(c)])
            x1, y1, x2, y2 = b
            if govdeye_dusuyor(maske, (x1 + x2) / 2, (y1 + y2) / 2, w, h): continue   # kendi govdesi (direk, guc kaynagi)
            b_sol, b_sag = math.atan((w / 2 - x1) / f), math.atan((w / 2 - x2) / f)
            orta, yari = wrap((b_sol + b_sag) / 2 + pan), (b_sol - b_sag) / 2 + 0.03   # robot cercevesinde yon
            esler = [k for k in kbilgi if abs(wrap(k['yon'] - orta)) <= yari]
            if esler:
                en_yakin = min(k['uz'] for k in esler); esler = [k for k in esler if k['uz'] < en_yakin + 0.9]
                uz, kaynak = en_yakin, 'lidar'
            else:
                alfa = math.atan((y2 - h / 2) / f) - EGIM
                uz, kaynak = (YUK / math.tan(alfa), 'zemin') if (y2 < h - 4 and alfa > 0.03) else (None, '')
            en_m = (x2 - x1) / f * uz if uz else None
            boy_m = (y2 - y1) / f * uz if (uz and y1 > 3 and y2 < h - 3) else None
            for k in esler:
                if k['sinif'] == 'cisim' or ad == 'insan': k['sinif'] = ad; k['kam'] = (en_m, boy_m, float(s))
            if not esler and uz and govdede(CAM_X + uz * math.cos(orta), CAM_Y + uz * math.sin(orta)):
                continue                                                 # robotun kendi parcasi (direk, guc kaynagi)
            # 2026-10-07 kullanici kurali: tek (derinliksiz) kamera ile PIKSELDEN MESAFE TAHMINI YASAK. Kamera sadece
            # siniflandirir; mesafe o acidaki LIDAR noktasindan (esler). Lidar karsiligi olmayan tespit engel SAYILMAZ.
            renk = RENK.get(ad, (0, 255, 0))
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), renk, 3 if ad == 'insan' else 2)
            etiket = f"{ad} {uz:.1f}m" if uz else ad
            if en_m: etiket += f" en {en_m*100:.0f}" + (f" boy {boy_m*100:.0f}cm" if boy_m else "cm")
            cv2.putText(img, etiket, (int(x1) + 3, max(14, int(y1) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2)
        # hitbox'lar
        nesneler = []
        # AYAKLAR: lidar 36 cm'de masa/sandalye/insanin sadece AYAKLARINI gorur, arasini bos sanir (masanin alti!).
        # Birbirine 75 cm'den yakin ince kumeler (ayak) tek mobilya sayilir ve ARASI DOLU olur.
        # Iki ayri masa arasindaki gecit (robot 59 cm + pay + tabla tasmasi) bundan genis oldugu icin acik kalir.
        ince = [k for k in kbilgi if max(np.ptp(k['pts'][:, 0]), np.ptp(k['pts'][:, 1])) <= 0.20 and k['uz'] < 5.0]
        ana = list(range(len(ince)))
        def kok(i):
            while ana[i] != i: ana[i] = ana[ana[i]]; i = ana[i]
            return i
        for i in range(len(ince)):
            for j in range(i + 1, len(ince)):
                if np.hypot(*(ince[i]['pts'].mean(0) - ince[j]['pts'].mean(0))) <= 0.75: ana[kok(i)] = kok(j)
        gruplar = {}
        for i, k in enumerate(ince): gruplar.setdefault(kok(i), []).append(k)
        birlesen = set()
        # MASA = AYAKLAR + USTU: 3-4 ince ayak bir DIKDORTGENIN KOSELERINE oturuyorsa (1.5 m'ye kadar) tek masa kutusu
        # (+8 cm tabla tasmasi). Kose kuralina uymayan (yan yana iki masa) birlestirilmez, aradaki gecit acik kalir.
        merk = [k['pts'].mean(0) for k in ince]
        for i in range(len(ince)):
            yakin = sorted([j for j in range(len(ince)) if np.hypot(*(merk[i] - merk[j])) <= 1.5 and id(ince[j]) not in birlesen],
                           key=lambda j: np.hypot(*(merk[i] - merk[j])))[:7]  # en yakin 7 ayak (hiz icin)
            if len(yakin) < 3 or id(ince[i]) in birlesen: continue
            for boyut in (4, 3):
                bulundu = False
                for secim in __import__('itertools').combinations(yakin, boyut):
                    if i not in secim: continue
                    pts = np.array([merk[j] for j in secim], np.float32)
                    (rcx, rcy), (rw, rh), ra = cv2.minAreaRect(pts * 100); rw, rh = rw / 100, rh / 100
                    if not (0.35 <= min(rw, rh) and max(rw, rh) <= 1.6): continue
                    kose = cv2.boxPoints(((rcx, rcy), (rw * 100, rh * 100), ra)) / 100
                    if all(np.min(np.hypot(*(kose - merk[j]).T)) < 0.12 for j in secim):   # her ayak bir kosede
                        kos = cv2.boxPoints(((rcx, rcy), ((rw + 0.16) * 100, (rh + 0.16) * 100), ra)) / 100
                        nesneler.append({'sinif': 'masa', 'x': rcx / 100, 'y': rcy / 100, 'en': rw + 0.16, 'derinlik': rh + 0.16,
                                         'boy': None, 'koseler': kos, 'uz': min(ince[j]['uz'] for j in secim), 'ham': False})
                        birlesen.update(id(ince[j]) for j in secim); bulundu = True; break
                if bulundu: break
        for grup in gruplar.values():
            grup = [k for k in grup if id(k) not in birlesen]               # masaya katilan ayaklar zaten kutuda
            if len(grup) < 2: continue
            siniflar = [k['sinif'] for k in grup]
            sinif = 'insan' if 'insan' in siniflar else ('masa' if 'masa' in siniflar else ('sandalye' if 'sandalye' in siniflar else 'mobilya'))
            if sinif == 'masa': continue                                 # masa asagida tabla eniyle birlikte birlesir
            pts = np.vstack([k['pts'] for k in grup]); cx, cy, en, der, aci, kos, _ = kutu(pts, 0.03)
            kam = next((k['kam'] for k in grup if k['kam']), None)
            nesneler.append({'sinif': sinif, 'x': cx, 'y': cy, 'en': en, 'derinlik': der, 'boy': kam[1] if kam else None,
                             'koseler': kos, 'uz': min(k['uz'] for k in grup)})
            birlesen.update(id(k) for k in grup)
        masalar = {}
        for k in kbilgi:                                                 # masa ayaklarini tek masada birlestir
            if id(k) in birlesen: continue
            if k['sinif'] == 'masa':
                anahtar = round(k['yon'] / 0.35); masalar.setdefault(anahtar, []).append(k)
        for k in kbilgi:
            if k['sinif'] == 'masa' or id(k) in birlesen: continue
            km = k['pts'].mean(0); dP = np.hypot(P[:, 0] - km[0], P[:, 1] - km[1]) if len(P) else np.zeros(0)
            tek_basina = not ((dP > 0.10) & (dP < 0.40)).any()                # 40 cm icinde baska HICBIR sey yok
            if max(np.ptp(k['pts'][:, 0]), np.ptp(k['pts'][:, 1])) <= 0.08 and k['sinif'] in ('cisim', 'mobilya') and tek_basina:
                # TEK BASINA INCE DIREK (vantilator, askilik, tek ayakli masa, semsiye): lidar 36 cm'de sadece diregi
                # gorur, yerdeki GENIS TABANI goremez (2026-10-07 vantilatore sol teker carpti) -> etrafi 28 cm dolu
                cx, cy = k['pts'].mean(0); r = 0.32                         # vantilator tabani ~60 cm cap
                kos = np.array([[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]])
                nesneler.append({'sinif': 'direk', 'x': float(cx), 'y': float(cy), 'en': 2 * r, 'derinlik': 2 * r, 'boy': None,
                                 'koseler': kos, 'uz': max(0.0, k['uz'] - r), 'ham': False})
                continue
            cx, cy, en, der, aci, kos, _ = kutu(k['pts'], 0.02)           # insan payi takipte eklenir (hatirlanan insana da)
            nesneler.append({'sinif': k['sinif'], 'x': cx, 'y': cy, 'en': en, 'derinlik': der, 'boy': (k['kam'] or (None, None))[1],
                             'koseler': kos, 'uz': k['uz']})
        for grup in masalar.values():                                    # masa: ayaklar + tabla eni
            pts = np.vstack([k['pts'] for k in grup]); cx, cy, en, der, aci, kos, rect = kutu(pts, 0.02)
            tabla = max([k['kam'][0] or 0 for k in grup] + [0.6])
            yeni_en, yeni_der = max(en, tabla), max(der, 0.6)
            kos = cv2.boxPoints(((cx * 100, cy * 100), (max(rect[1][0], tabla) * 100, max(rect[1][1], 0.6) * 100), rect[2])) / 100
            nesneler.append({'sinif': 'masa', 'x': cx, 'y': cy, 'en': yeni_en, 'derinlik': yeni_der,
                             'boy': grup[0]['kam'][1] if grup[0]['kam'] else None, 'koseler': kos, 'uz': min(k['uz'] for k in grup)})
        ox, oy, oth = ORTAK['odom']; co, so = math.cos(oth), math.sin(oth); simdi = time.time()
        for ad, uz, yon, en_m, boy_m, s in kamera_nes:                   # yeni gorulenleri hafizaya yaz (dunya cercevesi)
            r = max(0.15, en_m / 2) + (0.25 if ad == 'insan' else 0.0)
            bx, by = CAM_X + (uz + r) * math.cos(yon), CAM_Y + (uz + r) * math.sin(yon)
            hafiza = [hz for hz in hafiza if math.hypot(hz[2] - (ox + co * bx - so * by), hz[3] - (oy + so * bx + co * by)) > 0.5]
            hafiza.append((simdi, ad, ox + co * bx - so * by, oy + so * bx + co * by, r, boy_m))
        def gorunur(wx, wy):                                             # kamera su an o noktaya bakiyor mu
            dx, dy = wx - ox, wy - oy; cx, cy = co * dx + so * dy, -so * dx + co * dy
            return gecerli and abs(wrap(math.atan2(cy - CAM_Y, cx - CAM_X) - pan)) < FOV / 2 - 0.05 and math.hypot(cx - CAM_X, cy - CAM_Y) < 4.5
        # kamera bakarken artik gormuyorsa 1 sn'de silinir; kamera BASKA YONE donmusse hatirlanir (insan 4 sn, diger 20 sn)
        hafiza = [hz for hz in hafiza if simdi - hz[0] < (1.0 if gorunur(hz[2], hz[3]) else (4.0 if hz[1] == 'insan' else 20.0))]
        for _t, ad, wx, wy, r, boy_m in hafiza:
            dx, dy = wx - ox, wy - oy; cx, cy = co * dx + so * dy, -so * dx + co * dy
            uz = math.hypot(cx - CAM_X, cy - CAM_Y) - r
            kos = np.array([[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]])
            nesneler.append({'sinif': ad, 'x': cx, 'y': cy, 'en': 2 * r, 'derinlik': 2 * r, 'boy': boy_m, 'koseler': kos, 'uz': uz, 'ham': False})
        # 2026-10-07: parlak zeminde yansima/golgeyi engel saniyordu (koridoru kapatiyordu) -> varsayilan KAPALI, --zemin ile acilir
        # SADECE YAKIN (1.2 m) ve kamera one yakin bakarken: lidarin altinda kalan alcak tabanlar icin; uzak yansimalar yok
        # 2026-10-07: koyu panelleri/yansimayi engel saniyordu (kullanici: "bombos ama engel diyor") -> TAMAMEN KAPALI
        # (sadece --zemin ile acilir). Alcak tabanlar icin 'direk' kurali; kalici cozum: one yere yakin ToF/tampon.
        zemin = [z for z in zemin_engelleri(img, P, f, pan) if z[1] < 1.2] if (gecerli and '--zemin' in sys.argv) else []
        grup = []
        for yon, uz, u, v in zemin:
            if govdeye_dusuyor(maske, u, v, w, h): continue
            lid =[k['uz'] for k in kbilgi if abs(wrap(k['yon'] - yon)) < 0.06]
            if lid and min(lid) < uz + 0.3: continue                    # lidar zaten goruyor
            if any(abs(wrap(math.atan2(n['y'] - CAM_Y, n['x'] - CAM_X) - yon)) < 0.06 for n in nesneler): continue
            cv2.circle(img, (int(u), int(v)), 4, (0, 255, 255), -1)
            grup.append((yon, uz))
        grup.sort(key=lambda g: wrap(g[0] - pan)); parcalar = []
        for g in grup:                                                  # komsu sutunlari tek engelde topla
            if parcalar and abs(wrap(g[0] - parcalar[-1][-1][0])) < 0.05 and abs(g[1] - parcalar[-1][-1][1]) < 0.4: parcalar[-1].append(g)
            else: parcalar.append([g])
        for pr in parcalar:
            if len(pr) < 2: continue
            pts = np.array([[CAM_X + uz * math.cos(yon), CAM_Y + uz * math.sin(yon)] for yon, uz in pr])
            yb = np.mean([yon for yon, _u in pr])
            pts = np.vstack([pts, pts + [0.15 * math.cos(yb), 0.15 * math.sin(yb)]])   # bakis yonunde 15 cm derinlik
            cx, cy, en, der, aci, kos, _ = kutu(pts, 0.03)
            nesneler.append({'sinif': 'engel', 'x': cx, 'y': cy, 'en': en, 'derinlik': der, 'boy': None, 'koseler': kos,
                             'uz': float(min(uz for _y, uz in pr))})
        nesneler = takip.guncelle(nesneler)                            # kararli, yumusatilmis cisimler
        cp, sp = math.cos(pan), math.sin(pan)
        BOYLAR = {'insan': 1.7, 'masa': 0.75, 'sandalye': 0.9, 'cisim': LIDAR_Z + 0.25, 'engel': 0.45}
        for n in nesneler:                                              # LIDAR -> KAMERA: takip edilen her cismi (hatirlanan insan dahil) kameraya ciz
            if not gecerli: continue
            K0 = np.array(n['koseler']) - [CAM_X, CAM_Y]
            K = np.stack([cp * K0[:, 0] + sp * K0[:, 1], -sp * K0[:, 0] + cp * K0[:, 1]], 1); ileri = K[:, 0]   # kamera cercevesi
            if (ileri < 0.2).any(): continue
            us = w / 2 - f * K[:, 1] / ileri
            if us.max() < 0 or us.min() > w: continue
            yak = float(ileri.min()); vz = h / 2 + f * YUK / yak                    # yere bastigi satir
            vt = h / 2 - f * (BOYLAR.get(n['sinif'], 1.0) - YUK) / yak
            x1, x2 = int(max(0, us.min())), int(min(w - 1, us.max())); renk = RENK.get(n['sinif'], (0, 255, 0))
            kalin = 2 if n['sinif'] == 'insan' else 1
            cv2.rectangle(img, (x1, int(max(0, vt))), (x2, int(min(h - 1, vz))), renk, kalin)
            ek = '' if n['sinif'] in ('cisim', 'engel') else ' (takip)'
            cv2.putText(img, f"{n['id']} {n['sinif']}{ek} {n['uz']:.1f}m en {n['en']*100:.0f}cm", (x1 + 2, int(min(h - 30, max(12, vt - 4)))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, renk, 1)
        # gecitler: onundeki (+-70 derece, 3 m) komsu hitbox'lar arasi
        on = sorted([n for n in nesneler if n['uz'] < 5.0 and abs(math.atan2(n['y'], n['x'])) < math.radians(70)],
                    key=lambda n: math.atan2(n['y'], n['x']))
        if gecerli:                                                     # bakis kaydi: kamera bu karede hangi yonlere bakti
            BAKIS['son_kare'] = simdi
            for b in pan + np.linspace(-FOV / 2, FOV / 2, 12): BAKIS['log'][kutu_no(b)] = simdi
        # GECILIR (yesil) SADECE hepsi saglanirsa: yeterince genis + arkasi acik (masa alti/cikmaz degil) + kamera son 3 sn'de
        # bizzat bakti. Biri bile belirsizse SARI ('kontrol edilmedi'), robot sari/kirmizi gecitten GECMEZ.
        gecitler = []
        for a, b in zip(on, on[1:]):
            bosluk, pa, pb = poligon_arasi(a['koseler'], b['koseler'])
            if bosluk >= 3.0: continue
            orta = (pa + pb) / 2
            gerek = 2 * (ROBOT_R + GECIT_PAY) + 0.10 * sum(s in ('masa', 'mobilya') for s in (a['sinif'], b['sinif']))   # tabla tasmasi
            # ARKASI: araliga DIK yonde (koridor boyunca, robottan uzaga) robot genisliginde 0.8 m bos mu?
            # (robottan cizilen cizgi degil: capraz bakinca koridorun yanindaki duvari 'arka' saniyordu)
            k = pb - pa; nrm = np.array([-k[1], k[0]]) / (np.hypot(*k) + 1e-9)
            if nrm @ orta < 0: nrm = -nrm
            if len(P):
                dP = P - orta; ileri = dP @ nrm; yan = np.abs(dP @ np.array([-nrm[1], nrm[0]]))
                engel = (ileri > 0.05) & (ileri < 0.8) & (yan < min(bosluk / 2, ROBOT_R) - 0.02)
                arkasi_kapali = bool(engel.any())
            else: arkasi_kapali = True
            if bosluk < gerek: durum = 'dar'
            elif arkasi_kapali: durum = 'arkasi kapali'                  # masa alti ayrica ayak birlestirmeyle kapali
            elif simdi - bakildi(math.atan2(orta[1] - CAM_Y, orta[0] - CAM_X)) > 3.0: durum = 'kamera bakmadi'
            else: durum = 'gecilir'
            gecitler.append({'arasi': [a['id'], b['id']], 'siniflar': [a['sinif'], b['sinif']], 'bosluk': round(bosluk, 2), 'a': pa.tolist(), 'b': pb.tolist(),
                             'orta': [round(float(orta[0]), 2), round(float(orta[1]), 2)], 'durum': durum, 'gecilir': durum == 'gecilir'})
        for g in gecitler:                                              # gecitleri kamera goruntusune de (yere) ciz
            if not gecerli: break
            uc = []
            for x, y in (g['a'], g['b']):
                X, Y = cp * (x - CAM_X) + sp * (y - CAM_Y), -sp * (x - CAM_X) + cp * (y - CAM_Y)
                if X < 0.25: break
                uc.append((int(w / 2 - f * Y / X), int(h / 2 + f * YUK / X)))
            if len(uc) < 2: continue
            renk = (0, 220, 0) if g['gecilir'] else ((0, 0, 255) if g['durum'] != 'kamera bakmadi' else (0, 220, 255))
            cv2.line(img, uc[0], uc[1], renk, 3)
            cv2.putText(img, f"{g['bosluk']*100:.0f}cm {g['durum']}", ((uc[0][0] + uc[1][0]) // 2 - 40, min(h - 6, (uc[0][1] + uc[1][1]) // 2 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, renk, 2)
        # Nav2'ye: hitbox'lar dolu, kameranin bos gordugu yonler temiz
        isaret = []
        for n in nesneler: isaret += doldur(n['koseler_nav'])           # Nav2'ye PAYSIZ kutu: Nav2 kendi payini (sisirme) ekler
        # robotun kendi gÃ¶vdesine (+5 cm) tasan isaretler (yanindaki insanin payi gibi) Nav2'ye gitmez: "baslangic dolu" olmasin
        isaret = [p for p in isaret if math.hypot(p[0], p[1]) > ROBOT_R + 0.05]
        dolu = [math.atan2(n['y'] - CAM_Y, n['x'] - CAM_X) for n in nesneler]
        def govde_sutunu(b):                                             # bu yon goruntude govdenin arkasinda mi kaliyor
            if not isinstance(maske, list): return maske == 'tam'
            u = w / 2 - f * math.tan(wrap(b - pan))                     # zemini kapatan (alta inen) govde kutusu
            return any(x1 * w <= u <= x2 * w for x1, y1, x2, y2 in kutular_(maske) if y2 >= 0.9)
        bos = [(float(CAM_X + 3.0 * math.cos(b)), float(CAM_Y + 3.0 * math.sin(b)), 0.5)
               for b in pan + np.linspace(-FOV / 2, FOV / 2, 25)
               if all(abs(wrap(b - d)) > 0.12 for d in dolu) and not govde_sutunu(b)] if gecerli else []   # gormedigi yeri silmez
        # bakis kaydi + LIDAR -> KAMERA: kameranin yakindan bakmadigi / tanimadigi cisimler (yeni, kucuk) ve cok yakin noktalar
        dikkat = []
        for n in nesneler:
            a = math.atan2(n['y'] - CAM_Y, n['x'] - CAM_X)
            if n['sinif'] == 'cisim' and n['uz'] < 3.0 and n['en'] < 1.2 and n.get('yas', 99) < 15 and simdi - bakildi(a) > 5.0:
                dikkat.append((n['uz'], a))
        if len(P):
            d = np.hypot(P[:, 0], P[:, 1]); yakin = P[d < ROBOT_R + 0.45]
            for x, y in yakin[::5]:
                a = math.atan2(y - CAM_Y, x - CAM_X)
                if simdi - bakildi(a) > 8.0: dikkat.append((float(math.hypot(x, y)) - 1.0, a))   # yakin olan once
        BAKIS['dikkat'] = sorted(dikkat)[:3]
        BAKIS['yeni_yakin'] = sorted((n['uz'], math.atan2(n['y'] - CAM_Y, n['x'] - CAM_X)) for n in nesneler
                                     if n.get('yas', 99) < 4.0 and n['uz'] < 1.5 and n['sinif'] in ('cisim', 'insan')
                                     and simdi - bakildi(math.atan2(n['y'] - CAM_Y, n['x'] - CAM_X)) > 3.0)
        kb = kablo_bul(img) if (gecerli and abs(pan) < math.radians(15)) else (False, 0, 0.0)   # sadece kamera ondeyken
        if kb[0]:
            cv2.putText(img, f'KABLO ({kb[1]} px)', (6, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            cv2.line(img, (0, int(kb[2] * h)), (w - 1, int(kb[2] * h)), (0, 165, 255), 1)
        ozet = {'zaman': time.strftime('%H:%M:%S'), 'kablo': bool(kb[0]),
                'tarete': {'aci': round(math.degrees(pan), 1), 'gecerli': gecerli, 'sebep': BAKIS['sebep']},
                'nesneler': [{'id': n['id'], 'sinif': n['sinif'], 'x': round(n['x'], 2), 'y': round(n['y'], 2), 'en': round(n['en'], 2),
                              'derinlik': round(n['derinlik'], 2), 'boy': round(n['boy'], 2) if n['boy'] else None, 'uz': round(n['uz'], 2),
                              'koseler': [[round(float(x), 2), round(float(y), 2)] for x, y in n['koseler']]} for n in nesneler],
                'gecitler': [{k: (v if k not in ('a', 'b') else [round(float(x), 2) for x in v]) for k, v in g.items()} for g in gecitler]}
        node.yayinla(isaret, bos, ozet)
        if time.time() - t_kayit > 5: takip.kaydet(); t_kayit = time.time()
        # ekran: kamera + ustten bakis
        cv2.putText(img, time.strftime('%H:%M:%S') + f'  cisim {len(nesneler)}  insan {sum(n["sinif"] == "insan" for n in nesneler)}',
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if TARETE:
            yazi = f"kamera {math.degrees(pan):+.0f} derece ({BAKIS['sebep']})"
            if TARETE['durum'].value == 2: yazi = 'KAMERA KILITLI: one cevir + --sifirla'
            elif maske == 'tam': yazi += ' - KENDI GOVDESI kapatiyor, kare kullanilmadi'
            elif not gecerli: yazi += ' - DONUYOR, kare kullanilmadi'
            cv2.putText(img, yazi, (6, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255) if gecerli else (0, 0, 255), 1)
        # 2026-10-08 kullanici: canli ekranda SADECE kamera goruntusu (kus bakisi hitbox haritasi istege bagli: --harita)
        ekran = np.hstack([img, kus_bakisi(P, nesneler, gecitler, pan if TARETE else 0.0, gecerli)]) if '--harita' in sys.argv else img
        ORTAK['jpg'] = cv2.imencode('.jpg', ekran, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes(); ORTAK['n'] += 1
        if time.time() - ORTAK.get('kayit_t', 0) >= 1.0:                # KAYIT: saniyede 1 kare, son 15 dk
            ORTAK['kayit_t'] = time.time(); kd = os.path.expanduser('~/cryvex_araclar/kayit'); os.makedirs(kd, exist_ok=True)
            open(os.path.join(kd, time.strftime('%Y%m%d-%H%M%S') + '.jpg'), 'wb').write(ORTAK['jpg'])
            eski = sorted(os.listdir(kd))[:-900]
            for e in eski:
                try: os.remove(os.path.join(kd, e))
                except Exception: pass

SAYFA = b'''<!DOCTYPE html><html lang="tr"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cryvex Algilama</title><style>body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif;padding:12px}
img{width:100%;max-width:1200px;display:block;margin:0 auto;border-radius:8px}h1{font-size:1.1rem;text-align:center}
p{text-align:center;color:#aaa;font-size:.85rem}#d{color:#6c6}</style></head>
<body><h1>Cryvex yapay zeka + lidar: cisimler ve hitbox'lar <span id="d">&#9679;</span></h1><img id="g" alt="canli goruntu">
<p>Kirmizi: insan &middot; Turuncu: masa &middot; Mavi: sandalye &middot; Gri: lidar cismi &middot; Mor: ayaklari birlesen mobilya (masa alti/sandalye) &middot; Sari: zemin engeli &middot; Yesil cizgi: gecilir (genis + arkasi acik + kamera bakti) &middot; Sari: kamera bakmadi (gecmez) &middot; Kirmizi: dar / arkasi kapali (gecmez)</p>
<script>
const g = document.getElementById('g'), d = document.getElementById('d'); let bekliyor = false, son = Date.now();
function yenile() {
  if (bekliyor && Date.now() - son < 3000) return;
  bekliyor = true; const yeni = new Image();
  yeni.onload = () => { g.src = yeni.src; bekliyor = false; son = Date.now(); d.style.color = '#6c6'; };
  yeni.onerror = () => { bekliyor = false; d.style.color = '#c66'; };
  yeni.src = '/kare.jpg?t=' + Date.now();
}
setInterval(yenile, 250); yenile();
</script></body></html>'''
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
        elif self.path.startswith('/ham.jpg'):                          # cizimsiz kamera karesi + o anki kamera acisi
            j, aci = ORTAK.get('ham') or (b'', 0.0)
            self.send_response(200 if j else 503); self.send_header('Content-Type', 'image/jpeg'); self.send_header('X-Kamera-Aci', str(aci))
            self.send_header('Content-Length', str(len(j))); self.end_headers(); self.wfile.write(j)
        elif self.path.startswith('/kare.jpg'):
            j = ORTAK['jpg'] or b''
            self.send_response(200 if j else 503); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(j))); self.end_headers(); self.wfile.write(j)
        elif self.path.startswith('/nesneler.json'):
            try: j = open(os.path.expanduser('~/cryvex_araclar/nesneler.json'), 'rb').read()
            except Exception: j = b'[]'
            self.send_response(200); self.send_header('Content-Type', 'application/json; charset=utf-8'); self.send_header('Content-Length', str(len(j))); self.end_headers(); self.wfile.write(j)
        else:
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(SAYFA))); self.end_headers(); self.wfile.write(SAYFA)

def tarete_baslat():
    """kamera motoru surecini (kamera_tarete.surec) ROS ve is parcaciklarindan ONCE ayir"""
    try: d = json.load(open(kamera_tarete.KONUM_DOSYA)); aci0 = float(d.get('aci', 0.0))
    except Exception: aci0 = 0.0
    ctx = mp.get_context('fork')
    TARETE.update(hedef=ctx.Value('d', 0.0), aci=ctx.Value('d', aci0), hareket=ctx.Value('i', 0), durus=ctx.Value('d', 0.0),
                  durum=ctx.Value('i', 1), kapat=ctx.Value('i', 0), sifirla=ctx.Value('i', 0))
    BAKIS['hedef'] = 0.0
    pr = ctx.Process(target=kamera_tarete.surec, args=(TARETE['hedef'], TARETE['aci'], TARETE['hareket'], TARETE['durus'],
                                                      TARETE['durum'], TARETE['kapat'], os.getpid(), TARETE['sifirla']), daemon=False)
    pr.start(); log(f'kamera motoru sureci basladi (pid {pr.pid}), kamera {aci0:+.1f} derece')
    # TITREMESIN: motor sureci 3. cekirdekte GERCEK ZAMANLI oncelikle; yapay zeka (bu surec) 0-2. cekirdeklerde
    import subprocess
    r = subprocess.run(['sudo', '-n', 'chrt', '-f', '-p', '60', str(pr.pid)], capture_output=True, text=True)
    log('motor sureci gercek zamanli oncelik: ' + ('tamam' if r.returncode == 0 else 'ALINAMADI ' + r.stderr.strip()))
    try: os.sched_setaffinity(0, {0, 1, 2})
    except Exception: pass
    return pr

def main():
    pr = tarete_baslat() if '--tarete-yok' not in sys.argv else None
    rclpy.init(); node = Dugum()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=lambda: ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever(), daemon=True).start()
    if pr: threading.Thread(target=bakis_dongusu, args=(node,), daemon=True).start()
    log(f'algilama calisiyor - ekran http://0.0.0.0:{PORT}/')
    try:
        while rclpy.ok():
            try:
                ana_dongu(node)
            except KeyboardInterrupt:
                break
            except Exception as e:
                import traceback; log('HATA (devam ediyorum): ' + repr(e)); traceback.print_exc(); time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if pr:                                                           # kamerayi one getirip motor surecini kapat
            TARETE['kapat'].value = 1; pr.join(6.0)
            if pr.is_alive(): pr.terminate()
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass                                           # Ctrl+C ROS'u zaten kapatmis olabilir

if __name__ == '__main__':
    main()
