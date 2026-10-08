"""OTONOM GEZGIN (Nav2 tabanli) - Cryvex kafe robotu, 2026-10-05.

Hareketi Nav2 yapar (MPPI + Collision Monitor, bkz. cryvex_bringup/launch/otonom.launch.py):
 - dar yerde (iki masa arasi) ileri/geri/donusu MPPI kendisi optimize eder, geri vites dahil
 - onune biri gecerse/yol kapanirsa yeniden planlar; carpmaya 1 sn kala Collision Monitor durdurur
 - takilirsa davranis agaci: 2 sn bekle + 30 cm geri (yerinde fir donme YOK)
Bu dosya NEREYE gidilecegine karar verir:
 1) KESIF: frontier_exploration_ros2 haritanin bilinmeyen kenarlarina sirayla goturur
 2) DEVRIYE: haritadaki bos noktalara gider; son dakikalarda gectigi yerlerden UZAK noktalari secer
    (geldigi yere hemen donmez, once baska yerleri dolasir; hafiza zamanla silinir, sonra geri gelir)
 3) Robot kaldirilip baska yere konursa: haritayi kaydeder, orada sifirdan haritalar (KESIF)
 4) Harita 2 dk'da bir ~/cryvex_araclar/haritalar/ altina kaydedilir
Kullanim:  python3 -u otonom_gezgin.py [--mod kesif|devriye] [--yeni]
           (--yeni: SLAM'i sifirla, bulundugu yerde sifirdan basla)
Durdurma:  uygulamada Durdur, ya da SIGINT/SIGTERM (haritayi kaydedip durur)."""
import os, sys, time, math, json, random, signal, subprocess, urllib.request
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import tf2_ros

MOD = sys.argv[sys.argv.index('--mod') + 1] if '--mod' in sys.argv else 'kesif'
KAYIT_DIR = os.path.expanduser('~/cryvex_araclar/haritalar'); os.makedirs(KAYIT_DIR, exist_ok=True)
HEDEF_MIN, HEDEF_MAX = 2.5, 5.5       # devriye hedefi robottan bu kadar uzakta (Nav2 haritasi 12x12 m: en fazla ~5.5)
HAFIZA_SN = 420.0                   # gecilen yerlerin "yakin zamanda gidildi" sayilma suresi
HEDEF_ZAMAN_ASIMI = 90.0
LY, LX, LYY = math.radians(-90), 0.125, -0.045   # lidar'in govdedeki yeri (urdf ile ayni)

def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def api(path, body=None):
    data = json.dumps(dict(body or {}, password='1234')).encode()
    req = urllib.request.Request('http://127.0.0.1:8080' + path, data=data, headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=30).read())
def yaw_of(q): return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

class YeniYer(Exception): pass

class Gezgin(Node):
    def __init__(self):
        super().__init__('otonom_gezgin')
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map = None; self.cost = None; self.lod = None; self.wod = None
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map', m), tl)
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', lambda m: setattr(self, 'cost', m), 10)
        self.create_subscription(Odometry, 'odom_rf2o', lambda m: setattr(self, 'lod', m), 10)
        self.create_subscription(Odometry, 'wheel/odom', lambda m: setattr(self, 'wod', m), 10)
        self.eod = None; self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'eod', m), 10)
        self.create_subscription(String, 'patrol_command', self._cmd, 10)
        self.scan = None; self.hareket = []; self.nesneler = None
        self.create_subscription(String, 'nesneler', self._nesneler, 10)
        self.tarete = {}; self.create_subscription(String, 'tarete/durum', self._tarete, 10)
        self.istek_pub = self.create_publisher(String, 'tarete/istek', 10)   # kameraya "once suraya bak"
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Empty, 'exploration_complete', lambda m: setattr(self, 'kesif_bitti', True), 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)          # sadece DURDURMAK icin
        self.kurtar_pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)   # kurtarma hareketi: Collision Monitor'den gecer
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self, spin_thread=False)
        self.dur = False; self.kesif_bitti = False; self.iz = []; self.ziyaret = []; self.yasak = []

    def _nesneler(self, m):
        self.nesneler = m.data; self.nesneler_t = time.time()

    def _tarete(self, m):
        try: self.tarete = json.loads(m.data); self.tarete_t = time.time()
        except Exception: pass

    def once_bak(self, aci, sure=30.0, bekle=3.5):
        """KAMERA ONCE BAKSIN: kamerayi aci (derece, + sol; ya da 'arka') yonune cevir, oraya varip en az bir kareyi
        isleyene kadar bekle (en fazla 'bekle' sn). Kamera yoksa/kilitliyse beklemeden devam (lidar yine bakar)."""
        if time.time() - getattr(self, 'tarete_t', 0) > 2.0 or self.tarete.get('durum') != 'calisiyor': return False
        self.istek_no = getattr(self, 'istek_no', 0) + 1; kid = f'g{os.getpid()}-{self.istek_no}'
        a = aci if aci == 'arka' else f'{max(-180.0, min(180.0, float(aci))):.0f}'
        self.istek_pub.publish(String(data=f'{kid};{a};{sure}')); t = time.time()
        while time.time() - t < bekle and not self.dur:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tarete.get('istek_id') == kid and self.tarete.get('bakti'):
                log(f'  kamera {self.tarete.get("aci")} dereceye bakti'); return True
        log('  kamera zamaninda bakamadi - lidarla devam'); return False

    def bilerek_bekliyorum(self):
        """bekci.py'ye haber: robot BILEREK bekliyor (onu/arkasi kapali), 'hareketsiz' diye testi durdurmasin"""
        try: open(os.path.expanduser('~/cryvex_araclar/gezgin_bekliyor'), 'w').write(time.strftime('%H:%M:%S'))
        except Exception: pass

    def bakis_serbest(self):
        self.istek_pub.publish(String(data='serbest'))

    def kamera_noktalari(self):
        """KAMERA -> HAREKET: yapay zekanin tanidigi cisimlerin hitbox'lari (insan +25 cm pay, masa tablasi dahil),
        robot cercevesinde nokta olarak; en fazla 1 sn eski. Lidar 36 cm'de goremediklerini de kapsar."""
        if not self.nesneler or time.time() - getattr(self, 'nesneler_t', 0) > 1.0: return np.zeros((0, 2))
        try: d = json.loads(self.nesneler)
        except Exception: return np.zeros((0, 2))
        pts = []
        for n in d.get('nesneler', []):
            if n['sinif'] in ('cisim', 'engel') or 'koseler' not in n: continue
            K = np.array(n['koseler'], float)
            pts.append(np.vstack([np.linspace(K[i], K[(i + 1) % 4], 8) for i in range(4)]))
        if not pts: return np.zeros((0, 2))
        Q = np.vstack(pts)
        return Q[np.hypot(Q[:, 0], Q[:, 1]) > 0.295 + 0.05]               # govdeye tasan pay (yanimdaki insan) yonleri kapatmasin

    def _cmd(self, m):
        if m.data.strip() == 'stop' or m.data.startswith('teleop') or m.data.startswith('rescue'):
            self.dur = True

    def bekle(self, s):
        t = time.time()
        while time.time() - t < s and not self.dur:
            rclpy.spin_once(self, timeout_sec=0.05)

    def poz(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def aktif_mi(self, dugum):
        try:
            r = subprocess.run(['ros2', 'lifecycle', 'get', '/' + dugum], capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            return False
        return r.stdout.strip().startswith('active')

    def durdur_robot(self):
        for _ in range(5): self.cmd_pub.publish(Twist()); time.sleep(0.05)

    # ---- tasinma: lidarin gordugu yer degistirme tekerlerinkinden >1 m fazla (2 kez ust uste) ----
    def tasinma_kontrol(self):
        if self.lod is None or self.wod is None: return
        now = time.time()
        if self.iz and now - self.iz[-1][0] < 0.5: return
        p = lambda m: (m.pose.pose.position.x, m.pose.pose.position.y)
        self.iz.append((now, p(self.wod), p(self.lod)))
        while self.iz and now - self.iz[0][0] > 3.0: self.iz.pop(0)
        if now - self.iz[0][0] < 2.5: return
        (_, w0, l0), (_, w1, l1) = self.iz[0], self.iz[-1]
        dw = math.hypot(w1[0] - w0[0], w1[1] - w0[1]); dl = math.hypot(l1[0] - l0[0], l1[1] - l0[1])
        self.tas_say = getattr(self, 'tas_say', 0) + 1 if dl > dw + 1.0 else 0
        if self.tas_say >= 2:
            self.tas_say = 0; self.iz.clear()
            raise YeniYer(f'tekerler {dw:.2f} m, lidar {dl:.2f} m')

    # ---- harita ----
    def harita_kaydet(self, bekle=False):
        if self.map is None or not getattr(self, 'oda', None): return
        f = os.path.join(KAYIT_DIR, self.oda)
        p = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', '/map', '-f', f,
                              '--ros-args', '-p', 'save_map_timeout:=5.0', '-p', 'use_sim_time:=false'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if bekle:
            try: p.wait(timeout=20)
            except Exception: pass
        self.son_kayit = time.time()

    def baslat(self, yeni):
        """SLAM + Nav2 ayakta olsun; yeni=True ise harita sifirlanir"""
        slam_acikti = self.aktif_mi('slam_toolbox')
        log('haritalama + Nav2: ' + str(api('/api/start_mapping')) + (' (zaten acikti)' if slam_acikti else ' (yeni acildi)'))
        if yeni and slam_acikti:              # yeni acildiysa harita zaten bos; sifirlama Nav2 acilisini bozuyordu
            self.bekle(5.0)
            rs = subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'],
                                capture_output=True, text=True, timeout=30)
            log('  harita sifirlandi' if 'result=0' in rs.stdout.replace(' ', '') else '  harita sifirlanamadi')
        if yeni:
            self.oda = 'oda-' + time.strftime('%Y%m%d-%H%M%S'); self.ziyaret.clear(); self.yasak.clear()
        elif not getattr(self, 'oda', None):
            self.oda = 'oda-' + time.strftime('%Y%m%d-%H%M%S')
        log('  Nav2 bekleniyor (tum parcalar aktif olana kadar)...')
        t = time.time(); yeniden = False
        while not all(self.aktif_mi(x) for x in ('controller_server', 'planner_server', 'bt_navigator', 'collision_monitor')):
            self.bekle(3.0)
            if self.dur: raise RuntimeError('durduruldu')
            if time.time() - t > 180:
                if yeniden: raise RuntimeError('Nav2 acilmadi')
                log('  Nav2 3 dk icinde acilmadi - haritalamayi kapatip aciyorum')
                api('/api/cancel_mapping'); self.bekle(10.0); api('/api/start_mapping'); t = time.time(); yeniden = True
        self.nav.wait_for_server(timeout_sec=10.0)
        while self.poz() is None and time.time() - t < 150: self.bekle(0.5)
        self.son_kayit = time.time()
        log('  hazir')
        self.bekle(1.0); P = self.tarama()
        if len(P) and float(np.hypot(P[:, 0], P[:, 1]).min()) < 0.40:
            if not getattr(self, 'sadece_geri', False):
                log('  baslangicta dar yerdeyim'); self.dar_yer_kurtar()

    # ---- hedefe git ----
    def git(self, x, y, yaw, aralik_ara=False):
        g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2)
        fut = self.nav.send_goal_async(g)
        while not fut.done(): rclpy.spin_once(self, timeout_sec=0.05)
        h = fut.result()
        if not h.accepted: return 'reddedildi'
        res = h.get_result_async(); t = time.time(); self.hareket.clear(); t_ara = time.time()
        bas_poz = self.epoz() if (self.eod or self.wod) else None
        try:
            while not res.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if bas_poz and time.time() - t > 6.0:                       # Nav2 6 sn'de kipirdatamadiysa bekleme: kendin cik
                    x, y, th = self.epoz()
                    if math.hypot(x - bas_poz[0], y - bas_poz[1]) < 0.05 and abs(wrap(th - bas_poz[2])) < math.radians(10):
                        h.cancel_goal_async(); self.bekle(0.3); return 'kipirdamadi'
                    bas_poz = None
                if aralik_ara and time.time() - t_ara > 1.0:                # gezerken aralik gorursem hedefi birak
                    t_ara = time.time()
                    ad = self.aralik_adaylari()
                    if ad:
                        self.gorulen_aralik = ad[0]                         # gordugum araligi unutma
                        h.cancel_goal_async(); self.bekle(0.5); return 'aralik gordum'
                self.tasinma_kontrol(); self.ziyaret_isle()
                if time.time() - self.son_kayit > 120: self.harita_kaydet()
                if self.sikisma_kontrol():
                    if True:   # 2026-10-06 kullanici: yerinde ufak ufak donme yok, sadece geri cekil
                        h.cancel_goal_async(); self.bekle(0.5); return 'sikisti'   # 2026-10-06 kullanici: geri geri gelmesin
                    self.dar_yer_kurtar(); return 'dar yer'
                if self.dur or time.time() - t > HEDEF_ZAMAN_ASIMI:
                    h.cancel_goal_async(); self.bekle(0.5)
                    return 'durduruldu' if self.dur else 'zaman asimi'
        except YeniYer:
            h.cancel_goal_async(); raise
        return {4: 'ulasti', 5: 'iptal', 6: 'basarisiz'}.get(res.result().status, str(res.result().status))

    def ziyaret_isle(self):
        p = self.poz(); now = time.time()
        if p and (not self.ziyaret or now - self.ziyaret[-1][2] > 2.0):
            self.ziyaret.append((p[0], p[1], now))
        self.ziyaret = [v for v in self.ziyaret if now - v[2] < HAFIZA_SN]
        self.yasak = [v for v in self.yasak if now - v[2] < 120]

    # ---- devriye hedefi: bos, ulasilabilir, yakinda gidilmemis ----
    def hedef_sec(self):
        p = self.poz(); m = self.cost or self.map
        if p is None or m is None: return None
        w, h, res = m.info.width, m.info.height, m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
        g = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        serbest = (g >= 0) & (g < (30 if m is self.cost else 1))
        iy, ix = np.nonzero(serbest)
        if not len(ix): return None
        X = ox + (ix + 0.5) * res; Y = oy + (iy + 0.5) * res
        d = np.hypot(X - p[0], Y - p[1]); sel = (d > HEDEF_MIN) & (d < HEDEF_MAX)
        if sel.sum() < 10: sel = d > 0.8
        X, Y, d = X[sel], Y[sel], d[sel]
        if not len(X): return None
        k = np.random.choice(len(X), min(400, len(X)), replace=False); X, Y, d = X[k], Y[k], d[k]
        now = time.time(); skor = np.random.uniform(0, 0.3, len(X)) + 0.35 * np.minimum(d, HEDEF_MAX)   # UZAK yerler tercih
        for vx, vy, vt in self.ziyaret:            # yakinda gecilen yerlere GENIS ceza (zamanla azalir): ayni yerde dolasma
            agirlik = 1.0 - (now - vt) / HAFIZA_SN
            skor -= 3.0 * agirlik * np.exp(-((X - vx) ** 2 + (Y - vy) ** 2) / (2 * 1.5 ** 2))
        try:                                       # kalici yasak bolgeler (siparis alinan masa aralari)
            for vx, vy, vr in json.load(open(os.path.expanduser('~/cryvex_araclar/yasak_bolgeler.json'))):
                skor -= 20.0 * np.exp(-((X - vx) ** 2 + (Y - vy) ** 2) / (2 * vr ** 2))
        except Exception:
            pass
        for vx, vy, vt in self.yasak:              # ulasilamayan hedefler
            skor -= 5.0 * np.exp(-((X - vx) ** 2 + (Y - vy) ** 2) / (2 * 0.6 ** 2))
        i = int(np.argmax(skor))
        return X[i], Y[i], math.atan2(Y[i] - p[1], X[i] - p[0])

    def devriye(self, sure=None):
        log('DEVRIYE: haritadaki bos noktalara, yakinda gitmedigi yerlere oncelik vererek' + (f' ({sure:.0f} sn)' if sure else ''))
        hata = 0; t_bas = time.time()
        while not self.dur and (sure is None or time.time() - t_bas < sure):
            hd = self.hedef_sec()
            if hd is None: log('  hedef bulunamadi, bekliyorum'); self.bekle(3.0); continue
            x, y, yaw = hd; p = self.poz()
            log(f'  hedef ({x:+.1f}, {y:+.1f}) - {math.hypot(x - p[0], y - p[1]):.1f} m')
            s = self.git(x, y, yaw); log(f'  -> {s}')
            if s in ('kipirdamadi', 'sikisti', 'basarisiz'): self.acik_yere_cik(f'Nav2 {s}')
            if s == 'ulasti': hata = 0; self.bekle(1.0)
            elif s != 'durduruldu':
                pass   # geri cekilme yok (kullanici istegi)
                self.yasak.append((x, y, time.time())); hata += 1
                if hata >= 3: log('  ust uste 3 hedefe gidilemedi - kisa bekleme'); self.bekle(3.0); hata = 0

    # ---- ARALIK AVI: gez, masa aralarini KENDIN bul, once olc, sigiyorsa gec ----
    def aralik_adaylari(self):
        """algilamanin buldugu araliklar (harita cercevesinde): genis + arkasi acik olanlar. Darsa ya da arkasi kapaliysa
        (masa alti) hatirlanir, oraya hic gidilmez. Donus: [(gecit, merkez_harita, yaklasma_noktasi, yaklasma_yonu)]"""
        p = self.poz()
        try: d = json.loads(self.nesneler) if self.nesneler and time.time() - getattr(self, 'nesneler_t', 0) < 1.5 else None
        except Exception: d = None
        if not d or p is None: return []
        c, s = math.cos(p[2]), math.sin(p[2]); simdi = time.time(); sonuc = []
        hafiza = [h for h in getattr(self, 'aralik_hafiza', []) if simdi - h[3] < 240]; self.aralik_hafiza = hafiza
        for g in d['gecitler']:
            a, b, o = np.array(g['a']), np.array(g['b']), np.array(g['orta'])
            if math.hypot(*o) > 4.0 or g['bosluk'] > 2.0: continue            # cok uzak / aralik degil acik alan
            mx, my = p[0] + c * o[0] - s * o[1], p[1] + s * o[0] + c * o[1]
            if any(math.hypot(mx - hx, my - hy) < 0.5 for hx, hy, _n, _t in hafiza): continue   # bunu zaten denedim/gectim
            if g.get('durum') in ('dar', 'arkasi kapali'):
                log(f"  aralik {g['bosluk']*100:.0f} cm ({math.hypot(*o):.1f} m ileride): {g['durum']} - oraya HIC GITMIYORUM")
                hafiza.append((mx, my, g['durum'], simdi)); continue
            k = b - a; n = np.array([-k[1], k[0]]) / (np.hypot(*k) + 1e-9)    # araligin normali, robottan uzaga dogru
            if n @ o < 0: n = -n
            y = o - 0.9 * n                                                  # araligin 90 cm onu (robot cercevesi)
            yx, yy = p[0] + c * y[0] - s * y[1], p[1] + s * y[0] + c * y[1]
            yaw = p[2] + math.atan2(n[1], n[0])
            sonuc.append((g, (mx, my), (yx, yy), yaw))
        return sorted(sonuc, key=lambda t: math.hypot(t[1][0] - p[0], t[1][1] - p[1]))

    def araliga_gir(self, aday):
        """yaklasma noktasina git (yuzu araliga donuk), araligi yeniden bul, olc; sigiyorsa dumduz gec"""
        g, (mx, my), (yx, yy), yaw = aday
        self.aralik_hafiza.append((mx, my, 'denendi', time.time()))
        o = np.array(g['orta']); on_tarafta = math.hypot(*o) < 2.5 and abs(math.atan2(o[1], o[0])) < math.radians(35)
        if on_tarafta:                                                     # zaten onumde ve yakin: Nav2'ye gerek yok
            log(f"  ARALIK BULDUM: {g['bosluk']*100:.0f} cm ({g['siniflar'][0]} - {g['siniflar'][1]}), {math.hypot(*o):.1f} m onumde - Nav2'siz olcuyorum")
        else:
            log(f"  ARALIK BULDUM: {g['bosluk']*100:.0f} cm ({g['siniflar'][0]} - {g['siniflar'][1]}) - once karsisina geciyorum")
            s = self.git(yx, yy, yaw); log(f'  -> {s}')
            if s in ('kipirdamadi', 'sikisti', 'basarisiz'):
                p = self.poz()                                             # yaklastiysam Nav2'siz devam, degilse kendim cik
                if not (p and math.hypot(p[0] - mx, p[1] - my) < 2.5): self.acik_yere_cik(f'Nav2 {s}'); return False
                log('  araliga yakinim - Nav2 takildi ama kendim olcup devam ediyorum')
            elif s != 'ulasti': return False
        self.bekle(1.0); self.once_bak(0, sure=6.0)
        p = self.poz(); c, si = math.cos(p[2]), math.sin(p[2])
        try: d = json.loads(self.nesneler)
        except Exception: return False
        en_iyi = None
        for x in d['gecitler']:                                              # ayni araligi robot cercevesinde bul
            ox, oy = p[0] + c * x['orta'][0] - si * x['orta'][1], p[1] + si * x['orta'][0] + c * x['orta'][1]
            if math.hypot(ox - mx, oy - my) < 0.8 and (en_iyi is None or math.hypot(ox - mx, oy - my) < en_iyi[0]):
                en_iyi = (math.hypot(ox - mx, oy - my), x)
        if en_iyi is None: log('  karsisina gelince araligi goremedim - gecmiyorum'); return False
        ok, neden = self.girebilir_miyim(en_iyi[1])
        log(f"  {en_iyi[1]['bosluk']*100:.0f} cm aralik: " + ('GIREBILIRIM - ' if ok else 'GIREMEM, denemiyorum - ') + neden)
        if not ok: return False
        self.tekrar_hakki = 1
        return self.duz_gec(np.array(en_iyi[1]['orta']), np.array(en_iyi[1]['a']), np.array(en_iyi[1]['b']))

    def aralik_avi(self, sure):
        """GEZ + ARALIK BUL + OLC + GEC. Aralik yoksa devriye hedefine dogru gezer (en fazla 25 sn), gezerken araliklara bakar."""
        log(f'ARALIK AVI: gezip masa aralarini kendim buluyorum ({sure:.0f} sn)'); t_bas = time.time(); gecilen = 0
        self.aralik_hafiza = []
        P = self.tarama()
        if len(P) and float(np.hypot(P[:, 0], P[:, 1]).min()) < 0.295 + 0.30:
            self.acik_yere_cik('baslangicta etrafim dar')                # once dar yerden kendim cikayim
        while not self.dur and time.time() - t_bas < sure:
            adaylar = self.aralik_adaylari()
            if not adaylar and getattr(self, 'gorulen_aralik', None): adaylar = [self.gorulen_aralik]   # az once gorduk
            self.gorulen_aralik = None
            if adaylar:
                if self.araliga_gir(adaylar[0]): gecilen += 1; log(f'  toplam gecilen aralik: {gecilen}')
                continue
            hd = self.hedef_sec()
            if hd is None: self.bekle(2.0); continue
            x, y, yaw = hd; log(f'  aralik gormuyorum - gezmeye devam: hedef ({x:+.1f}, {y:+.1f})')
            s = self.git(x, y, yaw, aralik_ara=True); log(f'  -> {s}')
            if s in ('kipirdamadi', 'sikisti', 'basarisiz'): self.acik_yere_cik(f'Nav2 {s}')   # Nav2'yi bekleme
            if s not in ('ulasti', 'aralik gordum', 'durduruldu'): self.yasak.append((x, y, time.time()))
        log(f'ARALIK AVI BITTI: {gecilen} aralik gecildi')

    def masaya_git_ve_gir(self):
        """kaydedilen masa girisine Nav2 ile git, sonra koridor_cik ile ortalanip duz gir"""
        try:
            m = json.load(open(os.path.expanduser('~/cryvex_araclar/masa_girisi.json')))
        except Exception:
            log('kayitli masa girisi yok'); return
        hx, hy = m['x'] - 0.8 * math.cos(m['yaw']), m['y'] - 0.8 * math.sin(m['yaw'])   # girisin 80 cm onu
        log(f'MASAYA DONUYORUM: once girisin 80 cm onune ({hx:+.2f}, {hy:+.2f})')
        for deneme in range(3):
            s = self.git(hx, hy, m['yaw']); log(f'  -> {s}')
            p = self.poz()
            if p and math.hypot(p[0] - hx, p[1] - hy) < 0.5: s = 'ulasti'
            if s == 'ulasti' or self.dur: break
        if self.dur: return
        p = self.poz()
        if p: log(f'  giristeyim: hedefe {math.hypot(p[0]-m["x"], p[1]-m["y"])*100:.0f} cm, yon farki {math.degrees(wrap(p[2]-m["yaw"])):+.0f} deg')
        self.nav_iptal()
        r = subprocess.run(['python3', '-u', os.path.expanduser('~/cryvex_araclar/koridor_cik.py'), '--gir', '0.6', '--masa'],
                           capture_output=True, text=True, timeout=240)
        for satir in r.stdout.splitlines(): log('  [koridor] ' + satir.split(' ', 1)[-1])

    def geri_cekil(self, mesafe=0.25):
        """yol bulunamayinca: arkasi bossa yavasca geri gel (Collision Monitor uzerinden)"""
        P = self.tarama()
        if self.yol(P, math.pi) < mesafe + 0.10: log('  arkam dolu - geri cekilemiyorum'); return False
        self.once_bak('arka', sure=mesafe / 0.08 + 6.0)                 # once kamera arkaya baksin
        if self.yol(self.tarama(), math.pi) < mesafe + 0.10: log('  kamera arkada engel gordu - geri cekilmiyorum'); self.bakis_serbest(); return False
        log(f'  {mesafe*100:.0f} cm geri cekiliyorum')
        x0 = self.wod.pose.pose.position.x if self.wod else 0.0; y0 = self.wod.pose.pose.position.y if self.wod else 0.0
        def gitti():
            q = self.wod.pose.pose.position; return math.hypot(q.x - x0, q.y - y0)
        self.surus_cmd(-0.08, 0.0, mesafe / 0.08 + 2.0, lambda: gitti() >= mesafe or self.yol(self.tarama(), math.pi) < 0.10)
        self.bakis_serbest()
        return True

    def cikis_yonu(self, P):
        """360 derece: en az donusle en acik yon. Her yon icin: o yone dumduz kac m gidilir (yuvarlak govde) ve yol
        boyunca iki yanda kac cm kalir. Donus: (yon_rad, acik_m) ya da None (her yer kapali)"""
        en_iyi = None
        for d in range(-180, 180, 5):
            a = math.radians(d); acik = self.yol(P, a)
            if acik < 0.8: continue
            sol, sag = self.kenar_paylari(P, a, min(acik, 1.2), bas=0.25)
            if min(sol, sag) < 0.03: continue
            skor = min(acik, 2.5) - 1.2 * abs(a) / math.pi                  # acik yol iyi, buyuk donus kotu
            if en_iyi is None or skor > en_iyi[0]: en_iyi = (skor, a, acik)
        return None if en_iyi is None else (en_iyi[1], en_iyi[2])

    def donus_guvenli(self, P, a, r=0.295, yakin_sinir=0.45, pay=math.radians(20)):
        """yerinde 'a' kadar donerken iki tekerin (+-90 derece, r) SUPURECEGI yayda (15 cm icinde) bir sey var mi?
        (2026-10-07: saga donerken sol teker vantilator tabanina carpti; govde daire ama tekerler yayi supurur)"""
        if len(P) == 0: return True
        d = np.hypot(P[:, 0], P[:, 1]); Q = P[d < yakin_sinir]
        if len(Q) == 0: return True
        aci = np.arctan2(Q[:, 1], Q[:, 0])
        for teker in (math.pi / 2, -math.pi / 2):
            bas, son = (teker, teker + a) if a > 0 else (teker + a, teker)        # tekerin supurdugu yay
            orta = (bas + son) / 2; yari = (son - bas) / 2 + pay
            if (np.abs((aci - orta + math.pi) % (2 * math.pi) - math.pi) <= yari).any(): return False
        return True

    def acik_yere_cik(self, sebep=''):
        """SIKISINCA KENDIN CIK (Nav2'yi bekleme): lidar + kamera ile etrafina bak, en az donusle en acik yonu bul,
        kamera once oraya baksin, TEK SEFERDE yavas don (gerekirse once birkac cm duz acil), dumduz acik alana cik.
        En acik yol arkadaysa DONMEDEN dumduz geri cik. Yalpalama / bir saga bir sola donme YOK."""
        log(f'  CIKIS ARIYORUM ({sebep}): lidar + kamera ile etrafima bakiyorum')
        self.durdur_robot(); self.bekle(0.5)                               # (Nav2 hedefi git() icinde zaten iptal edildi)
        secim = self.cikis_yonu(self.tarama())
        if secim is None:
            log('  her yer kapali - kipirdamadan engelin kalkmasini bekliyorum (10 sn)'); t = time.time()
            while secim is None and time.time() - t < 10 and not self.dur:
                self.bilerek_bekliyorum(); self.bekle(1.0); secim = self.cikis_yonu(self.tarama())
            if secim is None: return False
        a = secim[0]
        if abs(a) <= math.radians(100):                                      # kamera once oraya baksin, sonra yine olc
            self.once_bak(math.degrees(a), sure=8.0, bekle=4.0)
        else:
            self.once_bak('arka', sure=15.0, bekle=4.0)
        secim = self.cikis_yonu(self.tarama())
        if secim is None: log('  kamera bakinca yol kapali cikti - bekliyorum'); self.bakis_serbest(); return False
        a, acik = secim
        log(f'  en iyi cikis: {math.degrees(a):+.0f} derece, {acik:.1f} m acik')
        def yakin(): Q = self.tarama(); return float(np.hypot(Q[:, 0], Q[:, 1]).min()) if len(Q) else 9.0
        def genis(): return yakin() > 0.295 + 0.45
        if abs(a) > math.radians(100):                                       # en acik yol ARKADA: donme, dumduz geri
            git = min(acik - 0.3, 1.5); x0, y0, _ = self.epoz()
            log(f'  en acik yol arkamda - DONMEDEN dumduz {git:.1f} m geri cikiyorum')
            self.surus_cmd(-0.10, 0.0, git / 0.10 + 3, lambda: math.hypot(self.epoz()[0] - x0, self.epoz()[1] - y0) >= git
                           or self.yol(self.tarama(), math.pi) < 0.15)
            self.bakis_serbest(); return True
        if abs(a) > math.radians(8) and not self.donus_guvenli(self.tarama(), a):
            # tekerlerin supurecegi yerde bir sey var (vantilator tabani gibi): ONCE bos tarafa dumduz acil, sonra don
            P = self.tarama(); f, b = self.yol(P, 0.0), self.yol(P, math.pi); yon = 1 if f >= b else -1
            acil = min(0.30, max(f, b) - 0.10)
            if acil > 0.05:
                log(f'  donersem teker carpabilir - once {acil*100:.0f} cm dumduz {"ileri" if yon > 0 else "geri"} aciliyorum')
                if yon < 0: self.once_bak('arka', sure=10.0, bekle=3.0)
                x0, y0, _ = self.epoz()
                self.surus_cmd(0.08 * yon, 0.0, acil / 0.08 + 2, lambda: math.hypot(self.epoz()[0] - x0, self.epoz()[1] - y0) >= acil
                               or self.yol(self.tarama(), 0.0 if yon > 0 else math.pi) < 0.08)
            secim = self.cikis_yonu(self.tarama())
            if secim is None: log('  acildim ama yol kapali - bekliyorum'); self.bakis_serbest(); return False
            a = secim[0]
            if abs(a) > math.radians(100):
                log('  simdi en acik yol arkamda - bir sonraki turda geri cikacagim'); self.bakis_serbest(); return False
            if abs(a) > math.radians(8) and not self.donus_guvenli(self.tarama(), a):
                log('  hala donersem teker carpabilir - DONMUYORUM, bekliyorum'); self.bakis_serbest(); return False
        if abs(a) > math.radians(8):
            if yakin() < 0.295 + 0.015:                                      # donecek yer yok (1.5 cm'den az): once birkac cm duz acil
                P = self.tarama(); f, b = self.yol(P, 0.0), self.yol(P, math.pi); yon = 1 if f > b else -1
                acil = min(0.15, max(f, b) - 0.08)
                if acil > 0.03:
                    log(f'  donecek yer yok - once {acil*100:.0f} cm {"ileri" if yon > 0 else "geri"} aciliyorum')
                    x0, y0, _ = self.epoz()
                    self.surus_cmd(0.06 * yon, 0.0, acil / 0.06 + 2, lambda: math.hypot(self.epoz()[0] - x0, self.epoz()[1] - y0) >= acil
                                   or self.yol(self.tarama(), 0.0 if yon > 0 else math.pi) < 0.06)
                if yakin() < 0.295 + 0.01: log('  hala donecek yer yok - bekliyorum'); self.bakis_serbest(); return False
            log(f'  {"SOLA" if a > 0 else "SAGA"} {abs(math.degrees(a)):.0f} derece donuyorum (tek seferde, yavas)')
            th0 = self.epoz()[2]
            self.surus_cmd(0.0, 0.3 if a > 0 else -0.3, abs(a) / 0.3 * 1.6 + 2,
                           lambda: abs(wrap(self.epoz()[2] - th0)) >= abs(a) - math.radians(4) or yakin() < 0.295 + 0.005)
        P = self.tarama(); acik = self.yol(P, 0.0); git = min(acik - 0.4, 2.0)
        if git > 0.1:
            log(f'  dumduz {git:.1f} m ilerleyip acik alana cikiyorum'); x0, y0, _ = self.epoz()
            self.surus_cmd(0.15, 0.0, git / 0.15 + 3, lambda: math.hypot(self.epoz()[0] - x0, self.epoz()[1] - y0) >= git
                           or self.yol(self.tarama(), 0.0) < 0.40 or (math.hypot(self.epoz()[0] - x0, self.epoz()[1] - y0) > 0.5 and genis()))
        self.bakis_serbest(); log('  acik alana ciktim - devam'); return True

    def kenar_paylari(self, P, yon, uzunluk, r=0.295, bas=-0.1):
        """o yone 'uzunluk' kadar dumduz gidilirse yol boyunca govdenin SOLUNDA ve SAGINDA kalan en dar bosluk (m)"""
        c, s = math.cos(yon), math.sin(yon); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        bant = (px > bas) & (px < uzunluk + r) & (np.abs(py) < 1.5)
        sol = py[bant & (py > 0)]; sag = -py[bant & (py < 0)]
        return (float(sol.min()) - r if len(sol) else 1.2), (float(sag.min()) - r if len(sag) else 1.2)

    def dolanma_yonu(self, P, tercih=0.0):
        """onumdeki engelin yanindan gecebilecegim yon (rad, robot cercevesi) ya da None: en fazla 60 derece sapma,
        o yonde en az 1 m yol acik ve yol boyunca govdenin iki yaninda en az 4 cm pay. Hedefe en yakin olani secilir."""
        for d in sorted(range(-60, 61, 5), key=lambda d: abs(math.radians(d) - tercih)):
            a = math.radians(d)
            if self.yol(P, a) < 1.0: continue
            sol, sag = self.kenar_paylari(P, a, 0.8)
            if min(sol, sag) >= 0.04: return a
        return None

    def girebilir_miyim(self, g):
        """INSAN GIBI: once bak, olc, sigiyorsam gir; sigmiyorsam HIC DENEME. Kamera gecidin sol kenarina, sag kenarina
        ve icine (arkasina) bakar; her bakista gecit yeniden olculur. Yol boyunca (gecit + 1 m otesi) govdenin iki
        yaninda en az 4 cm kalmali. Donus: (True/False, aciklama). Robot bu sirada HIC hareket etmez."""
        a, b, o = np.array(g['a']), np.array(g['b']), np.array(g['orta'])
        yon = math.atan2(o[1], o[0]); uzunluk = math.hypot(*o) + 1.0
        for ad, nokta in (('sol kenar', a if math.atan2(a[1], a[0]) > math.atan2(b[1], b[0]) else b),
                          ('sag kenar', b if math.atan2(a[1], a[0]) > math.atan2(b[1], b[0]) else a), ('ici', o)):
            self.once_bak(math.degrees(math.atan2(nokta[1], nokta[0])), sure=6.0)
            self.bekle(0.4)
            try: d = json.loads(self.nesneler) if self.nesneler else None
            except Exception: d = None
            if not d: return False, f'{ad}: algilama verisi yok'
            yakin = [x for x in d['gecitler'] if math.hypot(x['orta'][0] - o[0], x['orta'][1] - o[1]) < 0.6]
            if not yakin: return False, f'{ad}: bakinca gecit kayboldu (arasi dolu)'
            x = min(yakin, key=lambda x: math.hypot(x['orta'][0] - o[0], x['orta'][1] - o[1]))
            if not x['gecilir']: return False, f"{ad}: {x['bosluk']*100:.0f} cm, {x.get('durum', 'gecilmez')}"
        self.bakis_serbest()
        P = self.tarama()                                                # lidar + kameranin tanidiklari (insan payi, masa tablasi)
        serbest = self.yol(P, yon, w=0.295 + 0.04)
        if serbest < uzunluk: return False, f'yol boyunca {serbest:.2f} m sonra sigmiyorum (gereken {uzunluk:.2f} m)'
        sol, sag = self.kenar_paylari(P, yon, uzunluk)
        if min(sol, sag) < 0.04: return False, f'yanlarim dar: sol {sol*100:.0f} cm, sag {sag*100:.0f} cm'
        return True, f'sol {sol*100:.0f} cm, sag {sag*100:.0f} cm pay var, yol {uzunluk:.1f} m boyunca acik'

    def gecitten_gec(self):
        """robotun onu aciksa DONMEDEN dumduz git; degilse algilamanin buldugu gecilir araliga gec.
        Her iki durumda da ONCE bakip olcer: sigmiyorsa hic denemez."""
        self.once_bak(0, sure=16.0)                                       # once kamera da one baksin
        self.bekle(1.0); P = self.tarama(); acik = self.yol(P, 0.0)
        if acik >= 1.5:
            git = min(acik - 0.5, 3.0)
            sol, sag = self.kenar_paylari(P, 0.0, git + 1.0)
            if min(sol, sag) >= 0.04:
                log(f'  onum {acik:.1f} m acik, sol {sol*100:.0f} cm sag {sag*100:.0f} cm pay - DONMEDEN dumduz {git + 1.0:.1f} m gidiyorum')
                self.bakis_serbest(); return self.duz_gec(np.array([git, 0.0]), None, None)
            log(f'  onum acik ama yanlarim dar (sol {sol*100:.0f} cm, sag {sag*100:.0f} cm) - gecit ariyorum')
        secim = None; t = time.time()
        while secim is None and time.time() - t < 15 and not self.dur:
            self.bekle(0.5)
            try: d = json.loads(self.nesneler) if self.nesneler else None
            except Exception: d = None
            if not d: continue
            P = self.tarama(); aday = []
            for g in d['gecitler']:
                if not g['gecilir'] or g['orta'][0] < 0.3: continue
                yon = math.atan2(g['orta'][1], g['orta'][0]); uz = math.hypot(*g['orta'])
                if abs(yon) > math.radians(35): continue                  # sadece ondeki gecitler
                ar = P[np.abs(np.arctan2(P[:, 1], P[:, 0]) - yon) < math.radians(3)]
                ote = float(np.hypot(ar[:, 0], ar[:, 1]).min()) if len(ar) else 9.0
                if ote < uz + 1.0: continue                              # arkasi bos degil (duvar vb.) -> gercek gecit degil
                aday.append(g)
            for g in sorted(aday, key=lambda g: abs(math.atan2(g['orta'][1], g['orta'][0]))):   # tam ondeki once (en az donus)
                ok, neden = self.girebilir_miyim(g)
                log(f"  {g['bosluk']*100:.0f} cm gecit: " + ('GIREBILIRIM - ' if ok else 'GIREMEM, denemiyorum - ') + neden)
                if ok: secim = g; break
            if aday and secim is None: break                                # baktim, olctum, sigmiyor: zorlamiyorum
        self.bakis_serbest()
        if secim is None: log('  GECIT BULUNAMADI'); return False
        a, b, o = np.array(secim['a']), np.array(secim['b']), np.array(secim['orta'])
        log(f'  GECIT: {secim["siniflar"][0]} ile {secim["siniflar"][1]} arasi {secim["bosluk"]*100:.0f} cm, '
            f'{math.hypot(*o):.2f} m ileride - DUMDUZ geciyorum')
        return self.duz_gec(o, a, b)

    def epoz(self):
        p = (self.eod or self.wod).pose.pose; return p.position.x, p.position.y, yaw_of(p.orientation)

    def geri_cik_yol(self, iz):
        """ileri giderken kaydedilen izi (odom noktalari) TERSINDEN takip ederek geri geri cik:
        girilen yol guvenliyse cikilan yol da odur. Arkasi kapanirsa DURUR.
        Donus: ('cikti' | 'arka kapali' | 'zaman asimi', henuz geri gidilmemis iz)"""
        self.once_bak('arka', sure=46.0)                                # GERI gitmeden once kamera arkaya baksin
        tw = Twist(); t = time.time(); hedefler = list(reversed(iz)); durum = 'zaman asimi'
        while not self.dur and time.time() - t < 40 and hedefler:
            rclpy.spin_once(self, timeout_sec=0.0)
            x, y, th = self.epoz()
            while hedefler and math.hypot(hedefler[0][0] - x, hedefler[0][1] - y) < 0.08: hedefler.pop(0)
            if not hedefler: break
            hx, hy = hedefler[min(2, len(hedefler) - 1)]                 # ~10 cm geriden bak (geri pure pursuit)
            geri_yon = th + math.pi; hata = wrap(math.atan2(hy - y, hx - x) - geri_yon)
            if self.yol(self.tarama(), math.pi) < 0.12:
                log('  arkam kapandi - duruyorum'); durum = 'arka kapali'; break
            tw.linear.x = -0.10; tw.angular.z = max(-0.4, min(0.4, 1.5 * hata))
            self.kurtar_pub.publish(tw); time.sleep(0.05)
        if not hedefler: log('  geldigim izden giris noktasina geri ciktim'); durum = 'cikti'
        self.durdur_robot(); self.bakis_serbest()
        return durum, list(reversed(hedefler))

    def koridordan_kurtul(self, iz):
        """koridorda onune biri/bir sey cikti. INSAN GIBI:
        1) once kamera ARKAYA bakar, lidar + kamera birlikte arkayi olcer; arka bossa girdigi izden DUMDUZ geri cikar
        2) onu de arkasi da kapaliysa (sikisti) HIC KIPIRDAMAZ, BEKLER; hangi taraf once acilirsa oradan cikar
           (arka acilirsa geri cikar, on acilirsa gecmeye devam eder)
        Donus: 'ciktim' (giris noktasina geri cikti) | 'onum acildi' (ileri devam) | 'vazgectim'"""
        bas = time.time(); son_log = 0.0
        while not self.dur and time.time() - bas < 600:
            if time.time() - bas > 1.0 and self.yol(self.tarama(), 0.0) >= 0.60:   # koridorda: sadece duz ileri
                self.bakis_serbest(); log('  onum acildi - gecmeye devam ediyorum'); return 'onum acildi'
            self.once_bak('arka', sure=8.0)                              # 1) once kamera arkaya baksin
            x, y, _ = self.epoz()
            geri = math.hypot(x - iz[0][0], y - iz[0][1]) + 0.05          # giris noktasina kadar geri gidilecek yol
            P = self.tarama(); arka = self.yol(P, math.pi)                # lidar + kamera (insan payi dahil)
            if arka >= geri + 0.15:
                sol, sag = self.kenar_paylari(P, math.pi, geri)
                log(f'  arkam {arka:.2f} m bos (gereken {geri:.2f} m, sol {sol*100:.0f} cm sag {sag*100:.0f} cm) - DUMDUZ GERI GERI cikiyorum')
                durum, iz = self.geri_cik_yol(iz)
                if durum == 'cikti': return 'ciktim'
                if not iz: return 'ciktim'
                continue                                                 # arka kapandi: yeniden degerlendir
            if time.time() - son_log > 5.0:                               # 2) SIKISTIM: bekle
                on = self.yol(self.tarama(), 0.0)
                log(f'  SIKISTIM: onum {on:.2f} m, arkam {arka:.2f} m (gereken {geri:.2f}) - kipirdamadan engelin kalkmasini BEKLIYORUM')
                son_log = time.time()
            self.bilerek_bekliyorum(); self.bekle(0.5)
        self.bakis_serbest(); return 'vazgectim'

    def duz_gec(self, o, a, b):
        """Nav2'siz: gecidin ortasina don (tek seferde), dumduz ilerle, iki yandaki engellere gore ortada kal,
        gecidi 1 m gecince dur. Onune bir sey cikarsa DURUR (donup kacmaz)."""
        x0, y0, th0 = self.epoz(); c, s = math.cos(th0), math.sin(th0)
        ox, oy = x0 + c * o[0] - s * o[1], y0 + s * o[0] + c * o[1]          # gecit ortasi (odom)
        mesafe = math.hypot(*o) + 1.0
        # 1) gecidin ortasina don (en fazla 35 derece, normal hizda, tek hareket)
        hedef = math.atan2(o[1], o[0])
        if abs(hedef) > math.radians(3):
            self.once_bak(math.degrees(hedef), sure=8.0)                 # donmeden once kamera o yone baksin
            if self.yol(self.tarama(), hedef) < 0.35:
                log('  kamera donecegim yonde engel gordu - DURUYORUM'); self.bakis_serbest(); return False
            if not self.donus_guvenli(self.tarama(), hedef):
                log('  donersem teker yandaki bir seye carpabilir - gecmiyorum'); self.bakis_serbest(); return False
            th_bas = self.epoz()[2]; tw = Twist(); tw.angular.z = 0.5 if hedef > 0 else -0.5; t = time.time()
            while abs(wrap(self.epoz()[2] - th_bas)) < abs(hedef) - math.radians(2) and time.time() - t < 6 and not self.dur:
                rclpy.spin_once(self, timeout_sec=0.0); self.kurtar_pub.publish(tw); time.sleep(0.03)
            self.durdur_robot(); self.bakis_serbest()
        # 2) dumduz ilerle
        xb, yb, _ = self.epoz(); tw = Twist(); t = time.time(); iz = [(xb, yb)]
        while not self.dur and time.time() - t < 40:
            rclpy.spin_once(self, timeout_sec=0.0)
            x, y, th = self.epoz(); gitti = math.hypot(x - xb, y - yb)
            if math.hypot(x - iz[-1][0], y - iz[-1][1]) > 0.05: iz.append((x, y))
            if gitti >= mesafe: log(f'  GECTIM ({gitti:.2f} m)'); self.durdur_robot(); return True
            P = self.tarama()
            yan = P[(np.abs(P[:, 0]) < 0.35) & (np.abs(P[:, 1]) < 0.9)]
            koridorda = (yan[:, 1] > 0).any() and (yan[:, 1] < 0).any()       # iki yanimda da engel var (masa arasi)
            on = self.yol(P, 0.0)
            if on < 0.9 and not koridorda:                                   # ACIK YERDE onumde engel: DURMA, yanindan dolan
                hedef_rel = wrap(math.atan2(oy - y, ox - x) - th) if math.hypot(ox - x, oy - y) > 0.4 else 0.0
                alt = self.dolanma_yonu(P, hedef_rel)
                if alt is not None and not getattr(self, 'dolaniyor', False):  # once kamera o yana baksin, sonra yine olc
                    self.durdur_robot(); self.once_bak(math.degrees(alt), sure=4.0, bekle=1.5)
                    P = self.tarama(); alt = self.dolanma_yonu(P, hedef_rel)
                    if alt is not None:
                        log(f'  onumde engel ({on:.2f} m) - {"SOLUNDAN" if alt > 0 else "SAGINDAN"} dolanip devam ediyorum ({math.degrees(alt):+.0f} derece)')
                    self.dolaniyor = alt is not None
                if alt is not None:
                    tw.linear.x = 0.15; tw.angular.z = max(-0.5, min(0.5, 1.5 * alt))   # yururken kavis: yerinde donme YOK
                    self.kurtar_pub.publish(tw); time.sleep(0.05); continue
            if on >= 0.9: self.dolaniyor = False
            if on < 0.35:
                self.durdur_robot()
                if gitti < 0.10: log('  onum kapali - girmiyorum'); return False
                log(f'  ONUME BIRI/BIR SEY CIKTI ({gitti:.2f} m) - once arkama bakiyorum')
                sonuc = self.koridordan_kurtul(iz)
                if sonuc == 'onum acildi':                                 # ayni gecisi surdur; biraz geri gelmis olabilir:
                    x, y, _ = self.epoz()                                  # izi bulundugu noktaya kadar kisalt
                    iz = iz[:int(np.argmin([math.hypot(px - x, py - y) for px, py in iz])) + 1]
                    t = time.time(); continue
                if sonuc != 'ciktim': return False
                if getattr(self, 'tekrar_hakki', 1) > 0 and not self.dur:
                    self.tekrar_hakki = 0; log('  3 sn bekliyorum'); self.bekle(3.0)
                    if self.yol(self.tarama(), 0.0) >= 1.0:
                        log('  yol acildi - tekrar dumduz geciyorum'); return self.duz_gec(o, a, b)
                    log('  yol hala kapali - gecmekten vazgectim')
                return False
            # yon: gecit ortasina dogru; gecitin icindeyken iki yana gore ortala
            hata = wrap(math.atan2(oy - y, ox - x) - th) if math.hypot(ox - x, oy - y) > 0.4 else 0.0
            sol = yan[yan[:, 1] > 0][:, 1].min() if (yan[:, 1] > 0).any() else None
            sag = -yan[yan[:, 1] < 0][:, 1].max() if (yan[:, 1] < 0).any() else None
            if sol is not None and sag is not None: hata = 0.6 * hata + 1.2 * (sol - sag) / 2
            tw.linear.x = 0.18; tw.angular.z = max(-0.4, min(0.4, 1.2 * hata))   # gecis hizi (yavas, kontrollu)
            self.kurtar_pub.publish(tw); time.sleep(0.05)
        self.durdur_robot(); return False
    def kesif(self):
        log('KESIF: frontier_exploration_ros2 ile bilinmeyen kenarlara')
        self.kesif_bitti = False
        pr = subprocess.Popen(['ros2', 'launch', 'frontier_exploration_ros2', 'frontier_explorer.launch.py',
                               'use_sim_time:=false'], start_new_session=True,
                              stdout=open(os.path.expanduser('~/cryvex_araclar/frontier.log'), 'w'), stderr=subprocess.STDOUT)
        try:
            bilinen0 = -1; t_artis = time.time()
            while not self.dur and not self.kesif_bitti:
                self.bekle(1.0); self.tasinma_kontrol(); self.ziyaret_isle()
                if self.sikisma_kontrol(): self.dar_yer_kurtar(pr)
                if time.time() - self.son_kayit > 120: self.harita_kaydet()
                if self.map is not None:
                    bilinen = int((np.asarray(self.map.data) >= 0).sum())
                    if bilinen > bilinen0 * 1.01: bilinen0, t_artis = bilinen, time.time()
                    elif time.time() - t_artis > 150:
                        log('  2.5 dk boyunca harita buyumedi - kesif tamam sayiyorum'); break
                if pr.poll() is not None: log('  kesif paketi kapandi'); break
            if self.kesif_bitti: log('  KESIF TAMAM (bilinmeyen kenar kalmadi)')
        finally:
            try: os.killpg(os.getpgid(pr.pid), signal.SIGINT); pr.wait(timeout=10)
            except Exception:
                try: os.killpg(os.getpgid(pr.pid), signal.SIGKILL)
                except Exception: pass
            self.nav_iptal()

    # ---- DAR YER KURTARMA (iki masa arasi): Nav2 kipirdayamazsa lidarla en acik yone kendisi doner ----
    def tarama(self):
        """son lidar taramasi + kameranin tanidigi cisimlerin hitbox'lari, robot (base_footprint) cercevesinde noktalar"""
        m = self.scan
        if m is None: return self.kamera_noktalari()
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.15) & (r < 8.0)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
        L = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)
        L = L[np.hypot(L[:, 0], L[:, 1]) > 0.30]                          # govdenin ICI (kendi guc kablosu/fisi, -100..-120 derece): engel degil
        return np.vstack([L, self.kamera_noktalari()])

    def yol(self, P, ang, w=0.32):   # govde yaricapi 0.295 (59 cm) + 2.5 cm
        """o yone duz gidince (yuvarlak govde) degmeden once gidilebilecek yol"""
        c, s = math.cos(ang), math.sin(ang); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        sel = (px > 0) & (np.abs(py) < w)
        return float(max(0.0, (px[sel] - np.sqrt(w * w - py[sel] ** 2)).min())) if sel.any() else 9.0

    def teker_yaw(self):
        return yaw_of(self.wod.pose.pose.orientation) if self.wod else 0.0

    def surus_cmd(self, lx, az, sure, dur_kosul):
        tw = Twist(); tw.linear.x = float(lx); tw.angular.z = float(az); t = time.time()
        while time.time() - t < sure and not self.dur:
            rclpy.spin_once(self, timeout_sec=0.03)
            if dur_kosul(): break
            self.kurtar_pub.publish(tw); time.sleep(0.03)
        self.durdur_robot()

    def sikisma_kontrol(self):
        p = self.poz(); now = time.time()
        if p is None: return False
        self.hareket.append((now, p))
        while self.hareket and now - self.hareket[0][0] > 12.0: self.hareket.pop(0)
        if now - self.hareket[0][0] < 11.0 or now - getattr(self, 'son_kurtarma', 0) < 20: return False
        (_, a), (_, b) = self.hareket[0], self.hareket[-1]
        hareketsiz = math.hypot(b[0] - a[0], b[1] - a[1]) < 0.15 and abs(wrap(b[2] - a[2])) < math.radians(20)
        P = self.tarama()
        dar = len(P) > 0 and float(np.hypot(P[:, 0], P[:, 1]).min()) < 0.45
        return hareketsiz and dar

    def dar_yer_kurtar(self, explorer=None):
        log('  DAR YER: Nav2 12 sn kipirdayamadi - kendim cikiyorum')
        if explorer is not None:
            try: os.killpg(os.getpgid(explorer.pid), signal.SIGSTOP)   # kesif paketini duraklat
            except Exception: pass
        self.nav_iptal(); self.bekle(0.5)
        try:
            # 0) iki paralel engel arasindaysa (masa arasi): YERINDE DONME YOK, duz ileri/geri cik
            arg = ['--kaydet'] if getattr(self, 'masa_kaydet', False) else []; self.masa_kaydet = False
            r = subprocess.run(['python3', '-u', os.path.expanduser('~/cryvex_araclar/koridor_cik.py')] + arg,
                               capture_output=True, text=True, timeout=180)
            for satir in r.stdout.splitlines()[-8:]: log('  [koridor] ' + satir.split(' ', 1)[-1])
            if 'iki paralel engel' in r.stdout:
                if 'koridordan ciktim' not in r.stdout:
                    log('  masa arasindan cikamadim - DURUYORUM, yardim gerekli'); self.dur = True
                return
            P = self.tarama()
            if len(P) and float(np.hypot(P[:, 0], P[:, 1]).min()) < 0.325:   # donecek yer yok: sadece duz git
                f, b = self.yol(P, 0.0), self.yol(P, math.pi)
                log(f'  donecek yer yok - {"ileri" if f >= b else "geri"} duz aciliyorum')
                self.surus_cmd(0.06 if f >= b else -0.06, 0.0, 6.0,
                               lambda: self.yol(self.tarama(), 0.0 if f >= b else math.pi) < 0.15)
                P = self.tarama()
                if len(P) and float(np.hypot(P[:, 0], P[:, 1]).min()) < 0.325:
                    log('  hala donecek yer yok - Nav2 devralsin'); return
            # 1) en acik yon (360 derece)
            acilar = [math.radians(a) for a in range(-180, 180, 10)]
            hedef = max(acilar, key=lambda a: min(self.yol(P, a), 3.0) - 0.2 * abs(a))
            log(f'  en acik yon {math.degrees(hedef):+.0f} deg ({self.yol(P, hedef):.2f} m)')
            # 2) on/arka bosluklari esitle (ortalan)
            f, b = self.yol(P, 0.0), self.yol(P, math.pi); d = (f - b) / 2
            if abs(d) > 0.02:
                y0 = self.wod.pose.pose.position if self.wod else None
                x0 = (y0.x, y0.y) if y0 else (0, 0)
                def gitti():
                    q = self.wod.pose.pose.position; return math.hypot(q.x - x0[0], q.y - x0[1])
                self.surus_cmd(0.04 if d > 0 else -0.04, 0.0, 6.0,
                               lambda: gitti() >= min(abs(d), 0.15) or self.yol(self.tarama(), 0.0 if d > 0 else math.pi) < 0.08)
            # 3a) en acik yol ARKADA ise donme: duz geri geri cik (geri vites), sonra yan acikliga don
            if abs(hedef) > math.radians(120):
                log('  en acik yol arkamda - geri geri cikiyorum')
                self.surus_cmd(-0.07, 0.0, 12.0, lambda: self.yol(self.tarama(), math.pi) < 0.20)
                P = self.tarama()
                yanlar = [math.radians(a) for a in range(-120, 121, 10) if abs(a) >= 30]
                hedef = max(yanlar, key=lambda a: min(self.yol(P, a), 3.0) - 0.2 * abs(a))
                log(f'  simdi en acik yon {math.degrees(hedef):+.0f} deg ({self.yol(P, hedef):.2f} m)')
            # 3) yerinde YAVAS don, saga ya da sola (etrafta 30 cm'den yakina bir sey gelirse dur)
            if abs(hedef) > math.radians(5):
                y0 = self.teker_yaw(); hiz = 0.15 if hedef > 0 else -0.15
                def dondu():
                    return abs(wrap(self.teker_yaw() - y0)) >= abs(hedef) - math.radians(3)
                def cok_yakin():
                    Q = self.tarama(); return len(Q) > 0 and float(np.hypot(Q[:, 0], Q[:, 1]).min()) < 0.33
                self.surus_cmd(0.0, hiz, abs(hedef) / 0.15 * 2.5 + 2, lambda: dondu() or cok_yakin())
            # 4) acilan yone biraz ilerle
            # 4) acilan yone, etrafinda en az 50 cm bosluk olan yere kadar ilerle (en fazla 1.5 m)
            def genis_yer():
                Q = self.tarama(); return len(Q) > 0 and float(np.hypot(Q[:, 0], Q[:, 1]).min()) > 0.50
            self.surus_cmd(0.08, 0.0, 19.0, lambda: self.yol(self.tarama(), 0.0) < 0.30 or genis_yer())
            log('  dar yerden ciktim - Nav2 devam')
        finally:
            self.son_kurtarma = time.time(); self.hareket.clear()
            if explorer is not None:
                try: os.killpg(os.getpgid(explorer.pid), signal.SIGCONT)
                except Exception: pass

    def nav_iptal(self):
        """Nav2'deki TUM hedefleri iptal et (kesif paketinin verdigi dahil): bos goal_id + sifir zaman = hepsi"""
        subprocess.run(['ros2', 'service', 'call', '/navigate_to_pose/_action/cancel_goal',
                        'action_msgs/srv/CancelGoal', '{}'], capture_output=True, timeout=15)
        self.durdur_robot()

def main():
    rclpy.init(); n = Gezgin()
    signal.signal(signal.SIGTERM, lambda *a: setattr(n, 'dur', True))
    mod = MOD; yeni = '--yeni' in sys.argv
    if '--gecit-test' in sys.argv:                   # geciti bul, ortasindan gec, sonra sureli devriye
        sure = float(sys.argv[sys.argv.index('--gecit-test') + 1])
        try:
            n.sadece_geri = True
            n.baslat(False)
            if not n.dur: n.gecitten_gec()
            if not n.dur: n.devriye(sure)
        except YeniYer as e:
            log(f'tasindim ({e}) - test durdu')
        finally:
            n.nav_iptal(); n.harita_kaydet(bekle=True); log('GECIT TESTI BITTI')
            n.destroy_node(); rclpy.shutdown()
        return
    if '--aralik-avi' in sys.argv:                   # gez, masa aralarini kendin bul, olc, sigiyorsa gec
        sure = float(sys.argv[sys.argv.index('--aralik-avi') + 1])
        try:
            n.sadece_geri = True
            n.baslat(False)
            if not n.dur: n.aralik_avi(sure)
        except YeniYer as e:
            log(f'tasindim ({e}) - test durdu')
        finally:
            n.nav_iptal(); n.harita_kaydet(bekle=True); log('ARALIK AVI TESTI BITTI')
            n.destroy_node(); rclpy.shutdown()
        return
    if '--masa-test' in sys.argv:                  # cik (girisi kaydet) -> sureli devriye -> masaya don, gir
        sure = float(sys.argv[sys.argv.index('--masa-test') + 1])
        try:
            n.masa_kaydet = True; n.baslat(False)
            if n.masa_kaydet:                            # baslangicta dar degildi: yine de cikis+kayit dene
                n.dar_yer_kurtar()
            if not n.dur: n.devriye(sure)
            if not n.dur: n.masaya_git_ve_gir()
        except YeniYer as e:
            log(f'tasindim ({e}) - test durdu')
        finally:
            n.nav_iptal(); n.harita_kaydet(bekle=True); log('MASA TESTI BITTI')
            n.destroy_node(); rclpy.shutdown()
        return
    try:
        n.baslat(yeni)
        while not n.dur:
            try:
                if mod == 'kesif':
                    n.kesif()
                    if not n.dur: mod = 'devriye'
                else:
                    n.devriye()
            except YeniYer as e:
                n.durdur_robot(); log(f'BASKA BIR YERE TASINDIM ({e}) - haritayi kaydedip burada sifirdan basliyorum')
                n.harita_kaydet(bekle=True); n.bekle(10.0); n.baslat(True); mod = 'kesif'
    except KeyboardInterrupt:
        pass
    finally:
        n.nav_iptal(); n.harita_kaydet(bekle=True)
        log(f'DURDU - harita: {KAYIT_DIR}/{getattr(n, "oda", "?")}.pgm')
        n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
