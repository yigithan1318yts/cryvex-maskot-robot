"""KORIDOR CIKISI - iki engel (masa/palet) arasindan cikma, 2026-10-05.
Yontem: lidar noktalarina RANSAC ile iki dogru uydurulur (iki engelin kenari). Paralellerse koridordur,
koridor ekseni = dogrularin yonu. Robot eksene EN AZ donusle hizalanir (en fazla 90 derece: cikis arkada
kalirsa geri vitesle gider), enine/capraz kaldiysa once ortalanir, donerken degecekse birkac cm ileri/geri
yer acip (sag-sol manevrasi) donmeye devam eder. Hizalaninca SADECE duz ileri ya da geri gider,
yanlardaki engeller bitince (koridordan cikinca) durur.
Hareket komutlari Collision Monitor acikken onun uzerinden (cmd_vel_smoothed), degilse cmd_vel'e gider.
Kullanim: python3 -u koridor_cik.py [--olc]   (--olc: hareket etmeden sadece analiz)"""
import sys, os, json, time, math, subprocess, urllib.request, numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

R_GOVDE = 0.30          # govde yaricapi: sag tekerden sol tekere 59 cm (2026-10-05 olculdu) + 0.5 cm
DONME_PAYI = 0.025      # yerinde donerken en yakin nokta en az R_GOVDE + bu kadar uzakta olmali
ALT_PAY = 0.08          # lidarin gormedigi alt cikintilar (palet takozu, masa ayagi tabani) icin yan pay
KOR_HIZ = 0.08          # koridor icinde ileri/geri hiz (ortadan kayinca 0.03'e duser)
LY, LX, LYY = math.radians(-90), 0.125, -0.045
def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Cikis(Node):
    def __init__(self):
        super().__init__('koridor_cik')
        self.scan = None; self.od = None
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'od', m), 10)
        cm = subprocess.run(['ros2', 'lifecycle', 'get', '/collision_monitor'], capture_output=True, text=True, timeout=15)
        self.cm = cm.stdout.strip().startswith('active')
        self.pub = self.create_publisher(Twist, 'cmd_vel_smoothed' if self.cm else 'cmd_vel', 10)
        self.dur_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self, spin_thread=False)
        log('hareket: ' + ('Collision Monitor uzerinden' if self.cm else 'dogrudan (Collision Monitor kapali)'))

    def bekle(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.02)

    def noktalar(self):
        n0 = id(self.scan); t = time.time()
        while (self.scan is None or id(self.scan) == n0) and time.time() - t < 1.0: rclpy.spin_once(self, timeout_sec=0.02)
        m = self.scan; a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.12) & (r < 6.0)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
        return np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)

    def yaw(self):
        q = self.od.pose.pose.orientation; return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
    def xy(self):
        p = self.od.pose.pose.position; return p.x, p.y

    def dur(self):
        for _ in range(4): self.pub.publish(Twist()); self.dur_pub.publish(Twist()); time.sleep(0.03)

    # ---- geometri ----
    @staticmethod
    def yol(P, ang, w=R_GOVDE + 0.03):
        """o yone duz giderse degmeden gidebilecegi mesafe"""
        c, s = math.cos(ang), math.sin(ang); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        sel = (px > 0) & (np.abs(py) < w)
        return float(max(0.0, (px[sel] - np.sqrt(w * w - py[sel] ** 2)).min())) if sel.any() else 9.0

    @staticmethod
    def en_yakin(P):
        return float(np.hypot(P[:, 0], P[:, 1]).min()) if len(P) else 9.0

    @staticmethod
    def ransac(P, deneme=200, esik=0.03):
        """noktalara dogru uydur: (yon_acisi, inlier maskesi)"""
        if len(P) < 8: return None, None
        en_iyi = None; rng = np.random.default_rng()
        for _ in range(deneme):
            i, j = rng.choice(len(P), 2, replace=False); d = P[j] - P[i]; L = np.hypot(*d)
            if L < 0.15: continue
            nrm = np.array([-d[1], d[0]]) / L; mask = np.abs((P - P[i]) @ nrm) < esik
            if en_iyi is None or mask.sum() > en_iyi[1].sum(): en_iyi = (math.atan2(d[1], d[0]), mask)
        if en_iyi is None or en_iyi[1].sum() < 8: return None, None
        Q = P[en_iyi[1]] - P[en_iyi[1]].mean(0)               # inlier'lara PCA ile ince ayar
        v = np.linalg.svd(Q, full_matrices=False)[2][0]
        return math.atan2(v[1], v[0]), en_iyi[1]

    def koridor(self, P):
        """yakin (1.2 m) engellerden iki dogru; (eksen_acisi, aciklama) ya da (None, sebep)"""
        Y = P[np.hypot(P[:, 0], P[:, 1]) < 1.2]
        a1, m1 = self.ransac(Y)
        if a1 is None: return None, 'yakinda engel cizgisi yok'
        a2, m2 = self.ransac(Y[~m1])
        if a2 is None: return a1, 'tek engel cizgisi (duvar boyunca)'
        fark = abs(wrap(2 * (a1 - a2))) / 2                  # dogrular yonsuz: 180 derece farki ayni
        if fark > math.radians(25): return a1, f'iki cizgi paralel degil ({math.degrees(fark):.0f} derece) - en uzunu alindi'
        ort = math.atan2(math.sin(2 * a1) + math.sin(2 * a2), math.cos(2 * a1) + math.cos(2 * a2)) / 2
        return ort, 'iki paralel engel: KORIDOR'

    def yanlar(self, P, menzil=1.2):
        """koridor ekseni + sol/sag engel yuzeyine uzaklik (robot merkezinden): (eksen, sol, sag) ya da None"""
        Y = P[np.hypot(P[:, 0], P[:, 1]) < menzil]
        if menzil > 1.2: Y = Y[Y[:, 0] > -0.2]                  # uzaktan ararken sadece onumdeki noktalar
        a1, m1 = self.ransac(Y)
        if a1 is None: return None
        a2, m2 = self.ransac(Y[~m1])
        if a2 is None or abs(wrap(2 * (a1 - a2))) / 2 > math.radians(25): return None
        ax = math.atan2(math.sin(2 * a1) + math.sin(2 * a2), math.cos(2 * a1) + math.cos(2 * a2)) / 2
        ax = wrap(2 * ax) / 2                                   # -90..90: robotun ilerisine en yakin eksen yonu
        nrm = np.array([-math.sin(ax), math.cos(ax)])           # eksene dik, SOL tarafi pozitif
        o1 = float(np.median(Y[m1] @ nrm)); o2 = float(np.median(Y[~m1][m2] @ nrm))
        if o1 * o2 >= 0: return None                            # iki cizgi ayni tarafta: koridor degil
        return ax, max(o1, o2), -min(o1, o2)

    def masa_kaydet(self, ileri_cikti):
        """cikis noktasi = masanin girisi; yon: koridora BAKAN yon (ileri ciktiysa ters)"""
        try:
            t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            q = t.rotation; yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
            if ileri_cikti: yaw = wrap(yaw + math.pi)
            d = {'x': t.translation.x, 'y': t.translation.y, 'yaw': yaw, 'zaman': time.strftime('%H:%M:%S')}
            json.dump(d, open(os.path.expanduser('~/cryvex_araclar/masa_girisi.json'), 'w'))
            log(f'MASA GIRISI KAYDEDILDI: ({d["x"]:+.2f}, {d["y"]:+.2f}) yon {math.degrees(yaw):+.0f} deg')
        except Exception as e:
            log(f'masa girisi kaydedilemedi: {e}')

    def koridordan_cik(self, tercih_ileri):
        """GERCEKTEN cikana (yanlar bosalana) kadar: once tercih edilen yon, kapanirsa bekle/tekrar, sonra ters yon"""
        for ileri in (tercih_ileri, tercih_ileri, not tercih_ileri):
            self.koridorda_git(ileri, 3.0, True)
            if self.son_durum == 'cikti':
                if '--kaydet' in sys.argv: self.masa_kaydet(ileri)
                return True
            log(f'  cikamadim ({self.son_durum}) - 2 sn bekleyip tekrar deniyorum'); self.bekle(2.0)
        return False

    def koridorda_git(self, ileri, mesafe, dur_bosalinca):
        """eksen boyunca DUZ git: ortada kal, egriltme yok; yan bosluk ALT_PAY'in altina duserse dur"""
        v = KOR_HIZ if ileri else -KOR_HIZ; x0, y0 = self.xy(); t = time.time(); bos_say = 0; self.son_durum = 'sure'; self.son_kayit_m = -1
        while time.time() - t < 60:
            gitti = math.hypot(self.xy()[0] - x0, self.xy()[1] - y0)
            if gitti >= mesafe: log(f'  {gitti:.2f} m gittim'); self.son_durum = 'mesafe'; break
            P = self.noktalar()
            if self.yol(P, 0.0 if ileri else math.pi) < 0.15: log(f'  {"onum" if ileri else "arkam"} kapandi ({gitti:.2f} m) - duruyorum'); self.son_durum = 'kapandi'; break
            yan = P[(np.abs(P[:, 0]) < 0.40) & (np.abs(P[:, 1]) < 0.70)]
            bos_say = bos_say + 1 if len(yan) == 0 else 0
            if dur_bosalinca and bos_say >= 3: log(f'  yanlarim bosaldi - koridordan ciktim ({gitti:.2f} m)'); self.son_durum = 'cikti'; break
            k = self.yanlar(P); tw = Twist(); tw.linear.x = v
            if gitti - getattr(self, 'son_kayit_m', -1) >= 0.10 or gitti < getattr(self, 'son_kayit_m', 0):
                self.son_kayit_m = gitti
                log(f'    [{gitti:.2f} m] ' + (f'sol {k[1]*100:.0f} cm sag {k[2]*100:.0f} cm eksen {math.degrees(k[0]):+.0f} | ' if k else 'koridor cizgisi yok | ')
                    + f'{"on" if ileri else "arka"} yol {self.yol(P, 0.0 if ileri else math.pi)*100:.0f} cm')
            if k is not None:
                ax, sol, sag = k
                if min(sol, sag) < R_GOVDE + ALT_PAY:
                    log(f'  yana cok yakin (sol {sol*100:.0f}, sag {sag*100:.0f} cm) - durup ortalaniyorum')
                    self.dur(); self.bekle(0.3)
                    if sol + sag < 2 * (R_GOVDE + ALT_PAY): log('  koridor robot icin cok dar (alt pay ile) - duruyorum'); self.son_durum = 'dar'; break
                kay = (sol - sag) / 2                           # + : ortanin saginda, sola gitmeli
                if abs(kay) > 0.02: tw.linear.x = 0.03 if ileri else -0.03   # ortadan 2 cm+ kaydi: yavas, once ortala
                yon_hedef = max(-0.25, min(0.25, 2.5 * kay)) * (1 if ileri else -1)
                tw.angular.z = max(-0.10, min(0.10, 0.8 * (ax + yon_hedef)))
            self.pub.publish(tw)
        self.dur(); return math.hypot(self.xy()[0] - x0, self.xy()[1] - y0)

    # ---- hareketler ----
    def kay(self, v, mesafe, sure=8.0):
        """duz ileri (v>0) / geri (v<0), degmeden"""
        x0, y0 = self.xy(); tw = Twist(); tw.linear.x = float(v); t = time.time()
        while time.time() - t < sure:
            P = self.noktalar()
            if self.yol(P, 0.0 if v > 0 else math.pi) < 0.04: break
            if math.hypot(self.xy()[0] - x0, self.xy()[1] - y0) >= mesafe: break
            self.pub.publish(tw)
        self.dur(); return math.hypot(self.xy()[0] - x0, self.xy()[1] - y0)

    def ortalan(self):
        """onundeki ve arkasindaki bosluklari esitle (enine kaldiysa donebilmek icin)"""
        P = self.noktalar(); f, b = self.yol(P, 0.0), self.yol(P, math.pi); d = (f - b) / 2
        if abs(d) > 0.015 and min(f, b) < 0.25:
            log(f'  ortalaniyorum: on {f*100:.0f} cm, arka {b*100:.0f} cm -> {d*100:+.0f} cm')
            self.kay(0.04 if d > 0 else -0.04, abs(d))

    def don(self, hedef):
        """yerinde hedef kadar don; degecek gibiyse ileri/geri yer acip devam (sag-sol manevrasi)"""
        y0 = self.yaw(); kalan = hedef; deneme = 0
        while abs(kalan) > math.radians(3) and deneme < 12:
            P = self.noktalar()
            if self.en_yakin(P) < R_GOVDE + DONME_PAYI:
                f, b = self.yol(P, 0.0), self.yol(P, math.pi)
                log(f'  donecek yer dar ({self.en_yakin(P)*100:.0f} cm) - {"ileri" if f > b else "geri"} yer aciyorum')
                if max(f, b) < 0.03: log('  ne ileri ne geri yer var - duruyorum'); return False
                self.kay(0.04 if f > b else -0.04, min(0.05, max(f, b) / 2)); deneme += 1; continue
            tw = Twist(); tw.angular.z = 0.25 if kalan > 0 else -0.25; t = time.time()
            adim_bas = self.yaw()
            while time.time() - t < 1.5:                           # 10 derecelik adim (0.15 rad/s * ~1.2 s)
                if abs(wrap(self.yaw() - adim_bas)) >= min(abs(kalan), math.radians(10)): break
                if self.en_yakin(self.noktalar()) < R_GOVDE + 0.01: break
                self.pub.publish(tw)
            self.dur(); kalan = hedef - wrap(self.yaw() - y0); deneme += 1 if abs(wrap(self.yaw() - adim_bas)) < math.radians(2) else 0
        return abs(kalan) <= math.radians(5)

    def cik(self, eksen, geri):
        """eksen boyunca duz git, yanlar bosalinca dur; koridora paralel kalmak icin hafif duzelt"""
        v = -0.08 if geri else 0.08; x0, y0 = self.xy(); t = time.time(); bos_say = 0
        while time.time() - t < 30:
            P = self.noktalar()
            if self.yol(P, math.pi if geri else 0.0) < 0.12: log('  onum kapandi - duruyorum'); break
            yan = P[(np.abs(P[:, 0]) < 0.35) & (np.abs(P[:, 1]) < 0.65)]
            bos_say = bos_say + 1 if len(yan) == 0 else 0
            if bos_say >= 3: log(f'  yanlarim bosaldi - koridordan ciktim ({math.hypot(self.xy()[0]-x0, self.xy()[1]-y0):.2f} m)'); break
            ax, _ = self.koridor(P); tw = Twist(); tw.linear.x = v
            if ax is not None:                                     # eksene paralel kal (ileri/geri fark etmez)
                hata = wrap(2 * ax) / 2; tw.angular.z = max(-0.15, min(0.15, 0.8 * hata))
            self.pub.publish(tw)
        self.dur(); return math.hypot(self.xy()[0] - x0, self.xy()[1] - y0)

def main():
    rclpy.init(); n = Cikis()
    t = time.time()
    while (n.scan is None or n.od is None) and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
    if n.scan is None or n.od is None: log('lidar/odometri yok'); return
    try:
        if '--gir' in sys.argv:                               # test: onundeki koridora duz ve ortadan gir
            m = float(sys.argv[sys.argv.index('--gir') + 1])
            mf = os.path.expanduser('~/cryvex_araclar/masa_girisi.json')
            if '--masa' in sys.argv and os.path.exists(mf):   # kayitli giris yonune don (Nav2 tam varamamis olabilir)
                g = json.load(open(mf)); t0 = time.time(); poz = None
                while poz is None and time.time() - t0 < 5:
                    try:
                        tr = n.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform; poz = tr
                    except Exception: n.bekle(0.2)
                if poz is not None:
                    q = poz.rotation; yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
                    dx, dy = g['x'] - poz.translation.x, g['y'] - poz.translation.y
                    log(f'kayitli girise {math.hypot(dx, dy)*100:.0f} cm, giris yonune {math.degrees(wrap(g["yaw"] - yaw)):+.0f} deg')
                    if abs(wrap(g['yaw'] - yaw)) > math.radians(8): n.don(wrap(g['yaw'] - yaw))
            P = n.noktalar(); k = n.yanlar(P)
            if k is None:
                k = n.yanlar(P, menzil=2.0)
                if k is not None:
                    ax0, sol0, sag0 = k; kay0 = (sol0 - sag0) / 2
                    log(f'koridor ileride (2 m icinde): eksen {math.degrees(ax0):+.0f} deg - yaklasiyorum')
                    if abs(ax0) > math.radians(4): n.don(ax0)
                    acik = n.yol(n.noktalar(), 0.0); n.kay(0.06, max(0.0, min(0.8, acik - 0.5)))
                    P = n.noktalar(); k = n.yanlar(P)
            if k is None: log('onumde koridor goremedim'); return
            ax, sol, sag = k; log(f'koridor: eksen {math.degrees(ax):+.0f} deg, sol {sol*100:.0f} cm, sag {sag*100:.0f} cm - {m:.2f} m giriyorum')
            for deneme in range(3):                           # YERLES: hafif don -> ortaya kay -> eksene don
                ax, sol, sag = k; kay = (sol - sag) / 2       # - : ortanin solunda, saga kaymali
                if abs(kay) <= 0.03: break
                th = math.radians(35) * (1 if kay > 0 else -1)
                d = min(0.45, abs(kay) / math.sin(abs(th)))
                log(f'  yerlesme {deneme+1}: ortadan {kay*100:+.0f} cm kaymisim - {"sola" if th > 0 else "saga"} 35 deg donup {d*100:.0f} cm gidiyorum')
                if not n.don(ax + th): log('  donemedim - duruyorum'); return
                acik = n.yol(n.noktalar(), 0.0)
                if acik < 0.05: log('  onum kapali - duruyorum'); return
                n.kay(0.08, min(d, acik - 0.03))
                P = n.noktalar(); k = n.yanlar(P)
                if k is None: log('  koridoru kaybettim - duruyorum'); return
                n.don(k[0]); P = n.noktalar(); k = n.yanlar(P)
                if k is None: log('  koridoru kaybettim - duruyorum'); return
            ax, sol, sag = k
            log(f'yerlestim: sol {sol*100:.0f} cm, sag {sag*100:.0f} cm, on {n.yol(P,0)*100:.0f} cm acik')
            if min(sol, sag) < R_GOVDE + 0.03 or n.yol(P, 0.0) < 0.30:
                log('ortalanamadim ya da onum kapali - girmiyorum'); return
            if sol + sag < 2 * (R_GOVDE + ALT_PAY):
                log(f'koridor robot icin dar ({(sol+sag)*100:.0f} cm < {2*(R_GOVDE+ALT_PAY)*100:.0f} cm) - girmiyorum'); return
            if abs(ax) > math.radians(4): n.don(ax)
            git = n.koridorda_git(True, m, False)
            if '--siparis' in sys.argv:                       # masada siparis: bekle, yeri yasakla, cik, devriyeye gec
                try:
                    tr = n.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
                    yf = os.path.expanduser('~/cryvex_araclar/yasak_bolgeler.json')
                    yb = json.load(open(yf)) if os.path.exists(yf) else []
                    yb.append([tr.x, tr.y, 1.0]); json.dump(yb, open(yf, 'w'))
                    log(f'bu masa arasi yasak bolge olarak kaydedildi ({tr.x:+.2f}, {tr.y:+.2f})')
                except Exception as e:
                    log(f'yasak bolge kaydedilemedi: {e}')
                log('SIPARIS ALINIYOR - 5 sn bekliyorum'); n.bekle(5.0)
                P = n.noktalar(); ileri_acik, geri_acik = n.yol(P, 0.0), n.yol(P, math.pi)
                ileri = ileri_acik > max(1.5, geri_acik)
                log(f'siparis alindi - {"ILERI" if ileri else "GERI VITESLE"} cikiyorum (on {ileri_acik:.2f} m, arka {geri_acik:.2f} m acik)')
                if not n.koridordan_cik(ileri):
                    log('MASA ARASINDAN CIKAMADIM - devriyeyi BASLATMIYORUM, yardim gerekli'); return
                log('masa arasindan ciktim - DEVRIYE basliyor')
                req = urllib.request.Request('http://127.0.0.1:8080/api/otonom', data=json.dumps({'password': '1234', 'mod': 'devriye'}).encode(),
                                             headers={'Content-Type': 'application/json'})
                log('devriye: ' + urllib.request.urlopen(req, timeout=30).read().decode())
                return
            if '--geri-cik' in sys.argv:
                log(f'icerideyim ({git:.2f} m) - 2 sn sonra GERI VITESLE donmeden cikiyorum'); n.bekle(2.0)
                n.koridorda_git(False, git + 1.0, True)
            return
        P = n.noktalar(); eksen, neden = n.koridor(P)
        log(f'analiz: {neden}' + (f', eksen {math.degrees(eksen):+.0f} deg' if eksen is not None else '')
            + f' | on {n.yol(P, 0)*100:.0f} cm, arka {n.yol(P, math.pi)*100:.0f} cm, en yakin {n.en_yakin(P)*100:.0f} cm')
        if eksen is None: log('cikilacak koridor yok'); return
        secenek = []                                               # (donus, geri_mi, yol)
        for yon in (eksen, wrap(eksen + math.pi)):
            acik = n.yol(P, yon, 0.30)
            don, geri = (yon, False) if abs(yon) <= math.pi / 2 else (wrap(yon + math.pi), True)
            secenek.append((acik, don, geri))
        acik, don, geri = max(secenek, key=lambda s: min(s[0], 3.0) - 0.3 * abs(s[1]))
        log(f'cikis: {"GERI" if geri else "ILERI"} - once {math.degrees(don):+.0f} deg don, o yonde {acik:.2f} m acik')
        if '--olc' in sys.argv: return
        n.ortalan()
        if abs(don) > math.radians(4) and not n.don(don): log('hizalanamadim'); return
        P = n.noktalar(); ax, _ = n.koridor(P)
        log(f'hizalandim (eksen hatasi {math.degrees(wrap(2*ax)/2) if ax is not None else 0:+.0f} deg) - {"geri" if geri else "ileri"} cikiyorum')
        if not n.koridordan_cik(not geri): log('KORIDORDAN CIKAMADIM'); sys.exit(2)
    finally:
        n.dur(); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
