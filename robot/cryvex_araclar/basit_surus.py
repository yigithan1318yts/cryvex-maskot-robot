"""BASIT SURUS (Cryvex, 2026-10-07) - tek dongu, Nav2 planlamasi YOK. Saniyede 10 kez:
 1) BAK: lidar + kameranin kutulari (insan, masa, mobilya, vantilator tabani 'direk') -> onundeki +-90 derecede her yon
    icin 'robot (59 cm) iki yaninda 5 cm payla buradan kac m gidebilir'
 2) ONU BOSSA dumduz git
 3) ONUNDE ENGEL VARSA durmadan, gitmek istedigi yone en yakin BOS yonu sec ve KAVISLE oradan gec
    (= sigdigi araliktan gecer, sigmadigini hic denemez)
 4) +-90 derecede bos yol YOKSA dur; kamera arkaya baksin; arka bossa DUMDUZ geri gel, degilse KIPIRDAMA, bekle
 5) yerinde firil firil donme yok: secilen yon en az 1 sn korunur; buyuk donus sadece tekerler bir seye supurmuyorsa
Hiz zinciri: cmd_vel_smoothed -> collision_monitor (son guvenlik freni) -> cmd_vel
Kullanim: python3 -u basit_surus.py [sure_sn]   (once SLAM/Nav2 acik olmali: collision_monitor calisiyor olmali)"""
import sys, os, time, math, json, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

R = 0.295                      # robot yaricapi (59 cm)
PAY = 0.05                     # iki yanda birakilacak pay
HIZ, YAVAS, GERI = 0.18, 0.08, 0.08
LY, LX, LYY = math.radians(-90), 0.125, -0.045
def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Surucu(Node):
    def __init__(self):
        super().__init__('basit_surus')
        self.scan = None; self.odom = None; self.nes = None; self.nes_t = 0.0; self.tarete = {}
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(String, 'nesneler', self._nes, 5)
        self.create_subscription(String, 'tarete/durum', lambda m: self.tarete.update(json.loads(m.data)), 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.dur_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.istek = self.create_publisher(String, 'tarete/istek', 10)

    def _nes(self, m):
        try: self.nes = json.loads(m.data); self.nes_t = time.time()
        except Exception: pass

    def noktalar(self):
        """lidar (govdenin ici = kendi kablosu haric) + kameranin tanidigi cisimlerin kutu kenarlari (robot cercevesi)"""
        P = []
        m = self.scan
        if m is not None:
            a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
            ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
            x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
            L = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)
            P.append(L[np.hypot(L[:, 0], L[:, 1]) > 0.30])
        if self.nes and time.time() - self.nes_t < 1.0:
            for n in self.nes.get('nesneler', []):
                if n['sinif'] in ('cisim', 'engel') or 'koseler' not in n: continue
                K = np.array(n['koseler'], float)
                Q = np.vstack([np.linspace(K[i], K[(i + 1) % 4], 10) for i in range(4)])
                P.append(Q[np.hypot(Q[:, 0], Q[:, 1]) > R + 0.05])
        return np.vstack(P) if P else np.zeros((0, 2))

    @staticmethod
    def yol(P, a, w=R + PAY):
        """o yone dumduz gidince (yuvarlak govde + iki yanda pay) degmeden gidilebilecek yol (m)"""
        if len(P) == 0: return 9.0
        c, s = math.cos(a), math.sin(a); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        sel = (px > 0) & (np.abs(py) < w)
        return float(max(0.0, (px[sel] - np.sqrt(w * w - py[sel] ** 2)).min())) if sel.any() else 9.0

    @staticmethod
    def donus_guvenli(P, a, sinir=0.45, pay=math.radians(20)):
        """yerinde 'a' donerken tekerlerin (+-90 derece) supurecegi yayda 45 cm icinde bir sey var mi"""
        if len(P) == 0: return True
        d = np.hypot(P[:, 0], P[:, 1]); Q = P[d < sinir]
        if len(Q) == 0: return True
        aci = np.arctan2(Q[:, 1], Q[:, 0])
        for t in (math.pi / 2, -math.pi / 2):
            b, e = (t, t + a) if a > 0 else (t + a, t); o = (b + e) / 2; y = (e - b) / 2 + pay
            if (np.abs((aci - o + math.pi) % (2 * math.pi) - math.pi) <= y).any(): return False
        return True

    @staticmethod
    def yanlar(P):
        """govdenin hemen yanindaki (on-arka 40 cm) en yakin sol ve sag engel uzakligi (m) ya da None"""
        if len(P) == 0: return None, None
        b = P[np.abs(P[:, 0]) < 0.40]; s = b[(b[:, 1] > 0) & (b[:, 1] < 0.9)]; g = b[(b[:, 1] < 0) & (b[:, 1] > -0.9)]
        return (float(s[:, 1].min()) if len(s) else None), (float(-g[:, 1].max()) if len(g) else None)

    def ortala(self, P):
        """KORIDORDA ORTALAN (SAKIN): sadece UZUN yan yuzeyler (duvar/tahta; govde boyunca 80 cm'de en az 15 nokta) sayilir,
        ayak gibi kucuk parcalar degil; olcum yumusatilir, 3 cm alti kaymaya dokunulmaz. Donus: m (+ = sola gitmeli)"""
        b = P[np.abs(P[:, 0]) < 0.40] if len(P) else P
        s = b[(b[:, 1] > 0) & (b[:, 1] < 0.9)] if len(b) else b; g = b[(b[:, 1] < 0) & (b[:, 1] > -0.9)] if len(b) else b
        if len(s) < 15 or len(g) < 15: ham = 0.0
        else: ham = (float(np.median(np.sort(s[:, 1])[:15])) - float(np.median(np.sort(-g[:, 1])[:15]))) / 2
        self.orta_yumusak = 0.8 * getattr(self, 'orta_yumusak', 0.0) + 0.2 * ham
        return 0.0 if abs(self.orta_yumusak) < 0.03 else self.orta_yumusak

    def th(self):
        q = self.odom.pose.pose.orientation; return math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

    def gonder(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.pub.publish(t)

    def durdur(self):
        for _ in range(3): self.dur_pub.publish(Twist()); time.sleep(0.03)

    def bekle(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.05)

    def kamera_bak(self, aci, sure=6.0, bekle=3.0):
        """kamera o yone baksin (aci derece ya da 'arka'), bakana kadar bekle"""
        kid = f'b{int(time.time() * 10) % 100000}'; self.istek.publish(String(data=f'{kid};{aci};{sure}')); t = time.time()
        while time.time() - t < bekle:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tarete.get('istek_id') == kid and self.tarete.get('bakti'): return True
        return False

    @staticmethod
    def yan_paylar(P, a, uzun):
        """o yone 'uzun' m dumduz gidince yol boyunca (20 cm'den sonra) govdenin solunda/saginda kalan bosluk (m)"""
        if len(P) == 0: return 9.0, 9.0
        c, s = math.cos(a), math.sin(a); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        b = (px > 0.20) & (px < uzun) & (np.abs(py) < 1.2)
        sol = py[b & (py > 0)]; sag = -py[b & (py < 0)]
        return (float(sol.min()) - R if len(sol) else 9.0), (float(sag.min()) - R if len(sag) else 9.0)

    def poz(self):
        p = self.odom.pose.pose.position; return p.x, p.y

    def araliklar(self, P, th):
        """IKI ENGELIN ARASINDAKI ARALIKLAR (sadece olcum): o yonde 5 cm payla >= 1 m gidilebiliyor VE yol boyunca
        HEM solda HEM sagda 45 cm icinde engel var. Komsu yonler tek aralik. Donus: [(yon_rad, genislik_m, hedef_nokta)]"""
        x, y = self.poz(); bul = []
        for d in range(-90, 91, 3):
            a = math.radians(d); f = self.yol(P, a)
            if f < 1.0: continue
            sol, sag = self.yan_paylar(P, a, min(f, 1.8))
            if sol < 0.45 and sag < 0.45: bul.append((d, sol + sag + 2 * R))
        gruplar = []
        for d, gen in bul:
            if gruplar and d - gruplar[-1][-1][0] <= 3: gruplar[-1].append((d, gen))
            else: gruplar.append([(d, gen)])
        sonuc = []
        for g in gruplar:
            d = g[len(g) // 2][0]; a = math.radians(d); gen = min(x[1] for x in g)
            nok = (x + 1.2 * math.cos(th + a), y + 1.2 * math.sin(th + a))
            if any(math.hypot(nok[0] - gx, nok[1] - gy) < 0.9 and time.time() - gt < 240 for gx, gy, gt in self.gecilen): continue
            sonuc.append((a, gen, nok))
        return sonuc

    def calis(self, sure):
        log(f'DEVRIYE + ARALIK ({sure:.0f} sn): gez, iki engel arasi bul, sigiyorsa gec, sonra digerine')
        while (self.scan is None or self.odom is None) and rclpy.ok(): self.bekle(0.2)
        hedef = self.th()                                                # gitmek istedigim yon (dunya)
        secim, secim_t, durum, son_log, bas = 0.0, 0.0, '', 0.0, time.time()
        self.gecilen = []; self.ziyaret = []; aralik = None; t_ara = 0.0; t_yeni_yon = time.time(); gecilen_say = 0
        while rclpy.ok() and time.time() - bas < sure:
            rclpy.spin_once(self, timeout_sec=0.0)
            P = self.noktalar(); th = self.th(); px, py = self.poz()
            if not self.ziyaret or time.time() - self.ziyaret[-1][2] > 1.0: self.ziyaret.append((px, py, time.time()))
            # --- ARALIK: hedefteki araliktan gectim mi? yoksa yeni aralik ara ---
            if aralik:
                if math.hypot(px - aralik['nok'][0], py - aralik['nok'][1]) < 0.35 or time.time() - aralik['t'] > 40:
                    tamam = time.time() - aralik['t'] <= 40; gecilen_say += tamam
                    log(f"  {'ARALIKTAN GECTIM' if tamam else 'araliga 40 sn icinde ulasamadim, birakiyorum'} (toplam gecilen: {gecilen_say})")
                    self.gecilen.append((aralik['nok'][0], aralik['nok'][1], time.time())); aralik = None; t_yeni_yon = time.time()
            elif time.time() - t_ara > 0.5:
                t_ara = time.time(); bul = self.araliklar(P, th)
                if bul:
                    a, gen, nok = min(bul, key=lambda b: abs(b[0]))
                    aralik = {'nok': nok, 't': time.time()}; hedef = th + a; secim_t = 0.0
                    log(f'  ARALIK BULDUM: {gen*100:.0f} cm genis, {math.degrees(a):+.0f} derece - icinden geciyorum')
            # --- DEVRIYE: 25 sn aralik yoksa son 3 dk'da gitmedigim en acik yone ---
            if not aralik and time.time() - t_yeni_yon > 25:
                t_yeni_yon = time.time(); en_iyi = None
                for d in range(-90, 91, 10):
                    a = math.radians(d); f = min(self.yol(P, a), 3.0)
                    qx, qy = px + 1.5 * math.cos(th + a), py + 1.5 * math.sin(th + a)
                    ceza = sum(1 for zx, zy, zt in self.ziyaret if time.time() - zt < 180 and math.hypot(qx - zx, qy - zy) < 1.0)
                    s = f - 0.15 * ceza - 0.3 * abs(a)
                    if f >= 1.0 and (en_iyi is None or s > en_iyi[0]): en_iyi = (s, a)
                if en_iyi: hedef = th + en_iyi[1]; log(f'  devriye: gitmedigim yone donuyorum ({math.degrees(en_iyi[1]):+.0f} derece)')
            istenen = wrap(hedef - th)
            serbest = {d: self.yol(P, math.radians(d)) for d in range(-90, 91, 5)}
            if max(serbest.values()) < 0.8:                              # dar koridor: 5 cm pay yok -> 1.5 cm ile yeniden bak
                serbest = {d: self.yol(P, math.radians(d), w=R + 0.015) for d in range(-90, 91, 5)}
            on = serbest[0]
            # aday yonler: en az 0.8 m bos; puan = bos yol - istenen yondan sapma
            # YALPALAMA YOK: bir tarafa (saga/sola) dolanmaya basladiysa o taraf kapanmadikca obur tarafa gecme
            taraf = math.copysign(1, secim) if abs(secim) > math.radians(10) else 0
            aday = [(min(f, 2.5) - 1.0 * abs(math.radians(d) - istenen)
                     - (1.5 if taraf and d * taraf < -10 else 0.0), math.radians(d)) for d, f in serbest.items() if f >= 0.8]
            # secimi 2 sn koru, ama o yon kapandiysa hemen birak
            if time.time() - secim_t < 2.0 and self.yol(P, secim) >= 0.6: a = secim
            elif aday:
                a = max(aday)[1]
                if abs(a - secim) > math.radians(10): secim_t = time.time()
                secim = a
            else: a = None
            if a is None:                                                # +-90 derecede bos yol yok
                self.durdur(); arka = self.yol(P, math.pi)
                if arka >= 0.6:
                    log(f'  onum ve yanlarim kapali (on {on:.2f} m) - kamera arkaya bakiyor, arka {arka:.2f} m bos: DUMDUZ geri')
                    self.kamera_bak('arka'); P = self.noktalar(); geri = min(0.5, self.yol(P, math.pi) - 0.3)
                    if geri > 0.05:
                        x0 = self.odom.pose.pose.position.x; y0 = self.odom.pose.pose.position.y; t0 = time.time()
                        while time.time() - t0 < geri / GERI + 2:
                            rclpy.spin_once(self, timeout_sec=0.0); p = self.odom.pose.pose.position
                            if math.hypot(p.x - x0, p.y - y0) >= geri or self.yol(self.noktalar(), math.pi) < 0.15: break
                            self.gonder(-GERI, 0.0); time.sleep(0.05)
                        self.durdur()
                    self.istek.publish(String(data='serbest'))
                    hedef = th + math.pi / 2 * (1 if self.yol(self.noktalar(), math.pi / 2) > self.yol(self.noktalar(), -math.pi / 2) else -1)
                else:
                    if time.time() - son_log > 5: log(f'  her yer kapali (on {on:.2f}, arka {arka:.2f}) - KIPIRDAMADAN bekliyorum'); son_log = time.time()
                    try: open(os.path.expanduser('~/cryvex_araclar/gezgin_bekliyor'), 'w').write('1')
                    except Exception: pass
                    self.bekle(0.5)
                continue
            # surus: kucuk sapma -> dumduz/kavis; buyuk sapma -> yavas kavis; cok buyuk -> (tekerler guvenliyse) yavas don
            if abs(a) <= math.radians(35): v, w = HIZ, max(-0.5, min(0.5, 1.2 * a)); yeni = 'ileri'   # ortalama/kayma YOK
            elif abs(a) <= math.radians(60) or not self.donus_guvenli(P, a): v, w = YAVAS, max(-0.5, min(0.5, 1.0 * a)); yeni = 'kavis'
            else: v, w = 0.05, 0.40 * math.copysign(1, a); yeni = 'genis kavis'   # yerinde donme YOK: yavas ilerleyerek don
            if self.yol(P, a, w=R + 0.015) < 0.25 and v > 0: v = 0.0           # gidecegi yonde govdenin dibinde bir sey
            v *= max(0.35, min(1.0, (min(on, self.yol(P, a)) - 0.3) / 1.0))      # VFH+: engele yaklastikca yavasla
            if yeni != durum or time.time() - son_log > 5:
                log(f'  {yeni}: yon {math.degrees(a):+.0f} derece ({serbest[int(round(math.degrees(a) / 5) * 5)]:.1f} m bos), on {on:.2f} m')
                durum, son_log = yeni, time.time()
            if on >= 1.5 and abs(istenen) > math.radians(60): hedef = th + a   # istedigim yon kapali kaldiysa yeni yonu benimse
            self.gonder(v, w); time.sleep(0.1)
        self.durdur(); self.istek.publish(String(data='serbest')); log('BASIT SURUS bitti')

def duvar_cizgisi(Q):
    """yan duvar noktalarina SAGLAM cizgi uydur (ayak/cikinti gibi aykiri noktalar atilir).
    Donus: (aci_rad: duvarin robot x eksenine gore acisi, uzaklik_m: govde merkezinden duvara dik uzaklik) ya da None"""
    if len(Q) < 20: return None
    for _ in range(3):
        m = Q.mean(0); u, s, vt = np.linalg.svd(Q - m); yon = vt[0]
        if yon[0] < 0: yon = -yon
        nrm = np.array([-yon[1], yon[0]]); art = (Q - m) @ nrm
        tut = np.abs(art) < max(0.03, 2.5 * np.median(np.abs(art)))
        if tut.sum() < 15: return None
        Q = Q[tut]
    m = Q.mean(0); u, s, vt = np.linalg.svd(Q - m); yon = vt[0]
    if yon[0] < 0: yon = -yon
    if s[1] > 0.25 * s[0]: return None                                   # cizgi degil (daginik)
    return math.atan2(yon[1], yon[0]), abs(np.array([-yon[1], yon[0]]) @ m)

def koridor_olc(L, x1=-0.8, x2=0.8):
    """lidar noktalarindan (govde ici haric) sol ve sag duvar cizgileri, robotun x1..x2 araliginda.
    Donus: dict(sol=(aci,uzaklik)|None, sag=..., aci=ortalama duvar acisi|None, kayma=m (+ sola gitmeli)|None)"""
    b = L[(L[:, 0] > x1) & (L[:, 0] < x2)]
    sol = duvar_cizgisi(b[(b[:, 1] > 0.25) & (b[:, 1] < 0.9)])
    sag = duvar_cizgisi(b[(b[:, 1] < -0.25) & (b[:, 1] > -0.9)])
    acilar = [d[0] for d in (sol, sag) if d]
    aci = float(np.mean(acilar)) if acilar else None
    kayma = (sol[1] - sag[1]) / 2 if (sol and sag) else None
    return {'sol': sol, 'sag': sag, 'aci': aci, 'kayma': kayma}

def koridor_testi(n, tekrar=4, mesafe=0.8, bekleme=3.0, giris_max=2.5):
    """KORIDOR (DUVAR TAKIBI): 1) arkadaki koridora DUMDUZ geri gir (kamera arkaya bakar)
    2) icerde 'tekrar' kez: mesafe ileri -> bekle -> mesafe geri -> bekle. Duvarlar CIZGI olarak olculur, robot onlara
    PARALEL kalir, ortadan sapinca cok yumusak duzeltir (dusuk kazanc, olu bolge, yumusatma). Saniyede bir on/arka/sol/sag."""
    while (n.scan is None or n.odom is None) and rclpy.ok(): n.bekle(0.2)
    def lidar():
        m = n.scan; a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0); x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s = math.cos(LY), math.sin(LY); L = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1)
        return L[np.hypot(L[:, 0], L[:, 1]) > 0.30]
    def surus(yon, mesafe, giris=False):
        n.kamera_bak(0 if yon > 0 else 'arka', sure=40, bekle=3.0)
        x0, y0 = n.poz(); th0 = n.th(); t0 = time.time(); son = 0.0; wf = 0.0; ici = 0.0
        hiz, sinir = (0.12, 0.35) if yon > 0 else (0.09, 0.25)
        while rclpy.ok() and time.time() - t0 < mesafe / hiz + 20:
            rclpy.spin_once(n, timeout_sec=0.0); L = lidar(); px, py = n.poz(); gitti = math.hypot(px - x0, py - y0)
            k = koridor_olc(L); bos = n.yol(L, 0.0 if yon > 0 else math.pi, w=R + 0.015)
            if time.time() - son > 1.0:
                son = time.time(); sl = f"{k['sol'][1] - R:.2f}" if k['sol'] else '-'; sg = f"{k['sag'][1] - R:.2f}" if k['sag'] else '-'
                log(f"    on {n.yol(L, 0.0, w=R + 0.015):.2f} m | arka {n.yol(L, math.pi, w=R + 0.015):.2f} m | sol duvar {sl} m | sag duvar {sg} m | gittim {gitti:.2f} m")
            if giris:                                                    # girerken: iki duvar 0.5 m boyunca gorulunce icerdeyim
                ici = ici + 1 if (k['sol'] and k['sag']) else 0
                if ici >= 10 and gitti > 0.5: log('    koridorun icindeyim'); break
            elif gitti >= mesafe: break
            if bos < sinir: n.durdur(); log(f"    {'onumde' if yon > 0 else 'arkamda'} engel {bos:.2f} m - duruyorum"); return False
            # DUVARA PARALEL: duvar acisi varsa ona gore, yoksa basladigi yone gore; ortalama cok yumusak
            if k['aci'] is not None: ae = k['aci'] if abs(k['aci']) > math.radians(1.5) else 0.0
            else: ae = wrap(th0 - n.th())
            ke = 0.0
            if k['kayma'] is not None and abs(k['kayma']) > 0.02: ke = yon * max(-0.08, min(0.08, k['kayma']))
            w = max(-0.15, min(0.15, 0.8 * ae + 1.0 * ke)); wf = 0.7 * wf + 0.3 * w        # yumusatma: ani donus yok
            v = hiz * max(0.5, min(1.0, (bos - sinir) / 0.5))
            n.gonder(v * yon, wf); time.sleep(0.05)
        n.durdur(); return True
    L = lidar(); arka = koridor_olc(L, -1.8, -0.3)
    log(f"KORIDOR TESTI. Arkada duvarlar: sol {'var' if arka['sol'] else 'yok'}, sag {'var' if arka['sag'] else 'yok'}")
    if not (arka['sol'] or arka['sag']): log('  arkamda koridor goremiyorum - durdum'); return
    log('1) arkadaki koridora DUMDUZ geri giriyorum')
    if not surus(-1, giris_max, giris=True): log('  giremedim - durdum'); n.istek.publish(String(data='serbest')); return
    n.bekle(bekleme)
    for i in range(tekrar):
        log(f'TUR {i + 1}/{tekrar}: {mesafe:.1f} m ileri'); surus(+1, mesafe); log(f'  {bekleme:.0f} sn bekliyorum'); n.bekle(bekleme)
        log(f'TUR {i + 1}/{tekrar}: {mesafe:.1f} m geri'); surus(-1, mesafe); log(f'  {bekleme:.0f} sn bekliyorum'); n.bekle(bekleme)
    n.istek.publish(String(data='serbest')); log('KORIDOR TESTI bitti')

def ileri_geri(n, tekrar=4, mesafe=1.0, bekleme=3.0):
    """KORIDOR TESTI: dumduz ILERI mesafe -> 3 sn bekle -> dumduz GERI mesafe -> 3 sn bekle, 'tekrar' kez.
    Yon sabit tutulur (odom yaw). Gidecegi tarafta engel olursa durur, acilana kadar bekler (en fazla 15 sn)."""
    while (n.scan is None or n.odom is None) and rclpy.ok(): n.bekle(0.2)
    th0 = n.th(); log(f'ILERI-GERI TESTI: {tekrar} kez, {mesafe:.1f} m ileri / {mesafe:.1f} m geri, aralarda {bekleme:.0f} sn')
    def git(yon):
        n.kamera_bak(0 if yon > 0 else 'arka', sure=mesafe / 0.1 + 15, bekle=3.0)       # once kamera gidecegi yere baksin
        x0, y0 = n.odom.pose.pose.position.x, n.odom.pose.pose.position.y; t0 = time.time(); bekledi = 0.0
        hiz, sinir = (0.15, 0.35) if yon > 0 else (0.10, 0.25)
        while rclpy.ok() and time.time() - t0 < mesafe / hiz + 25:
            rclpy.spin_once(n, timeout_sec=0.0); p = n.odom.pose.pose.position
            gitti = math.hypot(p.x - x0, p.y - y0)
            if gitti >= mesafe: break
            P = n.noktalar()
            # durma karari: SADECE gittigi yonde, govdeye 1.5 cm'den yakin bir sey (ortalama/kayma YOK)
            bos = n.yol(P, 0.0 if yon > 0 else math.pi, w=R + 0.015)
            if bos < sinir:
                n.durdur()
                if bekledi == 0.0: log(f'  {"onumde" if yon > 0 else "arkamda"} engel ({bos:.2f} m) - duruyorum, acilmasini bekliyorum')
                bekledi += 0.2; n.bekle(0.2)
                if bekledi > 15: log('  15 sn acilmadi - bu adimi birakiyorum'); break
                continue
            bekledi = 0.0
            v = hiz * max(0.4, min(1.0, (bos - sinir) / 0.6))              # engele yaklastikca yavasla
            n.gonder(v * yon, max(-0.2, min(0.2, 1.0 * wrap(th0 - n.th()))))   # SADECE basladigi yonu koru: dumduz
            time.sleep(0.05)
        n.durdur(); p = n.odom.pose.pose.position
        log(f'  {"ileri" if yon > 0 else "geri"}: {math.hypot(p.x - x0, p.y - y0):.2f} m gittim')
    for i in range(tekrar):
        log(f'TUR {i + 1}/{tekrar}')
        git(+1); log(f'  {bekleme:.0f} sn bekliyorum'); n.bekle(bekleme)
        git(-1); log(f'  {bekleme:.0f} sn bekliyorum'); n.bekle(bekleme)
    n.istek.publish(String(data='serbest')); log('ILERI-GERI TESTI bitti')

if __name__ == '__main__':
    rclpy.init(); n = Surucu()
    sure = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('--') else 120.0
    try:
        if '--koridor' in sys.argv:
            koridor_testi(n)
        elif '--koridor-olc' in sys.argv:                                 # HAREKETSIZ: koridor duvarlarini olc
            t = time.time()
            while time.time() - t < 4: rclpy.spin_once(n, timeout_sec=0.1)
            m = n.scan; a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
            ok = np.isfinite(r) & (r > 0.05) & (r < 8.0); x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
            c, s = math.cos(LY), math.sin(LY); L = np.stack([c * x - s * y + LX, s * x + c * y + LYY], 1); L = L[np.hypot(L[:, 0], L[:, 1]) > 0.30]
            for ad, (x1, x2) in (('ARKADA', (-1.8, -0.3)), ('YANIMDA', (-0.8, 0.8)), ('ONUMDE', (0.3, 1.8))):
                k = koridor_olc(L, x1, x2)
                f = lambda d: f'aci {math.degrees(d[0]):+.1f} derece, govdeye {d[1] - R:.2f} m' if d else 'yok'
                print(f'{ad}: sol duvar {f(k["sol"])} | sag duvar {f(k["sag"])}')
            print('arka bos', round(n.yol(L, math.pi, w=R + 0.015), 2), 'm | on bos', round(n.yol(L, 0.0, w=R + 0.015), 2), 'm')
        elif '--ileri-geri' in sys.argv:
            ileri_geri(n)
        elif '--dene' in sys.argv:                                          # HAREKETSIZ: ne yapacagini goster
            t = time.time()
            while time.time() - t < 4: rclpy.spin_once(n, timeout_sec=0.1)
            P = n.noktalar(); print('nokta', len(P))
            print('  ' + '  '.join(f'{d:+d}:{n.yol(P, math.radians(d)):.1f}' for d in range(-90, 91, 15)))
            print('  arka:', round(n.yol(P, math.pi), 2))
            n.gecilen = []
            for a, gen, nok in n.araliklar(P, n.th()): print(f'  ARALIK: {math.degrees(a):+.0f} derece, {gen*100:.0f} cm genis')
        else:
            n.calis(sure)
    except KeyboardInterrupt:
        pass
    finally:
        n.durdur(); n.destroy_node(); rclpy.shutdown()
