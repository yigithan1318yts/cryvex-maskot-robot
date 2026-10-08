"""DEVRIYE (Cryvex, 2026-10-07) - kullanicinin videoda ayaklariyla gosterdigi davranis, TEK durum makinesi:
  GEZ   : dumduz git; onu kapanirsa bos tarafa KAVISLE don (yerinde donme yok), secimi 3 sn koru
  ARALIK: iki engelin arasinda sigdigi bir aralik gorurse -> kavisle araliga yonel
  GEC   : araligin icinden DUMDUZ (yon sabit); engel bitip SAGI-SOLU bosalinca DUR (3 sn)
  GERI  : DUMDUZ geri; girise gelip sagi-solu bosalinca DUR (3 sn)
  CIK   : bos tarafa kavisle don, GEZ'e don (bu araliga 4 dk tekrar girme)
Hiz: yumusak (ivme sinirli), dur-kalk yok. Komut dogrudan /cmd_vel (Nav2 carpisma freni aradan cikti: koridorda
dur-kalk yaptiriyordu). Kendi frenimiz: gidilen yonde govdeye 1.5 cm'den yakin bir sey -> dur.
Kullanim: python3 -u devriye.py [sure_sn]   |  --dene (hareketsiz: ne gordugunu/ne yapacagini yaz)"""
import os, sys, time, math, json, numpy as np, rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import basit_surus as B
from basit_surus import R, log, wrap, koridor_olc

HIZ, AZ, GERI = 0.22, 0.10, 0.10                 # 2026-10-08: daha hizli gezinme
# 2026-10-08 GUVENLIK (kullanici kurali: GEOMETRI ONCE, sinif guvenligi asla azaltmaz) - ayarlar guvenlik.json
GUV = {'dur_m': 0.35, 'yavas_m': 0.70, 'yavas_hiz': 0.08, 'kose_pay_m': 0.03, 'hitbox_ile_dur': [],
       'bilinmeyen': {'menzil_m': 1.5, 'en_cok_aci_der': 30, 'en_az_isin': 2}, 'insan_suzulme_min_m': 0.5}
try: GUV.update(json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'guvenlik.json'))))
except Exception as e: print('guvenlik.json okunamadi, varsayilan:', e)
DV_UP, DV, DW = 0.004, 0.0125, 0.015   # HIZLANMA 0.04 m/s2 (step motor adim kacirmasin), YAVASLAMA 0.25 m/s2, donus 0.3 rad/s2
# (yavaslamayi da 0.04 yapmak 0.15 m/s'den 28 cm fren mesafesi demek -> carpar; acil fren her zaman ani)

class Devriye(B.Surucu):
    def __init__(self):
        super().__init__()
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.v = 0.0; self.w = 0.0
        # KAYMA TESPITI (IMU yok -> referans LIDAR ODOMETRISI rf2o): tekerler donuyor ama lidar ilerleme gormuyorsa kayma
        from nav_msgs.msg import Odometry as _Od
        self.teker_v = 0.0; self.lidar_v = 0.0; self.kayma_bas = None
        self.teker_p = None; self.lidar_p = None; self.kayma_gecmis = []
        def _t(m): self.teker_v = m.twist.twist.linear.x; self.teker_p = (m.pose.pose.position.x, m.pose.pose.position.y)
        def _l(m): self.lidar_v = m.twist.twist.linear.x; self.lidar_p = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.create_subscription(_Od, 'wheel/odom', _t, 10)
        self.create_subscription(_Od, 'odom_rf2o', _l, 10)

    def kayma_var(self):
        """TAKILMA/KAYMA: son 2 sn'de TEKERLER > 6 cm gitti der ama LIDAR KONUMU < 2 cm degisti (ve komut hareket).
        (Anlik lidar hizi yavasta 0 gorunuyordu -> yanlis alarm; konum toplami kucuk hareketi de dogru yakalar.)"""
        if '--kayma' not in sys.argv: return False                          # 2026-10-08: rf2o yavasta yanlis alarm -> enkoder gelene kadar KAPALI
        if self.teker_p is None or self.lidar_p is None: return False
        simdi = time.time(); self.kayma_gecmis.append((simdi, self.teker_p, self.lidar_p, abs(self.v) > 0.03))
        while self.kayma_gecmis and simdi - self.kayma_gecmis[0][0] > 0.8: self.kayma_gecmis.pop(0)
        if simdi - self.kayma_gecmis[0][0] < 0.75 or not all(h[3] for h in self.kayma_gecmis): return False
        (_, t0, l0, _), (_, t1, l1, _) = self.kayma_gecmis[0], self.kayma_gecmis[-1]
        dt = math.hypot(t1[0] - t0[0], t1[1] - t0[1]); dl = math.hypot(l1[0] - l0[0], l1[1] - l0[1])
        if dt > 0.03 and dl < 0.008:
            self.teker_v, self.lidar_v = dt / 0.8, dl / 0.8                   # log icin ortalama hizlar
            self.kayma_gecmis = []; return True
        return False

    def takildi_kurtul(self):
        """STEP MOTOR ADIM KACIRMA SIFIRLAMASI (2026-10-07): 1) komut 250 ms TAM SIFIR (bobinler gevser, rotor oturur)
        2) gidis yonunun tersine 0.06 m/s x 0.5 sn dumduz (w=0), o taraf >= 30 cm bossa 3) takildigim yeri 60 sn TAKILMA
        isaretle (lidar altindaki zemin bozuklugu -> lidar 'bos' der, hayalet silme silmez), planlayici yeni yone YUMUSAK
        rampayla cikar. Cagiranlar self.takildi_t ile anlar."""
        yon = -1 if self.v > 0 else 1
        log(f'    ADIM KACIRMA: tekerler {self.teker_v:+.3f} m/s diyor ama robot ilerlemiyor (lidar {self.lidar_v:+.3f} m/s) - step sifirlama')
        th = self.th(); x, y = self.poz(); ileri = -yon
        if not hasattr(self, 'takilma'): self.takilma = []
        self.takilma.append((x + 0.4 * ileri * math.cos(th), y + 0.4 * ileri * math.sin(th), time.time() + 30))
        self.v = self.w = 0.0
        t0 = time.time()
        while time.time() - t0 < 0.25: self.cmd.publish(Twist()); time.sleep(0.05)      # bobin gevsetme
        if self.yol(self.lidar(), math.pi if yon < 0 else 0.0, w=R + 0.015) >= 0.30:
            log(f"    {'geri' if yon < 0 else 'ileri'} 0.06 m/s x 0.5 sn (w=0) - tumsekten uzaklasiyorum")
            t0 = time.time()
            while time.time() - t0 < 0.5:
                tw = Twist(); tw.linear.x = 0.06 * yon; self.cmd.publish(tw); time.sleep(0.05)
            for _ in range(5): self.cmd.publish(Twist()); time.sleep(0.05)
        else:
            log('    o taraf kapali - geri itmiyorum')
        self.v = self.w = 0.0; self.kayma_gecmis = []; self.takildi_t = time.time()

    def yakin_yan_dolu(self, L):
        """donerken govdeye 1 cm'den yakin bir sey girdi mi (acil kes)"""
        return bool(len(L)) and float(np.hypot(L[:, 0], L[:, 1]).min()) < R + 0.01

    def kablo_onde(self):
        """kamera robotun ~1 m onunde yerde KABLO gordu mu (son 2 sn icinde)"""
        if self.nes and time.time() - self.nes_t < 1.0 and self.nes.get('kablo'): self.kablo_t = time.time()
        return time.time() - getattr(self, 'kablo_t', 0) < 2.0

    def engel_noktalari(self, L, yon=None):
        """GUVENLIK icin engel geometrisi: lidar (bilinmeyen alan dolu sayilmis) + PAYLI hitbox kenarlari
        (masa/mobilya/insan...: egik ayak, yerdeki cerceve lidarin altinda kalir -> pay). Sinif SADECE ekler.
        Gövde kenarindaki 2 cm (kendi guc kablosu/parcalari) atilir. Hitbox SADECE robot ONA DOGRU gidiyorsa (merkezi
        gidis yonunde) eklenir: payin icine girmis robot dumduz UZAKLASABILSIN (o zaman sadece gercek lidar karar verir).
        yon: +1 ileri, -1 geri, None (yerinde donus: sadece lidar)"""
        L = L[np.hypot(L[:, 0], L[:, 1]) > R + 0.02] if len(L) else L
        E = [L] if len(L) else []
        # GERI: hitbox YOK, sadece gercek lidar (geri hep yavas, kisa, dumduz = geldigi yoldan cikis). 2026-10-08: masa/sandalye
        # ayaklarindan uydurulan PAYLI masa kutulari bos arkayi 'dolu' gosterip robotu masanin dibinde kilitliyordu.
        if yon is not None and yon > 0 and self.nes and time.time() - self.nes_t < 0.5:
            for n in self.nes.get('nesneler', []):
                if n.get('sinif') not in GUV['hitbox_ile_dur']: continue
                K = np.array(n['koseler'], float)
                if K.mean(0)[0] * yon <= 0: continue                         # nesneden uzaklasiyorum: payi beni kilitlemesin
                ic = all(np.cross(b - a, -a) >= 0 for a, b in zip(K, np.roll(K, -1, 0))) or all(np.cross(b - a, -a) <= 0 for a, b in zip(K, np.roll(K, -1, 0)))
                kn = np.vstack([a + np.linspace(0, 1, max(2, int(np.hypot(*(b - a)) / 0.05)))[:, None] * (b - a)
                                for a, b in zip(K, np.roll(K, -1, 0))])           # kenarlar boyunca 5 cm'de bir nokta
                if ic or np.hypot(kn[:, 0], kn[:, 1]).min() < R: continue    # kutu robotun GOVDESINE biniyor: guvenilmez, ham lidar karar versin
                E.append(kn)
        return np.vstack(E) if E else np.zeros((0, 2))

    def guvenlik(self, v, w):
        """TEK KAPI: her surus komutu buradan gecer. Gidis yonundeki seritte (govde eni) on/arka yuze dur_m'den yakin
        HERHANGI bir sey -> v = 0 (ANI). yavas_m icinde hiz <= yavas_hiz. Gidis yonundeki govde koselerine kose_pay_m'den
        yakin bir sey -> DUR. Yerinde donus: govde cevresinde kose_pay_m -> donme. Donus: (v, w, acil_dur_mu)"""
        if self.scan is None: return 0.0, 0.0, True
        L = self.lidar(); acil = False; neden = None
        if abs(v) > 0.003:
            yon = 1.0 if v > 0 else -1.0; E = self.engel_noktalari(L, yon)
            if len(E):
                X = E[:, 0] * yon; Y = E[:, 1]; yari = R + 0.03 + min(0.12, abs(w) * 0.3)   # donerken serit genisler
                serit = (np.abs(Y) < yari) & (X > -R) & ((X > 0) | (np.abs(Y) < R))   # pay seridindeki (|Y| > R) sadece ondeyse
                # yuvarlak govdenin gidis yonundeki yuzeyine mesafe (gövdenin yaninda duran, yolda olmayan sey engel degil)
                d_i = X[serit] - np.sqrt(np.maximum(R * R - Y[serit] ** 2, 0.0))
                d = float(d_i.min()) if len(d_i) else 9.0
                if d < GUV['dur_m']:                                         # DUR + DONME de yok (kose engele surtmesin);
                    neden = f"{'onumde' if yon > 0 else 'arkamda'} {max(d, 0) * 100:.0f} cm'de bir sey"   # calis() bunu gorup dumduz geri cikar
                    v = 0.0; w = 0.0; acil = True; self.guv_acil_t = time.time(); self.guv_acil_yon = yon
                elif d < GUV['yavas_m']: v = yon * min(abs(v), GUV['yavas_hiz'])
        if abs(w) > 0.02 and abs(v) < 0.05:                                 # yerinde donus: dikdortgen govdenin KOSELERI/tekerler supurur
            E = self.engel_noktalari(L)
            if len(E) and float(np.hypot(E[:, 0], E[:, 1]).min()) < GUV.get('donus_yaricap_m', 0.40):
                w = 0.0; neden = neden or 'yerinde donersem govde kosesi bir seye degecek'
        if neden and time.time() - getattr(self, 'guv_log', 0) > 2.0:
            self.guv_log = time.time(); log(f'    GUVENLIK: {neden} - DURDUM (sinif ne olursa olsun)')
        return v, w, acil

    def surt(self, v, w):
        """YUMUSAK surus: hiz ve donus kademeli degisir (dur-kalk yok).
        HISTEREZIS: donus yonu (w isareti) ancak yeni yon 5 dongu (~250 ms) ust uste istenirse tersine doner.
        2026-10-08: once GUVENLIK kapisi (geometri; hicbir mod/sinif bunu gecemez)."""
        if self.kayma_var():                                                  # TAKILMA / KAYMA ACIL DURUMU
            self.takildi_kurtul(); return
        v, w, acil = self.guvenlik(v, w)
        if acil: self.v = 0.0                                                 # acil fren: rampa yok
        if abs(w) > 0.02 and abs(self.w) > 0.02 and (w > 0) != (self.w > 0):
            self.ters_n = getattr(self, 'ters_n', 0) + 1
            if self.ters_n < 5: w = 0.0                                  # henuz kanitlanmadi: once duzle, ters cevirme
        else:
            self.ters_n = 0
        if v > 0 and self.kablo_onde():                                     # KABLO: yavas (0.07) ve DUZ gec, teker patinaj yapmasin
            if not getattr(self, 'kablo_log', False): log('    yerde kablo - yavas ve dumduz geciyorum'); self.kablo_log = True
            v = min(v, 0.07); w = 0.0
        else: self.kablo_log = False
        dv = v - self.v; hizlan = abs(v) > abs(self.v) and (v * self.v >= 0)              # hizlanma mi yavaslama mi
        self.v += max(-(DV_UP if hizlan else DV), min(DV_UP if hizlan else DV, dv)); self.w += max(-DW, min(DW, w - self.w))
        t = Twist(); t.linear.x = float(self.v); t.angular.z = float(self.w); self.cmd.publish(t)

    def dur(self, yumusak=True):
        if yumusak:
            for _ in range(15):
                self.surt(0.0, 0.0); time.sleep(0.05)
                if abs(self.v) < 1e-3 and abs(self.w) < 1e-3: break
        self.v = self.w = 0.0
        for _ in range(3): self.cmd.publish(Twist()); time.sleep(0.03)

    def bekle_dur(self, s):
        self.dur(); t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.05); self.cmd.publish(Twist())

    def lidar(self, ham=False):
        """lidar noktalari (robot cercevesi). ham=False: BILINMEYEN alan DOLU sayilir (2026-10-08 kullanici kurali:
        'nesne gorulmedi = bos' VARSAYILMAZ): iki YAKIN olcum arasinda yansima donmeyen isinlar (siyah/parlak yuzey)
        komsularin yakin olaninin mesafesinde engel noktasi olur. ham=True: sadece gercek olcumler (duvar uydurma vb.)"""
        m = self.scan; a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment; r = np.array(m.ranges)
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0); x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        if not ham:
            b = GUV['bilinmeyen']; idx = np.nonzero(ok)[0]
            if len(idx) > 1:
                bos = np.diff(idx) - 1; i0, i1 = idx[:-1], idx[1:]
                sec = (bos >= b['en_az_isin']) & (bos * abs(m.angle_increment) <= math.radians(b['en_cok_aci_der'])) \
                      & (r[i0] < b['menzil_m']) & (r[i1] < b['menzil_m'])
                ex, ey = [x], [y]
                for j0, j1 in zip(i0[sec], i1[sec]):
                    d = min(r[j0], r[j1]); aa = a[j0 + 1:j1]; ex.append(d * np.cos(aa)); ey.append(d * np.sin(aa))
                self.bilinmeyen_n = int(sec.sum()); x, y = np.concatenate(ex), np.concatenate(ey)
        c, s = math.cos(B.LY), math.sin(B.LY); L = np.stack([c * x - s * y + B.LX, s * x + c * y + B.LYY], 1)
        return L[np.hypot(L[:, 0], L[:, 1]) > 0.30]

    @staticmethod
    def yanlar_bos(P, ileri, yari=0.25, gen=0.65):
        """govdenin yaninda (on-arka +-yari m) sol ve sag 'gen' m icinde bir sey var mi -> (sol_dolu, sag_dolu)"""
        b = P[np.abs(P[:, 0] - ileri) < yari] if len(P) else P
        return bool(((b[:, 1] > 0) & (b[:, 1] < gen)).any()), bool(((b[:, 1] < 0) & (b[:, 1] > -gen)).any())

    def bos_yon(self, P, haric=None):
        """+-90 icinde en acik yon (haric verilen yonden 40 derece uzak), kucuk donus tercih"""
        en = None; th = self.th(); x, y = self.poz()
        for d in range(-90, 91, 10):
            a = math.radians(d)
            if haric is not None and abs(wrap(a - haric)) < math.radians(40): continue
            f = min(self.yol(P, a), 3.0)
            if f < 1.0: continue
            qx, qy = x + 1.2 * math.cos(th + a), y + 1.2 * math.sin(th + a)      # yakinda sikistigim cikmaza dogru mu?
            if any(math.hypot(qx - cx, qy - cy) < 0.9 and time.time() - ct < 30 for cx, cy, ct in getattr(self, 'cikmaz', []) + getattr(self, 'takilma', [])): continue
            s = f - 0.8 * abs(a)
            taraf = getattr(self, 'son_taraf', 0)
            if taraf and a * taraf < -math.radians(10) and time.time() - getattr(self, 'son_taraf_t', 0) < 6: s -= 0.6   # taraf degistirme
            if en is None or s > en[0]: en = (s, a, f)
        return en

    def dar_yerdeyim(self, P):
        """yanlarim (govde boyunca) 45 cm icinde dolu mu: dar yerde YERINDE DONME YASAK"""
        s, g = self.yanlar_bos(P, 0.0, yari=0.35, gen=0.45)
        return s or g

    def kavisle_don(self, aci, sure_max=8.0):
        """yerinde degil, YAVAS ILERLEYEREK 'aci' kadar don (yolu bossa). SIFIR-DONUS KURALI: dar yerdeysem ya da onum
        kapaliysa YERINDE DONMEM -> dururum, yonumu korurum. Donus: True (dondu) / False (donmedi)"""
        th0 = self.th(); t = time.time()
        while time.time() - t < sure_max and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            kalan = wrap(th0 + aci - self.th())
            if abs(kalan) < math.radians(5): return True
            P = self.lidar()
            if self.yol(P, 0.0, w=R + 0.015) < 0.25:
                if self.dar_yerdeyim(P) or not self.donus_guvenli(P, kalan):
                    self.dur(); log('    dar yerde yerinde donmuyorum (sifir-donus kurali) - yonumu koruyorum'); return False
                self.surt(0.0, 0.35 * math.copysign(1, kalan))                 # genis yerde ve tekerler bossa yerinde
            else: self.surt(AZ, max(-0.45, min(0.45, 1.2 * kalan)))
            time.sleep(0.05)
        return False

    def hafiza_temizle(self, P, hepsi=False):
        """HAYALET ENGEL YOK: lidar bir yonu SIMDI bos goruyorsa (1.5 m) o yondeki cikmaz kayitlarini hemen sil.
        hepsi=True: butun hafizayi sil (kilitlenme zaman asimi)"""
        if hepsi: self.cikmaz = []; self.son_insan = None; return
        th = self.th(); x, y = self.poz(); kalan = []
        for cx, cy, ct in self.cikmaz:
            if time.time() - ct > 30: continue                                # en fazla 30 sn hatirla
            a = wrap(math.atan2(cy - y, cx - x) - th); d = math.hypot(cx - x, cy - y)
            if self.yol(P, a, w=R + 0.015) >= max(1.5, d + 0.3): continue     # o yon artik bos: sil
            kalan.append((cx, cy, ct))
        self.cikmaz = kalan

    def duz_git(self, yon, mesafe_max, dur_kosul, ad):
        """DUMDUZ (yon sabit, odom) git; dur_kosul(P, gitti) True olunca ya da onu/arkasi kapaninca dur. Donus: gidilen m"""
        th0 = self.th(); x0, y0 = self.poz(); son = 0.0; t0 = time.time(); hiz = HIZ if yon > 0 else GERI
        while rclpy.ok() and time.time() - t0 < mesafe_max / (hiz * 0.5) + 10:
            rclpy.spin_once(self, timeout_sec=0.0); P = self.lidar(); px, py = self.poz(); gitti = math.hypot(px - x0, py - y0)
            if getattr(self, 'takildi_t', 0) > t0: log(f'    {ad}: takildim - bu hareketi biraktim'); return gitti
            on = self.yol(P, 0.0 if yon > 0 else math.pi, w=R + 0.015)
            sol, sag = self.yanlar_bos(P, 0.0)
            if time.time() - son > 1.0:
                son = time.time(); log(f'    {ad}: gittim {gitti:.2f} m | {"on" if yon > 0 else "arka"} {on:.2f} m | sol {"dolu" if sol else "bos"} | sag {"dolu" if sag else "bos"}')
            if dur_kosul(P, gitti) or gitti >= mesafe_max: break
            if on < (0.30 if yon > 0 else 0.25):
                self.dur(); log(f'    {"onumde" if yon > 0 else "arkamda"} engel {on:.2f} m - durdum'); return gitti
            v = hiz * max(0.45, min(1.0, (on - 0.3) / 0.6)) * max(0.5, self.hiz_carpani(P, yon))
            self.surt(v * yon, max(-0.2, min(0.2, 1.0 * wrap(th0 - self.th())))); time.sleep(0.05)
            if time.time() - getattr(self, 'guv_acil_t', 0) < 0.1:          # GUVENLIK kapisi durdurdu: bekleme, birak
                self.dur(); log(f'    {ad}: GUVENLIK durdurdu ({"on" if yon > 0 else "arka"} kapali)'); break
        self.dur(); p = self.poz(); return math.hypot(p[0] - x0, p[1] - y0)

    @staticmethod
    def parcalar(L):
        """lidar noktalarini aci sirasina gore kopukluklardan parcalara ayir; duz olanlari dondur:
        [(merkez, yon_birim, uzunluk, uc1, uc2, kalinlik)]"""
        if len(L) < 10: return []
        sira = np.argsort(np.arctan2(L[:, 1], L[:, 0])); Q = L[sira]
        d = np.hypot(*(Q[1:] - Q[:-1]).T); r = np.hypot(Q[:-1, 0], Q[:-1, 1])
        sonuc = []
        for p in np.split(Q, np.nonzero(d > 0.08 + 0.03 * r)[0] + 1):
            if len(p) < 8: continue
            m = p.mean(0); u, s, vt = np.linalg.svd(p - m); yon = vt[0]; nrm = np.array([-yon[1], yon[0]])
            boy = (p - m) @ yon; art = (p - m) @ nrm
            kal = 2 * float(np.percentile(np.abs(art), 90))
            if kal > 0.07: continue                                      # duz degil
            sonuc.append((m, yon, float(boy.max() - boy.min()), m + yon * boy.min(), m + yon * boy.max(), kal))
        return sonuc

    def tahta_koridoru(self, L):
        """TAHTA KORIDORU (kullanicinin test araligi): tek basina duran 0.9-2.0 m DUZ ince parca (tahta) + ona PARALEL
        (<12 derece) uzun bir yuzey (bolme) 0.80-1.25 m uzakta, tahtanin en az %60'i boyunca; arasi BOS.
        Donus: dict(giris, ic_yon, gen, boy) robot cercevesinde (giris = robota yakin agiz ortasi) ya da None"""
        P = self.parcalar(L); en = None
        for (ma, ua, la, a1, a2, _k) in P:
            if not 0.9 <= la <= 2.0: continue
            for (mb, ub, lb, b1, b2, _k2) in P:
                if mb is ma or lb < 1.0: continue
                if abs(abs(float(ua @ ub)) - 1.0) > 1 - math.cos(math.radians(12)): continue
                n = np.array([-ua[1], ua[0]]); dist = float((mb - ma) @ n)
                if dist < 0: n, dist = -n, -dist
                if not 0.80 <= dist <= 1.25: continue
                pb = sorted([float((b1 - ma) @ ua), float((b2 - ma) @ ua)])            # B'nin tahta ekseni uzerindeki izi
                ust = min(la / 2, pb[1]) - max(-la / 2, pb[0])
                if ust < 0.6 * la: continue
                c = ma + n * dist / 2                                     # koridor ekseni ortasi
                ara = L - c; boyuna = ara @ ua; enine = ara @ n            # koridor ici bos mu
                if ((np.abs(boyuna) < la / 2 - 0.1) & (np.abs(enine) < dist / 2 - 0.08)).sum() > 3: continue
                ag1, ag2 = a1 + n * dist / 2, a2 + n * dist / 2           # iki agiz
                giris, cikis = (ag1, ag2) if np.hypot(*ag1) < np.hypot(*ag2) else (ag2, ag1)
                ic = (cikis - giris) / (np.hypot(*(cikis - giris)) + 1e-9)
                aday = {'giris': giris, 'ic_yon': ic, 'gen': dist - 0.03, 'boy': la}
                if en is None or np.hypot(*giris) < np.hypot(*en['giris']): en = aday
        return en

    def koridora_git(self, k):
        """koridor girisinin 70 cm onune git, yuzunu koridora don (kendi surusumuzle, yumusak). Basarisizsa False."""
        on_nokta = k['giris'] - 0.7 * k['ic_yon']; hedef_yon = math.atan2(k['ic_yon'][1], k['ic_yon'][0])
        th = self.th(); x, y = self.poz(); c, s = math.cos(th), math.sin(th)
        ox, oy = x + c * on_nokta[0] - s * on_nokta[1], y + s * on_nokta[0] + c * on_nokta[1]   # odom'da
        yon_odom = th + hedef_yon
        mesafe = float(np.hypot(*on_nokta)); aci = math.atan2(on_nokta[1], on_nokta[0])
        log(f'  koridor girisinin onune gidiyorum: {mesafe:.2f} m, {math.degrees(aci):+.0f} derece')
        if mesafe > 0.15:
            self.dur(); self.kamera_bak(math.degrees(aci), sure=20, bekle=2.0); self.kavisle_don(aci)
            px, py = self.poz(); kalan = math.hypot(ox - px, oy - py)
            self.duz_git(+1, kalan, lambda P, g: False, 'girise git')
            px, py = self.poz()
            if math.hypot(ox - px, oy - py) > 0.35: log('  girisin onune varamadim'); return False
        log('  yuzumu koridora donuyorum'); self.dur(); self.kamera_bak(math.degrees(wrap(yon_odom - self.th())), sure=20, bekle=2.0)
        self.kavisle_don(wrap(yon_odom - self.th()))
        return True

    def gidip_gel(self):
        """ARALIKTA BIR GIDIS-GELIS: dumduz ileri, engel bitip sagi-solu bosalinca DUR (3 sn); dumduz geri, girise gelip
        sagi-solu bosalinca DUR (3 sn). Yolu kapanirsa False."""
        self.dur(); self.kamera_bak(0, sure=40, bekle=2.0)
        ic = {'gordu': False, 'son': None}
        def bitti(P, gitti):                                             # once yanlar dolu (icerdeyim), sonra bosaldi
            s, g = self.yanlar_bos(P, 0.0)
            if s or g: ic['gordu'] = True; ic['son'] = None; return False
            if not ic['gordu']: return gitti > 2.5
            ic['son'] = ic['son'] if ic['son'] is not None else gitti
            return gitti - ic['son'] > 0.10                              # sagi-solu 10 cm boyunca bos: engel bitti
        log('  GECIYORUM: dumduz, engel bitince duracagim')
        g1 = self.duz_git(+1, 4.0, bitti, 'gec')
        if not ic['gordu'] and g1 < 0.3: log('  ilerleyemedim'); return False
        log(f'  engel bitti, sagim solum bos - DURDUM ({g1:.2f} m). 3 sn bekliyorum'); self.bekle_dur(3.0)
        self.kamera_bak('arka', sure=40, bekle=3.0)
        ic2 = {'gordu': False, 'son': None}
        def giriste(P, gitti):
            s, g = self.yanlar_bos(P, 0.0)
            if s or g: ic2['gordu'] = True; ic2['son'] = None; return False
            if not ic2['gordu']: return gitti > g1 + 0.3
            ic2['son'] = ic2['son'] if ic2['son'] is not None else gitti
            return gitti - ic2['son'] > 0.10
        log('  DUMDUZ GERI geliyorum, girise gelince duracagim')
        g2 = self.duz_git(-1, g1 + 0.5, giriste, 'geri')
        log(f'  girisdeyim - DURDUM ({g2:.2f} m geri). 3 sn bekliyorum'); self.bekle_dur(3.0)
        self.istek.publish(String(data='serbest'))
        return g2 > 0.3

    # ---------------- GUVENLIK KURALLARI (kullanici, 2026-10-07) ----------------
    def hiz_carpani(self, P, yon=+1):
        """UYARI BOLGESI (hiza gore buyur): govde kenarindan 30 cm + fren mesafesi (hiz x 0.6 sn) icinde, gidis yonunun
        +-60 derecesinde bir sey varsa hiz %70 duser (0.3). Kamerada az once gorulup KAYBOLAN insan yonunde lidar
        1.5 m icinde bir sey goruyorsa da 0.3."""
        if len(P) == 0: return 1.0
        tampon = 0.30 + abs(self.v) * 0.6
        aci = np.arctan2(P[:, 1], P[:, 0]) if yon > 0 else np.arctan2(-P[:, 1], -P[:, 0])
        kenar = np.hypot(P[:, 0], P[:, 1]) - R
        if ((np.abs(aci) < math.radians(60)) & (kenar < tampon)).any(): return 0.3
        ki = self.kayip_insan()
        if ki is not None:
            a = np.arctan2(P[:, 1], P[:, 0])
            if ((np.abs((a - ki + math.pi) % (2 * math.pi) - math.pi) < math.radians(15)) & (np.hypot(P[:, 0], P[:, 1]) < 1.5)).any(): return 0.3
        return 1.0

    def insanlar(self):
        """kameranin taniyip LIDAR ile olctugu insanlar: [(aci_rad, uzaklik_m)] (robot cercevesi)"""
        if not self.nes or time.time() - self.nes_t > 1.0: return []
        return [(math.atan2(n['y'], n['x']), n['uz']) for n in self.nes.get('nesneler', []) if n['sinif'] == 'insan']

    def kayip_insan(self):
        """son 2 sn icinde 1.5 m icinde gorulen ama SIMDI gorulmeyen insanin yonu (yoksa None)"""
        ins = [i for i in self.insanlar() if i[1] < 1.5]
        if ins: self.son_insan = (time.time(), ins[0][0]); return None
        s = getattr(self, 'son_insan', None)
        return s[1] if s and time.time() - s[0] < 2.0 else None

    @staticmethod
    def dar_genislik(P, yon=0.0, bas=0.15, son=1.2):
        """gidis yonunde onumdeki yol boyunca (bas..son m) iki yanim ARASINDAKI en dar serbest genislik (m);
        iki yanda da engel yoksa None (acik alan)"""
        if len(P) == 0: return None
        c, s = math.cos(yon), math.sin(yon); px = c * P[:, 0] + s * P[:, 1]; py = -s * P[:, 0] + c * P[:, 1]
        en = None
        for x0 in np.arange(bas, son, 0.1):
            b = (px > x0) & (px < x0 + 0.15) & (np.abs(py) < 0.8)
            sol = py[b & (py > 0)]; sag = -py[b & (py < 0)]
            if len(sol) and len(sag):
                g = float(sol.min() + sag.min())
                en = g if en is None else min(en, g)
        return en

    @staticmethod
    def bosluklar(L, aci_max=45, menzil=2.5):                               # SADECE ON DILIM +-45 derece
        """IKI ENGEL ARASI HER BOSLUK = aday gecit. Lidar noktalari (onde +-aci_max, menzil icinde) aci sirasina gore
        engellere (kumelere) ayrilir; yan yana iki engelin birbirine bakan KENARLARI (R1,t1) ve (R2,t2):
        W = sqrt(R1^2 + R2^2 - 2 R1 R2 cos(t1 - t2))  (kosinus teoremi).
        Arkasi acik olmali (bosluk icinden bakinca kenarlardan en az 30 cm otesi bos) -> duvar girintisi degil.
        Donus: [dict(W, hedef_aci, sol=(x,y), sag=(x,y), uz)]"""
        if len(L) < 5: return []
        r = np.hypot(L[:, 0], L[:, 1]); t = np.arctan2(L[:, 1], L[:, 0])
        sec = (np.abs(t) < math.radians(aci_max)) & (r < menzil); Q, rq, tq = L[sec], r[sec], t[sec]
        if len(Q) < 4: return []
        o = np.argsort(tq); Q, rq, tq = Q[o], rq[o], tq[o]
        d = np.hypot(*(Q[1:] - Q[:-1]).T)
        kes = np.nonzero(d > 0.10 + 0.04 * rq[:-1])[0]                       # engeller arasi kopukluk
        sonuc = []
        for i in kes:                                                         # i: sagdaki engelin son, i+1: soldakinin ilk noktasi
            R2, t2, R1, t1 = rq[i], tq[i], rq[i + 1], tq[i + 1]
            W = math.sqrt(max(0.0, R1 * R1 + R2 * R2 - 2 * R1 * R2 * math.cos(t1 - t2)))
            sol, sag = Q[i + 1], Q[i]; m = (sol + sag) / 2; ha = math.atan2(m[1], m[0])
            # arkasi acik mi: tum lidar (menzil sinirsiz) bosluk acisi icinde kenarlardan 30 cm otesine kadar bos
            ic = (t > min(t1, t2) + 0.02) & (t < max(t1, t2) - 0.02)
            ote = float(r[ic].min()) if ic.any() else 9.0
            if ote < max(R1, R2) + 0.30: continue
            sonuc.append({'W': W, 'hedef_aci': ha, 'sol': sol, 'sag': sag, 'uz': float(np.hypot(*m))})
        return sonuc

    def gecit_ekseni_kenar(self, sol, sag):
        """iki kenar noktasindan orta cizgi: (eksen_aci, yan_kayma [+ = sola gitmeli], agiz_x)"""
        m = (np.asarray(sol) + np.asarray(sag)) / 2; d = np.asarray(sol) - np.asarray(sag)
        ax = math.atan2(-d[0], d[1])
        if abs(ax) > math.pi / 2: ax = wrap(ax + math.pi)
        e = -(-math.sin(ax) * (0 - m[0]) + math.cos(ax) * (0 - m[1]))
        return ax, e, float(m[0])

    def yeniden_olc(self, L, kenar):
        """huni sirasinda gecidi yeniden olc: kenarla geldiysem onume en yakin gecilir bosluk; yoksa duvar/kose"""
        if kenar is not None:
            bs = [b for b in self.bosluklar(L) if b['W'] >= 2 * R + 0.10 and abs(b['hedef_aci']) < math.radians(50)]
            if bs:
                b = min(bs, key=lambda b: abs(b['hedef_aci'])); return self.gecit_ekseni_kenar(b['sol'], b['sag'])
        return self.gecit_ekseni(L)

    def gecit_ekseni(self, L):
        """onumdeki (0.2-1.6 m) gecidin ORTA CIZGISI: (eksen_aci_rad, yan_kayma_m [+ = sola gitmeli], agiz_x_m) ya da None.
        Once iki yan duvar CIZGI olarak; olmazsa giris koselerinden (en yakin sol ve sag noktalar)."""
        k = koridor_olc(L, 0.2, 1.6)
        if k['sol'] and k['sag']:
            a = k['aci']; sl, sg = k['sol'][1], k['sag'][1]
            return a, (sl - sg) / 2, 0.2
        b = L[(L[:, 0] > 0.2) & (L[:, 0] < 1.4) & (np.abs(L[:, 1]) < 0.9)] if len(L) else L
        sol = b[b[:, 1] > 0]; sag = b[b[:, 1] < 0]
        if not len(sol) or not len(sag): return None
        cl = sol[np.argmin(sol[:, 1])]; cr = sag[np.argmax(sag[:, 1])]       # giris koseleri (iç kenarlar)
        m = (cl + cr) / 2; d = cl - cr; ax = math.atan2(-d[0], d[1])           # koselere dik, ileri dogru eksen
        if abs(ax) > math.pi / 2: ax = wrap(ax + math.pi)
        e = -(-math.sin(ax) * (0 - m[0]) + math.cos(ax) * (0 - m[1]))         # robotun eksen cizgisine gore sapmasi
        return ax, e, float(m[0])

    def huni(self, W, kenar=None):
        """HUNI (ON HIZALAMA): gecide 40 cm kala dur; orta cizgiyi olc; yan kayma > 3 cm ise once eksen uzerindeki giris
        noktasina git, sonra yuzunu eksene cevir; sol-on ve sag-on caprazlar (+-30 derece) esitlenince True (tunele gir).
        kenar=(sol, sag): kosinus teoremiyle bulunan iki engel kenari (yoksa duvar cizgileri / koseler)."""
        self.dur(); L = self.lidar()
        g = self.gecit_ekseni_kenar(*kenar) if kenar is not None else self.gecit_ekseni(L)
        if g is None: log('  HUNI: gecidin iki yanini olcemedim - girmiyorum'); return False
        ax, e, agiz = g
        log(f'HUNI: gecit {W*100:.0f} cm, orta cizgi {math.degrees(ax):+.1f} derece, yan kayma {e*100:+.0f} cm - hizalaniyorum')
        if abs(e) > 0.03:                                                     # once orta cizgi uzerindeki giris noktasina
            hx = max(0.25, agiz - 0.40); hy = e + math.tan(ax) * hx             # eksen uzerinde, agzin 40 cm onu
            hed = math.atan2(hy, hx); mes = math.hypot(hx, hy)
            if not self.kavisle_don(hed): log('  HUNI: donemedim - girmiyorum'); return False
            if self.yol(self.lidar(), 0.0, w=R + 0.015) < mes: log('  HUNI: giris noktasina yol kapali - girmiyorum'); return False
            self.duz_git(+1, mes, lambda P, g: False, 'huni: giris noktasina')
            L = self.lidar(); g = self.yeniden_olc(L, kenar)
            if g is None: log('  HUNI: yeniden olcemedim - girmiyorum'); return False
            ax, e, agiz = g
        if abs(ax) > math.radians(2) and not self.kavisle_don(ax): log('  HUNI: eksene donemedim - girmiyorum'); return False
        L = self.lidar(); sol_c = self.yol(L, math.radians(30), w=0.05); sag_c = self.yol(L, math.radians(-30), w=0.05)
        g = self.yeniden_olc(L, kenar)
        if g: ax, e, agiz = g
        log(f'  HUNI: hizalandim - eksene {math.degrees(ax):+.1f} derece, kayma {e*100:+.0f} cm, sol-on {sol_c:.2f} m / sag-on {sag_c:.2f} m')
        if g and (abs(ax) > math.radians(6) or abs(e) > 0.07): log('  HUNI: hala capraz/kayik - girmiyorum'); return False
        return True

    def tunelde_sikistim(self, icerde):
        """TUNELDE ONUM KAPANDI (kullanici karar matrisi):
        ARKA ACIK -> ters kurtarma: w=0 KILITLI, yerinde donme YOK, girdigim izden dumduz geri, tunelden cikinca dur.
        ARKA KAPALI -> kilitlenme bekleme: v=0 w=0; 20 Hz on/arka tara; ON 1.5 sn kesintisiz acik -> 'ileri';
        ARKA 1.5 sn kesintisiz acik -> ters kurtarma. Donus: 'ileri' | 'geri_cikti'"""
        self.dur(yumusak=False); log('  TUNEL: onum kapandi - DURDUM (v=0, w=0)')
        def on_acik(L):
            Wn = self.dar_genislik(L, 0.0, 0.15, 0.8)
            return self.yol(L, 0.0, w=R + 0.015) >= 0.5 and (Wn is None or Wn >= 2 * R + 0.10)
        def arka_acik(L): return self.yol(L, math.pi, w=R + 0.015) > 0.30
        L = self.lidar()
        if not arka_acik(L):
            log('  TUNEL: arkam da kapali - KILITLENME BEKLEME (kipirdamiyorum, on/arka 1.5 sn acik kalirsa cikacagim)')
            on_t = arka_t = None; son = 0.0
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0); self.cmd.publish(Twist()); L = self.lidar(); simdi = time.time()
                on_t = (on_t or simdi) if on_acik(L) else None
                arka_t = (arka_t or simdi) if arka_acik(L) else None
                if on_t and simdi - on_t >= 1.5: log('  TUNEL: onum 1.5 sn acik kaldi - ileri devam'); return 'ileri'
                if arka_t and simdi - arka_t >= 1.5: log('  TUNEL: arkam 1.5 sn acik kaldi - geri cikiyorum'); break
                if simdi - son > 5: log('    bekliyorum (onum ve arkam kapali)'); son = simdi
                try: open(os.path.expanduser('~/cryvex_araclar/gezgin_bekliyor'), 'w').write('1')
                except Exception: pass
                time.sleep(0.05)
        # TERS KURTARMA: w = 0 kilitli, dumduz geri; tunelden cikinca (iki yan bos 15 cm) ya da girdigim kadar gidince dur
        if abs(abs(self.tarete.get('aci', 0)) - 180) > 20: self.kamera_bak('arka', sure=40, bekle=2.0)
        log(f'  TUNEL: TERS KURTARMA - donmeden (w=0) girdigim izden dumduz geri ({icerde:.2f} m icerdeydim)')
        x0, y0 = self.poz(); bos_bas = None; t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 60:
            rclpy.spin_once(self, timeout_sec=0.0); L = self.lidar(); p = self.poz(); g = math.hypot(p[0] - x0, p[1] - y0)
            if not arka_acik(L) and self.yol(L, math.pi, w=R + 0.015) < 0.20:
                self.dur(yumusak=False); log('  TUNEL: geri cikarken arkam kapandi - bekliyorum'); return self.tunelde_sikistim(icerde - g)
            s, gg = self.yanlar_bos(L, 0.0)
            if not s and not gg:
                bos_bas = g if bos_bas is None else bos_bas
                if g - bos_bas > 0.15: break
            else: bos_bas = None
            if g >= icerde + 0.6: break
            self.surt(-0.08, 0.0); self.w = 0.0; time.sleep(0.05)            # w KILITLI 0
        self.dur(); self.istek.publish(String(data='serbest')); log('  TUNEL: tunelden geri ciktim'); return 'geri_cikti'

    def tunel(self, W):
        """TUNEL MODU: (W_r + 10 cm) <= W <= 1.2 m gecitte. Iki yan duvar CIZGI olarak olculur, orta cizginin yonu
        bir kez KILITLENIR; dinamik kacinma kapali. Yon sadece orta cizgiden sapmaya gore (olu bolge, sinirli, yumusak).
        SABIT HIZ 0.12 m/s (yan duvarlar hizi dusurmez). Cikis: iki yan bosalinca. Daralirsa: dur, dumduz geri cik.
        Donus: 'tamam' | 'daraldi' | 'engel'"""
        L = self.lidar(); k = koridor_olc(L, 0.0, 1.2)
        # TEK BASINA DURAN PANEL (tahta: 0.6-2 m duz parca) hangi yanda? Ayagi lidarin ALTINDA -> o yana 15 cm SANAL PAY
        buf_l = buf_r = 0.0
        for (m, u, boy, _a, _b, _k) in self.parcalar(L):
            if 0.6 <= boy <= 2.0 and -0.5 < m[0] < 1.5 and 0.2 < abs(m[1]) < 0.9:
                if m[1] > 0: buf_l = 0.15
                else: buf_r = 0.15
        if W - 2 * R - buf_l - buf_r < 0.10:
            log(f'TUNEL: gecit {W*100:.0f} cm ama panel ayagi payi ile ({(buf_l+buf_r)*100:.0f} cm) sigmiyorum - girmiyorum'); return 'geri_cikti'
        e_hedef = (buf_l - buf_r) / 2                                          # panelden uzak, duvara yakin gec
        # YON CAPASI: koridor = IKI YAN DUVAR; yoksa tunel YOK (acik alan). Robot eksene 15 dereceden fazla capraz -> iptal
        k_yan = koridor_olc(L, -0.5, 1.0)
        if not ((k['sol'] or k_yan['sol']) and (k['sag'] or k_yan['sag'])):
            log('TUNEL: iki yanimda duvar yok - bu koridor degil, acik alan (tunel iptal)'); return 'tamam'
        aci_k = k['aci'] if k['aci'] is not None else k_yan['aci']
        if aci_k is None or abs(aci_k) > math.radians(15):
            log(f'TUNEL: koridor eksenine {math.degrees(aci_k or 0):+.0f} derece caprazim (>15) - tunel iptal'); return 'tamam'
        th_kilit = self.th() + aci_k
        log(f'TUNEL MODU: gecit {W*100:.0f} cm - orta cizgi kilitlendi ({math.degrees(wrap(th_kilit - self.th())):+.1f} derece), sabit 0.12 m/s')
        x0, y0 = self.poz(); bos_bas = None; son = 0.0; ef = 0.0; t0 = time.time(); duvar_gordu = False
        while rclpy.ok() and time.time() - t0 < 120:
            rclpy.spin_once(self, timeout_sec=0.0); L = self.lidar(); px, py = self.poz(); gitti = math.hypot(px - x0, py - y0)
            if getattr(self, 'takildi_t', 0) > t0: log('  TUNEL: takildim - tunelden vazgectim'); return 'geri_cikti'
            if abs(wrap(self.th() - th_kilit)) > math.radians(15): self.dur(); log('  TUNEL: yonum koridor ekseninden 15 dereceden fazla saptı - tunel iptal'); return 'tamam'
            if gitti < 0.02 and time.time() - t0 > 2.0 and not (koridor_olc(L, -0.5, 0.8)['sol'] and koridor_olc(L, -0.5, 0.8)['sag']):
                self.dur(); self.hafiza_temizle(L, hepsi=True); log('  TUNEL: 2 sn ilerlemedim ve koridor dogrulanamadi - hayalet tunel, normal surus'); return 'tamam'
            Wn = self.dar_genislik(L, 0.0, 0.15, 0.8)
            if self.yol(L, 0.0, w=R + 0.015) < 0.25 or (Wn is not None and Wn < 2 * R + 0.10):
                sonuc = self.tunelde_sikistim(gitti)
                if sonuc == 'ileri': continue                                 # on acildi: tunelde devam
                return sonuc                                                  # 'geri_cikti'
            s, g = self.yanlar_bos(L, 0.0)
            if s and g: duvar_gordu = True; bos_bas = None                   # TUNEL = IKI YAN BIRLIKTE dolu
            elif duvar_gordu or gitti > 1.0:                                  # once iki yan dolu olmali; bir yan bosalinca
                bos_bas = gitti if bos_bas is None else bos_bas
                if gitti - bos_bas > 0.30: log(f'  TUNEL: ciktim ({gitti:.2f} m)'); return 'tamam'
            if time.time() > getattr(self, 'bitis', 1e18): log('  TUNEL: test suresi doldu'); self.dur(); return 'tamam'
            kk = koridor_olc(L, -0.4, 0.6)
            if kk['kayma'] is not None: e = kk['kayma']                        # + = sola gitmeli (iki duvar cizgisi)
            elif kk['sol'] is not None:                                       # TEK DUVAR TAKIBI: sol duvar referans
                e = (kk['sol'][1] - R) - (W - 2 * R) / 2
            elif kk['sag'] is not None:                                       # TEK DUVAR TAKIBI: sag duvar referans
                e = -((kk['sag'][1] - R) - (W - 2 * R) / 2)
            else:                                                             # cizgi yok (masa koseleri): sol/sag mesafe dengesi
                yb = L[(L[:, 0] > -0.3) & (L[:, 0] < 0.5) & (np.abs(L[:, 1]) < 0.9)]
                sl = np.sort(yb[yb[:, 1] > 0][:, 1])[:5]; sg = np.sort(-yb[yb[:, 1] < 0][:, 1])[:5]
                e = (float(np.median(sl)) - float(np.median(sg))) / 2 if len(sl) and len(sg) else 0.0
            e = e - e_hedef                                                   # panel ayagi payi: hedef kayik orta
            if abs(self.v) < 0.05: e = 0.0                                    # durgun/cok yavasken yanal duzeltme YOK (sadece yon kilidi)
            ef = 0.7 * ef + 0.3 * e                                           # olcum yumusatma
            if abs(ef) < 0.02: ef_k = 0.0
            else: ef_k = ef
            hedef = th_kilit + max(-0.08, min(0.08, 0.8 * ef_k))                # orta cizgiye en fazla ~4.5 derece sapma
            hata = wrap(hedef - self.th())
            w = 0.0 if abs(hata) < math.radians(1.5) else max(-0.05, min(0.05, 1.0 * hata))   # donus kilidi: sadece mikro duzeltme (en fazla 0.05 rad/s)
            if time.time() - son > 1.0:
                son = time.time(); log(f'    tunel: gittim {gitti:.2f} m | genislik {Wn*100 if Wn else 0:.0f} cm | hedef cizgiden sapma {e*100:+.0f} cm' + (f' | panel payi sol {buf_l*100:.0f} sag {buf_r*100:.0f} cm' if buf_l or buf_r else ''))
            on_t = self.yol(L, 0.0, w=R + 0.015)                              # ORANTILI HIZ: ani dur-kalk yok
            self.surt(0.12 * max(0.25, min(1.0, (on_t - 0.25) / (0.80 - 0.25))), w); time.sleep(0.05)
        self.dur(); return 'tamam'

    def calis(self, sure):
        log(f'DEVRIYE ({sure:.0f} sn): gez -> aralik gorunce gir -> dumduz gec, engel bitince dur -> dumduz geri -> cik -> gez')
        while (self.scan is None or self.odom is None) and rclpy.ok(): self.bekle(0.2)
        self.gecilen = []; self.cikmaz = []; bas = time.time(); self.bitis = bas + sure; secim = 0.0; secim_t = 0.0; son = 0.0; gecis = 0; bek_t = None
        while rclpy.ok() and time.time() - bas < sure:
            rclpy.spin_once(self, timeout_sec=0.0); P = self.lidar(); th = self.th()
            if time.time() - getattr(self, 'takildi_t', 0) < 0.3: secim_t = 0.0                    # takildiktan sonra yeni yon sec
            # 2026-10-08 kullanici kurali: GUVENLIK onumu kapattiysa ayni yere tekrar tekrar yonelme (masaya surtme) ->
            # arkam bossa DUMDUZ 30 cm geri, orayi cikmaz say (30 sn), baska bos yone; arkam da doluysa bekle
            if time.time() - getattr(self, 'guv_acil_t', 0) < 0.3 and getattr(self, 'guv_acil_yon', 0) > 0:
                self.guv_acil_t = 0.0; self.dur(); gx, gy = self.poz()
                self.cikmaz.append((gx + 0.6 * math.cos(th), gy + 0.6 * math.sin(th), time.time()))
                if self.yol(P, math.pi, w=R + 0.015) >= 0.70:
                    log('GEZ: GUVENLIK onumu kapatti - arkam bos, DUMDUZ 30 cm geri cikiyorum')
                    self.duz_git(-1, 0.30, lambda P_, g: False, 'guvenlik geri')
                else:
                    log('GEZ: GUVENLIK onumu kapatti, arkam da dolu - 1.5 sn bekliyorum'); self.bekle_dur(1.5)
                self.guv_acil_t = 0.0; secim_t = 0.0; continue
            self.hafiza_temizle(P)                                            # lidar bos goruyorsa hayalet kaydi sil
            # --- TAHTA KORIDORU gorunuyor mu? (360 derece lidar, uzaktan) ---
            k = self.tahta_koridoru(P) if ('--tahta' in sys.argv and time.time() - getattr(self, 'kor_t', 0) > 1.0) else None   # gidip-gelme gorevi istege bagli
            if k is not None: self.kor_t = time.time()
            x, y = self.poz(); c, s = math.cos(th), math.sin(th)
            if k is not None:
                gx, gy = x + c * k['giris'][0] - s * k['giris'][1], y + s * k['giris'][0] + c * k['giris'][1]
                if any(math.hypot(gx - hx, gy - hy) < 1.0 and time.time() - ht < 240 for hx, hy, ht in self.gecilen): k = None
            if k is not None:                                                 # sadece giris ONUMDEYSE ve icerde degilsem
                on_n = k['giris'] - 0.7 * k['ic_yon']; aci_g = math.atan2(on_n[1], on_n[0]); d_g = float(np.hypot(*on_n))
                s_d, g_d = self.yanlar_bos(P, 0.0)
                if abs(math.atan2(k['giris'][1], k['giris'][0])) > math.radians(60) or (s_d and g_d): k = None
                elif d_g > 0.15 and self.yol(P, aci_g, w=R + 0.015) < d_g - 0.05: k = None     # oraya giden yol kapali
            if k is not None:
                nok = (gx, gy)
                log(f"ARALIK BULDUM (tahta koridoru): {k['gen']*100:.0f} cm genis, {k['boy']:.1f} m uzun, girisi {np.hypot(*k['giris']):.1f} m uzakta")
                if not self.koridora_git(k):
                    self.gecilen.append((nok[0], nok[1], time.time())); secim_t = time.time(); continue
                for tur in range(3):                                       # videodaki gibi: aralikta 3 kez git-gel
                    log(f'  --- aralikta {tur + 1}/3. gidis-gelis ---')
                    if not self.gidip_gel(): break
                gecis += 1; log(f'  aralik tamam (toplam {gecis} aralik)')
                self.gecilen.append((nok[0], nok[1], time.time())); self.istek.publish(String(data='serbest'))
                b = self.bos_yon(self.lidar(), haric=0.0)
                if b: log(f'  bos tarafa cikiyorum ({math.degrees(b[1]):+.0f} derece)'); self.kavisle_don(b[1])
                secim_t = time.time(); continue
            # --- GEZ: dumduz; onu kapanirsa bos tarafa kavis, secimi 3 sn koru ---
            on = self.yol(P, 0.0, w=R + 0.05)
            qx, qy = x + 1.2 * math.cos(th), y + 1.2 * math.sin(th)
            if any(math.hypot(qx - cx, qy - cy) < 0.9 and time.time() - ct < 30 for cx, cy, ct in self.cikmaz + getattr(self, 'takilma', [])): on = 0.0   # cikmaza dogru gitme
            # INSAN YOLUMDA MI? (sadece gercekten yolumdaysa: on tarafta, yanal |y| < R + 30 cm, 1.5 m icinde)
            # SUZULEREK GEC: durma; SADECE INSAN icin yan pay 40 -> 15 cm; insanin KARSI tarafina oncelik, en genis taraftan
            # kavisle, hizi koruyarak (>= %70) gec. Sadece iki yan duvar-duvar kapaliysa dur ve 3 sn yol ver.
            ins = [(a, d) for a, d in self.insanlar() if math.cos(a) > 0.3 and abs(d * math.sin(a)) < R + 0.30
                   and GUV['insan_suzulme_min_m'] <= d < 1.5]                 # 2026-10-08: cok yakin insan = normal engel (suzulme yok)
            if ins and time.time() > getattr(self, 'suzul_bit', 0):
                ia, idist = min(ins, key=lambda i: i[1]); en = None
                for dd in range(-70, 71, 5):
                    a = math.radians(dd)
                    if abs(dd) < 10: continue
                    f = self.yol(P, a, w=R + 0.15)                            # insan icin daraltilmis pay: 15 cm
                    if f < min(1.0, idist + 0.5): continue
                    s = min(f, 2.5) - 0.6 * abs(a) + (0.4 if a * ia < 0 else 0.0)   # insanin karsi tarafi tercih
                    if en is None or s > en[0]: en = (s, a, f)
                if en is not None:
                    secim = en[1]; secim_t = time.time(); self.suzul_bit = time.time() + 3.0
                    self.son_taraf = math.copysign(1, secim); self.son_taraf_t = time.time()
                    log(f"GEZ: yolumda insan ({idist:.1f} m) - DURMADAN {'SOLUNDAN' if secim > 0 else 'SAGINDAN'} suzulup geciyorum ({math.degrees(secim):+.0f} derece, {en[2]:.1f} m acik)")
                else:
                    log(f'GEZ: yolumda insan ({idist:.1f} m), iki yani da kapali - durup 3 sn yol veriyorum'); self.dur(); t_y = time.time()
                    while time.time() - t_y < 3.0 and [i for i in self.insanlar() if math.cos(i[0]) > 0.3 and abs(i[1] * math.sin(i[0])) < R + 0.30 and i[1] < 1.5]: self.bekle(0.2)
                    self.suzul_bit = time.time() + 2.0; P = self.lidar(); on = self.yol(P, 0.0, w=R + 0.05)
                    if on < 0.9: secim_t = 0.0
            # IKI ENGEL ARASI GECIT (kosinus teoremi): onumde (+-20 derece) ya da onum kapandiysa (+-60) gecilir bir
            # bosluk varsa -> HUNI (kenarlara gore orta cizgi) -> TUNEL. Gecilmez bosluk o yone gitmeme sebebi.
            if time.time() > getattr(self, 'suzul_bit', 0) and time.time() - getattr(self, 'bosluk_t', 0) > 0.5:
                self.bosluk_t = time.time()
                bs = [b for b in self.bosluklar(P) if b['uz'] < 2.0]
                iyi = [b for b in bs if 2 * R + 0.10 <= b['W'] <= 1.2 and time.time() > getattr(self, 'huni_bekle', 0)
                       and abs(self.gecit_ekseni_kenar(b['sol'], b['sag'])[0]) < math.radians(45)
                       and abs(self.gecit_ekseni_kenar(b['sol'], b['sag'])[1]) < 0.5
                       and abs(b['hedef_aci']) < math.radians(20 if on >= 1.2 else 60)
                       and not any(math.hypot(x + math.cos(th) * b['sol'][0] - math.sin(th) * b['sol'][1] - hx, y + math.sin(th) * b['sol'][0] + math.cos(th) * b['sol'][1] - hy) < 0.8
                                   and time.time() - ht < 60 for hx, hy, ht in self.gecilen)]
                if iyi:
                    b = min(iyi, key=lambda b: abs(b['hedef_aci']))
                    log(f"GEZ: iki engel arasi gecit: W = {b['W']*100:.0f} cm (kosinus teoremi), {math.degrees(b['hedef_aci']):+.0f} derece, {b['uz']:.1f} m - GECILIR")
                    self.gecilen.append((x + math.cos(th) * b['sol'][0] - math.sin(th) * b['sol'][1], y + math.sin(th) * b['sol'][0] + math.cos(th) * b['sol'][1], time.time()))
                    if not self.huni(b['W'], kenar=(b['sol'], b['sag'])): self.huni_bekle = time.time() + 8.0
                    else:
                        sonuc = self.tunel(b['W'])
                        if sonuc == 'geri_cikti': log('  acik alanda 5 sn bekliyorum'); self.bekle_dur(5.0)
                    secim_t = time.time(); continue
                for b in bs:
                    if b['W'] < 2 * R + 0.10 and abs(b['hedef_aci']) < math.radians(20) and time.time() - getattr(self, 'dar_log', 0) > 5:
                        self.dar_log = time.time(); log(f"GEZ: onumdeki bosluk W = {b['W']*100:.0f} cm < {(2*R+0.10)*100:.0f} cm - GECILMEZ, girmiyorum")
            # DAR GECIT: onumdeki yol iki yandan daraliyorsa olc; robot + 10 cm'den darsa ASLA girme
            W = self.dar_genislik(P) if time.time() > getattr(self, 'suzul_bit', 0) else None   # suzulurken gecit/huni yok
            if W is not None and W < 2 * R + 0.10 and on > 0.3:
                if time.time() - son > 3: log(f'GEZ: onumdeki gecit {W*100:.0f} cm - robot+10 cm ({(2*R+0.10)*100:.0f} cm) yok, GIRMIYORUM'); son = time.time()
                self.cikmaz.append((qx, qy, time.time())); on = 0.0
            if W is not None and 2 * R + 0.10 <= W <= 1.2 and on >= 0.6:     # HUNI (on hizalama) -> TUNEL MODU
                s_i, g_i = self.yanlar_bos(P, 0.0, yari=0.30, gen=0.55)
                if not (s_i and g_i): W = None                                # SADECE zaten koridorun icindeysem tunel
            if W is not None and 2 * R + 0.10 <= W <= 1.2 and on >= 0.6:
                sonuc = self.tunel(W)
                if sonuc == 'geri_cikti':                                     # acik alanda bekle, sonra baska yone
                    log('  acik alanda 5 sn bekliyorum'); self.bekle_dur(5.0)
                    self.cikmaz.append((qx, qy, time.time()))
                secim_t = time.time() + (0.0 if sonuc == 'tamam' else 1.0); continue
            if on >= 0.9 and time.time() - secim_t > 3.0:
                k_h = self.hiz_carpani(P)
                v = HIZ * max(0.5, min(1.0, (on - 0.5) / 1.0))
                if time.time() - son > 3: log(f'GEZ: dumduz, onum {on:.1f} m bos' + (' (uyari bolgesi: yavas)' if k_h < 1 else '')); son = time.time()
                bek_t = None; self.surt(v * k_h, 0.0); time.sleep(0.05); continue
            if time.time() - secim_t > 3.0:
                b = self.bos_yon(P)
                if b is None:
                    arka = self.yol(P, math.pi, w=R + 0.015)
                    if arka > 0.6:
                        # SIKISTIM: cikmaza girdim. Dumduz geri, iki yanim BOSALANA kadar (koseden TAMAMEN cik), sonra
                        # girdigim yone DEGIL baska bos tarafa; bu cikmazi 2 dk hatirla (tekrar girme)
                        x, y = self.poz(); self.cikmaz.append((x + 0.8 * math.cos(th), y + 0.8 * math.sin(th), time.time()))
                        log('GEZ: SIKISTIM (onum ve yanlarim kapali) - kamera arkaya bakiyor, koseden tamamen cikana kadar DUMDUZ geri')
                        self.dur()
                        if abs(abs(self.tarete.get('aci', 0)) - 180) > 20: self.kamera_bak('arka', sure=30, bekle=2.5)
                        def disari(P, gitti):                                    # donup cikabilecegim bir yan acildi mi
                            return gitti >= 0.3 and self.bos_yon(P, haric=0.0) is not None
                        g = self.duz_git(-1, 2.0, disari, 'koseden cik'); self.istek.publish(String(data='serbest'))
                        b = self.bos_yon(self.lidar(), haric=0.0)
                        if b:
                            log(f'  koseden ciktim ({g:.2f} m) - girdigim yone degil, {math.degrees(b[1]):+.0f} derece tarafina donuyorum')
                            self.kavisle_don(b[1]); secim_t = time.time() + 2.0; secim = 0.0
                    else:
                        if time.time() - son > 5: log('GEZ: her yer kapali - bekliyorum'); son = time.time()
                        self.dur(); self.bekle(0.5); bek_t = bek_t or time.time()
                        if time.time() - bek_t > 3.0:                            # KILITLENME ZAMAN ASIMI: hafizayi sil, yeniden bak
                            log('  3 sn kipirdamadim - hafizayi temizleyip yeniden bakiyorum'); self.hafiza_temizle(P, hepsi=True); bek_t = None
                            if self.yol(self.lidar(), math.pi, w=R + 0.015) > 0.25:
                                log('  hala kapali - 10 cm geri gelip sensorlere yer aciyorum (w=0)'); self.duz_git(-1, 0.10, lambda P, g: False, 'mikro geri')
                        continue
                    continue
                secim = b[1]; secim_t = time.time(); log(f'GEZ: onum kapaniyor - {math.degrees(secim):+.0f} derece tarafina kavisle')
                if abs(secim) > math.radians(10): self.son_taraf = math.copysign(1, secim); self.son_taraf_t = time.time()
            # secilen tarafa kavis (ilerleyerek); secim ancak o yon GERCEKTEN kapanirsa (20 cm) ve 1 sn gectiyse bozulur
            if self.yol(P, secim, w=R + 0.015) < 0.2 and time.time() - secim_t > 1.0: secim_t = 0.0; self.dur(); continue
            k_h = self.hiz_carpani(P)
            if time.time() < getattr(self, 'suzul_bit', 0): k_h = max(k_h, 0.7)     # insanin yanindan suzulurken durma yok (GUVENLIK kapisi yine de durdurur)
            v_k = (0.10 if time.time() < getattr(self, 'suzul_bit', 0) else (AZ if abs(secim) > math.radians(30) else HIZ * 0.7))
            self.surt(v_k * k_h, max(-0.45, min(0.45, 1.2 * secim)))
            secim = wrap(secim - self.w * 0.05); time.sleep(0.05)              # don dukce secilen yon one gelir
        self.dur(); self.istek.publish(String(data='serbest')); log('DEVRIYE bitti')

if __name__ == '__main__':
    rclpy.init(); n = Devriye()
    try:
        if '--dene' in sys.argv:
            t = time.time()
            while time.time() - t < 4: rclpy.spin_once(n, timeout_sec=0.1)
            P = n.lidar(); n.gecilen = []
            print('on', round(n.yol(P, 0, w=R + 0.05), 2), 'arka', round(n.yol(P, math.pi, w=R + 0.015), 2), '| yanlar dolu (sol, sag):', n.yanlar_bos(P, 0.0))
            print('duz parcalar (uzunluk m):', [round(p[2], 2) for p in n.parcalar(P)])
            k = n.tahta_koridoru(P)
            print('TAHTA KORIDORU:', 'yok' if k is None else f"{k['gen']*100:.0f} cm genis, {k['boy']:.1f} m, giris {np.hypot(*k['giris']):.1f} m uzakta, {math.degrees(math.atan2(k['giris'][1], k['giris'][0])):+.0f} derece")
            b = n.bos_yon(P); print('bos yon:', None if b is None else (round(math.degrees(b[1])), round(b[2], 1)))
        else:
            n.calis(float(sys.argv[1]) if len(sys.argv) > 1 else 180.0)
    except KeyboardInterrupt:
        pass
    finally:
        n.dur(yumusak=False); n.destroy_node(); rclpy.shutdown()