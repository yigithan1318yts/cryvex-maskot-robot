"""HOME HIZALAMA (Cryvex gorev sistemi, Asama 8'in ilk hali - 2026-10-08)

Robotun ARKASI en yakin duz duvara PARALEL (sirti duvara dik bakar), ONU bos tarafa duz, KAMERA tam karsiya (0 derece),
arka duvara REFERANS mesafede (kullanicinin elle koydugu poz: ~/cryvex_veri/pozlar/home_referans.json, 0.44 m).
Mevcut dosyalara DOKUNMAZ; devriye.py'nin lidar / duz parca / teker supurme fonksiyonlarini kullanir.

  1) Her yonde, 3 m icinde, >= 60 cm DUZ duvarlardan EN YAKINI (ayni duvarin parcalari birlestirilir)
  2) O duvar tam arkada kalsin diye donus (KABLO: > 45 derece ise --izin gerekir)
     DONUS LIDARLA KAPALI DONGU: teker odometrisine guvenilmez (step adim kacirir) - her taramada duvarin acisi
     yeniden olculur, hata < 0.7 derece olunca durur (uzakta 0.20, yakinda 0.06 rad/s)
  3) Arka duvara referans mesafeye kadar DUMDUZ yavas geri (0.05 m/s, w = 0), en fazla 1 m
  4) Kamera tareti 0 dereceye (tam karsi)
Kullanim: python3 -u home_hizala.py [--izin]   |  --dene (hareketsiz: duvari ve gereken donusu yazar)"""
import os, sys, time, math, json
sys.path.insert(0, os.path.expanduser('~/cryvex_araclar'))
import numpy as np, rclpy
from geometry_msgs.msg import Twist
import devriye as D
from basit_surus import log, wrap

BUYUK_DONUS = math.radians(45)                                                # kablo kurali
REFERANS = os.path.expanduser('~/cryvex_veri/pozlar/home_referans.json')

def hedef_mesafe(varsayilan=0.44):
    try: return float(json.load(open(REFERANS))['arka_duvar_m'])
    except Exception: return varsayilan

def arka_duvar(n, L, menzil=3.0, boy_min=0.6, yakin=None, yakin_tol=15):
    """en yakin duz duvar: (donus_rad [duvar tam arkada kalsin diye], duvara_mesafe_m, boy_m) ya da None.
    Ayni duvarin parcalari (yon +-15, mesafe +-0.12 m) boy agirlikli birlestirilir. 'yakin': beklenen donus (takip icin)"""
    ps = []
    for (m, u, boy, _a, _b, kal) in n.parcalar(L):
        if kal > 0.04: continue
        nrm = np.array([-u[1], u[0]])
        if nrm @ m < 0: nrm = -nrm                                            # normal duvara dogru
        mesafe = float(nrm @ m)
        if mesafe > menzil: continue
        ps.append((wrap(math.atan2(nrm[1], nrm[0]) - math.pi), mesafe, float(boy)))   # duvar 180 derecede olsun
    if yakin is not None: ps = [p for p in ps if abs(wrap(p[0] - yakin)) < math.radians(yakin_tol)]
    buyuk = [p for p in ps if p[2] >= boy_min]
    if not buyuk: return None
    ana = min(buyuk, key=lambda p: p[1])
    es = [p for p in ps if abs(wrap(p[0] - ana[0])) < math.radians(15) and abs(p[1] - ana[1]) < 0.12]
    w = np.array([p[2] for p in es])
    aci = ana[0] + float(np.sum(w * np.array([wrap(p[0] - ana[0]) for p in es])) / w.sum())
    return wrap(aci), float(np.sum(w * np.array([p[1] for p in es])) / w.sum()), float(w.sum())

def on_bosluk(L, yari=0.25):
    """tam onumde (govde genisliginde) en yakin engel (m)"""
    on = L[(L[:, 0] > 0) & (np.abs(L[:, 1]) < yari)]
    return float(on[:, 0].min()) if len(on) else 8.0

def arka_bosluk(L, yari=0.25):
    ark = L[(L[:, 0] < 0) & (np.abs(L[:, 1]) < yari)]
    return float(-ark[:, 0].max()) if len(ark) else 8.0

def yeni_tarama(n, son):
    """yeni bir lidar taramasi gelene kadar spin (ayni taramayi iki kez olcmeyelim)"""
    t = time.time()
    while rclpy.ok() and n.scan is son and time.time() - t < 1.0: rclpy.spin_once(n, timeout_sec=0.02)
    return n.scan

def don_lidarla(n, tolerans, izlenen=None, boy_min=0.6):
    """duvarin acisini HER TARAMADA olcerek don; teker odometrisi kullanilmaz. Donus: kalan hata (rad) ya da None.
    izlenen: takip edilen duvarin son bilinen donus acisi (baska duvara atlamasin)"""
    son = n.scan; t0 = time.time(); kayip = 0
    while rclpy.ok() and time.time() - t0 < 30:
        son = yeni_tarama(n, son); L = n.lidar(ham=True)
        d = arka_duvar(n, L, boy_min=boy_min, yakin=izlenen)
        if d is None:
            kayip += 1
            if kayip > 4: log('  HOME HIZALAMA: duvari kaybettim - durdum'); break
            continue
        kayip = 0; izlenen = d[0]; hata = d[0]
        if abs(hata) < tolerans: break
        if n.yakin_yan_dolu(L): log('  HOME HIZALAMA: govdeye cok yakin bir sey - durdum'); break
        hiz = 0.20 if abs(hata) > math.radians(8) else 0.10 if abs(hata) > math.radians(3) else 0.06
        tw = Twist(); tw.angular.z = math.copysign(hiz, hata); n.cmd.publish(tw)
    for _ in range(5): n.cmd.publish(Twist()); time.sleep(0.05)
    n.bekle(0.8); d = arka_duvar(n, n.lidar(ham=True), yakin=izlenen)
    return None if d is None else d[0]

def geri_yaklas(n, hedef, en_fazla=1.0):
    """arka duvara 'hedef' m kalana kadar DUMDUZ yavas geri (w = 0); lidar ile olcer"""
    L = n.lidar(ham=True); bas = arka_bosluk(L)
    if bas <= hedef + 0.01: return bas
    if bas - hedef > en_fazla: log(f'  HOME HIZALAMA: duvar {bas:.2f} m - {en_fazla:.1f} m den fazla geri gitmem, burada kaliyorum'); return bas
    log(f'  HOME HIZALAMA: arka duvar {bas:.2f} m, {hedef:.2f} m ye kadar dumduz geri geliyorum')
    son = n.scan; t0 = time.time()
    while rclpy.ok() and time.time() - t0 < (bas - hedef) / 0.05 * 2 + 3:
        son = yeni_tarama(n, son); L = n.lidar(ham=True); ar = arka_bosluk(L)
        if ar <= hedef + 0.005 or ar < 0.36: break
        tw = Twist(); tw.linear.x = -(0.05 if ar - hedef > 0.08 else 0.03); n.cmd.publish(tw)
    for _ in range(5): n.cmd.publish(Twist()); time.sleep(0.05)
    n.bekle(0.6); return arka_bosluk(n.lidar(ham=True))

def hizala(n, tolerans_der=0.7, izin=False, beklenen=None):
    """arkam en yakin duvara paralel + referans mesafe + kamera tam karsi. Donus: (basarili_mi, kalan_hata_der, arka_duvar_m)
    beklenen: haritadan bilinen gereken donus (rad) - verilirse SADECE o yondeki (+-35) duvar, kisa parca da (>= 40 cm) olur"""
    while (n.scan is None or n.odom is None) and rclpy.ok(): n.bekle(0.2)
    n.bekle(0.6); L = n.lidar(ham=True)
    d = arka_duvar(n, L) if beklenen is None else arka_duvar(n, L, boy_min=0.4, yakin=beklenen, yakin_tol=35)
    if d is None: log('  HOME HIZALAMA: 3 m icinde duz duvar bulamadim (>= 60 cm) - hizalamadan devam'); n.kamera_bak(0, sure=40, bekle=2.0); return False, None, None
    donus, mesafe, boy = d; tol = math.radians(tolerans_der)
    log(f'  HOME HIZALAMA: en yakin duvar {boy:.2f} m, {mesafe:.2f} m uzakta; arkam ona duz olsun diye {math.degrees(donus):+.1f} derece donmeliyim')
    hata = donus
    if abs(donus) >= tol:
        if abs(donus) > BUYUK_DONUS and not izin: log('  HOME HIZALAMA: 45 dereceden buyuk donus - KABLO kurali, izinsiz donmuyorum (--izin)')
        elif not n.donus_guvenli(L, donus, sinir=0.40, pay=math.radians(10)): log('  HOME HIZALAMA: donersem teker bir seye supurur - donmuyorum')
        else: hata = don_lidarla(n, tol, izlenen=donus, boy_min=0.4 if beklenen is not None else 0.6)
    if hata is not None and abs(hata) < tol * 1.5:
        ar = geri_yaklas(n, hedef_mesafe())
        h2 = arka_duvar(n, n.lidar(ham=True), boy_min=0.4, yakin=0.0)                 # geri gelirken yamulduysa bir kez daha duzelt (ayni duvar)
        if h2 is not None and abs(h2[0]) >= tol: hata = don_lidarla(n, tol, izlenen=h2[0], boy_min=0.4)
        else: hata = None if h2 is None else h2[0]
    n.kamera_bak(0, sure=40, bekle=2.0)                                       # kamera tam karsiya
    L = n.lidar(ham=True); ar = arka_bosluk(L)
    ok = hata is not None and abs(hata) < tol * 1.5
    log(f'  HOME HIZALAMA {"TAMAM" if ok else "EKSIK"}: arka duvara {"?" if hata is None else f"{math.degrees(hata):+.2f}"} derece, '
        f'arka {ar:.2f} m (hedef {hedef_mesafe():.2f}), onum {on_bosluk(L):.2f} m bos, kamera 0 derece')
    return ok, (None if hata is None else math.degrees(hata)), ar

if __name__ == '__main__':
    rclpy.init(); n = D.Devriye()
    try:
        if '--dene' in sys.argv:
            t = time.time()
            while time.time() - t < 4: rclpy.spin_once(n, timeout_sec=0.1)
            L = n.lidar(ham=True); d = arka_duvar(n, L)
            print('en yakin duvar:', None if d is None else f'{d[1]:.2f} m uzakta, {d[2]:.2f} m uzun; arkam ona duz olsun diye {math.degrees(d[0]):+.1f} derece donmeliyim')
            if d: print('donus guvenli mi:', n.donus_guvenli(L, d[0], sinir=0.40, pay=math.radians(10)), '| 45 dereceden buyuk mu:', abs(d[0]) > BUYUK_DONUS)
            print(f'onum {on_bosluk(L):.2f} m bos, arkam {arka_bosluk(L):.2f} m (hedef {hedef_mesafe():.2f}), kamera tareti {n.tarete.get("aci", "?")} derece')
        else:
            print(hizala(n, izin='--izin' in sys.argv))
    except KeyboardInterrupt:
        pass
    finally:
        n.dur(yumusak=False); n.destroy_node(); rclpy.shutdown()
