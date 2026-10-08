"""HOME <-> MASA 1 TESTI (Cryvex gorev sistemi - 2026-10-08)

Kullanicinin istegi: HOME'da hizalan (arkasi duvara duz, kamera tam karsi) -> TAM KARSIDAKI kutuyu MASA 1 olarak kaydet ->
2 dk gezip haritayi cikar -> HOME'a don -> HOME'dan cik, Masa 1'e git, 10 sn siparis bekle -> HOME'a don.
Mevcut dosyalara DOKUNMAZ (devriye.py gezinme, home_hizala.py hizalama, haritala.py yardimcilari iceri alinir).

  * Masa 1 HOME'un TAM KARSISINDA: gidis DUMDUZ ileri (kutuyu lidarla hedef alir), donus DUMDUZ geri (kutuya bakarak) ->
    masa testinde yerinde donus YOK (kablo). Masanin 35 cm onunde (robotun on yuzu) durur.
  * Yolda onune biri cikarsa (kutudan once yeni bir sey belirirse) durur, gidene kadar bekler (en fazla 30 sn).
  * Haritalamadan sonra HOME'a donus: Nav2 sadece HOME'un 0.8 m onune goturur; son kisim lidarla geri yaklasma + hizalama.
Kullanim: python3 -u masa_testi.py [gezinti_sn=120]  |  --dene (hareketsiz: her seyi kontrol eder, hareket etmez)"""
import os, sys, time, math
sys.path.insert(0, os.path.expanduser('~/cryvex_araclar')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, rclpy, tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
import devriye as D
from basit_surus import log, R
import home_hizala as H
from haritala import api, servis, slam_acik, harita_pozu, yaml_yaz, HARITALAR

YAKLASMA = 0.35                                                               # masa ile robotun on yuzu arasi (m)
BEKLEME = 10.0                                                                # siparis bekleme (sn)

def masa_olc(L, aci=12, menzil=6.0):
    """tam onumdeki cisim (masa): (on yuzunun x'i [govde merkezinden], merkez y) ya da None"""
    a = np.degrees(np.arctan2(L[:, 1], L[:, 0])); r = np.hypot(L[:, 0], L[:, 1])
    s = (L[:, 0] > R + 0.05) & (r < menzil) & ((np.abs(a) < aci) | ((np.abs(L[:, 1]) < 0.40) & (L[:, 0] < 2.0)))
    on = L[s]
    if len(on) < 2: return None
    p = on[np.argmin(on[:, 0])]
    k = on[(on[:, 0] < p[0] + 0.25) & (np.abs(on[:, 1] - p[1]) < 0.40)]
    return (float(k[:, 0].min()), float(k[:, 1].mean())) if len(k) >= 2 else None

def masa_olc_kararli(n, sure=1.5):
    """birkac taramanin medyani (uzaktaki kutu her taramada gorunmeyebilir)"""
    ol = []; son = n.scan; t = time.time()
    while time.time() - t < sure:
        son = H.yeni_tarama(n, son); m = masa_olc(n.lidar())
        if m: ol.append(m)
    if len(ol) < 3: return None
    o = np.median(np.array(ol), 0); return float(o[0]), float(o[1])

def masaya_git(n):
    """DUMDUZ ileri, kutuyu hedef alarak; on yuz masaya YAKLASMA m kalinca dur. Basarili mi?"""
    hedef = R + YAKLASMA; son = n.scan; t0 = time.time(); onceki = None; kayip_t = None; bey = None
    log(f'  MASA 1 e gidiyorum (masanin {YAKLASMA * 100:.0f} cm onunde duracagim)'); n.kamera_bak(0, sure=40, bekle=0.5)
    while rclpy.ok() and time.time() - t0 < 120:
        son = H.yeni_tarama(n, son); L = n.lidar(); m = masa_olc(L)
        if m is None:                                                        # bir an goremedim: dumduz devam, 2 sn'yi gecerse dur
            kayip_t = kayip_t or time.time()
            if time.time() - kayip_t > 2.0: log('  masayi goremiyorum - durdum'); n.dur(); return False
            n.surt(min(n.v, 0.08), 0.0); continue
        kayip_t = None; x, y = m
        if onceki is not None and x < onceki - 0.25:                         # kutudan once yeni bir sey: onume biri cikti
            n.dur(); log(f'  onume bir sey cikti ({x:.2f} m) - gitmesini bekliyorum'); tb = time.time()
            while rclpy.ok() and time.time() - tb < 30:
                n.bekle_dur(0.5); m2 = masa_olc(n.lidar())
                if m2 and abs(m2[0] - onceki) < 0.15: break
            else: log('  30 sn gecti, onum hala kapali - durdum'); return False
            log('  onum acildi, devam'); son = n.scan; continue
        onceki = x
        if x <= hedef + 0.005: break
        bey = math.atan2(y, x)
        v = float(np.clip(0.5 * (x - hedef), 0.04, 0.18)); w = float(np.clip(0.8 * bey, -0.12, 0.12)) if x > hedef + 0.3 else 0.0
        n.surt(v, w)
    n.dur(); m = masa_olc_kararli(n, 1.0)
    log(f'  MASA 1 DEYIM: masaya {((m[0] - R) * 100 if m else 0):.0f} cm (hedef {YAKLASMA * 100:.0f})')
    return m is not None

def home_geri(n):
    """masadan HOME'a DUMDUZ geri (kutuya bakarak yon tutar); arka duvara ~0.7 m kalinca hizalama devralir"""
    hedef = H.hedef_mesafe() + 0.25; son = n.scan; t0 = time.time(); onceki = None
    log('  HOME a dumduz geri donuyorum')
    while rclpy.ok() and time.time() - t0 < 120:
        son = H.yeni_tarama(n, son); L = n.lidar(); ar = H.arka_bosluk(L, yari=0.32)
        if onceki is not None and ar < onceki - 0.25:                        # arkama biri girdi
            n.dur(); log(f'  arkama bir sey girdi ({ar:.2f} m) - gitmesini bekliyorum'); tb = time.time()
            while rclpy.ok() and time.time() - tb < 30:
                n.bekle_dur(0.5)
                if abs(H.arka_bosluk(n.lidar(), yari=0.32) - onceki) < 0.15: break
            else: log('  30 sn gecti, arkam hala kapali - durdum'); return False
            son = n.scan; continue
        onceki = ar
        if ar <= hedef: break
        m = masa_olc(L); w = float(np.clip(0.8 * math.atan2(m[1], m[0]), -0.10, 0.10)) if m else 0.0
        n.surt(-float(np.clip(0.5 * (ar - hedef), 0.04, 0.15)), w)
    n.dur(); ok, hata, ar = H.hizala(n, izin=True)
    return ok

def nav2_git(n, nav, gx, gy, yaw, sure=180):
    """tek Nav2 hedefi; Donus: basarili mi"""
    from nav2_simple_commander.robot_navigator import TaskResult
    g = PoseStamped(); g.header.frame_id = 'map'; g.header.stamp = nav.get_clock().now().to_msg()
    g.pose.position.x = gx; g.pose.position.y = gy; g.pose.orientation.z = math.sin(yaw / 2); g.pose.orientation.w = math.cos(yaw / 2)
    if not nav.goToPose(g): return False
    t = time.time()
    while not nav.isTaskComplete():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t > sure: nav.cancelTask(); log('  Nav2 zaman asimi'); return False
    return nav.getResult() == TaskResult.SUCCEEDED

def home_a_nav2(n, home, ileri=0.8, adim=4.5):
    """Nav2 ile HOME'un 'ileri' m onune (yuzu masaya) git. Nav2 global haritasi robot merkezli 12x12 m kayan pencere ->
    hedef 5 m'den uzaksa 4.5 m'lik ARA NOKTALARLA (ara nokta engeldeyse yanlari denenir)."""
    from nav2_simple_commander.robot_navigator import BasicNavigator
    nav = BasicNavigator(); x, y, yaw = home; hx, hy = x + ileri * math.cos(yaw), y + ileri * math.sin(yaw)
    try:
        for tur in range(8):
            p = harita_pozu(n, n._tfb, 10.0)
            if p is None: log('  harita konumum yok'); return False
            d = math.hypot(hx - p[0], hy - p[1])
            if d <= 5.0:
                log(f'  HOME a donuyorum (Nav2): HOME un {ileri:.1f} m onune ({d:.1f} m)')
                return nav2_git(n, nav, hx, hy, yaw) or yakinda(n, home, ileri)
            yon = math.atan2(hy - p[1], hx - p[0]); ok = False
            for yan in (0.0, 0.6, -0.6, 1.2, -1.2):                         # ara nokta engelde/planlanamazsa yanlari
                ax = p[0] + adim * math.cos(yon) - yan * math.sin(yon); ay = p[1] + adim * math.sin(yon) + yan * math.cos(yon)
                log(f'  HOME {d:.1f} m uzakta - ara nokta ({ax:+.1f}, {ay:+.1f}) ye gidiyorum')
                if nav2_git(n, nav, ax, ay, yon): ok = True; break
            if not ok: log('  ara noktaya gidemedim'); return False
        return False
    finally:
        nav.destroy_node()

def yakinda(n, home, ileri, tol=0.25):
    """Nav2 yonu tutturamasa da (yerinde donus kapali) KONUM yeterince yakinsa olur: yonu lidarla hizalama duzeltir"""
    tfb = getattr(n, '_tfb', None); p = harita_pozu(n, tfb, 5.0) if tfb else None
    if p is None: return False
    d = math.hypot(p[0] - (home[0] + ileri * math.cos(home[2])), p[1] - (home[1] + ileri * math.sin(home[2])))
    log(f'  HOME onune {d * 100:.0f} cm (yon farki {math.degrees(math.atan2(math.sin(p[2] - home[2]), math.cos(p[2] - home[2]))):+.0f} derece)')
    return d < tol

def yaml_oku():
    """waypoints.yaml (yaml_yaz bicimi) -> dict"""
    from haritala import NOKTALAR
    import json
    v = {}; bol = None
    for satir in open(NOKTALAR):
        if satir.startswith('#') or not satir.strip(): continue
        if not satir.startswith(' '): bol = satir.strip().rstrip(':'); v[bol] = {}; continue
        k, d = satir.strip().split(': ', 1); v[bol][k] = json.loads(d)
    return v

def home_beklenen(n, home):
    """haritaya gore HOME yonune gelmek icin gereken donus (rad) - arka duvari dogru yonde aramak icin"""
    p = harita_pozu(n, n._tfb, 5.0)
    return None if p is None else math.atan2(math.sin(home[2] - p[2]), math.cos(home[2] - p[2]))

def don_haritayla(n, yaw_hedef, tol=math.radians(2), sure=60, izin=False):
    """HARITA (SLAM) yonune bakarak yerinde don (teker odometrisi degil). Kablo: 45 dereceden buyukse izin (kullanici
    'basla' ile gorevi onayladi) + 10 sn bekleme olmadan donmez."""
    p = harita_pozu(n, n._tfb, 5.0)
    if p is None: return False
    fark = math.atan2(math.sin(yaw_hedef - p[2]), math.cos(yaw_hedef - p[2]))
    if abs(fark) > math.radians(45):
        if not izin: log(f'  {math.degrees(fark):+.0f} derece donmem lazim - KABLO kurali, sormadan donmuyorum'); return False
        log(f'  {math.degrees(fark):+.0f} derece buyuk donus (gorev onayli) - KABLO icin 10 sn bekliyorum'); n.bekle_dur(10.0)
    if not n.donus_guvenli(n.lidar(), fark, sinir=0.40, pay=math.radians(10)): log('  donersem teker bir seye supurur - donmuyorum'); return False
    log(f'  haritaya gore {math.degrees(fark):+.1f} derece donuyorum'); t0 = time.time()
    while rclpy.ok() and time.time() - t0 < sure:
        p = harita_pozu(n, n._tfb, 1.0)
        if p is None: break
        fark = math.atan2(math.sin(yaw_hedef - p[2]), math.cos(yaw_hedef - p[2]))
        if abs(fark) < tol: break
        if n.yakin_yan_dolu(n.lidar()): log('  govdeye cok yakin bir sey - durdum'); break
        tw = Twist(); tw.angular.z = math.copysign(0.15 if abs(fark) > math.radians(8) else 0.06, fark); n.cmd.publish(tw)
    for _ in range(5): n.cmd.publish(Twist()); time.sleep(0.05)
    n.bekle(0.8); return True

def home_yerles(n, home):
    """HOME onundeyken: once HARITAYA gore HOME yonune don, sonra arka duvar (artik tam arkada) lidarla ince ayar,
    dumduz geri + son hizalama"""
    if not don_haritayla(n, home[2], izin=True): return False
    ok, _h, _a = H.hizala(n, izin=False, beklenen=0.0)
    return home_geri(n) if ok else False

def main():
    dene = '--dene' in sys.argv
    arg = [a for a in sys.argv[1:] if not a.startswith('--')]
    gez = float(arg[0]) if arg else 120.0; ad = time.strftime('kafe_%Y%m%d_%H%M')
    rclpy.init(); n = D.Devriye(); tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n); n._tfb = tfb
    try:
        while (n.scan is None or n.odom is None) and rclpy.ok(): n.bekle(0.2)
        if '--devam' in sys.argv:                                            # haritalama bitti: HOME'a yerles + Masa 1 gorevi
            v = yaml_oku(); home = tuple(v['HOME']['poz'])
            log(f'DEVAM: HOME {home} - HOME a yerlesip Masa 1 gorevi')
            if dene:
                yakinda(n, home, 0.8); b = home_beklenen(n, home)
                log(f'  [dene] haritaya gore HOME yonu icin {math.degrees(b):+.1f} derece donus')
                d = H.arka_duvar(n, n.lidar(), boy_min=0.4, yakin=b, yakin_tol=35)
                log(f'  [dene] en yakin duvar: {None if d is None else f"{math.degrees(d[0]):+.1f} derece donus, {d[1]:.2f} m"}'); return
            if arg:                                                          # --devam <sn>: once bulundugum yerden gez (harita gunceller)
                log(f'  once {gez:.0f} sn gezinti (harita guncellenir, SLAM sifirlanmaz - HOME ayni kalir)'); n.calis(gez); n.dur()
            if not yakinda(n, home, 0.8) and not home_a_nav2(n, home): log('  HATA: HOME onune gidemedim - durdum'); return
            log('  10 sn sonra donuyorum (kablo)'); time.sleep(10)
            if not home_yerles(n, home): log('  HATA: HOME a yerlesemedim - durdum'); return
            log('  HOME DAYIM (hizali).')
            if '--sadece-home' in sys.argv: log('  --sadece-home: burada bitiyor'); return
            log('  Simdi Masa 1 gorevi.')
            if not masaya_git(n): return
            log(f'  SIPARIS BEKLIYORUM ({BEKLEME:.0f} sn)'); n.bekle_dur(BEKLEME)
            if not home_geri(n): log('  HATA: HOME a yerlesemedim - durdum'); return
            log('  GOREV TAMAM: HOME -> Masa 1 -> HOME'); return
        log(f'MASA TESTI: hizala -> Masa 1 kaydet -> {gez:.0f} sn harita -> HOME -> Masa 1 ({BEKLEME:.0f} sn) -> HOME')
        if not slam_acik(): log('  SLAM baslatiliyor: ' + str(api('/api/start_mapping'))); time.sleep(15)
        # 1) HOME hizalama
        if dene:
            d = H.arka_duvar(n, n.lidar()); log(f'  [dene] arka duvar: {None if d is None else f"{math.degrees(d[0]):+.1f} derece, {d[1]:.2f} m"}')
        else:
            log('  10 sn sonra basliyorum (kablo)'); time.sleep(10)
            ok, _h, _a = H.hizala(n, izin=True)
            if not ok: log('  HATA: HOME hizalanamadi - durdum'); return
            servis('/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'); time.sleep(4)   # harita HOME'dan baslasin
        home = harita_pozu(n, tfb, 30.0)
        if home is None: log('  HATA: harita konumu yok - durdum'); return
        log(f'  HOME: x {home[0]:+.2f}, y {home[1]:+.2f}, yon {math.degrees(home[2]):+.1f} derece')
        # 2) Masa 1 = tam karsidaki cisim
        m = masa_olc_kararli(n, 2.0)
        if m is None: log('  HATA: tam karsimda masa (kutu) goremiyorum - durdum'); return
        c, s_ = math.cos(home[2]), math.sin(home[2]); yuz = (home[0] + c * m[0] - s_ * m[1], home[1] + s_ * m[0] + c * m[1])
        dur = m[0] - R - YAKLASMA; durak = (home[0] + c * dur, home[1] + s_ * dur, home[2])
        log(f'  MASA 1: tam karsimda {m[0] - R:.2f} m (on yuzumden), {m[1] * 100:+.0f} cm yanda; {dur:.2f} m gidip duracagim')
        if not (0.3 < dur < 6.0 and abs(m[1]) < 0.5): log('  HATA: masa konumu mantiksiz - durdum'); return
        try:
            import nav2_simple_commander.robot_navigator  # noqa
            log('  Nav2 komutani hazir')
        except Exception as e: log(f'  HATA: nav2_simple_commander yok: {e}'); return
        if dene: log('  --dene: hareket yok, burada bitiyor'); return
        # 3) 2 dk haritalama
        n.calis(gez); n.dur()
        os.makedirs(HARITALAR, exist_ok=True); taban = os.path.join(HARITALAR, ad)
        o1 = servis('/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph', f'{{filename: "{taban}"}}', 60)
        o2 = servis('/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap', f'{{name: {{data: "{taban}"}}}}', 60)
        log(f"  harita kaydi: serialize {'TAMAM' if 'result=0' in o1.replace(' ', '') else 'HATA'}, pgm {'TAMAM' if 'result=0' in o2.replace(' ', '') else 'HATA'}")
        yaml_yaz({'harita': {'ad': ad, 'yol': taban},
                  'HOME': {'poz': [round(v, 3) for v in home], 'hizalandi': True, 'arka_duvar_m': H.hedef_mesafe()},
                  'MASA_1': {'yuz': [round(v, 3) for v in yuz], 'yaklasma_pozu': [round(v, 3) for v in durak], 'yaklasma_m': YAKLASMA}})
        # 4) HOME'a don
        if not home_a_nav2(n, home): log('  HATA: Nav2 HOME onune goturemedi - durdum (uygulamadan elle getirebilirsin)'); return
        if not home_yerles(n, home): log('  HATA: HOME a yerlesemedim - durdum'); return
        log('  HOME DAYIM (hizali). Simdi Masa 1 gorevi.')
        # 5) HOME -> Masa 1 -> 10 sn -> HOME
        if not masaya_git(n): return
        log(f'  SIPARIS BEKLIYORUM ({BEKLEME:.0f} sn)'); n.bekle_dur(BEKLEME)
        if not home_geri(n): log('  HATA: HOME a yerlesemedim - durdum'); return
        log('  GOREV TAMAM: HOME -> Masa 1 -> HOME')
    except KeyboardInterrupt:
        pass
    finally:
        n.dur(yumusak=False); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
