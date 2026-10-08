"""OTONOM HARITALAMA + HOME KAYDI (Cryvex gorev sistemi, Asama 6 - 2026-10-08)

Mevcut sisteme DOKUNMAZ: devriye.py'yi (bugunku gezinme davranisi: tunel, huni, kosinus teoremi, insan suzulme,
yumusak fren, sifir donus) ICERI ALIP kullanir; hicbir mevcut dosyayi degistirmez.

Akis:
  1) SLAM'i sifirdan baslat (uygulamanin /api/start_mapping'i; zaten aciksa /slam_toolbox/reset)
  2) map -> base_footprint bekle; robotun BASLADIGI yer = HOME (barmen) adayi (x, y, yaw)
  3) devriye.py ile 'sure' sn gez (harita gezerken olusur)
  4) haritayi KALICI kaydet: slam_toolbox serialize_map (yeniden yuklenebilir konum grafigi) + save_map (pgm/yaml)
  5) HOME'u ~/cryvex_veri/waypoints.yaml'a yaz (ince hizalama Asama 8'de: home_hizala.py)
Kullanim: python3 -u haritala.py [sure_sn=240] [harita_adi=kafe_YYYYMMDD_HHMM]   |  --dene (hareketsiz: sadece kontrol)"""
import os, sys, time, math, json, subprocess, urllib.request
sys.path.insert(0, os.path.expanduser('~/cryvex_araclar'))
import rclpy, tf2_ros
import devriye as D                                                      # mevcut gezinme davranisi (DEGISTIRILMEZ)
from basit_surus import log

VERI = os.path.expanduser('~/cryvex_veri'); HARITALAR = os.path.join(VERI, 'haritalar')
NOKTALAR = os.path.join(VERI, 'waypoints.yaml')

def api(yol):
    req = urllib.request.Request('http://127.0.0.1:8080' + yol, data=json.dumps({'password': '1234'}).encode(),
                                 headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=30).read())

def servis(ad, tip, istek, sure=30):
    r = subprocess.run(['ros2', 'service', 'call', ad, tip, istek], capture_output=True, text=True, timeout=sure)
    return r.stdout

def slam_acik():
    try: r = subprocess.run(['ros2', 'lifecycle', 'get', '/slam_toolbox'], capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired: return False
    return r.stdout.strip().startswith('active')

def harita_pozu(node, tfb, bekle=20.0):
    """map -> base_footprint (x, y, yaw) ya da None"""
    t = time.time()
    while time.time() - t < bekle:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tr = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p, q = tr.transform.translation, tr.transform.rotation
            return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            pass
    return None

def yaml_yaz(veri):
    """waypoints.yaml (basit, elle okunabilir; mevcut uygulamanin waypoints.json'una DOKUNMAZ)"""
    os.makedirs(VERI, exist_ok=True)
    with open(NOKTALAR, 'w') as f:
        f.write('# Cryvex gorev noktalari (gorev sistemi). Uygulamanin config/waypoints.json dosyasindan AYRIDIR.\n')
        for ad, d in veri.items():
            f.write(f'{ad}:\n')
            for k, v in d.items(): f.write(f'  {k}: {json.dumps(v)}\n')

def main():
    dene = '--dene' in sys.argv
    arg = [a for a in sys.argv[1:] if not a.startswith('--')]
    sure = float(arg[0]) if arg else 240.0
    ad = arg[1] if len(arg) > 1 else time.strftime('kafe_%Y%m%d_%H%M')
    rclpy.init(); n = D.Devriye(); tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
    try:
        log(f'HARITALAMA: {sure:.0f} sn otonom gezinti, harita adi "{ad}"')
        if not slam_acik():
            log('  SLAM baslatiliyor: ' + str(api('/api/start_mapping'))); time.sleep(15)   # yeni acilan SLAM zaten bos harita
        elif not dene:
            log('  SLAM zaten acik - haritayi sifirliyorum'); servis('/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'); time.sleep(3)
        duvar = None
        if not dene and '--hizalama-yok' not in sys.argv:                    # once arkam en yakin duvara duz, kamera tam karsi
            import home_hizala
            ok, hata, mes = home_hizala.hizala(n, izin='--izin' in sys.argv)
            if ok: duvar = {'hata_derece': round(hata, 2), 'arka_duvar_m': round(mes, 3)}
            if not ok: log('  HOME HIZALAMA tamamlanamadi - yamuk HOME kaydetmiyorum, durdum'); return
            if ok and slam_acik(): servis('/slam_toolbox/reset', 'slam_toolbox/srv/Reset', '{}'); time.sleep(3)   # harita hizali yonden baslasin
        p = harita_pozu(n, tfb, 40.0)
        if p is None: log('  HATA: harita konumu yok (map -> base_footprint) - durdum'); return
        log(f'  HOME (barmen) adayi: x {p[0]:+.2f}, y {p[1]:+.2f}, yon {math.degrees(p[2]):+.1f} derece')
        if dene: log('  --dene: hareket yok, burada bitiyor'); return
        n.calis(sure)                                                        # mevcut gezinme davranisi
        n.dur()
        os.makedirs(HARITALAR, exist_ok=True); taban = os.path.join(HARITALAR, ad)
        log('  harita kaydediliyor (konum grafigi + pgm/yaml)...')
        o1 = servis('/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph', f'{{filename: "{taban}"}}', 60)
        o2 = servis('/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap', f'{{name: {{data: "{taban}"}}}}', 60)
        log(f"  serialize: {'TAMAM' if 'result=0' in o1.replace(' ', '') else 'HATA ' + o1[-120:]}")
        log(f"  save_map:  {'TAMAM' if 'result=0' in o2.replace(' ', '') else 'HATA ' + o2[-120:]}")
        yaml_yaz({'harita': {'ad': ad, 'yol': taban},
                  'HOME': {'poz': [round(p[0], 3), round(p[1], 3), round(p[2], 4)], 'hizalandi': duvar is not None,
                           'arka_duvar': duvar}})
        log(f'  HOME ve harita {NOKTALAR} dosyasina yazildi. Bitti.')
    except KeyboardInterrupt:
        pass
    finally:
        n.dur(yumusak=False); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
