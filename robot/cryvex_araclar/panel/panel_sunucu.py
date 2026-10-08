"""CRYVEX GOREV PANELI sunucusu (2026-10-08) - http://<robot>:8090/

Mevcut uygulama sunucusuna (cafe_ui_server, :8080) DOKUNMAZ; ayri, kucuk bir sunucu. Gorev betiklerini (gorev/*.py,
devriye.py) baslatir/durdurur, durumlarini ve loglarini verir, guvenlik.json'u gosterir/duzenler.
Canli kamera algilama.py'den (:8081/akis), canli harita uygulamadan (:8080/api/live_map.png) dogrudan gelir.

KURALLAR (sahibin kurallari):
  * Hareket baslatmak = 'basla': sifre + 'kablo/etraf kontrol edildi' onayi ister.
  * Robotta zaten bir hareket programi calisiyorsa (baska oturum/arkadas dahil) YENISI BASLAMAZ. ~/HAREKET_KILIDI yazilir.
  * DUR sifresiz: tum hareket programlarini kapatir + 2 sn sifir /cmd_vel.
Calistirma: python3 -u panel_sunucu.py   (otomatik: crontab @reboot)"""
import os, sys, json, time, signal, subprocess, re
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

EV = os.path.expanduser('~'); ARAC = os.path.join(EV, 'cryvex_araclar'); GOREV = os.path.join(ARAC, 'gorev')
VERI = os.path.join(EV, 'cryvex_veri'); KILIT = os.path.join(EV, 'HAREKET_KILIDI')
GUV = os.path.join(ARAC, 'guvenlik.json'); BURASI = os.path.dirname(os.path.abspath(__file__))
SIFRE = '1234'                                                              # uygulamanin operator sifresiyle ayni
PORT = 8090
ROS = 'source /opt/ros/jazzy/setup.bash; source ~/cryvex_hw_ws/install/setup.bash; '
HAREKET = re.compile(r'(devriye\.py|masa_testi\.py|home_bul\.py|haritala\.py|home_hizala\.py)(?!.*--dene)')

GOREVLER = {
    'masa':      {'ad': 'HOME → Masa 1 → HOME (2 dk harita ile)', 'dizin': GOREV, 'komut': 'masa_testi.py 120', 'hareket': True,
                  'not': 'Robot HOME\'da olmalı. Hizalanır, Masa 1\'i kaydeder, 2 dk gezip haritayı çıkarır, Masa 1\'e gidip 10 sn bekler, döner.'},
    'masa_kisa': {'ad': 'HOME → Masa 1 → HOME (haritasız)', 'dizin': GOREV, 'komut': 'masa_testi.py --devam', 'hareket': True,
                  'not': 'Kayıtlı HOME\'a yerleşir, Masa 1\'e gidip 10 sn sipariş bekler, HOME\'a döner.'},
    'home':      {'ad': 'HOME\'a dön', 'dizin': GOREV, 'komut': 'home_bul.py', 'hareket': True,
                  'not': 'HOME\'u lidar taramasıyla bulur (harita bozuk olsa bile), döner, duvara hizalanıp yerleşir.'},
    'devriye':   {'ad': 'Kısa devriye (60 sn)', 'dizin': ARAC, 'komut': 'devriye.py 60', 'hareket': True,
                  'not': 'Serbest gezinti. Güvenlik kapısı: 35 cm\'de durur, arkası boşsa dümdüz geri çekilir.'},
    'kontrol':   {'ad': 'Hareketsiz kontrol', 'dizin': GOREV, 'komut': 'home_bul.py --dene', 'hareket': False,
                  'not': 'Robot kıpırdamaz. HOME\'a göre nerede olduğunu hesaplar.'},
}
SON = {'gorev': None, 'pid': None, 'log': None, 'bas': None}
STATIK = {'/logo.png': 'image/png', '/x.png': 'image/png', '/simge-192.png': 'image/png', '/simge-512.png': 'image/png',
          '/manifest.webmanifest': 'application/manifest+json'}

def hareket_surecleri():
    r = subprocess.run(['ps', '-eo', 'pid,etimes,args'], capture_output=True, text=True).stdout.splitlines()[1:]
    out = []
    for s in r:
        p = s.split(None, 2)
        if len(p) == 3 and HAREKET.search(p[2]) and 'bash -c' not in p[2] and 'grep' not in p[2]:
            out.append({'pid': int(p[0]), 'sure_sn': int(p[1]), 'komut': p[2][-90:]})
    return out

def kilit_oku():
    try: return json.load(open(KILIT))
    except Exception: return None

def log_kuyruk(yol, n=40):
    try: satirlar = open(yol, errors='replace').read().splitlines()
    except Exception: return []
    temiz = [s for s in satirlar if s.strip() and not s.startswith((' ', 'Traceback', '[INFO]', '[WARN')) and 'Error' not in s[:40]]
    return temiz[-n:]

def yaml_oku(yol):
    v = {}; bol = None
    try:
        for s in open(yol):
            if s.startswith('#') or not s.strip(): continue
            if not s.startswith(' '): bol = s.strip().rstrip(':'); v[bol] = {}; continue
            k, d = s.strip().split(': ', 1); v[bol][k] = json.loads(d)
    except Exception: pass
    return v

def baslat(ad):
    g = GOREVLER[ad]
    if g['hareket']:
        cal = hareket_surecleri()
        if cal: return False, f"Robotta zaten bir hareket programı çalışıyor (PID {cal[0]['pid']}). Önce DUR."
    log = f'/tmp/panel_{ad}.log'
    p = subprocess.Popen(['bash', '-c', f"{ROS}cd {g['dizin']}; exec python3 -u {g['komut']}"],
                         stdout=open(log, 'w'), stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    SON.update(gorev=ad, pid=p.pid, log=log, bas=time.time())
    if g['hareket']:
        json.dump({'kim': 'panel', 'gorev': g['ad'], 'pid': p.pid, 'zaman': time.strftime('%Y-%m-%d %H:%M:%S')}, open(KILIT, 'w'))
    return True, f"{g['ad']} başladı (PID {p.pid})"

def durdur():
    cal = hareket_surecleri()
    for c in cal:
        try: os.kill(c['pid'], signal.SIGINT)
        except Exception: pass
    time.sleep(1.0)
    for c in hareket_surecleri():
        try: os.kill(c['pid'], signal.SIGKILL)
        except Exception: pass
    subprocess.Popen(['bash', '-c', ROS + "timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1"],
                     start_new_session=True)
    try: os.remove(KILIT)
    except Exception: pass
    return f"{len(cal)} program durduruldu, sıfır hız gönderiliyor"

def durum():
    cal = hareket_surecleri(); k = kilit_oku()
    if k and not cal:                                                       # program bitti: kilit kalkar
        try: os.remove(KILIT)
        except Exception: pass
        k = None
    try: guv = json.load(open(GUV))
    except Exception: guv = {}
    return {'zaman': time.strftime('%H:%M:%S'), 'calisan': cal, 'kilit': k,
            'son': {'gorev': SON['gorev'], 'ad': GOREVLER.get(SON['gorev'] or '', {}).get('ad'), 'pid': SON['pid'],
                    'sure_sn': int(time.time() - SON['bas']) if SON['bas'] else None, 'log': log_kuyruk(SON['log']) if SON['log'] else []},
            'noktalar': yaml_oku(os.path.join(VERI, 'waypoints.yaml')),
            'guvenlik': {k2: v for k2, v in guv.items() if not k2.startswith('_')},
            'gorevler': {k2: {'ad': v['ad'], 'not': v['not'], 'hareket': v['hareket']} for k2, v in GOREVLER.items()}}

def guvenlik_yaz(d):
    guv = json.load(open(GUV)); sinir = {'dur_m': (0.20, 0.80), 'yavas_m': (0.30, 1.50), 'yavas_hiz': (0.03, 0.15), 'donus_yaricap_m': (0.32, 0.60)}
    for k, (a, b) in sinir.items():
        if k in d:
            v = float(d[k])
            if not a <= v <= b: return False, f'{k} {a}–{b} arasında olmalı'
            guv[k] = round(v, 3)
    for k, v in (d.get('pay_m') or {}).items():
        if k in guv.get('pay_m', {}):
            v = float(v)
            if not 0.0 <= v <= 0.40: return False, f'pay {k} 0–0.40 m arasında olmalı'
            guv['pay_m'][k] = round(v, 3)
    if guv['yavas_m'] <= guv['dur_m']: return False, 'yavaşlama mesafesi durma mesafesinden büyük olmalı'
    os.replace(GUV, GUV + '.onceki'); json.dump(guv, open(GUV, 'w'), indent=1, ensure_ascii=False)
    return True, 'Kaydedildi (bir sonraki başlatılan programda geçerli)'

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _json(self, v, kod=200):
        b = json.dumps(v, ensure_ascii=False).encode(); self.send_response(kod)
        self.send_header('Content-Type', 'application/json; charset=utf-8'); self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        if self.path in ('/', '/index.html') or self.path.startswith('/?'):
            b = open(os.path.join(BURASI, 'panel.html'), 'rb').read(); self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.startswith('/api/durum'): self._json(durum())
        elif self.path.split('?')[0] in STATIK:                             # logo, simgeler, telefona ekleme (manifest)
            ad = self.path.split('?')[0].lstrip('/'); b = open(os.path.join(BURASI, ad), 'rb').read(); self.send_response(200)
            self.send_header('Content-Type', STATIK['/' + ad]); self.send_header('Cache-Control', 'max-age=86400')
            self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        else: self._json({'hata': 'yok'}, 404)
    def do_POST(self):
        try: d = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0)) or 0) or b'{}')
        except Exception: d = {}
        if self.path == '/api/dur': return self._json({'ok': True, 'mesaj': durdur()})
        if str(d.get('sifre', '')) != SIFRE: return self._json({'ok': False, 'mesaj': 'Şifre yanlış'}, 403)
        if self.path == '/api/baslat':
            ad = d.get('gorev')
            if ad not in GOREVLER: return self._json({'ok': False, 'mesaj': 'Bilinmeyen görev'}, 400)
            if GOREVLER[ad]['hareket'] and not d.get('onay'): return self._json({'ok': False, 'mesaj': 'Kablo/etraf onayı gerekli'}, 400)
            ok, m = baslat(ad); return self._json({'ok': ok, 'mesaj': m}, 200 if ok else 409)
        if self.path == '/api/guvenlik':
            try: ok, m = guvenlik_yaz(d)
            except Exception as e: ok, m = False, f'Hata: {e}'
            return self._json({'ok': ok, 'mesaj': m}, 200 if ok else 400)
        self._json({'hata': 'yok'}, 404)

if __name__ == '__main__':
    print(f'Cryvex gorev paneli: http://0.0.0.0:{PORT}/', flush=True)
    ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
