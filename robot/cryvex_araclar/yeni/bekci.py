"""Bekci: kesif.py / gezgin.py calisirken robot 60 sn boyunca gercekten (lidar ICP ile) hareket etmezse
kesfi durdurur, robotu durdurur, haritayi kaydeder. kesif.py kendisi biterse bekci de cikar."""
import os, time, math, json, subprocess, urllib.request, rclpy
from ortak import Robot, icp
SURE_SN = 60; ORNEK_SN = 5; ACILIS_PAYI = 150   # ilk 2.5 dk sayilmaz (Nav2/SLAM acilisi)
def log(s): print(time.strftime('%H:%M:%S'), s, flush=True)
def kesif_calisiyor():
    return subprocess.run(['pgrep', '-f', 'python3 -u .*(kesif|gezgin)[.]py'], capture_output=True).returncode == 0
def api(path, body=None):
    req = urllib.request.Request('http://127.0.0.1:8080' + path, data=json.dumps(dict(body or {}, password='1234')).encode(),
                                headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=30).read())
rclpy.init(); r = Robot(); assert r.ready()
gecmis = []   # (zaman, hareket) - ardisik ornekler arasi gercek hareket
log(f'bekci basladi - ilk {ACILIS_PAYI} sn acilis payi'); t_bas = time.time()
while time.time() - t_bas < ACILIS_PAYI and kesif_calisiyor(): r.wait(1.0)
P0 = r.points(2); o0 = r.odo()
while kesif_calisiyor():
    r.wait(ORNEK_SN)
    P1 = r.points(2); o1 = r.odo()
    dth = math.atan2(math.sin(o1[2] - o0[2]), math.cos(o1[2] - o0[2]))
    c, s = math.cos(o0[2]), math.sin(o0[2]); dx, dy = o1[0] - o0[0], o1[1] - o0[1]
    g = (c * dx + s * dy, -s * dx + c * dy, dth)
    x, y, th, fit = max((icp(P1, P0, g), icp(P1, P0, (0.0, 0.0, 0.0))), key=lambda t: t[3])
    hareket = math.hypot(x, y) + 0.3 * abs(th)     # 10 deg donus ~ 5 cm sayilir
    gecmis.append((time.time(), hareket)); P0, o0 = P1, o1
    gecmis = [h for h in gecmis if time.time() - h[0] <= SURE_SN]
    try: bilerek = time.time() - os.path.getmtime(os.path.expanduser('~/cryvex_araclar/gezgin_bekliyor')) < 10
    except Exception: bilerek = False
    if bilerek:                                    # gezgin BILEREK bekliyor (onu/arkasi kapali): durdurma
        gecmis = []; continue
    if gecmis and time.time() - gecmis[0][0] >= SURE_SN - ORNEK_SN - 1 and sum(h[1] for h in gecmis) < 0.10:
        log(f'ROBOT {SURE_SN} SN HAREKETSIZ - test durduruluyor')
        subprocess.run(['pkill', '-f', 'python3 -u .*(kesif|gezgin)[.]py']); subprocess.run(['pkill', '-f', 'baslangica_don.py'])
        r.stop()
        try: log('otonom durduruluyor: ' + str(api('/api/otonom', {'mod': 'dur'})))
        except Exception as e: log(f'harita kaydedilemedi: {e}')
        break
log('bekci cikti'); r.destroy_node(); rclpy.shutdown()
