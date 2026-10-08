"""calisan algilamanin taramasi sirasinda (motoru ayrica DONDURMEDEN) her 15 derecelik aci icin bir gecerli kare toplar
ve bir pano yapar: kendi govdesinin (lidar diregi, guc kaynagi) hangi acida goruntunun neresini kapladigini gormek icin."""
import json, time, urllib.request, subprocess, numpy as np, cv2, os
cikti = os.path.expanduser('~/cryvex_araclar/govde'); os.makedirs(cikti, exist_ok=True)
toplanan = {int(f[4:-4]): cv2.imread(os.path.join(cikti, f)) for f in os.listdir(cikti) if f.startswith('aci_')}; bas = time.time()
while time.time() - bas < 150 and len(toplanan) < 25:
    try:
        d = json.loads(subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash; timeout 3 ros2 topic echo --once /nesneler std_msgs/msg/String --field data | head -n 1'],
                                      capture_output=True, text=True, timeout=8).stdout)
        tr = d['tarete']
        if not tr['gecerli']: continue
        k = int(round(tr['aci'] / 15.0)) * 15
        if k in toplanan or abs(tr['aci'] - k) > 6: continue
        j = urllib.request.urlopen('http://127.0.0.1:8081/kare.jpg', timeout=3).read()
        img = cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)[:, :640]
        cv2.imwrite(f'{cikti}/aci_{k:+04d}.jpg', img); toplanan[k] = img; print('aci', k, flush=True)
    except Exception as e:
        print('hata', e, flush=True)
anahtarlar = sorted(toplanan)
kucuk = [cv2.putText(cv2.resize(toplanan[k], (320, 240)), f'{k:+d}', (5, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2) for k in anahtarlar]
while len(kucuk) % 4: kucuk.append(np.zeros((240, 320, 3), np.uint8))
pano = np.vstack([np.hstack(kucuk[i:i + 4]) for i in range(0, len(kucuk), 4)])
cv2.imwrite(f'{cikti}/pano.jpg', pano, [cv2.IMWRITE_JPEG_QUALITY, 70]); print('toplanan acilar', anahtarlar)

