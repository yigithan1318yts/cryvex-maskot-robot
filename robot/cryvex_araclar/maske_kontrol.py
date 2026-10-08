"""govde_maske.json'u kalibrasyon karelerinin ustune ciz (kirmizi tarali = govde, yok sayilir)"""
import json, os, numpy as np, cv2
d = os.path.expanduser('~/cryvex_araclar/govde'); M = json.load(open(os.path.expanduser('~/cryvex_araclar/govde_maske.json')))
acilar = sorted(int(k) for k in M)
kucuk = []
for a in acilar:
    im = cv2.resize(cv2.imread(f'{d}/ham_{a:+04d}.jpg'), (320, 240)); kat = im.copy(); m = M[str(a)]
    kutular = [[0, 0, 1, 1]] if m == 'tam' else (m if isinstance(m[0], list) else [m])
    for x1, y1, x2, y2 in kutular: cv2.rectangle(kat, (int(x1 * 320), int(y1 * 240)), (int(x2 * 320), int(y2 * 240)), (0, 0, 255), -1)
    im = cv2.addWeighted(kat, 0.45, im, 0.55, 0)
    cv2.putText(im, f'{a:+d}' + (' KOR' if m == 'tam' else ''), (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2); kucuk.append(im)
while len(kucuk) % 3: kucuk.append(np.zeros((240, 320, 3), np.uint8))
cv2.imwrite(f'{d}/maske_pano.jpg', np.vstack([np.hstack(kucuk[i:i + 3]) for i in range(0, len(kucuk), 3)]), [cv2.IMWRITE_JPEG_QUALITY, 70])
print('ok', acilar)
