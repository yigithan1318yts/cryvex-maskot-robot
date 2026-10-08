"""canli /scan ile ayak birlestirmeyi MOTORSUZ dener: hangi ince kumeler tek mobilya olur"""
import sys, math, time, numpy as np, rclpy
sys.argv = ['x']
import algilama as A
rclpy.init(); n = A.Dugum()
t = time.time()
while time.time() - t < 3: rclpy.spin_once(n, timeout_sec=0.1)
P = A.ORTAK['P']; print('lidar noktasi', len(P))
kb = [{'pts': kp, 'uz': float(np.min(np.hypot(kp[:, 0] - A.CAM_X, kp[:, 1] - A.CAM_Y)))} for kp in A.kumele(P)]
ince = [k for k in kb if max(np.ptp(k['pts'][:, 0]), np.ptp(k['pts'][:, 1])) <= 0.20 and k['uz'] < 5.0]
print('kume', len(kb), 'ince (ayak olabilir)', len(ince))
ana = list(range(len(ince)))
def kok(i):
    while ana[i] != i: ana[i] = ana[ana[i]]; i = ana[i]
    return i
for i in range(len(ince)):
    for j in range(i + 1, len(ince)):
        if np.hypot(*(ince[i]['pts'].mean(0) - ince[j]['pts'].mean(0))) <= 0.75: ana[kok(i)] = kok(j)
g = {}
for i, k in enumerate(ince): g.setdefault(kok(i), []).append(k)
for grup in g.values():
    m = [tuple(np.round(k['pts'].mean(0), 2)) for k in grup]
    if len(grup) >= 2:
        cx, cy, en, der, *_ = A.kutu(np.vstack([k['pts'] for k in grup]), 0.03)
        print(f'MOBILYA {len(grup)} ayak, merkez ({cx:+.2f},{cy:+.2f}) {en*100:.0f}x{der*100:.0f} cm  ayaklar {m}')
    else:
        print('tek ayak/ince', m)
