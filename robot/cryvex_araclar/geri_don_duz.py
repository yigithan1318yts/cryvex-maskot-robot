import math, time, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot, KAYIT, icp
rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
ref = np.load(KAYIT); REF = ref['P']
def where(g=None):
    P = r.points(4); best = None
    for gg in ([g] if g else []) + [(x, 0, 0) for x in (0.0, 1.0, 2.0, 3.0)] + [(0, 0, k*math.pi/4) for k in range(1, 8)]:
        res = icp(P, REF, gg)
        if best is None or res[3] > best[3]: best = res
        if best[3] > 0.6: break
    return best
p = where(); x, y, th, fit = p
print(f'BASLANGICA GORE: {x*100:+.0f} cm ileride, {y*100:+.0f} cm solda, yon {math.degrees(th):+.1f} deg | eslesme %{fit*100:.0f}')
c, s = math.cos(-th), math.sin(-th); hx, hy = c*(-x) - s*(-y), s*(-x) + c*(-y)
behind = math.degrees(math.atan2(-hy, -hx)); dist = math.hypot(x, y)
print(f'baslangic robotun arkasindan {behind:+.0f} deg sapmayla, {dist:.2f} m uzakta')
if fit < 0.35: print('ESLESME ZAYIF - hareket etmiyorum')
elif abs(behind) > 10: print('BASLANGIC TAM ARKADA DEGIL - donmeden ulasilamaz, hareket etmiyorum')
elif dist < 0.05: print('zaten baslangicta')
else:
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = -0.12; why = 'tamam'
    try:
        while math.hypot(r.odo()[0]-x0, r.odo()[1]-y0) < dist - 0.01:
            f, b, nr = r.clearance()
            if b < 0.55: why = f'arkada engel {b:.2f} m'; break
            r.pub.publish(tw); r.wait(0.06)
    finally:
        r.stop(); r.wait(1.0)
    print(f'{math.hypot(r.odo()[0]-x0, r.odo()[1]-y0):.2f} m geri gidildi ({why})')
    p = where(); print(f'SON: baslangica gore {p[0]*100:+.0f} cm ileride, {p[1]*100:+.0f} cm solda, yon {math.degrees(p[2]):+.1f} deg | eslesme %{p[3]*100:.0f}')
r.destroy_node(); rclpy.shutdown()
