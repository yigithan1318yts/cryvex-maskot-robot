import math, time, sys, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot, KAYIT, icp
import sys as _s
yon = -1 if "--saat-yonu" in _s.argv else 1
HEDEF = math.radians(float([a for a in _s.argv[1:] if not a.startswith("--")][0])) if [a for a in _s.argv[1:] if not a.startswith("--")] else 2*math.pi
rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
print('kablo icin 10 sn bekleniyor...'); r.wait(10.0)
prev = r.odo()[2]; total = 0.0; tw = Twist(); tw.angular.z = 0.30 * yon; why = 'tamam'
try:
    while abs(total) < HEDEF - math.radians(2):
        h = r.odo()[2]; total += math.atan2(math.sin(h - prev), math.cos(h - prev)); prev = h
        f, b, nr = r.clearance()
        if nr < 0.36: why = f'yakin engel {nr:.2f} m'; break
        if abs(total) > HEDEF - math.radians(25): tw.angular.z = 0.15 * yon
        r.pub.publish(tw); r.wait(0.06)
finally:
    r.stop(); r.wait(1.0)
print(f'donulen: {math.degrees(total):+.0f} deg ({why})')
REF = np.load(KAYIT)['P']; P = r.points(4); best = None
for g in [(0, 0, k*math.pi/4) for k in range(8)]:
    res = icp(P, REF, g)
    if best is None or res[3] > best[3]: best = res
print(f'BASLANGICA GORE: {best[0]*100:+.0f} cm ileride, {best[1]*100:+.0f} cm solda, yon {math.degrees(best[2]):+.1f} deg | eslesme %{best[3]*100:.0f}')
r.destroy_node(); rclpy.shutdown()
