import math, time, os, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot, KAYIT, icp
rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
def sectors():
    P = r.points(2); a = np.degrees(np.arctan2(P[:,1], P[:,0])); d = np.hypot(P[:,0], P[:,1])
    return P, a, d
def straight(dist, speed=0.15):
    x0, y0, _ = r.odo(); tw = Twist(); tw.linear.x = speed; why = 'tamam'
    try:
        while math.hypot(r.odo()[0]-x0, r.odo()[1]-y0) < dist - 0.01:
            f, b, nr = r.clearance()
            if f < 0.60: why = f'onde engel {f:.2f} m'; break
            if nr < 0.36: why = f'yakin engel {nr:.2f} m'; break
            r.pub.publish(tw); r.wait(0.06)
    finally:
        r.stop(); r.wait(0.8)
    return math.hypot(r.odo()[0]-x0, r.odo()[1]-y0), why
print('1) dumduz ileri A ya (3.0 m)')
D, why = straight(3.0); print(f'   {D:.2f} m gidildi ({why})')
P = r.points(8); np.savez(os.path.expanduser('~/cryvex_araclar/a_noktasi.npz'), P=P, odo=np.array(r.odo()))
print('   A noktasi kaydedildi')
print('2) A da 10 sn bekleniyor'); r.wait(10.0)
P, a, d = sectors()
# U donusunun yapilacagi tarafta (yaricap 0.5 -> 1.0 m yana) bosluk
left = d[(a > 30) & (a < 150)].min(initial=99); right = d[(a < -30) & (a > -150)].min(initial=99)
side = 1 if left >= right else -1
print(f'   solda {left:.2f} m, sagda {right:.2f} m bos -> U donusu {"SOLA" if side > 0 else "SAGA"}')
if max(left, right) < 1.30:
    print('   U DONUSU ICIN YER YOK (iki yanda da 1.3 m den az) - burada duruyorum')
else:
    print('3) U donusu (yaricap ~0.5 m)')
    prev = r.odo()[2]; total = 0.0; tw = Twist(); tw.linear.x = 0.12; tw.angular.z = 0.24 * side; why = 'tamam'
    try:
        while abs(total) < math.pi - math.radians(3):
            h = r.odo()[2]; total += math.atan2(math.sin(h - prev), math.cos(h - prev)); prev = h
            f, b, nr = r.clearance()
            if f < 0.50: why = f'onde engel {f:.2f}'; break
            if nr < 0.38: why = f'yakin engel {nr:.2f}'; break
            r.pub.publish(tw); r.wait(0.06)
    finally:
        r.stop(); r.wait(0.8)
    print(f'   donulen {math.degrees(total):+.0f} deg ({why})')
    if abs(total) > math.radians(170):
        print(f'4) dumduz geri gelis ({D:.2f} m)')
        D2, why = straight(D); print(f'   {D2:.2f} m gidildi ({why})')
REF = np.load(KAYIT)['P']; P = r.points(4); best = None
for g in [(0, 0, math.pi), (0, 0.9, math.pi), (0, -0.9, math.pi)] + [(0, 0, k*math.pi/4) for k in range(8)]:
    res = icp(P, REF, g)
    if best is None or res[3] > best[3]: best = res
print(f'SON: baslangica gore {best[0]*100:+.0f} cm ileride, {best[1]*100:+.0f} cm solda, yon {math.degrees(best[2]):+.0f} deg | eslesme %{best[3]*100:.0f}')
r.destroy_node(); rclpy.shutdown()
