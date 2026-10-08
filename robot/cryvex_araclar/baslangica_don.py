"""Kayitli baslangica geri don. Arg: --kablo-bekle (buyuk donusten once 10 sn bekle)."""
import sys, math, time, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot, KAYIT, icp
KABLO = '--kablo-bekle' in sys.argv; SADECE_OLC = '--olc' in sys.argv
rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
ref = np.load(KAYIT); REF = ref['P']; o0 = ref['odo']
def where(prev=None):
    """robotun BASLANGICA gore konumu (x ileri, y sol, th) - ICP; birkac baslangic tahmini dener."""
    P = r.points(4); best = None
    guesses = [tuple(prev[:3])] if prev is not None else []
    o = r.odo(); c, s = math.cos(o0[2]), math.sin(o0[2]); dx, dy = o[0]-o0[0], o[1]-o0[1]
    guesses.append((c*dx + s*dy, -s*dx + c*dy, math.atan2(math.sin(o[2]-o0[2]), math.cos(o[2]-o0[2]))))
    guesses += [(0, 0, k*math.pi/4) for k in range(8)]
    for g in guesses:
        res = icp(P, REF, g)
        if best is None or res[3] > best[3]: best = res
        if best[3] > 0.6: break
    return best
def drive(lx, az, sec, back=False):
    tw = Twist(); tw.linear.x = float(lx); tw.angular.z = float(az); t = time.time()
    try:
        while time.time() - t < sec:
            f, b, nr = r.clearance()
            if nr < 0.36: print('  DUR: yakin engel'); return False
            if lx > 0 and f < 0.55: print('  DUR: onde engel'); return False
            if lx < 0 and b < 0.55: print('  DUR: arkada engel'); return False
            r.pub.publish(tw); r.wait(0.06)
        return True
    finally:
        r.stop(); r.wait(0.4)
def turn(rad):
    if abs(rad) > math.radians(45) and KABLO:
        print(f'  BUYUK DONUS {math.degrees(rad):+.0f} deg - kablo icin 10 sn bekleniyor'); r.wait(10.0)
    o = r.odo(); target = o[2] + rad; tw = Twist()
    try:
        for _ in range(1500):
            e = math.atan2(math.sin(target - r.odo()[2]), math.cos(target - r.odo()[2]))
            if abs(e) < math.radians(1.5): break
            if r.clearance()[2] < 0.36: print('  DUR: donerken yakin engel'); break
            tw.angular.z = max(0.12, min(0.35, abs(e))) * (1 if e > 0 else -1)
            r.pub.publish(tw); r.wait(0.06)
    finally:
        r.stop(); r.wait(0.5)
p = where(); print(f'BASLANGICA GORE: x={p[0]*100:+.0f} cm (ileri+), y={p[1]*100:+.0f} cm (sol+), yon {math.degrees(p[2]):+.1f} deg | eslesme %{p[3]*100:.0f}')
if p[3] < 0.35: print('ESLESME ZAYIF - baslangici taniyamadim, hareket etmiyorum'); SADECE_OLC = True
if not SADECE_OLC:
    for it in range(8):
        x, y, th, fit = p; dist = math.hypot(x, y)
        if dist < 0.05: break
        # hedef (0,0) robota gore nerede?
        bx, by = -x, -y; c, s = math.cos(-th), math.sin(-th); hx, hy = c*bx - s*by, s*bx + c*by
        ang = math.atan2(hy, hx)
        if abs(ang) > math.radians(120):   # baslangic arkada -> donmeden geri git
            back_ang = math.atan2(-hy, -hx)
            if abs(back_ang) > math.radians(4) and dist > 0.10: turn(back_ang)
            drive(-0.10, 0, min(dist, 0.6)/0.10)
        else:
            if abs(ang) > math.radians(4) and dist > 0.10: turn(ang)
            drive(0.10, 0, min(dist, 0.6)/0.10)
        r.wait(0.8); p = where(p); print(f'  adim {it+1}: x={p[0]*100:+.0f} y={p[1]*100:+.0f} cm yon {math.degrees(p[2]):+.1f} | eslesme %{p[3]*100:.0f}')
    if abs(p[2]) > math.radians(2): turn(-p[2])
    r.wait(0.8); p = where(p)
    print(f'SON: baslangica gore x={p[0]*100:+.0f} cm, y={p[1]*100:+.0f} cm, yon {math.degrees(p[2]):+.1f} deg | eslesme %{p[3]*100:.0f}')
r.destroy_node(); rclpy.shutdown()
