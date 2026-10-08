import math, time, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot, icp
rclpy.init(); r = Robot(); r.ready()
def w(a): return math.atan2(math.sin(a), math.cos(a))
for hedef, hiz in ((math.radians(90), 0.30), (math.radians(-90), 0.30)):
    r.wait(1.0); P0 = r.points(4); o0 = r.odo(); tw = Twist(); tw.angular.z = hiz if hedef > 0 else -hiz
    cum = 0.0; py = o0[2]
    while abs(cum) < abs(hedef):
        if r.clearance()[2] < 0.33: print('yakin engel, durdum'); break
        r.pub.publish(tw); r.wait(0.05); y = r.odo()[2]; cum += w(y - py); py = y
    r.stop(); r.wait(1.5); P1 = r.points(4); o1 = r.odo(); od = w(o1[2] - o0[2])
    best = max((icp(P1, P0, (0, 0, g)) for g in (od, od*1.2, od*0.8)), key=lambda t: t[3])
    print(f'odometri {math.degrees(od):+.1f} deg | lidar (gercek) {math.degrees(best[2]):+.1f} deg | eslesme %{best[3]*100:.0f} | oran {best[2]/od:.3f}')
r.destroy_node(); rclpy.shutdown()
