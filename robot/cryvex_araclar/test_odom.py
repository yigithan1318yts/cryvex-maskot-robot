import math, numpy as np, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ortak import Robot, icp
rclpy.init(); r = Robot(); r.ready(); E = {}
r.create_subscription(Odometry, 'odom', lambda m: E.__setitem__('m', m), 10)
r.create_subscription(Odometry, 'odom_rf2o', lambda m: E.__setitem__('l', m), 10)
def yaw(m): q = m.pose.pose.orientation; return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
def w(a): return math.atan2(math.sin(a), math.cos(a))
r.wait(2.0)
for hedef in (math.radians(90), math.radians(-90)):
    r.wait(1.0); P0 = r.points(4); o0 = r.odo(); e0 = yaw(E['m']); l0 = yaw(E['l']); tw = Twist(); tw.angular.z = 0.30 if hedef > 0 else -0.30
    cum = 0.0; py = o0[2]
    while abs(cum) < abs(hedef):
        if r.clearance()[2] < 0.33: print('yakin engel'); break
        r.pub.publish(tw); r.wait(0.05); y = r.odo()[2]; cum += w(y - py); py = y
    r.stop(); r.wait(1.5); P1 = r.points(4); od = w(r.odo()[2] - o0[2])
    best = max((icp(P1, P0, (0, 0, g)) for g in (od, od*1.2, od*0.8)), key=lambda t: t[3])
    print(f'teker {math.degrees(od):+.1f} | lidar-odom {math.degrees(w(yaw(E["l"])-l0)):+.1f} | EKF {math.degrees(w(yaw(E["m"])-e0)):+.1f} | GERCEK {math.degrees(best[2]):+.1f} deg (eslesme %{best[3]*100:.0f})')
r.destroy_node(); rclpy.shutdown()
