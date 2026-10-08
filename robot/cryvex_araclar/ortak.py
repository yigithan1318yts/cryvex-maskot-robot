"""Baslangic noktasi kaydet / geri don - ortak fonksiyonlar (haritadan bagimsiz, lidar ICP)."""
import time, math, json, os, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
LY, LX, LYY = math.radians(-90), 0.125, -0.045
KAYIT = os.path.expanduser('~/cryvex_araclar/baslangic.npz')
class Robot(Node):
    def __init__(self):
        super().__init__('baslangic_araci')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10); self.m = None; self.w = None
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'm', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'wheel/odom', lambda m: setattr(self, 'w', m), 10)
    def wait(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.02)
    def ready(self):
        t = time.time()
        while time.time() - t < 15 and (self.m is None or self.w is None): rclpy.spin_once(self, timeout_sec=0.1)
        return self.m is not None and self.w is not None
    def points(self, k=4):
        out = []; last = None; t = time.time()
        while len(out) < k and time.time() - t < 6:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.m is not None and self.m is not last: out.append(self.m); last = self.m
        P = []
        for m in out:
            a = m.angle_min + np.arange(len(m.ranges))*m.angle_increment; r = np.array(m.ranges); ok = np.isfinite(r) & (r > 0.15) & (r < 8)
            x, y = r[ok]*np.cos(a[ok]), r[ok]*np.sin(a[ok]); c, s = math.cos(LY), math.sin(LY)
            P.append(np.stack([c*x - s*y + LX, s*x + c*y + LYY], 1))
        return np.concatenate(P)
    def odo(self):
        p = self.w.pose.pose; q = p.orientation
        return p.position.x, p.position.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
    def clearance(self):
        P = self.points(1); ang = np.degrees(np.arctan2(P[:,1], P[:,0])); r = np.hypot(P[:,0], P[:,1])
        return r[np.abs(ang) < 35].min(initial=99), r[np.abs(ang) > 145].min(initial=99), r.min(initial=99)
    def stop(self):
        for _ in range(10): self.pub.publish(Twist()); time.sleep(0.05)
def icp(src, dst, guess):
    src = src[::max(1, len(src)//700)]; dst = dst[::max(1, len(dst)//900)]
    x, y, th = guess; dist = np.array([9.9]); ok = np.array([False])
    for it in range(50):
        c, s = math.cos(th), math.sin(th)
        S = src @ np.array([[c, s], [-s, c]]) + [x, y]
        d2 = ((S[:, None, :] - dst[None, :, :])**2).sum(2); j = d2.argmin(1); dist = np.sqrt(d2[np.arange(len(S)), j])
        lim = max(0.06, 0.8 * (0.9 ** it)); ok = dist < lim
        if ok.sum() < 30: break
        A, B = S[ok], dst[j[ok]]; ma, mb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb)); R = Vt.T @ U.T
        if np.linalg.det(R) < 0: Vt[1] *= -1; R = Vt.T @ U.T
        dth = math.atan2(R[1, 0], R[0, 0]); t = mb - R @ ma; c2, s2 = math.cos(dth), math.sin(dth)
        x, y = c2*x - s2*y + t[0], s2*x + c2*y + t[1]; th += dth
    fit = float((dist < 0.08).mean())
    return x, y, math.atan2(math.sin(th), math.cos(th)), fit
