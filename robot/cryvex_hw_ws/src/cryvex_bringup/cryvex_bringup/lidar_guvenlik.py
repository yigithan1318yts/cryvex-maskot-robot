"""LIDAR GUVENLIK KAPISI (2026-10-08) - Nav2 DISINDAN dogrudan /cmd_vel'e giden komutlar icin.

Sebep: patrol.py masadan ayrilirken 1 m DUZ GERI ve kurtulma manevrasinda geri gidiyordu; arkayi ULTRASONIK ile
kontrol ediyordu ama robotta ultrasonik TAKILI DEGIL (hep 'bos' okur) -> arkada duran insani ezdi.
Bu kapi LIDAR ile (360 derece) bakar; kural sahibin genel kuraliyla ayni (cryvex_araclar/guvenlik.json):
  * gidis yonundeki seritte (govde eni + 3 cm) govde yuzeyine dur_m'den (35 cm) yakin HERHANGI bir sey -> v = 0
  * yavas_m (70 cm) icinde hiz <= yavas_hiz
  * yerinde donus: donus_yaricap_m (40 cm) icinde bir sey -> w = 0
  * lidar verisi eski (> 0.5 sn) ya da yoksa -> HAREKET YOK (bilinmeyen = bos degil)
Govdenin 2 cm cevresi (kendi guc kablosu) sayilmaz."""
import json
import math
import os
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

R = 0.295                                         # govde yaricapi (59 cm en)
LX, LY, LYAW = 0.125, -0.045, math.radians(-90)   # lidar montaji (urdf/cryvex_real.urdf.xacro)
VARSAYILAN = {'dur_m': 0.35, 'yavas_m': 0.70, 'yavas_hiz': 0.08, 'kose_pay_m': 0.03, 'donus_yaricap_m': 0.40}


def _ayarlar():
    a = dict(VARSAYILAN)
    try:
        with open(os.path.expanduser('~/cryvex_araclar/guvenlik.json')) as f:
            a.update({k: v for k, v in json.load(f).items() if k in VARSAYILAN})
    except Exception:
        pass
    return a


class LidarKapisi:
    def __init__(self, node, scan_topic='/scan'):
        self.node = node
        self.L = np.zeros((0, 2))
        self.t = 0.0
        self.ayar = _ayarlar()
        self._ayar_t = time.time()
        self._log_t = 0.0
        node.create_subscription(LaserScan, scan_topic, self._scan, qos_profile_sensor_data)

    def _scan(self, m):
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.05) & (r < 8.0)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        c, s = math.cos(LYAW), math.sin(LYAW)
        L = np.stack([c * x - s * y + LX, s * x + c * y + LY], 1)
        self.L = L[np.hypot(L[:, 0], L[:, 1]) > R + 0.02]          # kendi govdesi / guc kablosu degil
        self.t = time.time()

    def _log(self, s):
        if time.time() - self._log_t > 2.0:
            self._log_t = time.time()
            self.node.get_logger().warn(f'LIDAR GUVENLIK: {s}')

    def engelli(self, yon):
        """yon +1 ileri / -1 geri: gidis seridinde govde yuzeyine en yakin mesafe (m), lidar yoksa None."""
        if time.time() - self.t > 0.5:
            return None
        L = self.L
        if not len(L):
            return 9.0
        X, Y = L[:, 0] * yon, L[:, 1]
        serit = (np.abs(Y) < R + self.ayar['kose_pay_m']) & (X > -R) & ((X > 0) | (np.abs(Y) < R))
        if not serit.any():
            return 9.0
        return float((X[serit] - np.sqrt(np.maximum(R * R - Y[serit] ** 2, 0.0))).min())

    def kapi(self, tw):
        """Twist'i guvenli hale getirir (yeni Twist doner)."""
        if time.time() - self._ayar_t > 5.0:
            self.ayar, self._ayar_t = _ayarlar(), time.time()
        v, w = float(tw.linear.x), float(tw.angular.z)
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            return tw
        out = Twist()
        if time.time() - self.t > 0.5:
            self._log('lidar verisi yok/eski - hareket yok')
            return out
        if abs(v) > 1e-3:
            yon = 1.0 if v > 0 else -1.0
            d = self.engelli(yon)
            if d is not None and d < self.ayar['dur_m']:
                self._log(f"{'onumde' if yon > 0 else 'ARKAMDA'} {max(d, 0) * 100:.0f} cm'de bir sey - DURDUM")
                return out                                         # ileri/geri durunca donme de yok
            if d is not None and d < self.ayar['yavas_m']:
                v = yon * min(abs(v), self.ayar['yavas_hiz'])
        if abs(w) > 1e-3 and abs(v) < 0.05 and len(self.L):
            if float(np.hypot(self.L[:, 0], self.L[:, 1]).min()) < self.ayar['donus_yaricap_m']:
                self._log('yerinde donersem govde kosesi bir seye degecek - donmuyorum')
                w = 0.0
        out.linear.x, out.angular.z = v, w
        return out
