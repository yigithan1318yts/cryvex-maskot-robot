import json, numpy as np, rclpy, math
from ortak import Robot, KAYIT
rclpy.init(); r = Robot()
assert r.ready(), 'lidar/odometri yok'
r.wait(1.0); P = r.points(8); o = r.odo()
np.savez(KAYIT, P=P, odo=np.array(o))
print(f'BASLANGIC KAYDEDILDI: {len(P)} lidar noktasi | odometri ({o[0]:.2f},{o[1]:.2f},{math.degrees(o[2]):+.1f})')
r.destroy_node(); rclpy.shutdown()
