import sys, math, time, numpy as np, rclpy
from geometry_msgs.msg import Twist
from ortak import Robot
dist = float(sys.argv[1]); speed = 0.12
rclpy.init(); r = Robot(); assert r.ready(), 'lidar/odometri yok'
f, b, nr = r.clearance(); print(f'once: onde {f:.2f} m bos, arkada {b:.2f} m')
x0, y0, h0 = r.odo(); tw = Twist(); tw.linear.x = speed if dist > 0 else -speed; why = 'tamam'
try:
    while math.hypot(r.odo()[0]-x0, r.odo()[1]-y0) < abs(dist) - 0.01:
        f, b, nr = r.clearance()
        if dist > 0 and f < 0.60: why = f'onde engel {f:.2f} m'; break
        if dist < 0 and b < 0.60: why = f'arkada engel {b:.2f} m'; break
        r.pub.publish(tw); r.wait(0.06)
finally:
    r.stop(); r.wait(0.8)
x, y, h = r.odo()
print(f'{math.hypot(x-x0, y-y0):.2f} m gidildi ({why}) | yon degisimi {math.degrees(h-h0):+.1f} deg (sayaca gore)')
r.destroy_node(); rclpy.shutdown()
