"""POZ KAYDI (HAREKETSIZ) - robotun su anki yerini kaydet: harita konumu + lidar noktalari + gordugu duz duvarlar.
Kullanim: python3 -u poz_kaydet.py <ad>   -> ~/cryvex_veri/pozlar/<ad>.json ve <ad>.npy"""
import os, sys, time, math, json
sys.path.insert(0, os.path.expanduser('~/cryvex_araclar')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, rclpy, tf2_ros
import devriye as D
from haritala import harita_pozu

KLASOR = os.path.expanduser('~/cryvex_veri/pozlar')

def duvarlar(n, L, boy_min=0.4):
    """gorulen duz duvarlar: robota gore yon (normalin acisi, 0 = tam on, 180 = tam arka), dik mesafe, boy"""
    out = []
    for (m, u, boy, _a, _b, kal) in n.parcalar(L):
        if boy < boy_min or kal > 0.05: continue
        nrm = np.array([-u[1], u[0]])
        if nrm @ m < 0: nrm = -nrm
        out.append({'yon_derece': round(math.degrees(math.atan2(nrm[1], nrm[0])), 1), 'mesafe_m': round(float(nrm @ m), 3),
                    'boy_m': round(float(boy), 2), 'merkez': [round(float(m[0]), 2), round(float(m[1]), 2)]})
    return sorted(out, key=lambda d: d['mesafe_m'])

if __name__ == '__main__':
    ad = sys.argv[1] if len(sys.argv) > 1 else time.strftime('poz_%H%M%S')
    rclpy.init(); n = D.Devriye(); tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
    try:
        t = time.time()
        while time.time() - t < 3 or n.scan is None: rclpy.spin_once(n, timeout_sec=0.1)
        p = harita_pozu(n, tfb, 10.0); L = n.lidar(); dv = duvarlar(n, L)
        on = L[(L[:, 0] > 0) & (np.abs(L[:, 1]) < 0.25)]; ark = L[(L[:, 0] < 0) & (np.abs(L[:, 1]) < 0.25)]
        kay = {'ad': ad, 'zaman': time.strftime('%Y-%m-%d %H:%M:%S'),
               'harita_poz': None if p is None else [round(p[0], 3), round(p[1], 3), round(math.degrees(p[2]), 2)],
               'on_bos_m': round(float(on[:, 0].min()), 2) if len(on) else 8.0,
               'arka_bos_m': round(float(-ark[:, 0].max()), 2) if len(ark) else 8.0,
               'kamera_derece': n.tarete.get('aci'), 'duvarlar': dv}
        os.makedirs(KLASOR, exist_ok=True)
        json.dump(kay, open(os.path.join(KLASOR, ad + '.json'), 'w'), indent=1); np.save(os.path.join(KLASOR, ad + '.npy'), L)
        print(json.dumps({k: v for k, v in kay.items() if k != 'duvarlar'}))
        for d in dv[:8]: print('  duvar', d)
    finally:
        n.destroy_node(); rclpy.shutdown()
