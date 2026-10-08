"""algilama.bakis_sec mantigini MOTORSUZ dener (sahte tarete degerleri, sahte hiz komutlari)."""
import sys, time, math, types
sys.argv = ['x']
import algilama as A

class V:
    def __init__(s, v): s.value = v
A.TARETE.update(hedef=V(0.0), aci=V(0.0), hareket=V(0), durus=V(time.time() - 5), durum=V(0), kapat=V(0))
node = types.SimpleNamespace(plan_acisi=lambda: None)

def dene(ad, **ortak):
    for k in ('cmd_niyet', 'cmd_cikis', 'istek'): A.ORTAK.pop(k, None)
    A.ORTAK.update(ortak)
    d, sebep, gorev = A.bakis_sec(node)
    print(f'{ad:38s} -> {d:+7.1f} derece  ({sebep})')
    return d

simdi = time.time()
A.BAKIS['goz_t'] = simdi                    # goz atma hemen baslamasin
dene('ileri duz 0.3 m/s', cmd_niyet=(simdi, 0.3, 0.0))
dene('ileri, sola kivrilarak (w +0.5)', cmd_niyet=(simdi, 0.3, 0.5))
dene('ileri, saga kivrilarak (w -0.5)', cmd_niyet=(simdi, 0.3, -0.5))
dene('yerinde sola donus', cmd_niyet=(simdi, 0.0, 0.6))
dene('yerinde saga donus', cmd_niyet=(simdi, 0.0, -0.6))
dene('geri duz', cmd_niyet=(simdi, -0.1, 0.0))
A.TARETE['aci'].value = -90.0
dene('geri duz (kamera sagdayken)', cmd_niyet=(simdi, -0.1, 0.0))
A.TARETE['aci'].value = 0.0
dene('geri, w +0.4', cmd_niyet=(simdi, -0.1, 0.4))
dene('istek: arka', istek={'id': 'a', 'aci': 'arka', 'bitis': simdi + 5, 'bakti': False})
dene('istek: -40', istek={'id': 'b', 'aci': -40.0, 'bitis': simdi + 5, 'bakti': False})
A.BAKIS['goz_t'] = 0
dene('ileri duz, 3 sn gecti -> goz atma', cmd_niyet=(simdi, 0.3, 0.0))
A.BAKIS['goz'] = None; A.BAKIS['goz_t'] = 0; A.BAKIS['dikkat'] = [(0.8, math.radians(70))]
dene('ileri duz, lidar solda cisim gosteriyor', cmd_niyet=(simdi, 0.3, 0.0))
A.BAKIS['dikkat'] = []; A.BAKIS['son_hareket'] = simdi - 10; A.BAKIS['gorev'] = None
print('dururken tarama (her bakista o yon "goruldu" sayilir):')
A.BAKIS['log'][:] = time.time() - 100
hepsi = []
for i in range(12):
    A.BAKIS['gorev'] = None
    d = dene(f'  adim {i}'); hepsi.append(d); A.TARETE['aci'].value = d
    for b in math.radians(d) + A.np.linspace(-A.FOV / 2, A.FOV / 2, 12): A.BAKIS['log'][A.kutu_no(b)] = time.time()
print('kablo kurali: tum hedefler -180..180 icinde ->', all(-180 <= x <= 180 for x in hepsi))
