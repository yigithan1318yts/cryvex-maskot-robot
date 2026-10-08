"""KAMERA TARETESI - NEMA 23 + DMA860H ile kamerayi saga/sola cevirir (Cryvex, 2026-10-06; 2026-10-07 DMA860H).
Baglanti: PUL+ = GPIO17 (pin 11), DIR+ = GPIO27 (pin 13), PUL-/DIR- = GND (pin 9); ENA bos.
Surucu: 6400 adim/tur (SW5..8 = on on off on; titremeyi azaltmak icin) -> 17.78 adim/derece.
KURAL (kablo dolanmasin): aci HER ZAMAN -180..+180 arasinda; ±185 asilacak olursa hareket ETMEZ.
  Hedefe hep MUTLAK aciyla gidilir: +170'ten -170'e gitmek onden gecerek 340 derece demektir, arkadan gecmez.
+aci = SOLA (ust taraftan bakinca saat yonunun tersi), 0 = tam on. Konum ~/cryvex_araclar/tarete_konum.json'da tutulur.
Hareket basinda dosyaya 'hareket': true yazilir; elektrik hareket ortasinda kesilirse konum belirsizdir ->
  bir sonraki acilista KILITLI baslar (kamerayi elle one cevir, sonra --sifirla).
algilama.py bunu ayri bir SURECTE calistirir (surec()), boylece yapay zeka adim zamanlamasini bozmaz.
Kullanim (test, algilama kapaliyken): python3 kamera_tarete.py --git 15 | --test | --uzun-test | --sifirla"""
import os, sys, json, time, math, gc, lgpio

CHIP, PUL, DIR = 4, 17, 27
ADIM_TUR = 6400
ADIM_DERECE = ADIM_TUR / 360.0
SINIR, KILIT = 180.0, 185.0
YON_TERS = True                        # 2026-10-07 testi: ters haliyle + komut SAGA donuyordu
KONUM_DOSYA = os.path.expanduser('~/cryvex_araclar/tarete_konum.json')

class Tarete:
    def __init__(self):
        try: os.sched_setaffinity(0, {3})          # pals zamanlamasi duzgun olsun: tek cekirdek
        except Exception: pass
        try: os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))   # root/chrt ile calisirsa
        except Exception: pass
        self.h = lgpio.gpiochip_open(CHIP)
        try:
            lgpio.gpio_claim_output(self.h, PUL, 0); lgpio.gpio_claim_output(self.h, DIR, 0)
        except Exception:
            lgpio.gpiochip_close(self.h); raise
        self.adim, self.kilitli = self._oku()

    def _oku(self):
        try:
            d = json.load(open(KONUM_DOSYA)); return int(d['adim']), bool(d.get('hareket'))
        except Exception:
            return 0, False
    def _yaz(self, hareket=False):
        json.dump({'adim': self.adim, 'aci': round(self.aci, 2), 'hareket': hareket, 'zaman': time.strftime('%H:%M:%S')},
                  open(KONUM_DOSYA, 'w'))

    @property
    def aci(self): return self.adim / ADIM_DERECE

    def git(self, hedef_aci, hiz=120.0, ivme=400.0):
        """hedef aciya (derece) git; hiz derece/s, ivme derece/s^2. Sinir disi -> HATA, hareket yok."""
        if self.kilitli:
            raise RuntimeError('KILITLI: son hareket yarida kesilmis, konum belirsiz (kamerayi one cevir + --sifirla)')
        if not -SINIR <= hedef_aci <= SINIR:
            raise ValueError(f'hedef {hedef_aci:.1f} deg sinir disi (±{SINIR})')
        hedef = int(round(hedef_aci * ADIM_DERECE)); fark = hedef - self.adim
        if fark == 0: return
        if abs((self.adim + fark) / ADIM_DERECE) > KILIT:
            raise RuntimeError('KILIT: kablo korumasi')
        ileri = fark > 0
        self._yaz(hareket=True)
        lgpio.gpio_write(self.h, DIR, int(ileri != YON_TERS)); time.sleep(0.0002)
        n = abs(fark); vmax = hiz * ADIM_DERECE; a = ivme * ADIM_DERECE
        t = time.perf_counter(); v = 0.0
        gc.disable()                                                    # hareket sirasinda duraklama olmasin
        try:
            for i in range(n):
                kalan = n - i
                v_fren = math.sqrt(2 * a * kalan)                       # trapez: hizlan - sabit - yavasla
                v = min(vmax, v_fren, max(a * 0.02, v + a / max(v, a * 0.02)))
                lgpio.gpio_write(self.h, PUL, 1); bekle = time.perf_counter() + 20e-6
                while time.perf_counter() < bekle: pass
                lgpio.gpio_write(self.h, PUL, 0)
                self.adim += 1 if ileri else -1
                t += 1.0 / v
                while time.perf_counter() < t: pass
        finally:
            gc.enable(); self._yaz()

    def kapat(self):
        lgpio.gpiochip_close(self.h)

def surec(hedef, aci, hareket, durus, durum, kapat, ana_pid, sifirla=None):
    """algilama.py'nin actigi ayri surec. Paylasilan degerler (multiprocessing.Value):
    hedef (deg, NaN = yok) -> buraya yazilir; aci (deg), hareket (0/1), durus (hareketin bittigi an, time.time()),
    durum (0 calisiyor, 1 GPIO bekleniyor, 2 KILITLI). kapat=1 ya da ana surec olurse: one don ve cik."""
    import signal
    def kapan(*_a): kapat.value = 1                                     # Ctrl+C / servis durdurma: once one don, sonra cik
    signal.signal(signal.SIGINT, kapan); signal.signal(signal.SIGTERM, kapan)
    t = None
    while t is None:
        try:
            t = Tarete()
        except Exception as e:
            durum.value = 1; print(time.strftime('%H:%M:%S'), 'tarete: GPIO acilamadi, bekliyorum:', e, flush=True)
            if kapat.value or os.getppid() != ana_pid: return
            time.sleep(2.0)
    aci.value = t.aci; durum.value = 2 if t.kilitli else 0
    if t.kilitli: print(time.strftime('%H:%M:%S'), 'tarete KILITLI: konum belirsiz, kamera DONMEYECEK', flush=True)
    try:
        while True:
            bitir = kapat.value or os.getppid() != ana_pid
            if sifirla is not None and sifirla.value == 1 and not bitir and not t.kilitli:
                # ORTALAMA: kamera su an fiziksel olarak TAM ONE bakiyor (once oraya goturuldu) -> burasi yeni 0
                print(time.strftime('%H:%M:%S'), f'tarete: {t.aci:+.2f} derece yeni SIFIR (tam on) olarak kaydedildi', flush=True)
                t.adim = 0; t._yaz(); aci.value = 0.0; hedef.value = 0.0; durus.value = time.time(); sifirla.value = 0
                continue
            h = 0.0 if bitir else hedef.value
            if not t.kilitli and h == h and abs(h - t.aci) >= 0.5:      # h == h: NaN degil
                h = max(-SINIR, min(SINIR, h))
                hareket.value = 1
                try: t.git(h, hiz=90.0, ivme=300.0)                    # 2 dk'lik sorunsuz testteki hiz (120'de titriyordu)
                except Exception as e: print(time.strftime('%H:%M:%S'), 'tarete hata:', e, flush=True)
                aci.value = t.aci; durus.value = time.time(); hareket.value = 0
                continue
            if bitir: return
            time.sleep(0.01)
    finally:
        t.kapat()

if __name__ == '__main__':
    t = Tarete(); print(f'baslangic: {t.aci:+.1f} deg' + ('  (KILITLI)' if t.kilitli else ''))
    try:
        if '--sifirla' in sys.argv:
            t.adim = 0; t.kilitli = False; t._yaz(); print('su anki konum 0 (on) olarak kaydedildi')
        elif '--git' in sys.argv:
            hedef = float(sys.argv[sys.argv.index('--git') + 1]); t.git(hedef); print(f'simdi {t.aci:+.1f} deg')
        elif '--test' in sys.argv:
            for hedef in (15, 0, -15, 0):
                t.git(hedef, hiz=30.0, ivme=120.0); print(f'  {t.aci:+.1f} deg', flush=True); time.sleep(1.0)
        elif '--uzun-test' in sys.argv:
            # gercek kullanima benzer: one bak, yanlara goz at, dondukce o yana bak; hep ayni yoldan geri
            # arkaya (±180) gidince ayni yoldan one doner; +180'den -180'e asla arkadan gecilmez (git() bunu garanti eder)
            plan = [30, 0, -30, 0, 60, 0, -60, 0, 90, 0, -90, 0, 45, -45, 0, 135, 0, -135, 0, 180, 0, -180, 0]
            bas = time.time()
            for tur in range(2):
                for hedef in plan:
                    t.git(hedef, hiz=90.0, ivme=300.0)
                    print(f'  {time.time() - bas:5.1f} s  {t.aci:+6.1f} deg', flush=True); time.sleep(1.2)
            t.git(0, hiz=60.0, ivme=200.0); print(f'bitti: {t.aci:+.1f} deg')
    finally:
        t.kapat()
