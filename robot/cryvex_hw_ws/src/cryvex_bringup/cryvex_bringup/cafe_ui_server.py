#!/usr/bin/env python3
"""
Cryvex Kafe Arayuzu - gercek donanim
====================================
~/cryvex_ws/src/cryvex_gazebo/web/index.html'i (goz animasyonlari, sanal
joystick, "Ortami Haritala" akisi) gercek donanimda sunar; sim'deki
tablet_server.py'nin gercek robot karsiligi.

  - Devriye beyni (patrol.py, 2026-09-25'te tasindi): telefon/ekran komutlari
    (/api/start_patrol, /goto, /place_order, /rescue_*, /send_message ...)
    sim ile AYNI duz metin komutlarina cevrilip /patrol_command'a gider,
    /patrol_status (JSON) /api/status ile arayuze doner. patrol.py yoksa
    /api/status "idle" yedegini doner (goz animasyonu yine calisir).
  - Nav2 <-> haritalama (LaunchManager): kayitli harita varsa acilista Nav2
    (AMCL + surus) baslar; "Ortami Haritala" slam_toolbox'a gecer, Bitir/
    Vazgec Nav2'ye doner. Ikisi ASLA ayni anda calismaz (/map + map->odom catisir).
  - Robotun haritadaki konumu da burada: son AMCL konumu config/last_pose.json'a
    kaydedilir, Nav2 her acildiginda (ayni haritaysa) geri verilir; haritalama
    bitince SLAM'in son konumu verilir; "Robot Burada" (/api/set_pose) ve
    "Robotu use sabitle" (/api/relocalize) elle ayar.
  - /live_map.png (1 m izgarali, canli LiDAR), /setup + /api/waypoints.
  - /api/tts_audio + /api/speak_here: robotun TEK sesi (edge-tts, kadin).
    Uretilen her cumle KALICI onbellege yazilir ve arayuzun sabit cumleleri
    internet gelir gelmez onceden uretilir - acilista internet/saat henuz
    hazir degilken de robot hep ayni sesle konusur. patrol.py'nin sesli
    mesajlari (say) da bu yoldan robotun kendi ekraninda okunur.
"""
import ast
import asyncio
import functools
import hashlib
import json
import math
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
import urllib.parse
import zlib
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time as RclpyTime
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener

try:
    import edge_tts  # kurulu degilse yalnizca onbellekteki cumleler calinir
    _EDGE_TTS_AVAILABLE = True
except ImportError:
    _EDGE_TTS_AVAILABLE = False

OPERATOR_PASSWORD = '1234'  # index.html/Flutter app ile AYNI (sim ile tutarli)

MAP_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

MAP_BACKUP_KEEP = 10  # "Bitir" oncesi yedeklenen eski haritalardan kac tanesi tutulsun

PATROL_STATUS_FRESH_S = 3.0   # bu kadar eski /patrol_status = patrol.py yok sayilir

# Siparisler (masada robot ekranindan verilir, garson telefonunda listelenir;
# barmen "Hazir - Robot Gotursun" deyince patrol.py'ye 'deliver' gider).
# config/orders.json'da kalici - sunucu yeniden baslasa da liste kaybolmaz.
ORDERS_KEEP = 60                    # dosyada tutulan en fazla siparis
ORDER_REVERT_GRACE_S = 4.0          # patrol teslimati bu sure icinde ustlenmezse geri al
DELIVERY_STATES = ('pickup', 'pickup_wait', 'serving', 'served_wait')
# preparing -> (barmen "Hazir") ready = robotun sirasinda -> delivering = robot aldi -> delivered
ACTIVE_ORDER_STATUSES = ('preparing', 'ready', 'delivering')
# patrol.py DELIVERY_INTERRUPTIBLE ile ayni: robot bu durumlardaysa teslimata hemen cikar
PATROL_INTERRUPTIBLE = ('idle', 'patrol', 'wander', 'going_home', 'greet_door')

# Robot Sagligi esikleri
BATTERY_LOW_V = 23.5              # stm32_bridge BATT_LOW_MV ile ayni (24 V LiFePO4; DM860H min 24 V)
LOCALIZATION_MAX_STD_M = 0.5      # AMCL belirsizligi bundan buyukse "emin degil"
LOCALIZATION_MAX_STD_DEG = 30.0
# Nav2 acilis bekcisi (2026-10-01): Pi acilirken (Chromium + tum dugumler ayni
# anda yuklenirken) bir lifecycle servis cevabi kaybolabiliyor; o zaman
# lifecycle_manager sonsuza kadar takiliyor, AMCL hic acilmiyor ve robot
# masalara GIDEMIYOR. Bekci iki yoneticiye de "aktif misin" diye sorar, bu
# sure icinde ikisi de evet demezse Nav2'yi temizce yeniden baslatir.
NAV2_STARTUP_TIMEOUT_S = 75.0     # Pi'de normal acilis ~20-40 sn
NAV2_MAX_START_ATTEMPTS = 3
NAV2_LIFECYCLE_MANAGERS = ('lifecycle_manager_localization', 'lifecycle_manager_navigation')
HEALTH_NAMES = {'brain': 'beyin', 'stm32': 'STM32', 'lidar': 'LiDAR', 'estop': 'acil stop',
                'bumper': 'tampon', 'battery': 'batarya', 'localization': 'konum'}

# Telefon uygulamasi robotu IP bilmeden bulur: ag yayinina "CRYVEX?" gonderir,
# robot bu porttan JSON ile cevap verir, uygulama cevabin geldigi adresi kullanir.
# Boylece modem IP'yi degistirse ya da robot baska bir kafenin agina gecse de
# kimse adres girmez (2026-09-26: Pi 192.168.1.7 -> .8 oldu, uygulama kopmustu).
DISCOVERY_PORT = 47474
DISCOVERY_QUERY = b'CRYVEX?'

# Robot yuzunun renkleri - garson telefondan secer, config/theme.json'da kalici.
# index.html bu anahtarlari CSS degiskenlerine uygular (applyTheme).
THEME_DEFAULT = {'eye': '#00d4ff', 'mouth': '#00d4ff', 'light': '#1fa2ff', 'face': '#06060e'}
THEME_COLOR_RE = re.compile(r'^#[0-9a-fA-F]{6}$')

# Govdesiz arayuz uclari -> patrol.py komutu (sim tablet_server.py ile AYNI)
PATROL_SIMPLE_COMMANDS = {
    '/api/start_patrol': 'start',
    '/api/go_home': 'go_home',
    '/api/wander': 'wander',
    '/api/greet_door': 'greet_door',
    '/api/resume_patrol': 'resume',
    '/api/interacting': 'interacting',
    '/api/done_interacting': 'done_interacting',
    '/api/msg_open': 'msg_open',
    '/api/msg_compose_start': 'msg_compose_start',
    '/api/msg_compose_cancel': 'msg_compose_cancel',
    '/api/rescue_stop': 'rescue_stop',
}
LAST_POSE_SAVE_S = 5.0        # son AMCL konumu en fazla bu siklikta diske yazilir

JOY_MAX_LIN = 0.30
JOY_MAX_ANG = 1.00

ROBOT_EXPRESSIONS = ('happy', 'love', 'alert', 'sad')  # index.html setExpression() ile ayni

TTS_VOICE = 'tr-TR-EmelNeural'
TTS_CACHE_DIR = os.path.expanduser('~/.cache/cryvex_tts')  # /tmp degil: acilista silinmesin


# ==================== PNG kodlama (tablet_server.py'den birebir) ====================
def _png_bytes(width, height, pixels, channels=1):
    """channels: 1 = gri tonlu, 3 = RGB."""
    stride = width * channels
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw.extend(pixels[y * stride:(y + 1) * stride])
    compressed = zlib.compress(bytes(raw), 1)  # 2026-10-03: 6 -> 1, Pi'de ~3 kat hizli (canli harita sik isteniyor)

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    color_type = 0 if channels == 1 else 2
    ihdr = struct.pack('>IIBBBBB', width, height, 8, color_type, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
            chunk(b'IDAT', compressed) + chunk(b'IEND', b''))


def _occgrid_gray(msg):
    """OccupancyGrid -> gri tonlu pikseller (PNG yonunde: ust satir = haritanin kuzeyi)."""
    w, h = msg.info.width, msg.info.height
    pixels = bytearray(w * h)
    for i, v in enumerate(msg.data):
        pixels[i] = 205 if v < 0 else max(0, min(255, round(254 - (v / 100.0) * 254)))
    flipped = bytearray(w * h)
    for row in range(h):
        src, dst = row * w, (h - 1 - row) * w
        flipped[dst:dst + w] = pixels[src:src + w]
    return flipped


SCAN_RGB = (255, 45, 45)   # LiDAR'in su an gordugu noktalar
ROBOT_RGB = (0, 170, 255)  # robotun konumu + onunun baktigi yon (ok)
PLAN_RGB = (255, 213, 0)   # Nav2'nin planladigi yol (telefon ana ekrani)
MARK_RGB = {'table': (0, 229, 255), 'barista': (0, 255, 136), 'door': (255, 170, 0)}
MARK_TEXT_RGB = (4, 18, 26)
PLAN_FRESH_S = 20.0        # bundan eski /plan cizilmez (robot artik o yolda degil)
GRID_RGB = np.array((0, 150, 185), dtype=np.float32)        # 1 m'lik izgara cizgileri
GRID_LABEL_RGB = np.array((0, 100, 130), dtype=np.float32)  # kare etiketleri (A1, B1, ...)
GRID_SCALE = 4       # izgarali resimde her harita hucresi 4x4 piksel - etiketler telefonda okunsun
GRID_FONT_SCALE = 5  # 3x5 harfin her pikseli 5x5 -> 15x25 piksel harf (1 m = 80 piksel)

# 3x5 piksel yazi tipi (satir satir, soldan saga) - sadece kare etiketleri icin.
_FONT_3X5 = {
    '0': '111101101101111', '1': '010110010010111', '2': '111001111100111', '3': '111001111001111',
    '4': '101101111001001', '5': '111100111001111', '6': '111100111101111', '7': '111001001001001',
    '8': '111101111101111', '9': '111101111001111',
    'A': '010101111101101', 'B': '110101110101110', 'C': '011100100100011', 'D': '110101101101110',
    'E': '111100110100111', 'F': '111100110100100', 'G': '011100101101011', 'H': '101101111101101',
    'I': '111010010010111', 'J': '001001001101010', 'K': '101101110101101', 'L': '100100100100111',
    'M': '101111111101101', 'N': '111101101101101', 'O': '010101101101010', 'P': '110101110100100',
    'Q': '010101101110011', 'R': '110101110101101', 'S': '011100010001110', 'T': '111010010010010',
    'U': '101101101101111', 'V': '101101101101010', 'W': '101101111111101', 'X': '101101010101101',
    'Y': '101101010010010', 'Z': '111001010100111',
}


def _grid_col_name(i):
    # setup.html colName() ile AYNI: A..Z, sonra AA, AB, ...
    return (chr(64 + i // 26) if i >= 26 else '') + chr(65 + i % 26)


@functools.lru_cache(maxsize=512)
def _text_mask(text, scale):
    """Metin -> bool maske (3x5 harfler, aralarinda 1 piksel bosluk, her piksel scale x scale)."""
    parts = []
    for k, ch in enumerate(text):
        if k:
            parts.append(np.zeros((5, 1), dtype=np.uint8))
        parts.append(np.array([int(b) for b in _FONT_3X5[ch]], dtype=np.uint8).reshape(5, 3))
    return np.kron(np.hstack(parts), np.ones((scale, scale), dtype=np.uint8)).astype(bool)


def _blend(region, color, alpha, mask=None):
    """region (resmin bir gorunumu) uzerine rengi alpha seffafligiyla boyar (harita alttan gorunur)."""
    if mask is None:
        region[...] = region * (1.0 - alpha) + color * alpha
    else:
        region[mask] = region[mask] * (1.0 - alpha) + color * alpha


LIVE_MAP_CACHE_S = 1.0          # canli harita PNG onbellek suresi (bkz. live_map_png_bytes)
_LIVE_PNG_CACHE = {}
_LIVE_PNG_LOCK = threading.Lock()


def _live_map_png(msg, scan_xy, robot_pose, grid=True, marks=None, plan_xy=None, crop=False):
    """Canli harita + LiDAR'in su an gordugu noktalar (kirmizi) + robot ve onunun
    baktigi yon (mavi daire + ok). grid=True: silik 1 m'lik izgara ve kare
    etiketleri (A1, B1, ...), etiketler okunsun diye resim GRID_SCALE kat buyuk.
    Kare adlari setup.html'deki izgarayla AYNI (resmin sol ustu = A1)."""
    w, h = msg.info.width, msg.info.height
    res = msg.info.resolution
    ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
    s = GRID_SCALE if grid else 1
    gray = np.frombuffer(bytes(_occgrid_gray(msg)), dtype=np.uint8).reshape(h, w)
    img = np.repeat(np.repeat(gray, s, axis=0), s, axis=1)[:, :, None].repeat(3, axis=2)
    img_h, img_w = img.shape[:2]

    if grid:
        step = s / res  # 1 m kac piksel
        for k in range(1, int(img_w / step) + 1):
            x = round(k * step)
            _blend(img[:, max(0, x - 1):x + 1], GRID_RGB, 0.4)
        for k in range(1, int(img_h / step) + 1):
            y = round(k * step)
            _blend(img[max(0, y - 1):y + 1, :], GRID_RGB, 0.4)
        pad = round(1.5 * s)
        for c in range(math.ceil(img_w / step)):
            for r in range(math.ceil(img_h / step)):
                mask = _text_mask(f'{_grid_col_name(c)}{r + 1}', GRID_FONT_SCALE)
                x0, y0 = round(c * step) + pad, round(r * step) + pad
                sub = img[y0:y0 + mask.shape[0], x0:x0 + mask.shape[1]]
                _blend(sub, GRID_LABEL_RGB, 0.55, mask[:sub.shape[0], :sub.shape[1]])

    def to_px(x, y):
        return (x - ox) / res * s, (h - (y - oy) / res) * s

    def square(px, py, half, color):
        c, r = math.floor(px), math.floor(py)
        img[max(0, r - half):max(0, r + half + 1), max(0, c - half):max(0, c + half + 1)] = color

    yy, xx = np.ogrid[:img_h, :img_w]

    # Planlanan yol (telefon: robot nereye, hangi yoldan gidiyor)
    if plan_xy:
        thick = max(0, s // 3)
        for (xa, ya), (xb, yb) in zip(plan_xy, plan_xy[1:]):
            pa, pb = to_px(xa, ya), to_px(xb, yb)
            n = max(1, int(math.hypot(pb[0] - pa[0], pb[1] - pa[1]) * 2))
            for i in range(n + 1):
                square(pa[0] + (pb[0] - pa[0]) * i / n, pa[1] + (pb[1] - pa[1]) * i / n, thick, PLAN_RGB)

    # Masa / Us / Kapi isaretleri: renkli daire + numara/harf
    for kind, label, mx, my in (marks or []):
        cx, cy = to_px(mx, my)
        rad = 0.22 / res * s
        img[(xx - cx) ** 2 + (yy - cy) ** 2 <= rad ** 2] = MARK_RGB[kind]
        fs = max(1, int(rad * 1.1 / 5))
        mask = _text_mask(label, fs)
        x0, y0 = int(cx - mask.shape[1] / 2), int(cy - mask.shape[0] / 2)
        if x0 >= 0 and y0 >= 0:
            sub = img[y0:y0 + mask.shape[0], x0:x0 + mask.shape[1]]
            sub[mask[:sub.shape[0], :sub.shape[1]]] = MARK_TEXT_RGB

    scan_half = max(1, (3 * s) // 2 - 1)
    for x, y in scan_xy:
        square(*to_px(x, y), scan_half, SCAN_RGB)
    if robot_pose is not None:
        rx, ry, yaw = robot_pose
        cx, cy = to_px(rx, ry)
        radius = 0.15 / res * s
        img[(xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2] = ROBOT_RGB
        # Ok: robotun ONU (base_footprint +x). 2026-10-02: eskiden LiDAR'in pozu
        # kullaniliyordu; LiDAR -128 derece donuk ve kacik takilinca ok yanlis
        # yonu gosteriyordu - artik base_footprint (bkz. _scan_in_map).
        length, thick = 0.6 / res * s, max(0, s // 2 - 1)
        tip = (cx + length * math.cos(yaw), cy - length * math.sin(yaw))
        head = length * 0.35
        for (sx, sy), ang, seg in (((cx, cy), yaw, length),
                                   (tip, yaw + math.radians(150), head),
                                   (tip, yaw - math.radians(150), head)):
            for i in range(int(seg * 2) + 1):  # yarim piksel adimlarla cizgi
                square(sx + math.cos(ang) * i / 2, sy - math.sin(ang) * i / 2, thick, ROBOT_RGB)
    if crop:
        # Telefon: sadece haritanin bilinen kismi (+ robot) ve 1 m pay - koca gri alan gitmesin.
        known = np.argwhere(gray != 205)
        if len(known):
            r0, c0 = known.min(0)
            r1, c1 = known.max(0)
            if robot_pose is not None:
                rc, rr = int((robot_pose[0] - ox) / res), int(h - (robot_pose[1] - oy) / res)
                r0, r1, c0, c1 = min(r0, rr), max(r1, rr), min(c0, rc), max(c1, rc)
            pad = int(1.0 / res)
            r0, c0 = max(0, r0 - pad) * s, max(0, c0 - pad) * s
            r1, c1 = min(h, r1 + pad + 1) * s, min(w, c1 + pad + 1) * s
            img = np.ascontiguousarray(img[r0:r1, c0:c1])
            img_h, img_w = img.shape[:2]
    return _png_bytes(img_w, img_h, img.tobytes(), 3)


def _parse_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    pos = 0

    def next_token():
        nonlocal pos
        while True:
            while pos < len(data) and data[pos:pos + 1].isspace():
                pos += 1
            if pos < len(data) and data[pos:pos + 1] == b'#':
                while pos < len(data) and data[pos:pos + 1] != b'\n':
                    pos += 1
                continue
            break
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        return data[start:pos]

    magic = next_token()
    width = int(next_token())
    height = int(next_token())
    int(next_token())
    pos += 1
    if magic == b'P5':
        pixels = data[pos:pos + width * height]
    elif magic == b'P2':
        vals = bytearray()
        while len(vals) < width * height:
            vals.append(int(next_token()))
        pixels = bytes(vals)
    else:
        raise ValueError(f'Desteklenmeyen PGM turu: {magic}')
    return width, height, pixels


def _write_pgm_atomic(path, width, height, pixels):
    """P5 PGM'i once .tmp'ye yazip yerine koyar - yazarken elektrik/surec
    kesilse bile harita dosyasi yarim kalmaz."""
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode() + bytes(pixels))
    os.replace(tmp, path)


# Harita duzenleme (kurulum ekranindaki fircalar): PGM piksel degerleri.
# map_saver trinary: 254 = bos, 0 = dolu (duvar), 205 = bilinmiyor.
MAP_EDIT_VALUES = {'free': 254, 'occ': 0, 'unknown': 205}
MAP_EDIT_MAX_RADIUS_PX = 40


def _pgm_to_png_bytes(path):
    width, height, pixels = _parse_pgm(path)
    return _png_bytes(width, height, pixels)


# ==================== Ses (TTS) ====================
# Robotun TEK sesi edge-tts (kadin). Baska bir sese (Piper/tarayici) dusmez:
# internet yokken ve cumle daha once hic uretilmediyse sessiz gecer.
def _tts_path(text):
    """text -> robotun sesiyle uretilmis mp3'un onbellek yolu ya da None."""
    key = hashlib.sha1(f'edge:{TTS_VOICE}:{text}'.encode('utf-8')).hexdigest()
    path = os.path.join(TTS_CACHE_DIR, key + '.mp3')
    if os.path.isfile(path) and os.path.getsize(path) > 0:
        return path
    if not _EDGE_TTS_AVAILABLE:
        return None
    os.makedirs(TTS_CACHE_DIR, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=TTS_CACHE_DIR, suffix='.part')
    os.close(fd)
    try:
        asyncio.run(asyncio.wait_for(edge_tts.Communicate(text, TTS_VOICE).save(tmp), timeout=8.0))
        if os.path.getsize(tmp) == 0:
            return None
        os.replace(tmp, path)  # atomik: yarim yazilmis dosya onbellekte asla gorunmez
        return path
    except Exception:  # noqa: BLE001
        return None
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


def _ui_phrases(web_dir):
    """index.html'deki sabit cumleler: speak/robotSays('...') + *_PHRASES/*_LINES dizileri."""
    try:
        with open(os.path.join(web_dir, 'index.html'), encoding='utf-8') as f:
            html = f.read()
    except OSError:
        return []
    phrases = set(re.findall(r"(?:speak|robotSays)\('([^'\\]+)'\)", html))
    for body in re.findall(r'const [A-Z_]+(?:PHRASES|LINES) = \[(.*?)\];', html, re.DOTALL):
        phrases.update(re.findall(r"'([^'\\]+)'", body))
    return sorted(phrases)


# ==================== Hoparlor sesi (PulseAudio) ====================
# cafe_ui_server systemd servisi olarak (User=main) calisiyor ama systemd
# system-unit'leri varsayilan olarak XDG_RUNTIME_DIR ayarlamaz - bu olmadan
# pactl "Connection refused" verir (kullanicinin oturum PulseAudio'suna
# ulasamaz). Asagida her cagrida acikca ekleniyor.
def _pactl_env():
    env = dict(os.environ)
    env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    return env


def get_volume_pct():
    try:
        result = subprocess.run(
            ['pactl', 'get-sink-volume', '@DEFAULT_SINK@'],
            capture_output=True, text=True, timeout=3.0, env=_pactl_env())
        if result.returncode != 0:
            return None
        m = re.search(r'(\d+)%', result.stdout)
        return int(m.group(1)) if m else None
    except Exception:  # noqa: BLE001
        return None


def set_volume_pct(pct):
    pct = max(0, min(100, int(pct)))
    try:
        result = subprocess.run(
            ['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{pct}%'],
            capture_output=True, text=True, timeout=3.0, env=_pactl_env())
        return result.returncode == 0
    except Exception:  # noqa: BLE001
        return False



class LaunchManager:
    """Nav2 (navigation.launch.py: AMCL + surus) ile canli haritalama
    (mapping.launch.py: slam_toolbox) SIRAYLA calisir, ASLA ayni anda degil -
    ikisi de /map + map->odom TF yayinlar, birlikte catisir (2026-09-25'te
    konum testi acikken haritalama acilinca LiDAR noktalari duvarlardan
    kaydi). Hangisinin ayakta oldugunu TEK YERDEN yonetir; her biri kendi
    surec grubunda, gecerken digeri temiz kapatilir (sim tablet_server deseni)."""

    def __init__(self, node):
        self.node = node
        self.proc = None
        self.kind = None   # 'nav' | 'slam' | None
        self._lock = threading.Lock()

    def is_running(self, kind=None):
        alive = self.proc is not None and self.proc.poll() is None
        return alive and (kind is None or self.kind == kind)

    def _stop_locked(self):
        if self.proc is not None and self.proc.poll() is None:
            pid = self.proc.pid
            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
                self.proc.wait(timeout=10.0)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(pid), signal.SIGKILL)
                    self.proc.wait(timeout=3.0)
                except Exception:  # noqa: BLE001
                    pass
            self.node.get_logger().info(f"[LaunchManager] '{self.kind}' durduruldu.")
        self.proc = None
        self.kind = None

    def stop(self):
        with self._lock:
            self._stop_locked()

    def start_slam(self):
        with self._lock:
            if self.is_running('slam'):
                return
            self._stop_locked()
            self.node.get_logger().info('[LaunchManager] Haritalama (slam_toolbox) basliyor...')
            cmd = ['ros2', 'launch', 'cryvex_bringup', 'mapping.launch.py']
            self.proc = subprocess.Popen(cmd, start_new_session=True)
            self.kind = 'slam'

    def start_nav(self, map_yaml):
        with self._lock:
            self._stop_locked()
            self.node.get_logger().info(f'[LaunchManager] Nav2 (AMCL + surus) basliyor: {map_yaml}')
            cmd = ['ros2', 'launch', 'cryvex_bringup', 'navigation.launch.py', f'map:={map_yaml}']
            self.proc = subprocess.Popen(cmd, start_new_session=True)
            self.kind = 'nav'


class CafeUiServerNode(Node):
    def __init__(self):
        super().__init__('cafe_ui_server')
        self.declare_parameter('http_port', 8080)
        port = self.get_parameter('http_port').value

        real_script = os.path.realpath(os.path.abspath(__file__))
        src_root = os.path.dirname(os.path.dirname(real_script))
        src_web_dir = os.path.join(src_root, 'web')
        src_maps_dir = os.path.join(src_root, 'maps')
        src_config_dir = os.path.join(src_root, 'config')
        if os.path.isdir(src_web_dir):
            self.web_dir = src_web_dir
            self.maps_dir = src_maps_dir
            self.config_dir = src_config_dir
        else:
            share_dir = get_package_share_directory('cryvex_bringup')
            self.web_dir = os.path.join(share_dir, 'web')
            self.maps_dir = os.path.join(share_dir, 'maps')
            self.config_dir = os.path.join(share_dir, 'config')
        os.makedirs(self.maps_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._live_map_msg = None
        self._map_png_cache = None
        self.mode = 'operating'
        self.screen_on = True
        self.speak_text, self.speak_expr, self.speak_seq = '', '', 0
        self.theme = self._load_theme()
        self._orders_lock = threading.Lock()
        self.orders = self._load_orders()
        self.orders_seq = 1   # her degisiklikte artar - telefon listeyi o zaman yeniden ceker

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, MAP_QOS)
        self._scan_msg, self._scan_rx = None, 0.0
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self._plan = ([], 0.0)   # Nav2 /plan: telefondaki canli haritada sari yol
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/patrol_command', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nomotion_cli = self.create_client(Empty, '/request_nomotion_update')
        self.launch_mgr = LaunchManager(self)

        # patrol.py'nin durumu (JSON) + robotun kendi ekraninda okunacak mesajlari
        self._patrol_status, self._patrol_rx = None, 0.0
        self._patrol_speak_seq = None
        self.create_subscription(String, '/patrol_status', self._patrol_status_cb, 10)
        # Son AMCL konumu -> config/last_pose.json (robot kapanip ayni yerde
        # acilinca kendini bilsin; "Robot Burada"ya gerek kalmasin)
        self._last_pose_saved = (None, 0.0)   # ((x, y, yaw), zaman)
        self._amcl_rx_t = 0.0                 # son /amcl_pose (monotonic)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, MAP_QOS)

        # "Robot Sagligi" (telefon ana paneli): STM32'nin her durum satiriyla
        # (20 Hz) gelen acil stop / tampon / batarya + kart bilgisi + konum guveni.
        self._hw = {'estop': None, 'bumper': None, 'battery_v': None, 'rx': 0.0, 'info': None}
        self._amcl_std = None          # (x m, y m, yaw derece) - son AMCL belirsizligi
        self._nav_started_t = 0.0
        self._nav_watch_gen = 0          # her Nav2 baslatmasinda artar (eski bekci kendini kapatir)
        self._nav_boot = None            # None | ('starting', deneme) | ('failed', deneme)
        self._nav_active_clients = {
            m: self.create_client(Trigger, f'/{m}/is_active') for m in NAV2_LIFECYCLE_MANAGERS}
        self.create_subscription(Bool, '/estop_state', lambda m: self._hw_rx('estop', m.data), 10)
        self.create_subscription(Bool, '/bumper_state', lambda m: self._hw_rx('bumper', m.data), 10)
        self.create_subscription(BatteryState, '/battery_state',
                                 lambda m: self._hw_rx('battery_v', float(m.voltage)), 10)
        self.create_subscription(
            String, '/stm32_info', self._stm32_info_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._httpd = ThreadingHTTPServer(('0.0.0.0', port), self._make_handler())
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()
        threading.Thread(target=self._prewarm_tts, daemon=True).start()
        threading.Thread(target=self._discovery_loop, args=(port,), daemon=True).start()
        self.get_logger().info(f'Kafe arayuzu hazir: http://0.0.0.0:{port}/')
        # Kayitli harita varsa Nav2'yi (AMCL + surus) hemen baslat; yoksa
        # "Ortami Haritala" beklenir. Masalar tanimli olmasa da baslar - kurulum
        # ekranindaki "Robot Burada" AMCL'e ihtiyac duyar.
        if self.has_saved_map():
            self.start_nav_and_localize()
        else:
            self.get_logger().warn('Kayitli harita yok - Nav2 baslatilmadi, "Ortami Haritala" bekleniyor.')

    # ---- yuz renkleri (telefondan) ----
    def _theme_path(self):
        return os.path.join(self.config_dir, 'theme.json')

    def _load_theme(self):
        theme = dict(THEME_DEFAULT)
        try:
            with open(self._theme_path(), encoding='utf-8') as f:
                saved = json.load(f)
            theme.update({k: v for k, v in saved.items()
                          if k in THEME_DEFAULT and THEME_COLOR_RE.match(str(v))})
        except (OSError, ValueError):
            pass
        return theme

    def update_theme(self, changes):
        """changes: {'eye'|'mouth'|'light'|'face': '#rrggbb'} ya da {'reset': true}."""
        if changes.get('reset'):
            theme = dict(THEME_DEFAULT)
        else:
            theme = dict(self.theme)
            for key, value in changes.items():
                if key not in THEME_DEFAULT:
                    raise ValueError(f'bilinmeyen renk: {key}')
                if not THEME_COLOR_RE.match(str(value)):
                    raise ValueError(f'gecersiz renk: {value}')
                theme[key] = str(value).lower()
        tmp = self._theme_path() + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(theme, f)
        os.replace(tmp, self._theme_path())
        self.theme = theme
        self.get_logger().info(f'Yuz renkleri: {theme}')
        return theme

    def _discovery_loop(self, http_port):
        """Uygulamanin "CRYVEX?" yayinina cevap (bkz. DISCOVERY_PORT)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', DISCOVERY_PORT))
        except OSError as e:
            self.get_logger().error(f'Kesif portu {DISCOVERY_PORT} acilamadi ({e}) - uygulama IP ile baglanmali.')
            return
        reply = json.dumps({'cryvex': 'robot', 'port': http_port, 'name': socket.gethostname()}).encode()
        while True:
            try:
                data, addr = sock.recvfrom(256)
                if data.strip() == DISCOVERY_QUERY:
                    sock.sendto(reply, addr)
            except OSError:
                time.sleep(1.0)

    def _prewarm_tts(self):
        # Arayuzun sabit cumleleri internet gelir gelmez kalici onbellege
        # uretilir; acilista (internet/saat henuz hazir degilken) da hazir olsunlar.
        pending = _ui_phrases(self.web_dir)
        delay = 10.0
        while pending:
            pending = [p for p in pending if _tts_path(p) is None]
            if pending:
                self.get_logger().warn(f'{len(pending)} cumlenin sesi uretilemedi (internet?), {delay:.0f}sn sonra tekrar.')
                time.sleep(delay)
                delay = min(delay * 2, 300.0)
        self.get_logger().info('Ses onbellegi hazir (tum arayuz cumleleri).')

    def _map_cb(self, msg):
        with self._lock:
            self._live_map_msg = msg

    def _scan_cb(self, msg):
        self._scan_msg, self._scan_rx = msg, time.monotonic()

    def _scan_in_map(self):
        """LiDAR'in su an gordugu noktalar ve kendi pozu (x, y, yaw), harita cercevesinde.
        Tarama 1sn'den eskiyse ya da harita cercevesi henuz yoksa ([], None)."""
        scan = self._scan_msg
        if scan is None or time.monotonic() - self._scan_rx > 1.0:
            return [], None
        try:
            tf = self.tf_buffer.lookup_transform('map', scan.header.frame_id, RclpyTime())
        except Exception:  # noqa: BLE001
            return [], None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        points = []
        angle = scan.angle_min + yaw
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max:  # NaN/inf de burada elenir
                points.append((t.x + r * math.cos(angle), t.y + r * math.sin(angle)))
            angle += scan.angle_increment
        # Robot isareti = govde (base_footprint), LiDAR degil: LiDAR kacik/donuk takili.
        try:
            tb = self.tf_buffer.lookup_transform('map', 'base_footprint', RclpyTime())
            bt, bq = tb.transform.translation, tb.transform.rotation
            byaw = math.atan2(2.0 * (bq.w * bq.z + bq.x * bq.y), 1.0 - 2.0 * (bq.y * bq.y + bq.z * bq.z))
            return points, (bt.x, bt.y, byaw)
        except Exception:  # noqa: BLE001
            return points, (t.x, t.y, yaw)

    def _plan_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._plan = (pts[::3] + pts[-1:] if pts else [], time.monotonic())

    def _map_marks(self):
        """Kayitli masa/Us/Kapi noktalari -> [(tur, etiket, x, y)] (haritada cizmek icin)."""
        cfg = self.load_waypoints_cfg()
        marks = []
        for i, t in enumerate(cfg.get('tables') or []):
            marks.append(('table', str(i + 1), float(t['x']), float(t['y'])))
        if cfg.get('barista'):
            marks.append(('barista', 'U', float(cfg['barista']['x']), float(cfg['barista']['y'])))
        if cfg.get('door'):
            marks.append(('door', 'K', float(cfg['door']['x']), float(cfg['door']['y'])))
        return marks

    def live_map_png_bytes(self, grid=True, marks=False, plan=False, crop=False):
        # 2026-10-03: canli harita her istekte sifirdan ~330 ms cizilip
        # sikistiriliyordu; robot ekrani (1.2 sn) + telefon birlikte isteyince
        # sunucu bogulup joystick/durum isteklerini geciktiriyordu (robot
        # titreyerek gidiyor, harita yanip sonuyordu). Ayni ayarla LIVE_MAP_CACHE_S
        # icinde gelen istekler hazir resmi alir; ayni anda tek cizim yapilir.
        key = (bool(grid), bool(marks), bool(plan), bool(crop))
        with _LIVE_PNG_LOCK:
            hit = _LIVE_PNG_CACHE.get(key)
            if hit and time.monotonic() - hit[0] < LIVE_MAP_CACHE_S:
                return hit[1]
            png = self._render_live_map_png_bytes(grid, marks, plan, crop)
            if png is not None:
                _LIVE_PNG_CACHE[key] = (time.monotonic(), png)
            return png

    def _render_live_map_png_bytes(self, grid=True, marks=False, plan=False, crop=False):
        with self._lock:
            msg = self._live_map_msg
        if msg is None or msg.info.width == 0:
            return None
        points, robot = self._scan_in_map()
        mk = self._map_marks() if marks and self.mode != 'mapping' else None
        pl = None
        if plan:
            pts, t = self._plan
            if pts and time.monotonic() - t < PLAN_FRESH_S and self.launch_mgr.is_running('nav'):
                pl = pts
        return _live_map_png(msg, points, robot, grid, mk, pl, crop)

    def current_robot_yaw(self):
        """Robotun bildigi yonelim (radyan, harita cercevesi): once canli
        map->base_footprint, yoksa config/last_pose.json, o da yoksa 0."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', RclpyTime())
            q = tf.transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:  # noqa: BLE001
            pass
        try:
            with open(self._last_pose_path(), encoding='utf-8') as f:
                return float(json.load(f)['yaw'])
        except (OSError, ValueError, KeyError, TypeError):
            return 0.0

    def set_robot_pose(self, x, y, yaw, yaw_known=False):
        """Kurulum ekranindaki "Robot Burada": AMCL'e kaba ipucu (±0.5m, ±30°;
        yon robotun kendi bildigi yonse ±15°).
        Tekerlek odometrisi yokken AMCL robot hareket etmedikce guncellenmez -
        bu yuzden ardindan ~5sn boyunca "hareketsiz guncelleme" istenir ve
        lidar ipucunu duvarlara oturtarak netlestirir (odom gelince de zararsiz)."""
        if self.initialpose_pub.get_subscription_count() == 0:
            raise RuntimeError('konum sistemi (AMCL) calismiyor')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.5 ** 2
        msg.pose.covariance[35] = math.radians(15.0 if yaw_known else 30.0) ** 2
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'Robot konumu ayarlandi: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')
        threading.Thread(target=self._refine_pose, daemon=True).start()

    def _refine_pose(self):
        time.sleep(0.5)  # AMCL once /initialpose'u islesin
        # lidar 7 Hz: her istek bir sonraki taramada islenir. 15 guncelleme
        # ±0.5 m ipucunu ±0.7 m'de birakabiliyordu (2026-09-26), 30 ~±0.3 m.
        for _ in range(30):
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            time.sleep(0.3)

    # ---- Nav2 + robotun haritadaki konumu ----
    def active_map_yaml_path(self):
        return os.path.join(self.maps_dir, 'cafe_map.yaml')

    def _map_stamp(self):
        """Kayitli haritanin kimligi: yeniden haritalayinca degisir, boylece
        eski haritaya ait kayitli konum yeni haritada kullanilmaz."""
        try:
            return f'{os.path.getmtime(self.active_map_yaml_path()):.0f}'
        except OSError:
            return None

    def _last_pose_path(self):
        return os.path.join(self.config_dir, 'last_pose.json')

    def _amcl_pose_cb(self, msg):
        self._amcl_rx_t = time.monotonic()
        c = msg.pose.covariance
        self._amcl_std = (math.sqrt(abs(c[0])), math.sqrt(abs(c[7])), math.degrees(math.sqrt(abs(c[35]))))
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        (prev, prev_t) = self._last_pose_saved
        now = time.monotonic()
        if prev is not None and (now - prev_t < LAST_POSE_SAVE_S or (
                math.hypot(p.x - prev[0], p.y - prev[1]) < 0.05
                and abs(math.remainder(yaw - prev[2], math.tau)) < math.radians(3))):
            return
        self._last_pose_saved = ((p.x, p.y, yaw), now)
        data = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 4), 'map': self._map_stamp()}
        try:
            tmp = self._last_pose_path() + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(data, f)
            os.replace(tmp, self._last_pose_path())
        except OSError as e:
            self.get_logger().warn(f'last_pose.json yazilamadi: {e}')

    def load_last_pose(self):
        try:
            with open(self._last_pose_path(), encoding='utf-8') as f:
                d = json.load(f)
        except (OSError, ValueError):
            return None
        if d.get('map') != self._map_stamp():
            return None   # baska bir haritaya ait (yeniden haritalandi)
        return float(d['x']), float(d['y']), float(d['yaw'])

    def start_nav_and_localize(self, pose=None, _attempt=1):
        """Nav2'yi kayitli haritayla baslatir ve AMCL hazir olunca robotun
        konumunu verir: pose (haritalama sonrasi SLAM'in son konumu) ya da
        son kaydedilen konum. Ikisi de yoksa "Robot Burada" ile elle verilir -
        Nav2'nin surus kismi konum gelene kadar beklemede kalir."""
        self.launch_mgr.start_nav(self.active_map_yaml_path())
        self._nav_started_t = time.monotonic()
        pose = pose or self.load_last_pose()
        self._nav_watch_gen += 1
        self._nav_boot = ('starting', _attempt)
        threading.Thread(target=self._nav_watchdog, args=(self._nav_watch_gen, _attempt, pose),
                         daemon=True).start()
        if pose is None:
            self.get_logger().warn('Bilinen robot konumu yok - kurulum ekranindan "Robot Burada" ile verin.')
            return
        threading.Thread(target=self._localize_when_ready, args=(pose, self._nav_watch_gen),
                         daemon=True).start()

    def _nav2_is_active(self, timeout=3.0):
        """Iki lifecycle_manager da 'aktif' diyorsa True. Takilmis bir yonetici
        cevap vermez - zaman asimi da 'aktif degil' sayilir."""
        for cli in self._nav_active_clients.values():
            if not cli.service_is_ready():
                return False
            fut = cli.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < timeout:
                time.sleep(0.1)
            if not fut.done():
                fut.cancel()
                return False
            res = fut.result()
            if res is None or not res.success:
                return False
        return True

    def _nav_watchdog(self, gen, attempt, pose):
        log = self.get_logger()
        t0 = time.monotonic()
        while time.monotonic() - t0 < NAV2_STARTUP_TIMEOUT_S:
            if gen != self._nav_watch_gen or not self.launch_mgr.is_running('nav'):
                return   # bu arada haritalama ya da yeni bir Nav2 baslatmasi devraldi
            if self._nav2_is_active():
                self._nav_boot = None
                log.info(f'[Nav2 bekcisi] Nav2 hazir ({time.monotonic() - t0:.0f} sn, deneme {attempt}).')
                return
            time.sleep(2.0)
        if gen != self._nav_watch_gen or not self.launch_mgr.is_running('nav'):
            return
        if attempt >= NAV2_MAX_START_ATTEMPTS:
            self._nav_boot = ('failed', attempt)
            log.error(f'[Nav2 bekcisi] Nav2 {attempt} denemede de acilamadi - robotu yeniden baslatin.')
            return
        log.warn(f'[Nav2 bekcisi] Nav2 {NAV2_STARTUP_TIMEOUT_S:.0f} sn icinde acilmadi '
                 f'(deneme {attempt}/{NAV2_MAX_START_ATTEMPTS}) - yeniden baslatiliyor.')
        self.start_nav_and_localize(pose, _attempt=attempt + 1)

    def _localize_when_ready(self, pose, gen=None):
        # AMCL /initialpose'a abone olsa bile AKTIF olana kadar gelen konumu yok
        # sayar - bu yuzden AMCL'in cevap olarak /amcl_pose yayinladigini
        # gorene kadar tekrarlanir (Pi'de Nav2'nin acilmasi ~15-30 sn).
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and self.launch_mgr.is_running('nav'):
            if gen is not None and gen != self._nav_watch_gen:
                return   # Nav2 bu arada yeniden baslatildi; konumu yeni deneme verir
            if self.initialpose_pub.get_subscription_count() > 0:
                sent = time.monotonic()
                try:
                    self.set_robot_pose(*pose)
                except RuntimeError:
                    pass
                time.sleep(2.5)
                if self._amcl_rx_t > sent:
                    self.get_logger().info(
                        f'Robot konumu verildi: x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f}°')
                    return
            else:
                time.sleep(1.0)
        if gen is not None and gen != self._nav_watch_gen:
            return   # yeniden baslatma sirasinda cikildi - uyari yeni denemenin isi
        self.get_logger().warn('Robot konumu AMCL\'e verilemedi (Nav2 acilmadi?) - "Robot Burada" ile verin.')

    def _capture_slam_pose(self):
        """Haritalama BITMEDEN (slam_toolbox hala ayaktayken) robotun o anki
        konumu: yeni haritanin cercevesinde, AMCL'e baslangic olarak verilir."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', RclpyTime())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Robot konumu SLAM\'den okunamadi ({e}).')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def relocalize_to_home(self):
        """"Robotu use sabitle": robotu barmen noktasinda kabul et (elle istenirse)."""
        b = self.load_waypoints_cfg().get('barista')
        if not b:
            raise RuntimeError('barmen noktasi tanimli degil')
        self.set_robot_pose(float(b['x']), float(b['y']), float(b.get('yaw', 0.0)))

    # ---- Siparisler ----
    def _orders_path(self):
        return os.path.join(self.config_dir, 'orders.json')

    def _load_orders(self):
        try:
            with open(self._orders_path(), encoding='utf-8') as f:
                orders = json.load(f)
            return orders if isinstance(orders, list) else []
        except (OSError, ValueError):
            return []

    def _save_orders_locked(self):
        self.orders = self.orders[-ORDERS_KEEP:]
        self.orders_seq += 1
        tmp = self._orders_path() + '.tmp'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(self.orders, f, ensure_ascii=False)
            os.replace(tmp, self._orders_path())
        except OSError as e:
            self.get_logger().warn(f'orders.json yazilamadi: {e}')

    def _find_order_locked(self, order_id):
        return next((o for o in self.orders if o['id'] == order_id), None)

    def add_order(self, raw):
        """Robot ekranindan gelen siparis: {"items": [{name, qty, price}], "table", "total"}."""
        try:
            data = json.loads(raw or '{}')
            items = [{'name': str(i['name']), 'qty': int(i['qty']), 'price': float(i.get('price', 0))}
                     for i in data.get('items') or []]
        except (ValueError, TypeError, KeyError):
            items, data = [], {}
        if not items:
            return None
        with self._orders_lock:
            order = {'id': max((o['id'] for o in self.orders), default=0) + 1,
                     'table': str(data.get('table') or 'Masa'), 'items': items,
                     'total': data.get('total') or sum(i['qty'] * i['price'] for i in items),
                     'created': time.time(), 'updated': time.time(), 'status': 'preparing'}
            self.orders.append(order)
            self._save_orders_locked()
        self.get_logger().info(f"Yeni siparis #{order['id']}: {order['table']} - "
                               + ', '.join(f"{i['qty']}x {i['name']}" for i in items))
        return order

    def order_ready(self, order_id):
        """Barmen "Hazir - Robot Gotursun": ASLA "mesgul" diye reddedilmez -
        robot siraya alir. Yolda/bostaysa hemen gelir; musteriyle ilgileniyor
        ya da baska teslimattaysa o isi biter bitmez gelir (patrol.py).
        Donus: (siparis, robot hemen mi geliyor)."""
        if not self.patrol_alive():
            raise RuntimeError('devriye beyni (patrol.py) çalışmıyor')
        tables = [t.get('isim') for t in (self.load_waypoints_cfg().get('tables') or [])]
        with self._orders_lock:
            order = self._find_order_locked(order_id)
            if order is None or order['status'] != 'preparing':
                raise RuntimeError('sipariş bulunamadı ya da zaten sırada/yolda/teslim edildi')
            if order['table'] not in tables:
                raise RuntimeError(f"{order['table']} kurulumda tanımlı değil")
            order['status'], order['updated'] = 'ready', time.time()
            self._save_orders_locked()
        status = self._patrol_status or {}
        now = (status.get('state') in PATROL_INTERRUPTIBLE and not status.get('delivery_order_id')
               and not status.get('delivery_queue'))
        self.publish_patrol(f"deliver:{tables.index(order['table']) + 1}:{order_id}")
        return order, now

    def order_cancel(self, order_id):
        with self._orders_lock:
            order = self._find_order_locked(order_id)
            if order is None or order['status'] not in ('preparing', 'ready'):
                raise RuntimeError('robot bu siparişi aldı; iptal için önce robotu durdurun')
            was_queued = order['status'] == 'ready'
            order['status'], order['updated'] = 'cancelled', time.time()
            self._save_orders_locked()
        if was_queued:
            self.publish_patrol(f'undeliver:{order_id}')   # robotun teslimat sirasindan cikar

    def robot_continue(self):
        """Kocaman "Devam Et": barmende/masada bekleyen robot yola devam eder."""
        state = (self._patrol_status or {}).get('state', '')
        if state not in ('at_barista', 'pickup_wait', 'served_wait'):
            raise RuntimeError('robot şu an bir onay beklemiyor')
        if state == 'served_wait':   # musteri aldi -> teslim edildi
            order_id = (self._patrol_status or {}).get('delivery_order_id')
            with self._orders_lock:
                order = self._find_order_locked(order_id)
                if order is not None and order['status'] == 'delivering':
                    order['status'], order['updated'] = 'delivered', time.time()
                    self._save_orders_locked()
        self.publish_patrol('continue')

    def _check_delivery(self, status):
        # Siparis durumunu patrol.py'nin GERCEK teslimat durumuyla esle:
        # robotun elindeki -> 'delivering', sirasindaki -> 'ready'. Ikisinde de
        # yoksa (Durdur, haritalama, cokme...) "hazirlaniyor"a geri al ki barmen
        # tekrar gonderebilsin (komut yeni gonderildiyse kisa bir sure bekle).
        current = status.get('delivery_order_id')
        queue = status.get('delivery_queue') or []
        with self._orders_lock:
            changed = False
            for order in self.orders:
                if order['status'] not in ('ready', 'delivering'):
                    continue
                if order['id'] == current:
                    new = 'delivering'
                elif order['id'] in queue:
                    new = 'ready'
                elif time.time() - order['updated'] > ORDER_REVERT_GRACE_S:
                    new = 'preparing'
                    self.get_logger().warn(f"Siparis #{order['id']} teslimati yarida kaldi -> tekrar 'hazirlaniyor'.")
                else:
                    continue
                if new != order['status']:
                    order['status'], order['updated'] = new, time.time()
                    changed = True
            if changed:
                self._save_orders_locked()

    def orders_view(self):
        with self._orders_lock:
            active = [o for o in self.orders if o['status'] in ACTIVE_ORDER_STATUSES]
            done = [o for o in self.orders if o['status'] in ('delivered', 'cancelled')][-10:]
            return {'seq': self.orders_seq, 'now': time.time(),
                    'orders': sorted(active, key=lambda o: o['id']) + list(reversed(done))}

    def delivering_order(self):
        with self._orders_lock:
            return next((dict(o) for o in self.orders if o['status'] == 'delivering'), None)

    # ---- Robot Sagligi ----
    def _hw_rx(self, key, value):
        self._hw[key] = value
        self._hw['rx'] = time.monotonic()

    def _stm32_info_cb(self, msg):
        try:
            self._hw['info'] = json.loads(msg.data)
        except ValueError:
            pass

    def health_payload(self):
        """Telefonun "Robot Sagligi" karti: her satir {ok: True|False|None, text}.
        ok=None = bilgi yok / gecerli degil (gri)."""
        now = time.monotonic()
        hw = self._hw
        stm_ok = now - hw['rx'] < 1.5
        info = hw['info'] or {}
        h = {}
        h['brain'] = {'ok': self.patrol_alive(),
                      'text': 'çalışıyor' if self.patrol_alive() else 'ÇALIŞMIYOR'}
        h['stm32'] = {'ok': stm_ok,
                      'text': (f"bağlı · yazılım {info.get('fw', '?')}"
                               + ('' if info.get('imu') else ' · IMU yok')) if stm_ok else 'BAĞLI DEĞİL'}
        scan_ok = self._scan_msg is not None and now - self._scan_rx < 1.5
        h['lidar'] = {'ok': scan_ok, 'text': 'çalışıyor' if scan_ok else 'VERİ YOK'}
        for key, name in (('estop', 'estop'), ('bumper', 'bumper')):
            if not stm_ok or hw[key] is None:
                h[name] = {'ok': None, 'text': '—'}
            else:
                h[name] = {'ok': not hw[key], 'text': 'BASILI / kablo kopuk' if hw[key] else 'normal'}
        volt = hw['battery_v']
        if not stm_ok or volt is None:
            h['battery'] = {'ok': None, 'text': '—'}
        elif volt < 5.0:
            h['battery'] = {'ok': None, 'text': 'takılı değil'}
        else:
            low = volt < BATTERY_LOW_V
            h['battery'] = {'ok': not low, 'text': f'{volt:.1f} V' + (' · DÜŞÜK, şarj edin' if low else '')}
        h['localization'] = self._localization_health()
        problems = [k for k, v in h.items() if v['ok'] is False]
        h['summary'] = {'ok': not problems, 'text': 'Her şey yolunda' if not problems
                        else 'Dikkat: ' + ', '.join(HEALTH_NAMES[k] for k in problems)}
        # Motorsuz hizli varis acik unutulmasin (patrol.py fast_arrival_seconds)
        if (self._patrol_status or {}).get('fast_arrival'):
            h['summary']['text'] += ' · ⚡ Motorsuz hızlı varış AÇIK'
        return h

    def _localization_health(self):
        if self.mode == 'mapping':
            return {'ok': None, 'text': 'haritalama sürüyor'}
        if not self.launch_mgr.is_running('nav'):
            return {'ok': False if self.has_saved_map() else None,
                    'text': 'konum sistemi kapalı' if self.has_saved_map() else 'harita yok'}
        if self._nav_boot is not None:
            phase, attempt = self._nav_boot
            if phase == 'failed':
                return {'ok': False, 'text': 'Nav2 AÇILAMADI · robotu yeniden başlatın'}
            return {'ok': None, 'text': 'konum sistemi açılıyor'
                    + (f' (yeniden deneme {attempt}/{NAV2_MAX_START_ATTEMPTS})' if attempt > 1 else '')}
        if self._amcl_rx_t < self._nav_started_t or self._amcl_std is None:
            return {'ok': False, 'text': 'KONUM BİLİNMİYOR · Kurulum → "Robot Burada"'}
        sx, sy, syaw = self._amcl_std
        err = max(sx, sy)
        if err > LOCALIZATION_MAX_STD_M or syaw > LOCALIZATION_MAX_STD_DEG:
            return {'ok': False, 'text': f'emin değil (±{err:.1f} m) · "Robot Burada" ile düzeltin'}
        return {'ok': True, 'text': f'biliyor (±{max(err, 0.01) * 100:.0f} cm)'}

    # ---- patrol.py (devriye beyni) ----
    def missing_target(self, path, table=None):
        """Gorev komutu yapilamiyorsa arayuze gosterilecek sebep (yoksa None).
        patrol.py da ayni kontrolu yapar ve robotta sesli soyler; bu, telefona
        aninda anlasilir bir hata donmesi icin."""
        if path not in ('/api/start_patrol', '/api/wander', '/api/go_home',
                        '/api/greet_door', '/api/goto'):
            return None
        if self.mode in ('mapping', 'tagging'):
            return 'Kurulum sürüyor: önce haritayı bitirip noktaları kaydedin'
        cfg = self.load_waypoints_cfg()
        tables = cfg.get('tables') or []
        if path in ('/api/start_patrol', '/api/wander') and not tables:
            return 'Masalar kurulumda tanımlı değil'
        if path == '/api/go_home' and not cfg.get('barista'):
            return 'Barmen noktası kurulumda tanımlı değil'
        if path == '/api/greet_door' and not cfg.get('door'):
            return 'Kapı kurulumda tanımlı değil'
        if path == '/api/goto' and not 1 <= (table or 0) <= len(tables):
            return f'Masa {table} kurulumda tanımlı değil'
        return None

    def publish_patrol(self, cmd):
        self.command_pub.publish(String(data=cmd))
        self.get_logger().info(f'patrol komutu: {cmd[:80]}')

    def patrol_alive(self):
        return self._patrol_status is not None and time.monotonic() - self._patrol_rx < PATROL_STATUS_FRESH_S

    def _patrol_status_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        self._patrol_status, self._patrol_rx = status, time.monotonic()
        self._check_delivery(status)
        # patrol.py'nin sesli mesajlari (say) robotun KENDI ekraninda, ayni
        # kadin sesiyle okunsun: sunucunun konusma sirasina aktarilir.
        seq = status.get('speak_seq', 0)
        if self._patrol_speak_seq is None:
            self._patrol_speak_seq = seq     # ilk mesaj: eski konusmayi tekrar okuma
        elif seq != self._patrol_speak_seq:
            self._patrol_speak_seq = seq
            text = (status.get('speak_text') or '').strip()
            if text:
                threading.Thread(target=self.request_robot_speech, args=(text, ''), daemon=True).start()

    def map_info(self):
        info = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0], 'image': 'cafe_map.pgm'}
        yaml_path = os.path.join(self.maps_dir, 'cafe_map.yaml')
        try:
            with open(yaml_path, encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or ':' not in line:
                        continue
                    key, val = (p.strip() for p in line.split(':', 1))
                    if key == 'resolution':
                        info['resolution'] = float(val)
                    elif key == 'origin':
                        info['origin'] = list(ast.literal_eval(val))
                    elif key == 'image':
                        info['image'] = val
            w, h, _ = _parse_pgm(os.path.join(self.maps_dir, info['image']))
            info['width'], info['height'] = w, h
        except Exception:  # noqa: BLE001
            info['width'] = info['height'] = 0
        return info

    def map_png_bytes(self):
        info = self.map_info()
        pgm_path = os.path.join(self.maps_dir, info.get('image', 'cafe_map.pgm'))
        mtime = os.path.getmtime(pgm_path)
        if self._map_png_cache and self._map_png_cache[0] == mtime:
            return self._map_png_cache[1]
        png = _pgm_to_png_bytes(pgm_path)
        self._map_png_cache = (mtime, png)
        return png

    def edit_map(self, strokes):
        """Kurulum ekranindaki beyaz/siyah firca: kayitli haritaya boya darbeleri.
        strokes: [{'v': 'free'|'occ'|'unknown', 'r': yaricap (harita pikseli),
        'pts': [[fx, fy], ...]}] - fx/fy resmin 0..1 kesirleri (PNG ve PGM ayni
        yonde: satir 0 = ust). Once eski harita maps/yedek/'e alinir, dosya
        atomik yazilir, Nav2 robotun su anki konumuyla yeniden baslatilir."""
        if self.mode == 'mapping':
            raise RuntimeError('Haritalama sürerken harita düzenlenemez')
        info = self.map_info()
        width, height = info.get('width', 0), info.get('height', 0)
        if not width:
            raise RuntimeError('kayıtlı harita yok')
        pgm_path = os.path.join(self.maps_dir, info.get('image', 'cafe_map.pgm'))
        _, _, pixels = _parse_pgm(pgm_path)
        img = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width).copy()
        stamps = 0
        for stroke in list(strokes)[:300]:
            val = MAP_EDIT_VALUES[stroke['v']]
            r = max(1.0, min(float(stroke['r']), MAP_EDIT_MAX_RADIUS_PX))
            pts = [(float(fx) * width, float(fy) * height) for fx, fy in list(stroke['pts'])[:5000]]
            if not pts:
                continue
            for (x0, y0), (x1, y1) in zip(pts, pts[1:] or pts):
                steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / max(1.0, r / 2.0)))
                for i in range(steps + 1):
                    cx = x0 + (x1 - x0) * i / steps
                    cy = y0 + (y1 - y0) * i / steps
                    xa, xb = max(0, int(cx - r)), min(width, int(cx + r) + 1)
                    ya, yb = max(0, int(cy - r)), min(height, int(cy + r) + 1)
                    if xa >= xb or ya >= yb:
                        continue
                    gy, gx = np.ogrid[ya:yb, xa:xb]
                    disc = (gx + 0.5 - cx) ** 2 + (gy + 0.5 - cy) ** 2 <= r * r
                    img[ya:yb, xa:xb][disc] = val
                    stamps += 1
        if not stamps:
            raise RuntimeError('boyanacak bir şey yok')
        pose = self._capture_slam_pose() if self.launch_mgr.is_running('nav') else None   # map->base_footprint (AMCL)
        self._backup_current_map()
        _write_pgm_atomic(pgm_path, width, height, img.tobytes())
        self._map_png_cache = None
        self.get_logger().info(f'Harita fircayla duzenlendi ({len(strokes)} darbe) - Nav2 yeni haritayla yeniden baslatiliyor.')
        if self.launch_mgr.is_running('nav'):
            self.start_nav_and_localize(pose)
        return len(strokes)

    def has_saved_map(self):
        return os.path.isfile(os.path.join(self.maps_dir, 'cafe_map.yaml'))

    # ---- masa/kapi/barmen noktalari (kurulum ekrani kaydeder) ----
    def _waypoints_path(self):
        return os.path.join(self.config_dir, 'waypoints.json')

    def load_waypoints_cfg(self):
        path = self._waypoints_path()
        if os.path.exists(path):
            try:
                with open(path, encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'waypoints.json okunamadi: {e}')
        return {'barista': None, 'door': None, 'tables': []}

    def save_waypoints_cfg(self, cfg):
        os.makedirs(self.config_dir, exist_ok=True)
        with open(self._waypoints_path(), 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'waypoints.json kaydedildi ({len(cfg.get("tables", []))} masa).')

    def is_configured(self):
        return bool(self.load_waypoints_cfg().get('tables'))

    def reset_waypoints_cfg(self):
        # yeniden haritalama sonrasi eski noktalar YENI haritada anlamsizdir
        # (SLAM haritayi farkli bir orijine gore cizer) - kurulum ekrani BOS acilsin.
        self.save_waypoints_cfg({'barista': None, 'door': None, 'tables': []})

    def _backup_current_map(self):
        """"Bitir" eski haritanin ve masalarin USTUNE yazar (2026-09-25'te iyi bir
        harita 12 saniyelik bir denemeyle boyle kayboldu). Once maps/yedek/NNN-tarih/
        altina kopyalanir, en yeni MAP_BACKUP_KEEP tanesi tutulur. Sira numarasi
        tarihten bagimsiz: Pi acilista saati gec ayarlayabiliyor (RTC pili yok)."""
        files = [os.path.join(self.maps_dir, n) for n in ('cafe_map.yaml', 'cafe_map.pgm')]
        if not all(os.path.isfile(p) for p in files):
            return
        backup_root = os.path.join(self.maps_dir, 'yedek')
        os.makedirs(backup_root, exist_ok=True)
        existing = sorted(d for d in os.listdir(backup_root) if re.match(r'^\d{3}-', d))
        seq = int(existing[-1][:3]) + 1 if existing else 1
        dest = os.path.join(backup_root, f'{seq % 1000:03d}-{time.strftime("%Y%m%d-%H%M%S")}')
        os.makedirs(dest)
        for path in files + [self._waypoints_path()]:
            if os.path.isfile(path):
                shutil.copy2(path, dest)
        for old in existing[:max(0, len(existing) + 1 - MAP_BACKUP_KEEP)]:
            shutil.rmtree(os.path.join(backup_root, old), ignore_errors=True)
        self.get_logger().info(f'Eski harita + masalar yedeklendi: {dest}')

    def finish_mapping(self):
        """Haritayi kaydet, Nav2'ye don. Donus: robotun konumu SLAM'den
        yakalanip AMCL'e verildi mi (False -> kurulumda "Robot Burada")."""
        if self._live_map_msg is None:
            raise RuntimeError('Henuz canli harita verisi yok - biraz daha surup dolasin.')
        captured_pose = self._capture_slam_pose()   # slam_toolbox HALA ayaktayken
        self._backup_current_map()
        map_path_noext = os.path.join(self.maps_dir, 'cafe_map')
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
               '-t', '/map', '-f', map_path_noext,
               '--ros-args', '-p', 'save_map_timeout:=5.0', '-p', 'use_sim_time:=false']
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15.0)
        if result.returncode != 0:
            raise RuntimeError(f'map_saver_cli basarisiz: {result.stderr[-400:]}')
        self.get_logger().info('Harita diske kaydedildi (cafe_map.pgm/.yaml).')
        self._map_png_cache = None
        self.reset_waypoints_cfg()
        self.publish_patrol('reload_waypoints')   # eski masalar yeni haritada anlamsiz
        self.start_nav_and_localize(captured_pose)
        # Kurulum ekrani "Kaydet"e basana kadar 'tagging': patrol.py devriyeyi
        # kilitli tutar (mapping_done orada gonderilir) - sim ile ayni akis.
        self.mode = 'tagging'
        return captured_pose is not None

    # ---- 2026-10-05 OTONOM: ~/cryvex_araclar/otonom_gezgin.py (Nav2 tabanli kesif/devriye) ----
    OTONOM_LOG = os.path.expanduser('~/cryvex_araclar/otonom.log')

    def otonom_calisiyor(self):
        p = getattr(self, '_otonom_proc', None)
        return p is not None and p.poll() is None

    def otonom_baslat(self, mod, yeni=False):
        self.otonom_durdur()
        cmd = ['python3', '-u', os.path.expanduser('~/cryvex_araclar/otonom_gezgin.py'), '--mod', mod]
        if yeni:
            cmd.append('--yeni')
        self.mode = 'mapping'
        self._otonom_proc = subprocess.Popen(cmd, cwd=os.path.expanduser('~/cryvex_araclar'),
                                             stdout=open(self.OTONOM_LOG, 'w'), stderr=subprocess.STDOUT,
                                             start_new_session=True)
        self.get_logger().info(f'OTONOM basladi: {mod} (yeni={yeni})')

    def otonom_durdur(self):
        p = getattr(self, '_otonom_proc', None)
        if p is not None and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                p.wait(timeout=25)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass
        self._otonom_proc = None
        self.publish_cmd(0.0, 0.0)

    def otonom_log(self, n=25):
        try:
            with open(self.OTONOM_LOG) as f:
                return f.read().splitlines()[-n:]
        except FileNotFoundError:
            return []

    def publish_cmd(self, lx, az):
        lx = max(-JOY_MAX_LIN, min(JOY_MAX_LIN, float(lx)))
        az = max(-JOY_MAX_ANG, min(JOY_MAX_ANG, float(az)))
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.cmd_pub.publish(msg)

    def request_robot_speech(self, text, expr):
        _tts_path(text)  # ses simdiden hazir olsun: kiosk istediginde aninda calsin
        with self._lock:
            self.speak_text, self.speak_expr = text, expr
            self.speak_seq += 1

    def status_payload(self):
        # patrol.py'nin publish_status_now() anahtarlari - index.html/Flutter
        # app bunlari bekliyor. patrol.py calismiyorsa "idle" yedegi (goz
        # animasyonu normal durumda gorunur, hata vermez).
        status = {
            'state': 'idle', 'waypoint': None, 'table_index': -1,
            'wait_total': 0.0, 'wait_remaining': 0.0,
            'interacting': False, 'greeting': False, 'cute': False,
            'screen_on': False, 'order': None,
            'msg_phase': None, 'msg_from': None, 'msg_to': None, 'msg_text': None,
            'msg_total': 0.0, 'msg_remaining': 0.0, 'msg_composing': False,
            'map_ready': not self.launch_mgr.is_running('slam'),
            'rescue_active': False,
        }
        alive = self.patrol_alive()
        if alive:
            status.update(self._patrol_status)
        # Konusma TEK siradan (sunucunun): patrol'un mesajlari da oraya aktariliyor
        # (bkz. _patrol_status_cb) - iki ayri sayac arayuzde cift okumaya yol acar.
        status.update(speak_text=self.speak_text, speak_seq=self.speak_seq, speak_expr=self.speak_expr)
        # Ekran: gercek robotta varsayilan ACIK (gozler); patrol is yaparken de acar.
        status['screen_on'] = self.screen_on or bool(status.get('screen_on'))
        # Teslim edilen siparis (robot ekraninda barmende/masada fis olarak gosterilir)
        delivering = self.delivering_order()
        status['delivery_order'] = json.dumps(delivering, ensure_ascii=False) if delivering else ''
        age = round(time.monotonic() - self._patrol_rx, 1) if alive else 0.1
        with self._orders_lock:
            active = sum(o['status'] in ACTIVE_ORDER_STATUSES for o in self.orders)
        return {'status': json.dumps(status), 'count': 1, 'age': age, 'patrol': alive,
                'theme': self.theme, 'health': self.health_payload(),
                'orders_seq': self.orders_seq, 'orders_active': active}

    def _make_handler(node_self):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: A003
                pass

            def _read_body(self):
                length = int(self.headers.get('Content-Length', 0) or 0)
                return self.rfile.read(length).decode('utf-8') if length else ''

            def _send_json(self, data):
                body = json.dumps(data).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_file(self, rel_path, content_type):
                path = os.path.join(node_self.web_dir, rel_path)
                try:
                    with open(path, 'rb') as f:
                        body = f.read()
                except FileNotFoundError:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                # Telefonun WebView'i sayfayi saklayip guncellemeden sonra da ESKISINI
                # gosteriyordu (kurulum izgarasi gorunmedi) - her acilista yeniden alsin.
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split('?')[0]
                # Ayni sayfa, adrese gore rol degistirir (index.html <body> basi):
                # /eyes = kafadaki yuz ekrani, /panel = govdedeki dev dokunmatik
                # ekran, / = ikisi bir arada (eski tek ekranli duzen).
                if path in ('/', '/index.html', '/eyes', '/panel'):
                    self._serve_file('index.html', 'text/html; charset=utf-8')
                elif path in ('/setup', '/setup.html'):
                    self._serve_file('setup.html', 'text/html; charset=utf-8')
                elif path in ('/app', '/cryvex.apk'):
                    # Telefon uygulamasinin guncel surumu: telefon tarayicisindan
                    # http://<robot>:8080/cryvex.apk ile indirip kurulur (USB gerekmez).
                    apk = os.path.join(node_self.web_dir, 'cryvex.apk')
                    try:
                        with open(apk, 'rb') as f:
                            body = f.read()
                    except FileNotFoundError:
                        self.send_error(404, 'cryvex.apk yok')
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/vnd.android.package-archive')
                    self.send_header('Content-Disposition', 'attachment; filename="cryvex.apk"')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif path in ('/otonom', '/otonom.html'):
                    self._serve_file('otonom.html', 'text/html; charset=utf-8')
                elif path == '/api/otonom':
                    self._send_json({'calisiyor': node_self.otonom_calisiyor(), 'log': node_self.otonom_log()})
                elif path == '/api/mode':
                    self._send_json({'mode': node_self.mode, 'configured': node_self.is_configured()})
                elif path == '/api/status':
                    self._send_json(node_self.status_payload())
                elif path == '/api/live_map.png':
                    # ?grid=0: izgarasiz, haritayla ayni boyut (setup.html kendi izgarasini cizer)
                    qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                    # ?marks=1: masa/Us/Kapi isaretleri, ?plan=1: Nav2'nin planladigi yol (telefon ana ekrani)
                    png = node_self.live_map_png_bytes(grid=qs.get('grid', ['1'])[0] != '0',
                                                       marks=qs.get('marks', ['0'])[0] == '1',
                                                       plan=qs.get('plan', ['0'])[0] == '1',
                                                       crop=qs.get('crop', ['0'])[0] == '1')
                    if png is None:
                        self.send_error(503, 'Henuz canli harita yok')
                        return
                    self._serve_png(png)
                elif path == '/api/map.png':
                    try:
                        png = node_self.map_png_bytes()
                    except Exception as e:  # noqa: BLE001
                        self.send_error(500, str(e))
                        return
                    self._serve_png(png)
                elif path == '/api/map_info':
                    self._send_json(node_self.map_info())
                elif path == '/api/waypoints':
                    self._send_json(node_self.load_waypoints_cfg())
                elif path == '/api/tts_audio':
                    self._serve_tts_audio()
                elif path == '/api/volume':
                    vol = get_volume_pct()
                    self._send_json({'volume': vol if vol is not None else -1})
                elif path == '/api/theme':
                    self._send_json(node_self.theme)
                elif path == '/api/health':
                    self._send_json(node_self.health_payload())
                elif path == '/api/orders':
                    self._send_json(node_self.orders_view())
                else:
                    self.send_error(404)

            def _serve_tts_audio(self):
                qs = urllib.parse.urlparse(self.path).query
                text = urllib.parse.parse_qs(qs).get('text', [''])[0].strip()
                if not text:
                    self.send_error(400, 'text parametresi gerekli')
                    return
                path = _tts_path(text)
                if path is None:
                    self.send_error(503, 'Ses uretilemedi (internet yok ve onbellekte yok)')
                    return
                with open(path, 'rb') as f:
                    audio = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'audio/mpeg')
                # Tarayici SAKLAMASIN: ses degisirse eski ses kendi disk
                # onbelleginden calinmaya devam ediyordu. Sunucu onbellegi zaten hizli.
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(audio)))
                self.end_headers()
                self.wfile.write(audio)

            def _serve_png(self, png):
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(png)))
                self.end_headers()
                self.wfile.write(png)

            def do_POST(self):
                path = self.path.split('?')[0]
                raw = self._read_body()
                try:
                    d = json.loads(raw or '{}')
                except Exception:  # noqa: BLE001
                    d = {}
                if not isinstance(d, dict):
                    d = {}

                def need_password():
                    return str(d.get('password', '')) != OPERATOR_PASSWORD

                def ok(**extra):
                    self._send_json(dict(result='ok', **extra))

                def error(reason):
                    self._send_json({'result': 'error', 'reason': reason})

                def to_patrol(cmd):
                    """Devriye beynine komut; beyin calismiyorsa arayuze dürüst hata."""
                    if not node_self.patrol_alive():
                        error('devriye beyni (patrol.py) calismiyor')
                        return
                    node_self.publish_patrol(cmd)
                    ok()

                def joy():
                    try:
                        return float(d.get('lx', 0.0)), float(d.get('az', 0.0))
                    except (TypeError, ValueError):
                        return 0.0, 0.0

                def clean(value):
                    return str(value or '').replace('|', '/').replace('\n', ' ').strip()

                if path in PATROL_SIMPLE_COMMANDS:
                    reason = node_self.missing_target(path)
                    if reason:
                        error(reason)
                        return
                    to_patrol(PATROL_SIMPLE_COMMANDS[path])
                elif path == '/api/stop_patrol':
                    # DURDUR her zaman calismali: beyin olmasa da motora 0 hiz.
                    node_self.publish_cmd(0.0, 0.0)
                    node_self.publish_patrol('stop')
                    ok()
                elif path == '/api/teleop':
                    # Beyin varsa ONUN uzerinden (Nav2 gorevini keser, 10 cm sonar
                    # korumasi + 0.6 sn bekci); yoksa dogrudan (haritalama yedegi -
                    # stm32_bridge 0.5 sn komutsuz kalirsa zaten durdurur).
                    lx, az = joy()
                    if node_self.patrol_alive():
                        node_self.publish_patrol(f'teleop:{lx:.3f}:{az:.3f}')
                    else:
                        node_self.publish_cmd(lx, az)
                    ok()
                elif path == '/api/teleop_stop':
                    if node_self.patrol_alive():
                        node_self.publish_patrol('teleop_stop')
                    node_self.publish_cmd(0.0, 0.0)
                    ok()
                elif path == '/api/goto':
                    # govde: {"table": 1..N, "action": "welcome_menu"}
                    try:
                        table = int(d['table'])
                    except (KeyError, TypeError, ValueError):
                        error('bad body')
                        return
                    reason = node_self.missing_target(path, table)
                    if reason:
                        error(reason)
                        return
                    to_patrol(f"goto:{table}:{d.get('action') or 'welcome_menu'}")
                elif path == '/api/place_order':
                    # Once kaydet (garson telefonu bildirim + liste), sonra robot
                    # siparisi barmene iletsin (beyin yoksa da kayit dusmesin).
                    order = node_self.add_order(raw)
                    if node_self.patrol_alive():
                        node_self.publish_patrol(f'order:{raw or "{}"}')
                    ok(order_id=order['id'] if order else None)
                elif path == '/api/order_ready':
                    try:
                        order, coming_now = node_self.order_ready(int(d.get('id')))
                    except (TypeError, ValueError):
                        error('sipariş no gerekli')
                        return
                    except RuntimeError as e:
                        error(str(e))
                        return
                    # coming_now=False: robot musteriyle/baska teslimatla ilgileniyor,
                    # siraya aldi - o is biter bitmez gelecek.
                    ok(table=order['table'], coming_now=coming_now)
                elif path == '/api/order_cancel':
                    try:
                        node_self.order_cancel(int(d.get('id')))
                    except (TypeError, ValueError):
                        error('sipariş no gerekli')
                        return
                    except RuntimeError as e:
                        error(str(e))
                        return
                    ok()
                elif path == '/api/continue':
                    try:
                        node_self.robot_continue()
                    except RuntimeError as e:
                        error(str(e))
                        return
                    ok()
                elif path == '/api/send_message':
                    frm, to, text = clean(d.get('from')), clean(d.get('to')), clean(d.get('text'))
                    if not (frm and to and text):
                        error('missing field')
                        return
                    to_patrol(f'msg:{frm}|{to}|{text}')
                elif path == '/api/msg_reply':
                    to_patrol(f"msg_reply:{clean(d.get('text'))}")
                elif path == '/api/rescue_start':
                    if need_password():
                        error('wrong password')
                        return
                    to_patrol('rescue_start')
                elif path == '/api/rescue_teleop':
                    lx, az = joy()
                    to_patrol(f'rescue_teleop:{lx:.3f}:{az:.3f}')
                elif path == '/api/relocalize':
                    try:
                        node_self.relocalize_to_home()
                    except RuntimeError as e:
                        error(str(e))
                        return
                    ok()
                elif path in ('/api/wake', '/api/sleep'):
                    node_self.screen_on = path == '/api/wake'
                    if node_self.patrol_alive():
                        node_self.publish_patrol(path.rsplit('/', 1)[1])
                    ok()
                elif path == '/api/start_mapping':
                    if need_password():
                        error('wrong password')
                        return
                    node_self.publish_patrol('mapping_start')
                    node_self.mode = 'mapping'
                    node_self.launch_mgr.start_slam()   # Nav2'yi once kapatir
                    ok()
                elif path == '/api/finish_mapping':
                    if need_password():
                        error('wrong password')
                        return
                    try:
                        pose_captured = node_self.finish_mapping()
                    except Exception as e:  # noqa: BLE001
                        error(str(e))
                        return
                    ok(pose_captured=pose_captured)
                elif path == '/api/otonom':
                    if need_password():
                        error('wrong password')
                        return
                    mod = str(d.get('mod', ''))
                    if mod in ('kesif', 'devriye'):
                        node_self.otonom_baslat(mod, yeni=bool(d.get('yeni', False)))
                        ok()
                    elif mod == 'dur':
                        node_self.publish_patrol('stop')
                        threading.Thread(target=node_self.otonom_durdur, daemon=True).start()
                        ok()
                    else:
                        error('mod: kesif | devriye | dur')
                elif path == '/api/cancel_mapping':
                    if need_password():
                        error('wrong password')
                        return
                    # Kaydedilmedi: eski harita diskte duruyor, onunla Nav2'ye don.
                    if node_self.has_saved_map():
                        node_self.start_nav_and_localize()
                    else:
                        node_self.launch_mgr.stop()
                    node_self.mode = 'operating'
                    node_self.publish_patrol('mapping_done')
                    ok()
                elif path == '/api/waypoints':
                    # govde: {"barista": {...}, "door": {...}, "tables": [{...}, ...]}
                    try:
                        node_self.save_waypoints_cfg(d)
                    except Exception as e:  # noqa: BLE001
                        error(str(e))
                        return
                    node_self.publish_patrol('reload_waypoints')
                    # Haritalamadan sonraki nokta isaretleme ("tagging") bu kayitla
                    # biter, devriye tekrar kullanilabilir olur.
                    if node_self.mode in ('tagging', 'mapping'):
                        node_self.mode = 'operating'
                        node_self.publish_patrol('mapping_done')
                    ok(mode=node_self.mode)
                elif path == '/api/volume':
                    # govde: {"volume": 0-100} - sifre gerekmez (zararsiz, geri alinabilir).
                    try:
                        pct = int(d.get('volume'))
                    except (TypeError, ValueError):
                        self._send_json({'result': 'error', 'reason': 'bad volume'})
                        return
                    vol_ok = set_volume_pct(pct)
                    self._send_json({'result': 'ok' if vol_ok else 'error', 'volume': pct})
                elif path == '/api/theme':
                    # govde: {"eye"|"mouth"|"light"|"face": "#rrggbb", ...} ya da {"reset": true}
                    try:
                        theme = node_self.update_theme(d)
                    except (ValueError, OSError) as e:
                        error(str(e))
                        return
                    ok(theme=theme)
                elif path == '/api/speak_here':
                    # govde: {"text": "...", "expr": "happy|love|alert|sad" (istege bagli)}
                    # Telefon vb. uzak istemciler icin: robotun KENDI ekrani bir
                    # sonraki durum sorgusunda konusur, agzi oynar, ifadesi degisir.
                    text = str(d.get('text', '')).strip()
                    if not text:
                        self._send_json({'result': 'error', 'reason': 'text gerekli'})
                        return
                    expr = d.get('expr', '')
                    node_self.request_robot_speech(text, expr if expr in ROBOT_EXPRESSIONS else '')
                    self._send_json({'result': 'ok'})
                elif path == '/api/map_edit':
                    # govde: {"strokes": [{"v": "free"|"occ", "r": px, "pts": [[fx, fy], ...]}]}
                    try:
                        n = node_self.edit_map(d['strokes'])
                    except (KeyError, TypeError, ValueError):
                        self._send_json({'result': 'error', 'reason': 'geçersiz fırça verisi'})
                        return
                    except RuntimeError as e:
                        self._send_json({'result': 'error', 'reason': str(e)})
                        return
                    self._send_json({'result': 'ok', 'strokes': n})
                elif path == '/api/set_pose':
                    # govde: {"x": m, "y": m, "yaw": rad} (harita cercevesi). yaw
                    # verilmezse/null ise robotun BILDIGI son yonelim korunur
                    # (2026-10-02: tek dokunusla "Robot Burada" - teker odometrisi
                    # donusu dogru olcuyor, kayan sey konum; yonu her seferinde
                    # elle vermek hem zahmetli hem de el ile verilen yon cogu
                    # zaman robotun gercek yonunden daha kotu).
                    try:
                        x, y = float(d['x']), float(d['y'])
                        yaw = d.get('yaw')
                        keep = yaw is None
                        yaw = node_self.current_robot_yaw() if keep else float(yaw)
                        node_self.set_robot_pose(x, y, yaw, yaw_known=keep)
                    except (KeyError, TypeError, ValueError):
                        self._send_json({'result': 'error', 'reason': 'x, y gerekli (yaw istege bagli)'})
                        return
                    except RuntimeError as e:
                        self._send_json({'result': 'error', 'reason': str(e)})
                        return
                    self._send_json({'result': 'ok', 'yaw': yaw, 'yaw_kept': keep})
                else:
                    # Bilinmeyen uc: eskiden yalandan "ok" donuyordu (buton
                    # calisiyor sanilirdi) - artik durustce hata.
                    self.send_response(404)
                    body = json.dumps({'result': 'error', 'reason': f'bilinmeyen komut: {path}'}).encode()
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

        return Handler

    def destroy_node(self):
        self.launch_mgr.stop()
        self._httpd.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = CafeUiServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
