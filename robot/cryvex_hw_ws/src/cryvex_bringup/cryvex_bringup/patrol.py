#!/usr/bin/env python3
"""
Cryvex Restaurant Patrol Node - GERCEK DONANIM
==============================================
~/cryvex_ws/src/cryvex_gazebo/scripts/patrol.py'nin (Gazebo'da aylarca test
edilmis devriye beyni) gercek robota tasinmis hali (2026-09-25). Durum
makinesi, Nav2 surusu, kurtulma/kurtarma/ayrilma manevralari, siparis ve
mesajlasma AYNEN korundu. Sim'den farklari:

  - Masa/kapi/barmen konumlari SADECE kurulum ekranindan (config/waypoints.json)
    gelir. Sim'deki cafe.world'e gore sabit koordinatlar YOK - bos kurulumda
    robot uydurma noktalara gitmez, devriye/yonlendirme komutlarini reddeder.
  - Sosyallesme (wander) noktalari ve "oda merkezi", kayitli haritanin bos
    alanlarindan (engelden >= WANDER_CLEARANCE_M uzak) hesaplanir.
  - Baslangic/relocalize/"Robot Burada" konumu cafe_ui_server.py'de (Nav2'yi o
    baslatiyor, ne zaman ayaga kalktigini o biliyor) - burada degil.
  - use_sim_time YOK (gercek saat). Mesaj alicisi, tanimli masalardan biri
    ya da 'Garson' olmali (arayuzun sabit Masa 1-8 listesi olmayan masayi da
    icerebilir).

Kafe beyni ile arayuz arasi: cafe_ui_server.py HTTP komutlarini /patrol_command
(duz metin, sim'deki tablet_server.py ile AYNI komutlar) olarak yayinlar,
/patrol_status'u (JSON) okuyup /api/status ile index.html/Flutter'a verir.

Kafe garson-robotu davranis beyni. nav2_simple_commander uzerinden calisir.

Durum makinesi:
  idle -> patrol -> waiting_at_table -> (siparis) delivering_order
       -> at_barista -> (resume) patrol ...
  wander     : bos zamanda oda icinde sosyallesir (Wander & Greet senaryosu)
  going_home : barmen/us noktasina doner

Senaryolar:
  * Wander & Greet   : `wander` komutu ile bos alanlarda gezip pazarlama yapar.
  * Kapida Karsilama : `greet_door` -> kapi noktasinda 3 dk "hos geldiniz".
  * Mesaj Tasima     : `msg:<from>|<to>|<text>` -> birinden birine not goturur.
  * Garson Yonlendirme: `goto:<masa>:<welcome|menu|welcome_menu|cute>`.

(Goruntu isleme / "yol isteme" senaryolari kaldirildi.)
"""
import ast
import os
import math
import json
import time
import random
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


# --- Masa / barmen (us) / kapi ---
# BOS baslar; kurulum ekraninin kaydettigi config/waypoints.json'dan doldurulur
# (apply_waypoints_cfg YERINDE gunceller, calisan dongu yeni degerleri gorur).
# Bos sozluk = o nokta tanimli degil.
WAYPOINTS = []
BARISTA_POS = {}      # siparis buraya goturulur
HOME_POSITION = {}    # = barmen noktasi (robotun "evi")
DOOR_POS = {}         # kapida karsilama noktasi

# Wander & Greet noktalari ve oda merkezi kayitli haritadan hesaplanir
# (refresh_map_geometry): engelden/bilinmeyenden bu kadar uzak bos hucreler.
WANDER_CLEARANCE_M = 0.6
WANDER_POINT_COUNT = 8
WANDER_POINTS = []
ROOM_CENTER = (0.0, 0.0)
RETRY_OFFSETS = [0.0, 0.35, 0.75]     # hedef reddedilirse oda merkezine dogru kaydir

# --- Ultrasonik kurtulma (takilip kalmayi onler) ---
# 4x HC-SR04 taban kosede; base_link'e gore yaklasik yon aci (rad).
SONAR_YAW = {'fl': 0.30, 'fr': -0.30, 'rl': 2.84, 'rr': -2.84}
ESCAPE_TRIGGER = 0.10        # < 10 cm -> kurtulma manevrasi (fiziksel temas esigi)
SONAR_STALE_SECONDS = 1.0    # bu suredir veri gelmeyen sensoru yok say
STALL_ESCAPE_SECONDS = 15.0  # hedefe 15 sn ilerleme yoksa "kor" kurtulma
MAX_ESCAPES_PER_GOAL = 5

# Masaya/hedefe FAZLA yanasmayi onler: kurtulmadan (fiziksel tehlike, 10 cm) AYRI,
# daha yumusak bir sinir. On sensorlerden biri bu mesafenin altina duserse (ve
# hedefe zaten yaklasilmisken) Nav2 gorevi TEMIZ iptal edilip "vardim" sayilir -
# robot daha fazla ittirmeye/yanasmaya calismaz, takilmaz.
APPROACH_STANDOFF = 0.25     # bu mesafeden sonra yaklasma, yeterli say (25 cm)
APPROACH_GATE_DIST = 1.5     # sadece hedefe bu kadar YAKINKEN gecerli (koridorda degil)

# Masadan AYRILIRKEN: donup gitmeden once duz geri cekil - masaya/sandalyeye
# yakinken hemen donmeye/ilerlemeye calismak surtunmeye/takilmaya sebep oluyordu.
DEPART_BACK_METERS = 1.0
DEPART_BACK_LIN = 0.15                              # m/s
DEPART_BACK_SECONDS = DEPART_BACK_METERS / DEPART_BACK_LIN

# Kurtulma SABIT ADIMLI bir manevra (surekli itme DEGIL - o donguye giriyordu):
#   1) DUR, 2 sn bekle (algiyi dogrula)
#   2) duz GERI (onde engel) veya duz ILERI (arkada engel)
#   3) DON (yakin taraftan uzaga; sol yakinsa saga, sag yakinsa sola)
#   4) az ILERI
#   5) ayni hedefe tekrar dene (robot artik farkli yerde/yonde -> Nav2 yeniden planlar)
ESCAPE_SETTLE_SECONDS = 2.0
ESCAPE_BACK_SECONDS = 1.0
ESCAPE_TURN_SECONDS = 0.9
ESCAPE_FWD_SECONDS = 0.6
ESCAPE_BACK_LIN = 0.12
ESCAPE_TURN_ANG = 0.80
ESCAPE_FWD_LIN = 0.10

# --- Operator ekrani: canli haritalama joystick'i (STATE_TELEOP) ---
# Wifi/arayuz kopukluguna karsi guvenlik: bu suredir taze 'teleop:' komutu
# gelmezse (parmak ekrandan kalkti, baglanti koptu) motor otomatik durur.
TELEOP_WATCHDOG_SECONDS = 0.6
TELEOP_MAX_LIN = 0.30
TELEOP_MAX_ANG = 1.00

TABLE_WAIT_SECONDS = 10.0             # masada siparis icin bekleme
INTERACT_TIMEOUT_SECONDS = 90.0      # menu bu kadar acik kalirsa sayac zorla devam
WANDER_GREET_SECONDS = 4.0           # wander duraginda selamlasma molasi
DOOR_GREET_SECONDS = 180.0          # kapida "hos geldiniz" molasi (3 dakika)
DIRECTED_GREET_SECONDS = 8.0        # garsonun yolladigi masada selam/sevimlilik suresi

# Garsonun secebilecegi hazir mesaj kaliplari (robot bunlari sesli okur).
DIRECTED_MESSAGES = {   # robot sesli okur (edge-tts) - Turkce karakterler SART
    'welcome':      'Hoş geldiniz! Cryvex kafeye buyurun, keyifli vakit geçirmenizi dileriz.',
    'menu':         'Menümüze göz atmak için ekrana dokunabilirsiniz.',
    'welcome_menu': 'Hoş geldiniz! Menümüzü görmek için ekrana dokunun, siparişinizi hemen alalım.',
    'cute':         'Merhabaa! Seni gördüğüme çok sevindim, hoş geldin küçük dostum!',
}
DIRECTED_ACTIONS = tuple(DIRECTED_MESSAGES.keys())

# --- Mesajlasma (robot postaci) ---
# Kisiler = tanimli masalar (isimleriyle) + 'Garson' (barmen noktasi); bkz. person_location.
MSG_OPEN_TIMEOUT = 25.0     # alici dokunmazsa mesaj acilmadan vazgec
MSG_READ_SECONDS = 20.0     # mesaji okuma + "cevap ver"e karar verme suresi
MSG_COMPOSE_TIMEOUT = 45.0  # "cevap ver"e basildi -> yazma icin guvenlik suresi
                            #   (arayuz 30 sn'de kendisi iptal eder; bu yedek)

# --- Durumlar ---
STATE_IDLE = 'idle'
STATE_PATROL = 'patrol'
STATE_WAITING = 'waiting_at_table'
STATE_DELIVERING = 'delivering_order'
STATE_AT_BARISTA = 'at_barista'
STATE_GOING_HOME = 'going_home'
STATE_WANDER = 'wander'
STATE_GREET_DOOR = 'greet_door'      # garson "kapida karsila" dedi -> 3 dk kapida
STATE_DIRECTED = 'directed'          # garson telefondan bir masaya yonlendirdi
STATE_MESSENGER = 'messenger'        # birinden birine mesaj tasiyor
STATE_TELEOP = 'teleop'              # operator ekranindan canli joystick suruşu
# Hazir siparisi masaya goturme (2026-09-26): barmen telefondan "Hazir -
# Robot Gotursun" der -> barmene git -> YUKLENMESINI bekle -> masaya git ->
# musterinin ALMASINI bekle -> devriye. Bekleme durumlarinda robot "Devam"
# (robot ekrani ya da garson telefonu) basilmadan KIPIRDAMAZ.
STATE_PICKUP = 'pickup'              # hazir siparisi almaya barmene gidiyor
STATE_PICKUP_WAIT = 'pickup_wait'    # barmende: siparis yukleniyor, "Devam" bekleniyor
STATE_SERVING = 'serving'            # siparisi masaya goturuyor
STATE_SERVED_WAIT = 'served_wait'    # masada: musteri aliyor, "Devam" bekleniyor
ORDER_STATES = (STATE_DELIVERING, STATE_AT_BARISTA, STATE_PICKUP, STATE_PICKUP_WAIT,
                STATE_SERVING, STATE_SERVED_WAIT)
# Hazir siparis teslimati SIRAYA alinir ve su durumlardaysa HEMEN baslar (robot
# yolda/bosta - birakabilecegi bir is). Musteriyle ilgilenirken (masada menu,
# mesaj, garson yonlendirmesi, joystick) aklinda tutar; o isten cikar cikmaz
# onceligi teslimat olur. Gercek robotta siparis bilgisi garson telefonuna
# gider - robot artik siparisi barmene TASIMAZ (STATE_DELIVERING kullanilmaz).
DELIVERY_INTERRUPTIBLE = (STATE_IDLE, STATE_PATROL, STATE_WANDER, STATE_GOING_HOME, STATE_GREET_DOOR)
ORDER_THANKS_SECONDS = 3.0   # siparis verildi: tesekkur ekrani gorunsun, sonra masadan ayril


class CommandListener(Node):
    """Arayuz komutlarini dinler, durumu /patrol_status (JSON) ile yayinlar."""

    def __init__(self):
        super().__init__('patrol_command_listener')
        self.state = STATE_IDLE
        self.current_waypoint = ''
        self.table_index = 0
        self.order_data = ''
        self.interacting = False
        self.wait_total = TABLE_WAIT_SECONDS
        self.wait_remaining = 0.0
        self.last_pos = None
        self.greeting = False           # kapida/wander/masa'da selamlama molasi
        self.cute = False               # sevimlilik modu (garson 'goto:N:cute')
        self.directed_table = 0         # garsonun yonlendirdigi masa index'i
        self.directed_action = 'menu'   # DIRECTED_ACTIONS'tan biri
        self.screen_on = False          # musteri ekrani acik mi
        self.speak_text = ''            # robotun sesli okuyacagi hazir mesaj
        self.speak_seq = 0              # her yeni mesajda artar (arayuz dedupe icin)
        # mesajlasma
        self.msg_from = ''
        self.msg_to = ''
        self.msg_text = ''
        self.msg_phase = ''             # '' | travel | pending | reading | reply | replying
        self.msg_open_req = False       # alici "ac" icin ekrana dokundu
        self.msg_composing = False      # alici "cevap ver"e basti, yaziyor -> BEKLE
        self.msg_reply_text = None      # cevap metni (None = cevap yok)
        self.msg_total = 0.0
        self.msg_remaining = 0.0
        # hazir siparis teslimati (deliver komutu)
        self.delivery_table = 0         # WAYPOINTS index'i
        self.delivery_order_id = 0      # cafe_ui_server'daki siparis no (0 = teslimat yok)
        self.delivery_queue = []        # sirada bekleyen teslimatlar: [(masa_index, siparis_no)]
        self.delivery_lock = threading.Lock()
        self.next_wp_index = None       # teslimattan sonra devriye bu masadan devam etsin
        self.depart_pending = False     # siradaki yola cikmadan once masadan geri cekil
        self.order_placed = False       # masada siparis verildi -> tesekkur et, ayril
        self.sonar = None              # main()'de SonarReader baglanir
        # operator kurulum ekrani: harita+masalar KAYDEDILENE kadar devriye/goto
        # komutlari reddedilir (yeni haritada anlamsiz eski WAYPOINTS'e gitmesin).
        # Var olan kafede (baslangicta mevcut waypoints.json) True baslar.
        self.map_ready = True
        self.teleop_lx = 0.0            # operator joystick (canli haritalama)
        self.teleop_az = 0.0
        self.teleop_last_t = 0.0
        # Kurtarma modu: robot herhangi bir gorev icinde (devriye/siparis/vs)
        # takilirsa garson joystick ile devralir; drive() icinden yonetilir,
        # STATE'i DEGISTIRMEZ - bittiginde ayni gorev kaldigi yerden devam eder.
        self.rescue_active = False
        self.rescue_lx = 0.0
        self.rescue_az = 0.0
        self.rescue_last_t = 0.0

        self.create_subscription(String, '/patrol_command', self._command_callback, 10)
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        # Beynin KENDI manevralari (joystick, kurtarma, kurtulma, masadan
        # ayrilis) SADECE /cmd_vel'e (stm32_bridge) gider. Sim ikisine birden
        # (/cmd_vel + /cmd_vel_nav) basiyordu; Jazzy'de /cmd_vel_nav ->
        # yumusatici -> carpisma izleyici -> /cmd_vel zinciri var: engel
        # yakinken izleyici "0", beyin "0.2" basar, STM32 ikisi arasinda
        # titrerdi (2026-09-25 testi). Otonom surus Nav2 zincirinden,
        # manuel/kurtulma dogrudan - tek yazici.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # NOT: /patrol_status yayini bir ROS timer'i ile DEGIL, main()'deki
        # duvar-saati (wall-clock) thread'i ile yapilir. Boylece use_sim_time /
        # executor sorunlarindan bagimsiz olarak arayuz her zaman guncellenir.
        self.get_logger().info('Restaurant komut dinleyici baslatildi.')

    # ---- komutlar ----
    def _command_callback(self, msg):
        cmd = msg.data.strip()

        # Siparis tasirken (DELIVERING/AT_BARISTA) garson override'lari yok sayilir.
        busy_with_order = self.state in ORDER_STATES

        # Kurulum/haritalama surerken (map_ready=False) devriye/yonlendirme
        # komutlari YOK SAYILIR - operator "Kaydet"e basip yeni noktalari
        # onaylayana kadar robot eski (artik anlamsiz) WAYPOINTS'e gitmeye
        # calismaz. Teleop bu kontrolden MUAF (haritalama tam da bunu kullanir).
        nav_cmd = cmd in ('start', 'wander', 'go_home', 'greet_door') or cmd.startswith('goto:')
        if nav_cmd and not self.map_ready:
            self.get_logger().warn(
                f"KOMUT '{cmd}' reddedildi: kurulum/haritalama surüyor (map_ready=False).")
            return
        # Gercek robotta noktalar SADECE kurulumdan gelir - tanimli olmayan bir
        # yere gitmesi istenirse uydurma koordinata degil, hicbir yere gitmez.
        missing = self._missing_target(cmd)
        if missing:
            self.get_logger().warn(f"KOMUT '{cmd}' reddedildi: {missing} kurulumda tanimli degil.")
            self.say(f'{missing} henüz tanımlı değil, önce kurulumdan işaretleyin.')
            return

        if cmd == 'start':
            if busy_with_order:
                self.get_logger().warn("KOMUT: 'start' - once siparis teslim edilmeli.")
            else:
                self.state = STATE_PATROL
                self.greeting = False
                self.cute = False
                self.patrol_restart = True   # TEK_TUR: her 'start' ilk masadan baslar
                self.get_logger().info('KOMUT: Devriye BASLAT')

        elif cmd == 'stop':
            # ACIL STOP > telefon > otonom oncelik zincirinin (malzeme listesi
            # rev.4 bolum 2.3) yazilim tarafi burada uygulanir - ayri bir
            # twist_mux node'una GEREK YOK: patrol.py zaten /cmd_vel'e YAZAN
            # TEK yer, o yuzden 'stop' RESCUE/TELEOP dahil HER SEYI kesmezse
            # oncelik zinciri gercekte calismaz. 2026-09-17'ye kadar bu
            # eksikti: 'stop' sadece state'i IDLE yapiyordu, rescue_active
            # AKTIF KALIYORDU - yani garson kurtarma joystick'iyle surerken
            # biri STOP'a basarsa robot DURMUYORDU (rescue_active=True oldugu
            # surece _rescue_maneuver kendi dongusunde donmeye devam ediyordu).
            self.state = STATE_IDLE
            self.interacting = False
            self.wait_remaining = 0.0
            self.rescue_active = False
            self.rescue_lx = 0.0
            self.rescue_az = 0.0
            self.teleop_lx = 0.0
            self.teleop_az = 0.0
            # yarim kalan ve siradaki teslimatlar iptal: sunucu siparisleri
            # "Hazirlaniyor"a geri alir, barmen tekrar "Hazir - Robot Gotursun" diyebilir
            with self.delivery_lock:
                self.delivery_order_id = 0
                self.delivery_queue = []
            self.depart_pending = False
            self.next_wp_index = None
            self.order_placed = False
            self.get_logger().info('KOMUT: DURDUR (rescue/teleop dahil hepsi kesildi)')

        elif cmd == 'wander':
            if not busy_with_order:
                self.state = STATE_WANDER
                self.greeting = False
                self.get_logger().info('KOMUT: Sosyallesme (Wander) modu')

        elif cmd == 'go_home':
            if not busy_with_order:
                self.state = STATE_GOING_HOME
                self.interacting = False
                self.greeting = False
                self.cute = False
                self.get_logger().info('KOMUT: Use DON')

        elif cmd == 'interacting':
            self.interacting = True
            self.get_logger().info('KOMUT: Musteri menuye bakiyor (sayac donduruldu)')

        elif cmd == 'done_interacting':
            self.interacting = False
            self.get_logger().info('KOMUT: Menu kapandi (sayac devam)')

        elif cmd.startswith('order:'):
            # Siparis bilgisi garson telefonuna (cafe_ui_server) gitti - robot
            # onu barmene TASIMAZ: tesekkur eder, masadan ayrilip devam eder.
            if self.state == STATE_WAITING:
                self.order_data = cmd[len('order:'):]
                self.interacting = False
                self.order_placed = True
                self.get_logger().info('KOMUT: Siparis alindi (telefona iletildi) -> masadan ayrilinacak')
            else:
                self.get_logger().info(f'Siparis kaydedildi, robot {self.state} durumunda - bir sey degismez.')

        elif cmd == 'resume':
            if self.state == STATE_AT_BARISTA:
                self.state = STATE_PATROL
                self.order_data = ''
                self.get_logger().info('KOMUT: Barmen onayladi -> Devriyeye devam')

        elif cmd.startswith('deliver:'):
            # deliver:<masa_no>:<siparis_id> - barmen "Hazir - Robot Gotursun" dedi
            try:
                _, table_s, order_s = cmd.split(':')
                idx, order_id = int(table_s) - 1, int(order_s)
            except ValueError:
                self.get_logger().warn(f'Gecersiz deliver komutu: {cmd}')
                return
            if not 0 <= idx < len(WAYPOINTS):
                self.get_logger().warn(f'KOMUT: deliver - Masa {idx + 1} tanimli degil.')
                return
            with self.delivery_lock:
                if order_id == self.delivery_order_id or any(o == order_id for _, o in self.delivery_queue):
                    return   # zaten sirada / yolda
                self.delivery_queue.append((idx, order_id))
            self.get_logger().info(
                f'KOMUT: Siparis #{order_id} hazir -> siraya alindi ({WAYPOINTS[idx]["isim"]}), '
                f'robot su an: {self.state}')
            self.maybe_start_delivery()

        elif cmd.startswith('undeliver:'):
            # sirada bekleyen (henuz alinmamis) teslimat iptal edildi
            try:
                order_id = int(cmd.split(':')[1])
            except (IndexError, ValueError):
                return
            with self.delivery_lock:
                self.delivery_queue = [(i, o) for i, o in self.delivery_queue if o != order_id]
            self.get_logger().info(f'KOMUT: Siparis #{order_id} teslimat sirasindan cikarildi.')

        elif cmd == 'continue':
            # Robot ekranindaki / garson telefonundaki kocaman "Devam Et" butonu.
            if self.state == STATE_AT_BARISTA:
                self.state = STATE_PATROL
                self.order_data = ''
                self.get_logger().info('KOMUT: Devam -> siparis iletildi, devriyeye devam')
            elif self.state == STATE_PICKUP_WAIT:
                self.state = STATE_SERVING
                self.get_logger().info('KOMUT: Devam -> siparis yuklendi, masaya gidiliyor')
            elif self.state == STATE_SERVED_WAIT:
                self.next_wp_index = self.delivery_table + 1   # servis edilen masadan sonrakine
                self.depart_pending = True                      # once masadan 1 m geri cekil
                self.delivery_order_id = 0
                self.state = STATE_PATROL if WAYPOINTS else STATE_IDLE
                self.say('Afiyet olsun!')
                self.get_logger().info('KOMUT: Devam -> siparis teslim edildi, devriyeye devam')
                self.maybe_start_delivery()   # siradaki teslimat varsa once o

        elif cmd == 'greet_door':
            if not busy_with_order:
                self.interacting = False
                self.state = STATE_GREET_DOOR
                self.get_logger().info('KOMUT: Kapida karsilama (3 dk)')

        elif cmd == 'relocalize' or cmd.startswith('set_pose:'):
            # Gercek robotta AMCL'e konum verme cafe_ui_server.py'de (/api/relocalize,
            # /api/set_pose) - Nav2'yi o baslatiyor. Burada yapacak bir sey yok.
            pass

        elif cmd == 'reload_waypoints':
            # Kurulum ekranindan yeni noktalar kaydedildi -> restart gerekmeden uygula.
            # Yeniden haritalamadan sonra da gelir: harita degismis olabilir.
            if apply_waypoints_cfg(load_waypoints_cfg(), logger=self.get_logger()):
                self.get_logger().info('KOMUT: Noktalar (waypoints.json) yeniden yuklendi.')
            else:
                self.get_logger().warn('KOMUT: reload_waypoints basarisiz, eski degerler korundu.')
            refresh_map_geometry(logger=self.get_logger())

        elif cmd == 'wake':
            self.screen_on = True
            self.get_logger().info('KOMUT: Robot ekrani ACIK')

        elif cmd == 'sleep':
            self.screen_on = False
            self.get_logger().info('KOMUT: Robot ekrani KAPALI')

        elif cmd.startswith('msg:'):
            # msg:<gonderen>|<alici>|<metin>
            parts = cmd[len('msg:'):].split('|', 2)
            if busy_with_order:
                self.get_logger().warn('KOMUT: msg - once siparis teslim edilmeli.')
            elif (len(parts) == 3 and parts[0].strip() and parts[2].strip()
                  and person_location(parts[1].strip()) is not None):
                self.msg_from = parts[0].strip()
                self.msg_to = parts[1].strip()
                self.msg_text = parts[2].strip()
                self.msg_phase = 'travel'
                self.msg_open_req = False
                self.msg_reply_text = None
                self.interacting = False
                self.greeting = False
                self.screen_on = True
                self.state = STATE_MESSENGER
                self.get_logger().info(f'KOMUT: Mesaj {self.msg_from} -> {self.msg_to}')
            else:
                self.get_logger().warn(f'Gecersiz msg komutu: {cmd}')

        elif cmd == 'msg_open':
            self.msg_open_req = True

        elif cmd == 'msg_compose_start':
            self.msg_composing = True      # cevap yaziliyor -> robot bekleyecek

        elif cmd == 'msg_compose_cancel':
            self.msg_composing = False

        elif cmd.startswith('msg_reply:'):
            self.msg_composing = False
            self.msg_reply_text = cmd[len('msg_reply:'):].strip()

        elif cmd.startswith('goto:'):
            # goto:<masa_no>:<eylem>   eylem = welcome | menu | welcome_menu | cute
            parts = cmd.split(':')
            try:
                idx = int(parts[1]) - 1
            except (IndexError, ValueError):
                idx = -1
            action = parts[2] if len(parts) > 2 else 'menu'
            if busy_with_order:
                self.get_logger().warn("KOMUT: goto - once siparis teslim edilmeli.")
            elif 0 <= idx < len(WAYPOINTS) and action in DIRECTED_ACTIONS:
                self.directed_table = idx
                self.directed_action = action
                self.interacting = False
                self.greeting = False
                self.cute = False
                self.screen_on = True     # garson yonlendirince ekran otomatik acilir
                self.state = STATE_DIRECTED
                self.get_logger().info(
                    f'KOMUT: Garson -> {WAYPOINTS[idx]["isim"]} ({action})')
            else:
                self.get_logger().warn(f'Gecersiz goto komutu: {cmd}')

        elif cmd.startswith('teleop:'):
            # teleop:<lx>:<az>  - operator ekrani sanal joystick (canli haritalama)
            if busy_with_order:
                self.get_logger().warn('KOMUT: teleop - siparis tasinirken kabul edilmez.')
            else:
                try:
                    _, lx_s, az_s = cmd.split(':')
                    lx = max(-TELEOP_MAX_LIN, min(TELEOP_MAX_LIN, float(lx_s)))
                    az = max(-TELEOP_MAX_ANG, min(TELEOP_MAX_ANG, float(az_s)))
                except (ValueError, IndexError):
                    lx = az = 0.0
                self.teleop_lx = lx
                self.teleop_az = az
                self.teleop_last_t = time.time()
                if self.state != STATE_TELEOP:
                    self.get_logger().info('KOMUT: Teleop (canli haritalama joystick) basladi')
                self.state = STATE_TELEOP

        elif cmd == 'teleop_stop':
            self.teleop_lx = 0.0
            self.teleop_az = 0.0
            if self.state == STATE_TELEOP:
                self.state = STATE_IDLE
                self.get_logger().info('KOMUT: Teleop durdu')

        elif cmd == 'rescue_start':
            # busy_with_order/map_ready KONTROLU YOK - robot siparis tasirken
            # bile takilabilir, o an tam da kurtarmaya en cok ihtiyac duyulan an.
            self.rescue_active = True
            self.rescue_last_t = time.time()
            self.get_logger().warn('KOMUT: KURTARMA MODU basladi (garson manuel suruyor).')

        elif cmd.startswith('rescue_teleop:'):
            try:
                _, lx_s, az_s = cmd.split(':')
                lx = max(-TELEOP_MAX_LIN, min(TELEOP_MAX_LIN, float(lx_s)))
                az = max(-TELEOP_MAX_ANG, min(TELEOP_MAX_ANG, float(az_s)))
            except (ValueError, IndexError):
                lx = az = 0.0
            self.rescue_lx = lx
            self.rescue_az = az
            self.rescue_last_t = time.time()
            self.rescue_active = True   # joystick'e her dokunuldugunda kurtarma AKTIF kalsin

        elif cmd == 'rescue_stop':
            # Sadece hizi sifirla + kurtarma modundan CIK (gorev kaldigi yerden
            # devam eder). Parmagin ekrandan gecici kalkmasiyla (rescue_teleop:0:0)
            # KARISTIRILMAMALI - o ayri, kurtarma modunda KALIR.
            self.rescue_active = False
            self.rescue_lx = 0.0
            self.rescue_az = 0.0
            self.get_logger().info('KOMUT: KURTARMA MODU bitti - gorev kaldigi yerden devam.')

        elif cmd == 'mapping_start':
            if busy_with_order:
                self.get_logger().warn('KOMUT: mapping_start - once siparis teslim edilmeli.')
            else:
                self.map_ready = False
                self.interacting = False
                self.greeting = False
                with self.delivery_lock:   # yeni haritada eski masalar gecersiz
                    self.delivery_order_id = 0
                    self.delivery_queue = []
                self.state = STATE_IDLE
                self.get_logger().warn('KOMUT: Haritalama/kurulum BASLADI - devriye kilitlendi.')

        elif cmd == 'mapping_done':
            self.map_ready = True
            self.get_logger().info('KOMUT: Kurulum TAMAMLANDI - devriye tekrar kullanilabilir.')

        else:
            self.get_logger().warn(f'Bilinmeyen komut: {cmd}')

    def maybe_start_delivery(self):
        """Siradaki teslimati, robot birakabilecegi bir isteyse (yolda/bosta)
        HEMEN baslatir. Musteriyle ilgileniyorsa bir sey yapmaz - o isten
        cikinca (ana dongu her turda cagirir) teslimat onceliklidir."""
        with self.delivery_lock:
            if (self.delivery_order_id or not self.delivery_queue or self.rescue_active
                    or self.state not in DELIVERY_INTERRUPTIBLE or not self.map_ready):
                return
            idx, order_id = self.delivery_queue.pop(0)
            if not 0 <= idx < len(WAYPOINTS):   # arada kurulum masalari degistirdiyse
                self.get_logger().warn(f'Teslimat #{order_id}: Masa {idx + 1} artik tanimli degil, atlandi.')
                return
            self.delivery_table, self.delivery_order_id = idx, order_id
            self.interacting = self.greeting = self.cute = False
            self.screen_on = True
            prev = self.state
            self.state = STATE_PICKUP
        self.get_logger().info(
            f'TESLIMAT #{order_id}: {prev} birakildi -> barmenden alinip {WAYPOINTS[idx]["isim"]} masasina')

    def _missing_target(self, cmd):
        """Komutun gidecegi nokta kurulumda tanimli degilse okunur adini dondurur."""
        if cmd in ('start', 'wander') and not WAYPOINTS:
            return 'Masalar'
        if cmd == 'go_home' and not HOME_POSITION:
            return 'Barmen noktası'
        if cmd == 'greet_door' and not DOOR_POS:
            return 'Kapı'
        if cmd.startswith('order:') and not BARISTA_POS:
            return 'Barmen noktası'
        if cmd.startswith('goto:'):
            try:
                idx = int(cmd.split(':')[1]) - 1
            except (IndexError, ValueError):
                return None                      # bicim hatasini asagidaki kontrol yakalar
            if not 0 <= idx < len(WAYPOINTS):
                return f'Masa {idx + 1}'
        return None

    def say(self, text):
        """Robotun (musteri ekrani hoparloru uzerinden) sesli okuyacagi mesaj."""
        self.speak_text = text
        self.speak_seq += 1
        self.get_logger().info(f'ROBOT KONUSUR: {text}')

    def drive_raw(self, twist):
        self.cmd_pub.publish(twist)

    def stop_motion(self):
        self.drive_raw(Twist())

    # ---- ultrasonik (SonarReader varsa; yoksa hepsi 'engel yok' doner) ----
    def ultra_blocked(self):
        return self.sonar.ultra_blocked() if self.sonar else False

    def blocked_sides(self):
        return self.sonar.blocked_sides() if self.sonar else (False, False, False, False)

    def closest_range(self):
        """4 sensorun en yakin okumasi (m) - sadece on degil, her yon. Veri yoksa sonsuz."""
        return self.sonar.min_range(SONAR_YAW.keys()) if self.sonar else float('inf')

    def publish_status_now(self):
        status = {
            'state': self.state,
            'waypoint': self.current_waypoint,
            'table_index': self.table_index,
            'wait_total': round(self.wait_total, 1),
            'wait_remaining': round(self.wait_remaining, 1),
            'interacting': self.interacting,
            'greeting': self.greeting,
            'cute': self.cute,
            'screen_on': self.screen_on,
            'speak_text': self.speak_text,
            'speak_seq': self.speak_seq,
            'order': self.order_data,
            'msg_phase': self.msg_phase,
            'msg_from': self.msg_from,
            'msg_to': self.msg_to,
            'msg_text': self.msg_text,
            'msg_total': round(self.msg_total, 1),
            'msg_remaining': round(self.msg_remaining, 1),
            'msg_composing': self.msg_composing,
            'map_ready': self.map_ready,
            'rescue_active': self.rescue_active,
            'fast_arrival': fast_arrival_seconds() > 0,
            'delivery_order_id': self.delivery_order_id,
            'delivery_queue': [o for _, o in self.delivery_queue],
            'delivery_table': (WAYPOINTS[self.delivery_table]['isim']
                               if self.delivery_order_id and self.delivery_table < len(WAYPOINTS) else ''),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


class SonarReader(Node):
    """4x HC-SR04'u AYRI bir node + executor'da dinler. Burada bir sorun olsa
    bile (deserialize hatasi vb.) patrol beyni etkilenmez."""

    def __init__(self):
        super().__init__('sonar_reader')
        self.ranges = {k: float('inf') for k in SONAR_YAW}
        self.range_t = {k: 0.0 for k in SONAR_YAW}
        for k in SONAR_YAW:
            self.create_subscription(
                Range, f'/ultrasonic/{k}',
                lambda m, key=k: self._cb(key, m),
                qos_profile_sensor_data)          # sensor verisi = BEST_EFFORT

    def _cb(self, key, msg):
        try:
            self.ranges[key] = float(msg.range)
            self.range_t[key] = time.time()
        except Exception:  # noqa: BLE001
            pass

    def _fresh(self):
        now = time.time()
        return {k: r for k, r in self.ranges.items()
                if now - self.range_t[k] < SONAR_STALE_SECONDS and 0.0 < r < 100.0}

    def ultra_blocked(self):
        return any(r < ESCAPE_TRIGGER for r in self._fresh().values())

    def min_range(self, keys):
        vals = [r for k, r in self._fresh().items() if k in keys]
        return min(vals) if vals else float('inf')

    def blocked_sides(self):
        """Hangi taraf(lar) < ESCAPE_TRIGGER? -> (on, arka, sol, sag) bool'lari."""
        trig = {k for k, r in self._fresh().items() if r < ESCAPE_TRIGGER}
        front = bool(trig & {'fl', 'fr'})
        rear = bool(trig & {'rl', 'rr'})
        left = bool(trig & {'fl', 'rl'})
        right = bool(trig & {'fr', 'rr'})
        return front, rear, left, right


# ==================== yardimcilar ====================
def person_location(name):
    """Mesajlasmada kisi adini bir hedef poza cevirir (tanimsizsa None)."""
    if name == 'Garson':
        return dict(BARISTA_POS, isim='Garson') if BARISTA_POS else None
    for wp in WAYPOINTS:
        if wp['isim'] == name:
            return wp
    return None


# ==================== masa/kapi/barmen noktalari (kurulum ekranindan) ====================
# web/setup.html haritaya tiklayarak nokta toplar, cafe_ui_server.py bunu
# config/waypoints.json'a yazar, biz burada okuyup WAYPOINTS/BARISTA_POS/
# HOME_POSITION/DOOR_POS'u YERINDE guncelleriz (restart gerekmez, 'reload_waypoints'
# komutuyla veya acilista otomatik).
def _pkg_dir(sub):
    """config/ veya maps/ - cafe_ui_server.py ile AYNI kural: --symlink-install ile
    kaynak agaci varsa ORASI (kurulum ekrani oraya yazar), yoksa paylasim dizini."""
    real_script = os.path.realpath(os.path.abspath(__file__))
    src_dir = os.path.join(os.path.dirname(os.path.dirname(real_script)), sub)
    if os.path.isdir(os.path.join(os.path.dirname(os.path.dirname(real_script)), 'web')):
        return src_dir
    return os.path.join(get_package_share_directory('cryvex_bringup'), sub)


def fast_arrival_seconds():
    """config/robot_settings.json -> "fast_arrival_seconds" (0/yok = KAPALI).

    MOTORLAR TAKILI DEGILKEN (tanitim videosu, masa testi) robot hedefe
    gidemez; drive() 15 sn'lik takilma + 5 kurtulma denemesiyle masa basina
    ~107 sn beklerdi. Bu ayar > 0 iken Nav2'ye hedef gitmez, robot yerinde
    kalir, hedefe bu kadar saniyede "varilmis" sayilir ve masadan geri
    cekilme atlanir. MOTORLAR TAKILINCA 0 YAPIN (dosya her suruste okunur,
    yeniden baslatma gerekmez; telefonun Robot Sagligi karti uyarir)."""
    try:
        with open(os.path.join(_pkg_dir('config'), 'robot_settings.json'), encoding='utf-8') as f:
            return max(0.0, float(json.load(f).get('fast_arrival_seconds', 0) or 0))
    except (OSError, ValueError, TypeError, AttributeError):
        return 0.0


def single_pass_enabled():
    """TEK_TUR - config/robot_settings.json -> "single_pass" (true = devriye
    masalari BIR KEZ sirayla gezer, sonra barmen/us noktasina donup durur;
    false/yok = eskisi gibi sonsuz dongu). Her turda okunur, restart gerekmez."""
    try:
        with open(os.path.join(_pkg_dir('config'), 'robot_settings.json'), encoding='utf-8') as f:
            return bool(json.load(f).get('single_pass', False))
    except (OSError, ValueError, TypeError, AttributeError):
        return False


def load_waypoints_cfg():
    """config/waypoints.json'u okur. Sim'den farkli: YOKSA uydurma nokta
    uretmez - bos kurulum bos kalir (robot tanimsiz yere gitmez)."""
    path = os.path.join(_pkg_dir('config'), 'waypoints.json')
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return {'barista': None, 'door': None, 'tables': []}
    except Exception as exc:  # noqa: BLE001
        print(f'[UYARI] waypoints.json okunamadi ({exc}).')
        return None


def _pose_dict(p, default_name):
    return {'isim': p.get('isim') or default_name, 'x': float(p['x']),
            'y': float(p['y']), 'yaw': float(p.get('yaw', 0.0))}


def apply_waypoints_cfg(cfg, logger=None):
    """cfg -> WAYPOINTS / BARISTA_POS / HOME_POSITION / DOOR_POS (yerinde degistirir,
    boylece calisan patrol dongusu de aninda yeni degerleri gorur). Sim'den farkli:
    cfg'de olmayan nokta BOSALTILIR (yeniden haritalama noktalari siler)."""
    def log(msg):
        logger.info(msg) if logger else print(msg)

    if cfg is None:
        log('[UYARI] Gecersiz waypoints konfigurasyonu, degisiklik yapilmadi.')
        return False
    try:
        tables = [_pose_dict(t, f'Masa {i + 1}') for i, t in enumerate(cfg.get('tables') or [])]
        b, d = cfg.get('barista'), cfg.get('door')
        barista = _pose_dict(b, 'Barmen') if b else {}
        door = _pose_dict(d, 'Kapi') if d else {}
    except Exception as exc:  # noqa: BLE001
        log(f'[HATA] waypoints uygulanamadi: {exc}')
        return False
    WAYPOINTS[:] = tables
    BARISTA_POS.clear()
    BARISTA_POS.update(barista)
    HOME_POSITION.clear()
    if barista:
        HOME_POSITION.update(dict(barista, isim='Us'))
    DOOR_POS.clear()
    DOOR_POS.update(door)

    def fmt(p):
        return f'({p["x"]:.2f},{p["y"]:.2f})' if p else 'YOK'
    log(f'[BILGI] Noktalar yuklendi: {len(WAYPOINTS)} masa, barmen/us={fmt(BARISTA_POS)}, '
        f'kapi={fmt(DOOR_POS)}')
    return True


# ==================== haritadan sosyallesme noktalari ====================
def _read_map():
    """maps/cafe_map.yaml + .pgm -> (piksel dizisi [satir=kuzeyden guneye], cozunurluk, orijin)."""
    maps_dir = _pkg_dir('maps')
    info = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0], 'image': 'cafe_map.pgm'}
    with open(os.path.join(maps_dir, 'cafe_map.yaml'), encoding='utf-8') as f:
        for line in f:
            key, _, val = line.partition(':')
            key, val = key.strip(), val.strip()
            if key == 'resolution':
                info['resolution'] = float(val)
            elif key == 'origin':
                info['origin'] = list(ast.literal_eval(val))
            elif key == 'image':
                info['image'] = val
    with open(os.path.join(maps_dir, info['image']), 'rb') as f:
        data = f.read()
    # P5 (ikili) PGM: "P5\n# yorum\n<w> <h>\n<max>\n<pikseller>" (map_saver_cli bicimi)
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError(f'desteklenmeyen PGM: {tokens[0]!r}')
    w, h = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)
    return pixels, info['resolution'], info['origin']


def refresh_map_geometry(logger=None):
    """WANDER_POINTS ve ROOM_CENTER'i kayitli haritadan hesaplar: bos (beyaz)
    ve cevresi WANDER_CLEARANCE_M boyunca engel/bilinmeyen icermeyen hucreler
    arasindan birbirinden uzak noktalar (en uzak nokta ornekleme)."""
    def log(msg):
        logger.info(msg) if logger else print(msg)
    try:
        pixels, res, origin = _read_map()
    except Exception as exc:  # noqa: BLE001
        log(f'[UYARI] Harita okunamadi ({exc}) - sosyallesme noktalari yok.')
        WANDER_POINTS[:] = []
        return
    blocked = (pixels < 250).astype(np.int32)       # 254 = bos; engel ve bilinmeyen = dolu
    r = max(1, int(round(WANDER_CLEARANCE_M / res)))
    padded = np.pad(blocked, r, constant_values=1)  # harita kenari da "dolu" sayilir
    csum = padded.cumsum(0).cumsum(1)
    csum = np.pad(csum, ((1, 0), (1, 0)))
    k = 2 * r + 1
    window = csum[k:, k:] - csum[:-k, k:] - csum[k:, :-k] + csum[:-k, :-k]
    rows, cols = np.nonzero(window == 0)             # cevresi tamamen bos hucreler
    h = pixels.shape[0]
    if len(rows) == 0:
        log('[UYARI] Haritada yeterince genis bos alan yok - sosyallesme noktalari yok.')
        WANDER_POINTS[:] = []
        return
    xs = origin[0] + (cols + 0.5) * res
    ys = origin[1] + (h - rows - 0.5) * res
    global ROOM_CENTER
    ROOM_CENTER = (float(xs.mean()), float(ys.mean()))
    chosen = [random.randrange(len(xs))]
    dist = np.hypot(xs - xs[chosen[0]], ys - ys[chosen[0]])
    while len(chosen) < min(WANDER_POINT_COUNT, len(xs)):
        i = int(dist.argmax())
        chosen.append(i)
        dist = np.minimum(dist, np.hypot(xs - xs[i], ys - ys[i]))
    WANDER_POINTS[:] = [(round(float(xs[i]), 2), round(float(ys[i]), 2)) for i in chosen]
    log(f'[BILGI] Haritadan {len(WANDER_POINTS)} sosyallesme noktasi, oda merkezi '
        f'({ROOM_CENTER[0]:.2f},{ROOM_CENTER[1]:.2f}).')


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(navigator, x, y, yaw=0.0, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    z, w = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def _nudge_to_center(x, y, dist):
    if dist <= 0.0:
        return x, y
    dx, dy = ROOM_CENTER[0] - x, ROOM_CENTER[1] - y
    n = math.hypot(dx, dy) or 1.0
    return x + dx / n * dist, y + dy / n * dist


def _escape_maneuver(listener, reason, expected_state):
    """Nav2 gorevi IPTAL edildikten sonra cagrilir. SABIT ADIMLI manevra:
    dur+bekle -> duz geri/ileri -> don -> az ileri -> hedefe tekrar dene.
    Surekli "it/yakinlas" dongusu YERINE tek seferlik, kesin bir hareket -
    boylece robot ayni noktaya/yone geri donup tekrar tekrar sikismiyor."""
    log = listener.get_logger()

    def _still_active():
        return listener.state == expected_state

    def _phase(vx, wz, seconds, label):
        log.info(f'  kurtulma adimi: {label} ({seconds:.1f} sn, vx={vx:.2f} wz={wz:.2f})')
        t0 = time.time()
        while time.time() - t0 < seconds:
            if not _still_active():
                return False
            tw = Twist()
            tw.linear.x = vx
            tw.angular.z = wz
            listener.drive_raw(tw)
            time.sleep(0.1)
        return True

    log.warn(f'KURTULMA MANEVRASI baslıyor ({reason})...')

    # 1) DUR ve ESCAPE_SETTLE_SECONDS bekle (aninda tepki yerine algiyi dogrula)
    listener.stop_motion()
    t0 = time.time()
    while time.time() - t0 < ESCAPE_SETTLE_SECONDS:
        if not _still_active():
            return
        time.sleep(0.1)

    # Hangi taraf tetiklendi? -> yon kararlari
    front, rear, left, right = listener.blocked_sides() if reason == 'ultra' else (False,) * 4

    # On yakinsa GERI, SADECE arka yakinsa ILERI git.
    back_vx = ESCAPE_BACK_LIN if (rear and not front) else -ESCAPE_BACK_LIN
    # Sol yakinsa SAGA don (REP-103: -wz = saga/CW), sag yakinsa SOLA; bilinmezse varsayilan SAG.
    turn_wz = ESCAPE_TURN_ANG if (right and not left) else -ESCAPE_TURN_ANG

    if not _phase(back_vx, 0.0, ESCAPE_BACK_SECONDS, 'duz geri' if back_vx < 0 else 'duz ileri'):
        listener.stop_motion(); return
    if not _phase(0.0, turn_wz, ESCAPE_TURN_SECONDS, 'saga don' if turn_wz < 0 else 'sola don'):
        listener.stop_motion(); return
    _phase(ESCAPE_FWD_LIN, 0.0, ESCAPE_FWD_SECONDS, 'az ileri')

    listener.stop_motion()
    time.sleep(0.3)
    log.info('KURTULMA tamam -> hedefe tekrar deneniyor.')


def _rescue_maneuver(listener, expected_state):
    """Garson '🆘 Kurtar' ile joystick uzerinden MANUEL suruyor (drive() Nav2
    gorevini zaten iptal etti). rescue_active False olana kadar (yani
    'Bitti, Devam Et'e basana kadar) burada kalinir - suresiz, otomatik
    zaman asimi YOK (garson karar verir). Bittiginde drive() cagirani AYNI
    hedefe kaldigi yerden devam eder; ne state ne de gorev degisti, sadece
    robot fiziksel olarak baska bir yerde/yonde - Nav2 oradan yeniden planlar."""
    log = listener.get_logger()
    log.warn('KURTARMA MODU basladi - garson manuel suruyor, "Bitti"ye basmasi bekleniyor...')
    while listener.rescue_active and listener.state == expected_state:
        if time.time() - listener.rescue_last_t > TELEOP_WATCHDOG_SECONDS:
            listener.stop_motion()          # parmak kalkti/baglanti koptu -> guvenlik durusu
            time.sleep(0.05)
            continue
        lx, az = listener.rescue_lx, listener.rescue_az
        front, rear, _left, _right = listener.blocked_sides()
        if lx > 0.0 and front:              # kurtarirken bile 10 cm engele SURME
            lx = 0.0
        if lx < 0.0 and rear:
            lx = 0.0
        tw = Twist()
        tw.linear.x = lx
        tw.angular.z = az
        listener.drive_raw(tw)
        time.sleep(0.05)
    listener.stop_motion()
    time.sleep(0.3)
    log.info('KURTARMA MODU bitti -> hedefe kaldigi yerden tekrar deneniyor.')


def _depart_maneuver(listener, expected_state):
    """Masadan ayrilirken donup gitmeden once ~1 m DUZ geri cekilir. Boylece
    Nav2 donusu/sonraki hedefe gidisi masaya/sandalyeye cok yakinken degil,
    acik alanda yapar. Arkada bir sey algilanirsa (10 cm) erken durur."""
    log = listener.get_logger()
    if fast_arrival_seconds() > 0:   # motorsuz hizli varis: robot yerinde kalir
        return
    log.info(f'[AYRILIS] {listener.current_waypoint} -> {DEPART_BACK_METERS:.0f} m geri cekiliniyor...')
    t0 = time.time()
    while time.time() - t0 < DEPART_BACK_SECONDS:
        if listener.state != expected_state:
            break
        if listener.ultra_blocked():
            log.warn('[AYRILIS] Arkada engel algilandi, geri cekilme erken kesildi.')
            break
        tw = Twist()
        tw.linear.x = -DEPART_BACK_LIN
        listener.drive_raw(tw)
        time.sleep(0.1)
    listener.stop_motion()
    time.sleep(0.3)
    log.info('[AYRILIS] Tamam -> sonraki konuma donuluyor.')


def drive(navigator, listener, target, expected_state):
    """target -> hedefe git. Durum expected_state disina cikarsa iptal.

    - Hedef reddedilirse oda merkezine dogru kademeli kaydirip tekrar dener.
    - Hedefe yaklasilmisken on sensor < APPROACH_STANDOFF (25 cm) okursa: fazla
      yanasmaya calismadan gorevi TEMIZ bitirip basarili sayar (masaya/insana
      cok yaklasip takilmayi onler).
    - Ultrasonik < 10 cm (fiziksel temas) veya 15 sn ilerleme yok -> kurtulma
      manevrasi + ayni hedefe tekrar dene.
    Donus: True basarili / False kesildi veya basarisiz.
    """
    log = listener.get_logger()

    fast = fast_arrival_seconds()
    if fast > 0:
        # Motorsuz hizli varis (bkz. fast_arrival_seconds): Nav2'ye hedef
        # gitmez, robot yerinde kalir, kisa bir "yol"dan sonra varilmis sayilir.
        listener.current_waypoint = target['isim']
        log.info(f"[HIZLI VARIS] {target['isim']} -> {fast:.0f} sn sonra varilmis sayilacak.")
        t0 = time.time()
        while time.time() - t0 < fast:
            if listener.state != expected_state:
                return False
            time.sleep(0.2)
        listener.last_pos = target
        return listener.state == expected_state

    if not navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        log.error(
            "NAV2 YOK: 'navigate_to_pose' sunucusu bulunamadi. cafe_ui_server "
            "Nav2'yi kayitli haritayla baslatti mi (haritalama suruyor olabilir)?")
        for _ in range(6):
            if listener.state != expected_state:
                return False
            time.sleep(0.5)
        return False

    tx, ty = float(target['x']), float(target['y'])
    yaw = target.get('yaw')
    if yaw is None:
        prev = listener.last_pos or HOME_POSITION
        yaw = math.atan2(ty - prev['y'], tx - prev['x']) if prev else 0.0
    listener.current_waypoint = target['isim']

    oi = 0                 # offset index (sadece gercek nav hatasinda artar)
    escapes = 0
    while oi < len(RETRY_OFFSETS):
        off = RETRY_OFFSETS[oi]
        gx, gy = _nudge_to_center(tx, ty, off)
        tag = target['isim'] + ('' if off == 0.0 else f' (+{off:.2f}m ic)')
        log.info(f"Hedef gonderiliyor -> {tag}  x={gx:.2f} y={gy:.2f}")

        accepted = navigator.goToPose(make_pose(navigator, gx, gy, yaw))
        if accepted is False:
            log.warn(f"{tag}: hedef REDDEDILDI. Kaydirip deniyorum.")
            oi += 1
            if listener.state != expected_state:
                return False
            continue

        escape_reason = None
        best_dist = None
        last_progress = time.time()
        last_fb_log = 0.0
        while not navigator.isTaskComplete():
            if listener.state != expected_state:
                navigator.cancelTask()
                listener.last_pos = target
                return False

            fb = navigator.getFeedback()
            dist_left = None
            now = time.time()
            if fb is not None:
                try:
                    dist_left = float(fb.distance_remaining)
                except Exception:  # noqa: BLE001
                    dist_left = None
                if dist_left is not None and (best_dist is None or dist_left < best_dist - 0.10):
                    best_dist = dist_left
                    last_progress = now
                if dist_left is not None and now - last_fb_log > 4.0:
                    last_fb_log = now
                    log.info(f"  ...{tag} yolunda, kalan ~{dist_left:.2f} m")

            # Hedefe zaten YAKINKEN herhangi bir sensor cok yakin okursa fazla
            # yanasmaya calisma - "yeterince yaklasildi" say ve orada dur.
            near_r = listener.closest_range()
            if (dist_left is not None and dist_left < APPROACH_GATE_DIST
                    and near_r < APPROACH_STANDOFF):
                log.info(f"{tag}: sensor {near_r*100:.0f} cm -> "
                         f"yeterince yaklasildi, fazla yanasilmiyor.")
                navigator.cancelTask()
                listener.stop_motion()
                listener.last_pos = target
                return True

            if listener.rescue_active:                    # garson "Kurtar" ile devraldi
                escape_reason = 'rescue'
                break

            if listener.ultra_blocked():                 # < 10 cm fiziksel engel
                escape_reason = 'ultra'
                break

            if now - last_progress > STALL_ESCAPE_SECONDS:  # 15 sn takildi kaldi
                escape_reason = 'stall'
                break
            time.sleep(0.2)

        if escape_reason:
            navigator.cancelTask()
            if escape_reason == 'rescue':
                # Garsonun manuel suruşu MAX_ESCAPES_PER_GOAL'a SAYILMAZ - bu bir
                # hata degil, bilincli mudahale. Bittiginde AYNI hedefe (kaldigi
                # yerden, gorev/state degismeden) tekrar denenir.
                _rescue_maneuver(listener, expected_state)
            else:
                escapes += 1
                _escape_maneuver(listener, escape_reason, expected_state)
                if escapes >= MAX_ESCAPES_PER_GOAL:
                    log.error(f"{target['isim']}: {escapes} kurtulma denemesi yetmedi, "
                              "adim atlaniyor -> devriyeye devam.")
                    listener.last_pos = target
                    return False
            listener.last_pos = target
            if listener.state != expected_state:
                return False
            continue                                     # ayni offset, tekrar dene

        listener.last_pos = target
        if listener.state != expected_state:
            return False

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            log.info(f"{tag}: ULASILDI.")
            return True
        log.warn(f"{tag}: Nav2 sonucu {result}, kaydirip deniyorum.")
        oi += 1
        if listener.state != expected_state:
            return False

    log.error(f"{target['isim']}: tum denemeler basarisiz, sonraki adima geciliyor.")
    return False


# ==================== ana dongu ====================
def main():
    rclpy.init()

    listener = CommandListener()
    # Kurulum ekranindan (web/setup.html) kaydedilmis noktalar varsa yukle
    # (yoksa bos baslar) ve sosyallesme noktalarini haritadan hesapla.
    apply_waypoints_cfg(load_waypoints_cfg(), logger=listener.get_logger())
    refresh_map_geometry(logger=listener.get_logger())
    executor = SingleThreadedExecutor()
    executor.add_node(listener)

    def _resilient_spin(exc, node, label, give_up_after=0):
        """Take/deserialize hatasi spin'i OLDURMESIN. give_up_after > 0 ise, o kadar
        art arda hatadan sonra bu executor'i komple birak (opsiyonel ozellik icin)."""
        fails = 0
        warned = False
        while rclpy.ok():
            try:
                exc.spin_once(timeout_sec=0.2)
                fails = 0
            except Exception as e:  # noqa: BLE001
                fails += 1
                if not warned:
                    warned = True
                    node.get_logger().warn(f'{label}: abonelik hatasi ({e}) - yok sayiliyor.')
                if give_up_after and fails >= give_up_after:
                    node.get_logger().warn(
                        f'{label}: surekli hata -> BU OZELLIK DEVRE DISI. '
                        '(patrol normal calismaya devam eder)')
                    return
                time.sleep(0.05)
    threading.Thread(target=_resilient_spin, args=(executor, listener, 'patrol'),
                     daemon=True).start()

    # Ultrasonik: AYRI node + AYRI executor. Sorun cikarsa patrol etkilenmez;
    # surekli hata verirse bu is parcacigi kendini kapatir (sonar'siz devam).
    sonar = SonarReader()
    sonar_exec = SingleThreadedExecutor()
    sonar_exec.add_node(sonar)
    threading.Thread(target=_resilient_spin,
                     args=(sonar_exec, sonar, 'sonar', 30), daemon=True).start()
    listener.sonar = sonar

    # --- Yinelenen node kontrolu (UYARIR, cikmaz) ---
    # Genelde diger 'patrol_node' onceki calismadan kalma OLU bir orphan'dir
    # (executor'i cokmus, hedef gondermiyor). O yuzden bu SAGLIKLI kopya devam
    # eder; sadece kullaniciyi uyaririz. Kesin cozum: eski process'i oldur.
    def _dup_check():
        my_name = listener.get_name()
        warned = False
        while rclpy.ok():
            twins = [n for n in listener.get_node_names() if n == my_name]
            if len(twins) > 1 and not warned:
                warned = True
                listener.get_logger().warn(
                    f"\n{'='*58}\n"
                    f"  UYARI: {len(twins)} adet '{my_name}' graf'ta gorunuyor.\n"
                    f"  Muhtemelen eski calismadan kalma OLU bir process. Bu\n"
                    f"  kopya calismaya devam ediyor. Emin olmak icin:\n"
                    f"     ps -eo pid,cmd | grep patrol.py | grep -v grep\n"
                    f"     kill -9 <ESKI PID>       (bu process haric)\n"
                    f"  Cift makinede/domain'de ise:  export ROS_DOMAIN_ID=7\n"
                    f"{'='*58}")
            elif len(twins) <= 1:
                warned = False
            time.sleep(5.0)
    threading.Thread(target=_dup_check, daemon=True).start()

    # /patrol_status'u duvar saatiyle sabit ~4 Hz yayinla (executor/sim-time'dan bagimsiz)
    def _status_loop():
        while rclpy.ok():
            try:
                listener.publish_status_now()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.25)
    threading.Thread(target=_status_loop, daemon=True).start()
    listener.get_logger().info('/patrol_status yayini basladi (wall-clock, 4 Hz).')

    # Gercek saat (sim'deki use_sim_time=True YOK). Baslangic konumu ve
    # "Robotu use sabitle" cafe_ui_server.py'de - bkz. modul docstring'i.
    navigator = BasicNavigator()

    listener.get_logger().info('Nav2 action sunucusu bekleniyor (en fazla 20 sn)...')
    try:
        if navigator.nav_to_pose_client.wait_for_server(timeout_sec=20.0):
            listener.get_logger().info('Nav2 hazir.')
        else:
            listener.get_logger().warn(
                'Nav2 sunucusu 20 sn icinde gelmedi. Node calisiyor; acilinca baglanacak.')
    except Exception as exc:  # noqa: BLE001
        listener.get_logger().warn(f'Nav2 sunucu kontrolu atlandi: {exc}')

    listener.get_logger().info(
        '==================================================\n'
        '  PATROL HAZIR.\n'
        '   Robot ekrani / telefon : http://<ip>:8080/  (cafe_ui_server)\n'
        '   Komutlar: start / stop / go_home / wander / greet_door /\n'
        '             goto:<masa>:<menu|welcome|cute>\n'
        '==================================================')

    wp_index = 0
    wander_prev = None

    while rclpy.ok():
        # Sirada hazir siparis varsa ve robot birakabilecegi bir isteyse
        # (yolda/bosta) once teslimat - musteriden ayrildigi an burada baslar.
        listener.maybe_start_delivery()
        state = listener.state

        # Robot bir is yapiyorsa musteri ekrani otomatik acilsin
        if state != STATE_IDLE:
            listener.screen_on = True
        if state != STATE_MESSENGER and listener.msg_phase:
            listener.msg_phase = ''
            listener.msg_remaining = 0.0
            listener.msg_composing = False

        # ---------------- DEVRIYE (sadece masalar; kapi karsilamasi AYRI komut) ----------------
        if state == STATE_PATROL:
            if not WAYPOINTS:   # kurulum hicbir masa kaydetmeden bir sekilde buraya dusulduyse
                listener.get_logger().error('WAYPOINTS bos - devriye bekletiliyor.')
                listener.state = STATE_IDLE
                continue
            if listener.depart_pending:            # siparis teslim edilen masadan ayrilis
                listener.depart_pending = False
                _depart_maneuver(listener, STATE_PATROL)
                if listener.state != STATE_PATROL:
                    continue
            if getattr(listener, 'patrol_restart', False):   # TEK_TUR: yeni 'start' -> ilk masa
                listener.patrol_restart = False
                wp_index = 0
            if listener.next_wp_index is not None:  # teslimattan sonra sonraki masadan devam
                wp_index = listener.next_wp_index
                listener.next_wp_index = None
            if wp_index >= len(WAYPOINTS) and single_pass_enabled():
                # TEK_TUR: tum masalar bir kez gezildi -> barmen/us noktasina don ve dur
                wp_index = 0
                listener.get_logger().info('[TEK TUR] Masalar bitti -> use donuluyor.')
                listener.state = STATE_GOING_HOME if HOME_POSITION else STATE_IDLE
                continue
            wp_index %= len(WAYPOINTS)
            wp = WAYPOINTS[wp_index]
            listener.table_index = wp_index
            listener.get_logger().info(f'--> {wp["isim"]} hedefine gidiliyor...')
            reached = drive(navigator, listener, wp, STATE_PATROL)
            if listener.state != STATE_PATROL:
                continue
            if reached:
                listener.get_logger().info(f'{wp["isim"]} ulasildi.')
            else:
                listener.get_logger().warn(f'{wp["isim"]} ulasilamadi, yine de siparis bekleniyor.')
            listener.state = STATE_WAITING

        # ---------------- MASADA BEKLEME ----------------
        elif state == STATE_WAITING:
            listener.wait_total = TABLE_WAIT_SECONDS
            listener.wait_remaining = TABLE_WAIT_SECONDS
            listener.interacting = False
            listener.order_placed = False
            interact_elapsed = 0.0
            listener.get_logger().info(
                f'[{listener.current_waypoint}] Siparis bekleniyor ({TABLE_WAIT_SECONDS:.0f}s)...')

            while listener.state == STATE_WAITING:
                time.sleep(0.5)
                if listener.order_placed:
                    # Siparis telefona gitti: tesekkur ekrani gorunsun, sonra ayril.
                    # (Sirada teslimat varsa ana dongu hemen onu baslatir.)
                    t = 0.0
                    while listener.state == STATE_WAITING and t < ORDER_THANKS_SECONDS:
                        time.sleep(0.25)
                        t += 0.25
                    listener.order_placed = False
                    if listener.state != STATE_WAITING:
                        break
                    _depart_maneuver(listener, STATE_WAITING)
                    if listener.state == STATE_WAITING:
                        wp_index += 1
                        listener.state = STATE_PATROL
                    break
                if listener.interacting:
                    interact_elapsed += 0.5
                    if interact_elapsed >= INTERACT_TIMEOUT_SECONDS:
                        listener.get_logger().warn('Menu cok uzun acik kaldi, sayac devam.')
                        listener.interacting = False
                        interact_elapsed = 0.0
                    continue
                interact_elapsed = 0.0
                listener.wait_remaining = max(0.0, listener.wait_remaining - 0.5)
                if listener.wait_remaining <= 0.0:
                    listener.get_logger().info(
                        f'[{listener.current_waypoint}] Siparis yok, masadan ayriliniyor.')
                    _depart_maneuver(listener, STATE_WAITING)  # once 1 m duz geri
                    if listener.state == STATE_WAITING:        # arada komut gelmediyse
                        wp_index += 1
                        listener.state = STATE_PATROL
                    break

        # ---------------- SIPARISI BARMENE GOTUR ----------------
        elif state == STATE_DELIVERING:
            listener.wait_remaining = 0.0
            listener.interacting = False
            listener.get_logger().info('[SIPARIS] Barmene gidiliyor...')
            reached = drive(navigator, listener, BARISTA_POS, STATE_DELIVERING)
            if listener.state != STATE_DELIVERING:
                continue
            if reached:
                listener.get_logger().info('Barmene ulasildi. Onay bekleniyor...')
            else:
                listener.get_logger().warn('Barmene ulasilamadi, yine de onay bekleniyor.')
            listener.state = STATE_AT_BARISTA

        # ---------------- BARMENDE BEKLE (sadece resume ile cikar) ----------------
        elif state == STATE_AT_BARISTA:
            time.sleep(0.3)
            if listener.state == STATE_PATROL:
                wp_index += 1

        # ---------------- HAZIR SIPARISI MASAYA GOTUR ----------------
        elif state == STATE_PICKUP:
            listener.wait_remaining = 0.0
            if listener.depart_pending:            # az once siparis teslim edilen masadan ayrilis
                listener.depart_pending = False
                _depart_maneuver(listener, STATE_PICKUP)
                if listener.state != STATE_PICKUP:
                    continue
            listener.get_logger().info(f'[TESLIMAT #{listener.delivery_order_id}] Barmene gidiliyor...')
            if not drive(navigator, listener, BARISTA_POS, STATE_PICKUP):
                if listener.state != STATE_PICKUP:
                    continue
                listener.get_logger().warn('[TESLIMAT] Barmene ulasilamadi, yine de yukleme bekleniyor.')
            if listener.state != STATE_PICKUP:
                continue
            listener.current_waypoint = BARISTA_POS.get('isim', 'Barmen')
            listener.state = STATE_PICKUP_WAIT
            table = WAYPOINTS[listener.delivery_table]['isim']
            listener.say(f'{table} siparişini alabilirim. Yerleştirince Devam\'a basın.')

        elif state == STATE_SERVING:
            idx = listener.delivery_table
            if not 0 <= idx < len(WAYPOINTS):   # arada kurulum masalari degistirdiyse
                listener.get_logger().error(f'[TESLIMAT] Masa {idx + 1} artik tanimli degil - teslimat iptal.')
                listener.delivery_order_id = 0
                listener.state = STATE_IDLE
                continue
            wp = WAYPOINTS[idx]
            listener.table_index = idx
            listener.get_logger().info(f'[TESLIMAT #{listener.delivery_order_id}] {wp["isim"]} masasina gidiliyor...')
            if not drive(navigator, listener, wp, STATE_SERVING) and listener.state == STATE_SERVING:
                listener.get_logger().warn(f'[TESLIMAT] {wp["isim"]} masasina ulasilamadi, yine de burada bekleniyor.')
            if listener.state != STATE_SERVING:
                continue
            listener.current_waypoint = wp['isim']
            listener.state = STATE_SERVED_WAIT
            listener.say(f'{wp["isim"]}, siparişiniz geldi! Afiyet olsun. Aldıktan sonra Devam\'a basın.')

        elif state in (STATE_PICKUP_WAIT, STATE_SERVED_WAIT):
            # Bilerek ZAMAN ASIMI YOK: robot ekrani ya da garson telefonundan
            # "Devam" gelmeden kipirdamaz ('continue' komutu).
            time.sleep(0.3)

        # ---------------- WANDER & GREET ----------------
        elif state == STATE_WANDER:
            # Haritadan bos alan bulunamadiysa masalarin arasinda dolas.
            pool = WANDER_POINTS or [(wp['x'], wp['y']) for wp in WAYPOINTS]
            if not pool:
                listener.get_logger().warn('[WANDER] Gidilecek nokta yok - bosta bekleniyor.')
                listener.state = STATE_IDLE
                continue
            choices = [p for p in pool if p != wander_prev] or pool
            px, py = random.choice(choices)
            wander_prev = (px, py)
            tgt = {"isim": "Sosyallesme", "x": px, "y": py}
            listener.get_logger().info(f'[WANDER] ({px:.1f}, {py:.1f}) noktasina gidiliyor...')
            drive(navigator, listener, tgt, STATE_WANDER)
            if listener.state != STATE_WANDER:
                continue
            # Selamlasma molasi (arayuz burada pazarlama anonsu yapar)
            listener.current_waypoint = 'Sosyallesme'
            listener.greeting = True
            t = 0.0
            while listener.state == STATE_WANDER and t < WANDER_GREET_SECONDS:
                time.sleep(0.5)
                t += 0.5
            listener.greeting = False

        # ---------------- KAPIDA KARSILAMA (garson komutu, 3 dk) ----------------
        elif state == STATE_GREET_DOOR:
            listener.get_logger().info('--> Kapiya gidiliyor (karsilama)...')
            drive(navigator, listener, DOOR_POS, STATE_GREET_DOOR)
            if listener.state != STATE_GREET_DOOR:
                continue
            listener.current_waypoint = 'Kapi'
            listener.greeting = True
            listener.get_logger().info(f'Kapida "Hos geldiniz" ({DOOR_GREET_SECONDS/60:.0f} dk)...')
            t = 0.0
            while listener.state == STATE_GREET_DOOR and t < DOOR_GREET_SECONDS:
                time.sleep(0.5)
                t += 0.5
            listener.greeting = False
            if listener.state == STATE_GREET_DOOR:      # sure doldu -> use don
                listener.get_logger().info('Karsilama bitti -> Use donuluyor.')
                listener.state = STATE_GOING_HOME

        # ---------------- GARSONUN YONLENDIRDIGI MASA ----------------
        elif state == STATE_DIRECTED:
            idx = listener.directed_table
            action = listener.directed_action
            if not 0 <= idx < len(WAYPOINTS):   # arada kurulum masalari degistirdiyse
                listener.get_logger().warn(f'Yonlendirilen masa {idx + 1} artik tanimli degil.')
                listener.state = STATE_IDLE
                continue
            wp = WAYPOINTS[idx]
            listener.table_index = idx
            listener.get_logger().info(f'--> Garson: {wp["isim"]} ({action}) hedefine gidiliyor...')
            drive(navigator, listener, wp, STATE_DIRECTED)
            if listener.state != STATE_DIRECTED:
                continue
            listener.current_waypoint = wp['isim']

            # Garsonun sectigi hazir mesaji robot sesli okur
            listener.say(DIRECTED_MESSAGES.get(action, DIRECTED_MESSAGES['menu']))

            if action in ('menu', 'welcome_menu'):
                # 10 sn menu sureci; dolarsa normal devriyeye devam et
                wp_index = idx
                listener.state = STATE_WAITING
            elif action == 'welcome':
                listener.greeting = True
                t = 0.0
                while listener.state == STATE_DIRECTED and t < DIRECTED_GREET_SECONDS:
                    time.sleep(0.5)
                    t += 0.5
                listener.greeting = False
                if listener.state == STATE_DIRECTED:
                    listener.state = STATE_IDLE
            elif action == 'cute':
                listener.cute = True
                t = 0.0
                while listener.state == STATE_DIRECTED and t < DIRECTED_GREET_SECONDS:
                    time.sleep(0.5)
                    t += 0.5
                listener.cute = False
                if listener.state == STATE_DIRECTED:
                    listener.state = STATE_IDLE

        # ---------------- MESAJ TASIMA (robot postaci) ----------------
        elif state == STATE_MESSENGER:
            target = person_location(listener.msg_to)
            if target is None:
                listener.get_logger().warn(f'Bilinmeyen alici: {listener.msg_to}')
                listener.msg_phase = ''
                listener.state = STATE_PATROL
                continue

            # 1) Aliciya git
            listener.msg_phase = 'travel'
            listener.get_logger().info(f'[MESAJ] {listener.msg_to} noktasina goturuluyor...')
            drive(navigator, listener, target, STATE_MESSENGER)
            if listener.state != STATE_MESSENGER:
                continue
            listener.current_waypoint = listener.msg_to

            # 2) "Acmak icin dokun" - alici dokununcaya kadar bekle
            #    (SESLI OKUMA YOK - mesaj sadece ekranda gosterilir)
            listener.msg_phase = 'pending'
            listener.msg_open_req = False
            t = 0.0
            while (listener.state == STATE_MESSENGER and not listener.msg_open_req
                   and t < MSG_OPEN_TIMEOUT):
                time.sleep(0.5)
                t += 0.5
            if listener.state != STATE_MESSENGER:
                continue
            if not listener.msg_open_req:
                listener.get_logger().info('[MESAJ] Acilmadi -> devriyeye donuluyor.')
                listener.msg_phase = ''
                listener.state = STATE_PATROL
                continue

            # 3) Okuma + cevap penceresi (15 sn). Mesaj ve "Cevap Ver" butonu
            #    bu 15 sn boyunca ekrandadir. Butona basilirsa (msg_composing)
            #    geri sayim DONAR ve kullanici yazmayi bitirene kadar beklenir.
            #    (Ayri 5 sn'lik pencere kaldirildi - cok kisaydi, robot kaciyordu.)
            listener.msg_phase = 'reading'
            listener.msg_reply_text = None
            listener.msg_composing = False
            listener.msg_total = MSG_READ_SECONDS
            listener.msg_remaining = MSG_READ_SECONDS
            compose_wait = 0.0
            while (listener.state == STATE_MESSENGER
                   and listener.msg_reply_text is None):
                time.sleep(0.5)
                if listener.msg_composing:
                    listener.msg_phase = 'replying'
                    compose_wait += 0.5
                    if compose_wait >= MSG_COMPOSE_TIMEOUT:
                        listener.get_logger().info('[MESAJ] Cevap yazma zaman asimi.')
                        break
                    continue
                listener.msg_phase = 'reading'
                listener.msg_remaining = max(0.0, listener.msg_remaining - 0.5)
                if listener.msg_remaining <= 0.0:
                    # Kisa lutuf suresi: "Cevap Ver"e yeni basildiysa komut ag
                    # gecikmesiyle henuz gelmemis olabilir; 3 sn daha bekle.
                    grace = 0.0
                    while (listener.state == STATE_MESSENGER and grace < 3.0
                           and not listener.msg_composing
                           and listener.msg_reply_text is None):
                        time.sleep(0.2)
                        grace += 0.2
                    if not listener.msg_composing and listener.msg_reply_text is None:
                        break
            if listener.state != STATE_MESSENGER:
                continue

            if listener.msg_reply_text:
                # Cevabi ters yone (alici -> gonderen) tasi, dongu tekrar eder
                listener.msg_from, listener.msg_to = listener.msg_to, listener.msg_from
                listener.msg_text = listener.msg_reply_text
                listener.msg_reply_text = None
                listener.msg_open_req = False
                listener.get_logger().info(f'[MESAJ] Cevap -> {listener.msg_to}')
            else:
                listener.get_logger().info('[MESAJ] Cevap yok -> devriyeye donuluyor.')
                listener.msg_phase = ''
                listener.msg_remaining = 0.0
                listener.state = STATE_PATROL

        # ---------------- TELEOP (operator canli haritalama joystick'i) ----------------
        elif state == STATE_TELEOP:
            if time.time() - listener.teleop_last_t > TELEOP_WATCHDOG_SECONDS:
                # Taze komut yok (parmak kalkti / baglanti koptu) -> guvenlik durusu.
                listener.stop_motion()
                time.sleep(0.05)
                continue
            lx, az = listener.teleop_lx, listener.teleop_az
            front, rear, _left, _right = listener.blocked_sides()
            if lx > 0.0 and front:      # 10 cm altinda engele dogru ilerlemeyi engelle
                lx = 0.0
            if lx < 0.0 and rear:
                lx = 0.0
            tw = Twist()
            tw.linear.x = lx
            tw.angular.z = az
            listener.drive_raw(tw)
            time.sleep(0.05)

        # ---------------- USE DON ----------------
        elif state == STATE_GOING_HOME:
            if not HOME_POSITION:
                listener.get_logger().warn('[US] Barmen noktasi tanimli degil - bosta bekleniyor.')
                listener.state = STATE_IDLE
                continue
            listener.get_logger().info('[US] Barmen noktasina donuluyor...')
            drive(navigator, listener, HOME_POSITION, STATE_GOING_HOME)
            if listener.state != STATE_GOING_HOME:
                continue
            listener.current_waypoint = ''
            listener.state = STATE_IDLE

        # ---------------- BOSTA ----------------
        else:
            listener.wait_remaining = 0.0
            time.sleep(0.3)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
