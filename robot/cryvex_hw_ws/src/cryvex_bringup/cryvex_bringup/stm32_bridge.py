#!/usr/bin/env python3
"""
Cryvex STM32 Koprusu
====================
Raspberry Pi 5 <-> STM32 (Nucleo-F446RE) arasindaki satir-tabanli seri
protokolu (bkz. ~/cryvex_hw_ws/docs/stm32_protokol.md) ROS 2'ye baglar.

TASARIM ILKESI: Gazebo simulasyonundaki (~/cryvex_ws) patrol.py'nin
kurtulma/yaklasma mantigi AYNEN calissin diye, gercek donanimin sensorleri
SIMULASYONLA BIREBIR AYNI topic isimleri/tipleriyle yayinlanir
(/ultrasonic/fl|fr|rl|rr, sensor_msgs/Range). patrol.py'ye TEK satir
degisiklik gerekmez.

  /cmd_vel (geometry_msgs/Twist)  -> STM32'ye "V <lx_mm_s> <az_mrad_s>"
  STM32'den "S ..." satiri        -> /ultrasonic/{fl,fr,rl,rr} (Range)
                                   -> /imu/data_raw (Imu, sadece wz)
                                   -> /wheel/odom (Odometry, ACIK CEVRIM -
                                      bkz. asagidaki not)
                                   -> /battery_state (BatteryState, sadece
                                      ham voltaj - SoC/yuzde HESAPLANMAZ)
                                   -> e-stop/bumper/dusuk-batarya YUKSELEN
                                      kenarda mevcut (ZATEN TEST EDILMIS)
                                      'stop' komutunu /patrol_command'a
                                      yayinlar - patrol.py'de HICBIR yeni
                                      kod gerekmez.

2026-09-18: motor surucu DM556 (STEP/DIR) oldu - geri besleme HATTI YOK,
bu yuzden STM32'nin kendi komutladigi step sayisindan turettigi 'left_mm'/
'right_mm' ACIK CEVRIM'dir (gercek pozisyon degil tahmin, adim kaybi
sessizce sapmaya yol acar). Burada bu degerlerden standart diferansiyel
suruş odometri entegrasyonuyla (x,y,yaw) hesaplanip '/wheel/odom' olarak
yayinlanir - robot_localization (EKF) bunu LiDAR/AMCL ile surekli duzeltir
(motor surucu ekibinin notu). TF YAYINLANMAZ BURADA - odom->base_footprint
TF'i zaten EKF'nin kendisi yayinliyor (ekf.yaml publish_tf:true), ikisi
ayni TF'i yayinlarsa catisir.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Imu, BatteryState
from std_msgs.msg import String, Bool

try:
    import serial
except ImportError:
    serial = None  # dugum yine de acilir, seri baglanti olmadan uyarip bekler


SONAR_ORDER = ('fl', 'fr', 'rl', 'rr')
SONAR_MAX_RANGE = 4.0    # HC-SR04 datasheet (m) - Gazebo'daki 0.6 yapay sinir DEGIL
SONAR_MIN_RANGE = 0.02
SONAR_FOV = 0.35         # rad - cryvex.urdf.xacro'daki sonar aciklik degeriyle ayni

CMD_RESEND_PERIOD_S = 0.10   # STM32'nin 200ms watchdog'unu rahat besler
# Bu suredir YENI /cmd_vel gelmediyse STM32'ye 0 hiz gonderilir. Olmasa son
# komut SONSUZA KADAR tekrarlaniyordu: telefonla surerken WiFi koparsa robot
# son hizla gitmeye devam ederdi (STM32 watchdog'u da tetiklenmez, cunku biz
# beslemeye devam ediyoruz). Nav2/patrol hareket ederken 10-20 Hz yayinlar.
CMD_TIMEOUT_S = 0.5

WHEEL_BASE_M = 0.540   # app_config.h WHEEL_BASE_MM ile AYNI TUTULMALI (2026-10-02 olculdu: 54 cm)
# DONUS_KALIBRASYON 2026-10-03: lidar-ICP olcumu (4 yerinde donus, 0.4 ve 0.8 rad/s):
# robot komut edilen/odometrinin hesapladigi donusun ~%80'ini yapiyor (teker
# kaymasi, sarhos tekerler). Etkin iz genisligi 0.54/0.80 = 0.675 m. Odometri
# bunu kullanir; donus komutlari ANGULAR_CMD_GAIN ile buyutulur ki Nav2 ne
# istiyorsa robot gercekten o kadar donsun. Duz gidis olcumu: odometri = gercek.
ODOM_WHEEL_BASE_M = 0.549   # 2026-10-05 sol teker kamalandiktan sonra lidar ICP ile yeniden olculdu (eski 0.675, %23 az donus gosteriyordu)
ANGULAR_CMD_GAIN = 1.02    # 2026-10-05 yeniden olculdu (eski 1.25)
BATT_LOW_MV = 23500    # app_config.h BATT_LOW_MV ile AYNI TUTULMALI - 2026-10-02: 22000'den yukseltildi - DM860H en az 24 V ister; LiFePO4'te 23.5 V ~%5-8 kalan sarj
# Bunun altindaki okuma = batarya TAKILI DEGIL (tezgah testi: Pi/STM32 adaptorden
# besleniyor, bolucu girisi bos -> ADC 1-3 V arasi gurultu okur). 24 V LiFePO4
# paket 5 V'a hic inmez (BMS cok once keser). cafe_ui_server.py'deki
# "takili degil" esigiyle (volt < 5.0) AYNI. 2026-10-01'e kadar bu ayrim yoktu:
# bataryasiz tezgahta her acilista "BATARYA DUSUK" deyip devriyeyi durduruyordu.
BATT_PRESENT_MV = 5000


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value

        self.range_pubs = {
            k: self.create_publisher(Range, f'/ultrasonic/{k}', qos_profile_sensor_data)
            for k in SONAR_ORDER
        }
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.wheel_odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop_state', 10)
        self.bumper_pub = self.create_publisher(Bool, '/bumper_state', 10)
        # Kartin READY/INFO bilgisi (JSON: fw, reset, imu) - sonradan acilan
        # dugumler de (arayuz "Robot Sagligi") son degeri alsin: kalici (latched).
        self.info_pub = self.create_publisher(
            String, '/stm32_info', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.command_pub = self.create_publisher(String, '/patrol_command', 10)

        self._lx_mm_s = 0
        self._az_mrad_s = 0
        self._last_cmd_t = 0.0
        self._last_estop = False
        self._last_bumper = False
        self._last_batt_low = False
        self._batt_absent_logged = False
        self._ser = None
        self._stop_flag = False
        self._imu_ok = None   # kart READY/INFO'da bildirir; None = bilinmiyor (eski yazilim)
        self._info_received = False

        # Acik-cevrim teker odometrisi entegrasyon durumu (bkz. dosya basi notu).
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_left_mm = None
        self._last_right_mm = None
        self._last_odom_stamp = None
        self._last_real_odom_t = 0.0   # son GERCEK (STM32'den) odometri, monotonic

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(CMD_RESEND_PERIOD_S, self._resend_cmd)
        self.create_timer(0.05, self._odom_keepalive)   # 20 Hz, STM32'nin S satiri hiziyla ayni

        if serial is None:
            self.get_logger().error(
                "pyserial kurulu degil ('pip install pyserial' / 'apt install python3-serial') "
                "- STM32 koprusu calismayacak.")
            return

        # Port acilista yoksa ya da kablo cikip takilirsa kendiliginden
        # (yeniden) baglanir - eskiden sadece acilista bir kez deneniyordu,
        # kart sonradan takilinca servisi yeniden baslatmak gerekiyordu.
        self._port, self._baud = port, baud
        self._rx_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._rx_thread.start()

    def _serial_loop(self):
        warned = False
        while not self._stop_flag and rclpy.ok():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.2)
            except Exception as exc:  # noqa: BLE001
                if not warned:
                    self.get_logger().error(
                        f'Seri port acilamadi ({self._port} @ {self._baud}): {exc} - '
                        'STM32 takilinca otomatik baglanilacak.')
                    warned = True
                time.sleep(2.0)
                continue
            warned = False
            self._reset_odom_baseline()
            self._info_received = False
            self._ser = ser
            self.get_logger().info(f'STM32 baglandi: {self._port} @ {self._baud}')
            self._rx_loop(ser)         # kablo cikana kadar burada
            self._ser = None
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
            if not self._stop_flag:
                self.get_logger().warn('STM32 baglantisi koptu - yeniden baglanmaya calisiliyor.')
                time.sleep(1.0)

    def _reset_odom_baseline(self):
        # Kart yeniden baslayinca (guc, bekci, yukleme) teker sayaclari 0'dan
        # baslar - eski degerle fark alinirsa robot bir anda metrelerce geri
        # gitmis sanilir. Bir sonraki S satiri yeni baslangic olur.
        self._last_left_mm = None
        self._last_right_mm = None

    # ---- Pi5 -> STM32 ----
    def _cmd_vel_cb(self, msg: Twist):
        self._lx_mm_s = int(msg.linear.x * 1000.0)
        self._az_mrad_s = int(msg.angular.z * 1000.0)
        self._last_cmd_t = time.monotonic()
        self._send_velocity()

    def _resend_cmd(self):
        # STM32'nin 200ms watchdog'u surekli beslensin - Nav2/patrol.py her
        # dongude YENI bir /cmd_vel yayinlamayabilir (ozellikle dururken).
        if time.monotonic() - self._last_cmd_t > CMD_TIMEOUT_S:
            self._lx_mm_s = self._az_mrad_s = 0  # komut kesildi -> dur (bkz. CMD_TIMEOUT_S)
        self._send_velocity()

    def _send_velocity(self):
        if self._ser is None:
            return
        # YON_DUZELTME 2026-10-03: kart robotun on/arkasini ters taniyor (ileri
        # komutu robotu lidarin tersine suruyordu, donus dogruydu) -> sadece
        # ileri hizin isareti cevrilir.
        line = f'V {-self._lx_mm_s} {int(self._az_mrad_s * ANGULAR_CMD_GAIN)}\n'   # DONUS_KALIBRASYON
        try:
            self._ser.write(line.encode('ascii'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'STM32 yazma hatasi: {exc}')

    # ---- STM32 -> Pi5 ----
    def _rx_loop(self, ser):
        buf = b''
        info_tries, next_info = 0, 0.0
        while not self._stop_flag and rclpy.ok():
            # Kart bilgisini (surum / acilis nedeni / IMU) cevap gelene kadar
            # saniyede bir, en fazla 5 kez sor. Bastaki '\n': onceki oturumdan
            # kartta yarim kalmis bir satir varsa (ornegin "V 20") INFO onun
            # devamina eklenip anlasilmaz olmasin (2026-09-26'da boyle kayboldu).
            if not self._info_received and info_tries < 5 and time.monotonic() >= next_info:
                try:
                    ser.write(b'\nINFO\n')
                except Exception:  # noqa: BLE001
                    pass
                info_tries += 1
                next_info = time.monotonic() + 1.0
            try:
                chunk = ser.read(256)
            except Exception as exc:  # noqa: BLE001
                if not self._stop_flag:   # kapanirken port kapatildi - uyari degil
                    self.get_logger().warn(f'STM32 okuma hatasi: {exc}')
                return   # kablo cikti vb. - _serial_loop yeniden baglanir
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle_line(line.decode('ascii', errors='ignore').strip())

    def _handle_line(self, line):
        if not line:
            return
        if line.startswith('READY'):
            # "READY fw=1.1.0 reset=power imu=0" (eski yazilim: sadece "READY")
            info = dict(tok.split('=', 1) for tok in line.split()[1:] if '=' in tok)
            self._info_received = True
            if info.get('reset') and info.get('reset') != '?':
                self._reset_odom_baseline()   # kart yeniden basladi: sayaclar 0'dan
            if 'imu' in info:
                self._imu_ok = info['imu'] == '1'
            self.get_logger().info(
                f"STM32 hazir: yazilim {info.get('fw', '?')}, acilis nedeni {info.get('reset', '?')}, "
                f"IMU {'var' if self._imu_ok else 'YOK - /imu/data_raw yayinlanmiyor'}")
            self.info_pub.publish(String(data=json.dumps(
                {'fw': info.get('fw', '?'), 'reset': info.get('reset', '?'), 'imu': self._imu_ok})))
            if info.get('reset') == 'iwdg':
                self.get_logger().error(
                    'STM32 TAKILMIS ve donanim bekcisi (IWDG) yeniden baslatmis - kart yazilimi '
                    'incelenmeli (motorlar o anda durdu).')
            return
        if line.startswith('ERR'):
            self.get_logger().warn(f'STM32 hata bildirdi: {line}')
            return
        if not line.startswith('S '):
            return
        parts = line[2:].split()
        if len(parts) != 10:
            return  # bozuk/yarim satir - sessizce atla, bir sonraki gelir
        try:
            (fl, fr, rl, rr, imu_wz, estop, bumper,
             left_mm, right_mm, batt_mv) = (int(p) for p in parts)
        except ValueError:
            return

        # YON_DUZELTME: kartin sol/sag sayaclari robot 180 derece ters tanindigi
        # icin gercek tekerin tersi -> yer degistir + isaret cevir (yaw ayni kalir,
        # ileri/geri duzelir).
        left_mm, right_mm = -right_mm, -left_mm

        now = self.get_clock().now().to_msg()
        for key, mm in zip(SONAR_ORDER, (fl, fr, rl, rr)):
            self._publish_range(key, mm, now)

        # IMU takili degilse kart 0 gonderir; bunu yayinlamak EKF'ye "robot hic
        # donmuyor" dedirtir (tekerlekler donerken bile). Kart READY/INFO'da
        # imu=0 derse yayinlama. (Eski yazilim bilgi vermez: None -> yayinla.)
        if self._imu_ok is not False:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.angular_velocity.z = imu_wz / 1000.0  # mrad/s -> rad/s
            imu_msg.orientation_covariance[0] = -1.0       # oryantasyon yok (REP-145)
            imu_msg.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu_msg)

        self._publish_wheel_odom(left_mm, right_mm, now)
        self._publish_battery(batt_mv, now)

        estop_now, bumper_now = bool(estop), bool(bumper)
        self.estop_pub.publish(Bool(data=estop_now))
        self.bumper_pub.publish(Bool(data=bumper_now))
        # batt_mv < BATT_PRESENT_MV = batarya takili degil / ADC okunamadi: "dusuk" SAYMA
        batt_low_now = BATT_PRESENT_MV <= batt_mv < BATT_LOW_MV
        if batt_mv < BATT_PRESENT_MV and not self._batt_absent_logged:
            self.get_logger().info(
                f'Batarya takili degil ({batt_mv} mV) - dusuk batarya korumasi devre disi '
                '(tezgah/adaptor ile calisma).')
            self._batt_absent_logged = True
        elif batt_mv >= BATT_PRESENT_MV:
            self._batt_absent_logged = False
        if (estop_now and not self._last_estop) or (bumper_now and not self._last_bumper):
            # YUKSELEN KENAR - zaten test edilmis 'stop' komutunu kullan,
            # patrol.py'de YENI hicbir kod gerekmiyor.
            self.get_logger().warn(
                f'DONANIMSAL DURDURMA algilandi (estop={estop_now} bumper={bumper_now}) '
                '-> devriye durduruluyor.')
            cmd = String()
            cmd.data = 'stop'
            self.command_pub.publish(cmd)
        if batt_low_now and not self._last_batt_low:
            # Motor surucu ekibinin notu: "22V altina inince şarja don" -
            # bu sadece bir ESIK UYARISI (SoC/yuzde takibi/otomatik sarja
            # DONUS DEGIL, o kapsam disi - bkz. malzeme_listesi_rev5.pdf).
            # AYNI test edilmis 'stop' komutu, yeni kod gerekmiyor.
            self.get_logger().warn(
                f'BATARYA DUSUK ({batt_mv} mV < {BATT_LOW_MV} mV) -> devriye durduruluyor, sarj gerekiyor.')
            cmd = String()
            cmd.data = 'stop'
            self.command_pub.publish(cmd)
        self._last_estop, self._last_bumper, self._last_batt_low = estop_now, bumper_now, batt_low_now

    def _publish_wheel_odom(self, left_mm, right_mm, stamp):
        # Ilk satir: baslangic referansi olarak al, entegrasyona baslama
        # (yoksa ilk "delta" STM32 acilistan bu yana biriken TUM mesafe olur).
        if self._last_left_mm is None:
            self._last_left_mm = left_mm
            self._last_right_mm = right_mm
            self._last_odom_stamp = stamp
            return

        dt = (stamp.sec + stamp.nanosec * 1e-9) - \
             (self._last_odom_stamp.sec + self._last_odom_stamp.nanosec * 1e-9)
        d_left = (left_mm - self._last_left_mm) / 1000.0   # mm -> m
        d_right = (right_mm - self._last_right_mm) / 1000.0
        # 2026-10-03: hareket halinde tek bir S satiri bozuk gelebiliyor (sayac
        # ~0.8 m sicrayip sonraki satirda geri donuyor) -> EKF/costmap sicriyor,
        # DWB "Trajectory Goes Off Grid" verip robot yerinde donuyordu. Fiziksel
        # olarak imkansiz adimi (0.6 m/s ustu) yok say; taban degismez, sonraki
        # saglam satir dogru farki verir. Ust uste 5 bozuk = sayac gercekten
        # degismis (kart reset) -> yeni taban al.
        max_step = 0.6 * max(dt, 0.05) + 0.05
        if abs(d_left) > max_step or abs(d_right) > max_step:
            self._odom_bad = getattr(self, '_odom_bad', 0) + 1
            self._odom_bad_total = getattr(self, '_odom_bad_total', 0) + 1
            if self._odom_bad_total <= 5 or self._odom_bad_total % 50 == 0:
                self.get_logger().warn(
                    f'Teker odometrisinde imkansiz sicrama yok sayildi (sol {d_left:+.3f} m, '
                    f'sag {d_right:+.3f} m, dt {dt:.2f} s) - toplam {self._odom_bad_total}')
            if self._odom_bad >= 5:
                self._last_left_mm, self._last_right_mm, self._last_odom_stamp = left_mm, right_mm, stamp
                self._odom_bad = 0
            return
        self._odom_bad = 0
        self._last_left_mm, self._last_right_mm, self._last_odom_stamp = left_mm, right_mm, stamp

        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / ODOM_WHEEL_BASE_M   # DONUS_KALIBRASYON (eskiden WHEEL_BASE_M)
        # Orta-nokta entegrasyonu: donus/ilerleme ayni anda oluyormus gibi
        # kabul edip aci degisiminin YARISINDAKI yonu kullanmak, tek adimda
        # once-don-sonra-ilerle varsayimindan daha az sapma biriktirir.
        mid_yaw = self._odom_yaw + d_yaw / 2.0
        self._odom_x += d_center * math.cos(mid_yaw)
        self._odom_y += d_center * math.sin(mid_yaw)
        self._odom_yaw = math.atan2(math.sin(self._odom_yaw + d_yaw), math.cos(self._odom_yaw + d_yaw))

        # YON_DUZELTME: S satirlari Pi yogunken toplu gelir, dt ~0 olur ve hiz
        # sahte olarak dev cikar (EKF robotu metrelerce isinliyordu). Kart 20 Hz
        # yollar -> dt'yi en az 45 ms say.
        # HIZ_PENCERE 2026-10-03: tek aralikla hesaplanan hiz, satirlarin duzensiz
        # gelmesi yuzunden her 4-5 olcumde bir sahte olarak ~0'a dusuyordu
        # (0.27 -> 0.05 m/s). Hiz son ~0.15 sn'deki toplam yoldan hesaplanir.
        tnow = stamp.sec + stamp.nanosec * 1e-9
        hist = getattr(self, '_vel_hist', None)
        if hist is None:
            hist = self._vel_hist = []
        self._cum_c = getattr(self, '_cum_c', 0.0) + d_center
        self._cum_y = getattr(self, '_cum_y', 0.0) + d_yaw
        hist.append((tnow, self._cum_c, self._cum_y))
        while len(hist) > 2 and tnow - hist[1][0] >= 0.15:
            hist.pop(0)
        t_old, c_old, y_old = hist[0]
        span = tnow - t_old
        if span >= 0.04:
            vx = (self._cum_c - c_old) / span
            vyaw = (self._cum_y - y_old) / span
        else:
            dt_v = max(dt, 0.045)
            vx = d_center / dt_v
            vyaw = d_yaw / dt_v
        self._emit_odom(stamp, vx, vyaw)
        self._last_real_odom_t = time.monotonic()

    def _emit_odom(self, stamp, vx, vyaw):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        half = self._odom_yaw / 2.0
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vyaw
        self.wheel_odom_pub.publish(msg)

    def _odom_keepalive(self):
        # STM32 bagli degil / sessizken motorlar zaten donemez: robot DURUYOR.
        # Son konumu sifir hizla yayinlamaya devam et ki EKF odom->base_footprint
        # TF'ini hep yayinlasin. Boylece launch dosyalarinda "sahte (sabit) odom"
        # gerekmez ve kart takilip cikarilinca hicbir ayar degismez - eskiden
        # STM32 baglaninca sabit TF ile EKF catisiyordu (elle kapatmak gerekiyordu).
        if time.monotonic() - self._last_real_odom_t > 0.3:
            self._emit_odom(self.get_clock().now().to_msg(), 0.0, 0.0)

    def _publish_battery(self, batt_mv, stamp):
        # Durum ekrani/loglama icin - SoC/yuzde HESAPLANMAZ (o kapsam disi,
        # bkz. app_config.h BATT_LOW_MV yorumu), sadece ham voltaj yayinlanir.
        msg = BatteryState()
        msg.header.stamp = stamp
        if batt_mv > 0:
            msg.voltage = batt_mv / 1000.0
            msg.present = True
        else:
            msg.voltage = float('nan')  # ADC okunamadi (bkz. battery.c)
            msg.present = False
        msg.percentage = float('nan')  # kasten hesaplanmiyor
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_DEAD if 0 < batt_mv < BATT_LOW_MV
            else BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        self.battery_pub.publish(msg)

    def _publish_range(self, key, mm, stamp):
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = f'sonar_{key}_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = SONAR_FOV
        msg.min_range = SONAR_MIN_RANGE
        msg.max_range = SONAR_MAX_RANGE
        # 9999 = STM32'nin "menzil disi/timeout" isareti (bkz. stm32_protokol.md)
        msg.range = SONAR_MAX_RANGE if mm >= 9999 else mm / 1000.0
        self.range_pubs[key].publish(msg)

    def destroy_node(self):
        self._stop_flag = True
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():   # Ctrl+C/launch SIGINT'i ROS'u zaten kapatmis olabilir
            rclpy.shutdown()


if __name__ == '__main__':
    main()
