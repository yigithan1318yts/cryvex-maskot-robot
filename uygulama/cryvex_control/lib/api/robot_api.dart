import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:http/http.dart' as http;

/// tablet_server.py'nin REST API'sine ince bir istemci. Backend'de HİÇBİR
/// değişiklik yok - web arayüzünün (index.html) kullandığı UÇLARIN AYNISINI
/// çağırıyoruz, sadece WiFi üzerinden farklı bir cihazdan (telefon).
class RobotApi {
  RobotApi({required this.baseUrl});

  /// Örn: "http://192.168.1.9:8080"
  String baseUrl;

  Uri _u(String path) => Uri.parse('$baseUrl$path');

  Future<Map<String, dynamic>> _postJson(String path,
      [Map<String, dynamic>? body, Duration timeout = const Duration(seconds: 6)]) async {
    try {
      final res = await http
          .post(_u(path),
              headers: {'Content-Type': 'application/json'},
              body: body != null ? jsonEncode(body) : null)
          .timeout(timeout);
      if (res.body.isEmpty) return {'result': 'ok'};
      return jsonDecode(res.body) as Map<String, dynamic>;
    } catch (e) {
      return {'result': 'error', 'reason': e.toString()};
    }
  }

  // start_mapping/finish_mapping/cancel_mapping tablet_server.py tarafinda
  // Nav2/SLAM sureclerini SIGINT ile durdurup (bkz. LaunchManager._stop_locked,
  // 8sn + 3sn'ye kadar bekleyebilir) yeniden baslatiyor; finish_mapping ayrica
  // map_saver_cli'yi de bekliyor (15sn'ye kadar). Kisa bir timeout (6sn) bu
  // isteklerin GERCEKTEN calisirken zaman asimina ugrayip hatali "basarisiz"
  // gostermesine yol aciyordu ("ekran acilmiyor" bug'i) - bu ucler icin cok
  // daha uzun bir sure taniyoruz.
  static const _longOp = Duration(seconds: 30);

  Future<Map<String, dynamic>> _getJson(String path) async {
    final res = await http.get(_u(path)).timeout(const Duration(seconds: 6));
    return jsonDecode(res.body) as Map<String, dynamic>;
  }

  // ---- durum ----
  Future<Map<String, dynamic>> status() => _getJson('/api/status');
  Future<Map<String, dynamic>> mode() => _getJson('/api/mode');
  Future<Map<String, dynamic>> waypoints() => _getJson('/api/waypoints');

  /// Canlı/harita PNG'sinin URL'i - Image.network ile dogrudan kullanilir.
  /// Onbellegi asmak icin her seferinde farkli bir cache-bust query eklenir.
  String liveMapUrl() => '$baseUrl/api/live_map.png?_=${DateTime.now().millisecondsSinceEpoch}';
  String mapUrl() => '$baseUrl/api/map.png?_=${DateTime.now().millisecondsSinceEpoch}';

  // ---- devriye kontrolleri ----
  Future<void> startPatrol() => _postJson('/api/start_patrol');
  Future<void> stopPatrol() => _postJson('/api/stop_patrol');
  Future<void> wander() => _postJson('/api/wander');
  Future<void> goHome() => _postJson('/api/go_home');
  Future<void> greetDoor() => _postJson('/api/greet_door');
  Future<void> wake() => _postJson('/api/wake');
  Future<void> sleep() => _postJson('/api/sleep');

  /// action: welcome | menu | welcome_menu | cute (varsayilan welcome_menu)
  Future<Map<String, dynamic>> goto(int tableNumber, {String action = 'welcome_menu'}) =>
      _postJson('/api/goto', {'table': tableNumber, 'action': action});

  // ---- serbest sürüş (harita/kontrol modlarinda ortak) ----
  Future<void> teleop(double lx, double az) => _postJson('/api/teleop', {'lx': lx, 'az': az});
  Future<void> teleopStop() => _postJson('/api/teleop_stop');

  // ---- kurtarma ----
  Future<Map<String, dynamic>> rescueStart(String password) =>
      _postJson('/api/rescue_start', {'password': password});
  Future<void> rescueTeleop(double lx, double az) =>
      _postJson('/api/rescue_teleop', {'lx': lx, 'az': az});
  Future<void> rescueStop() => _postJson('/api/rescue_stop');

  // ---- haritalama (uzun zaman asimi - yukarida _longOp aciklamasina bakin) ----
  Future<Map<String, dynamic>> startMapping(String password) =>
      _postJson('/api/start_mapping', {'password': password}, _longOp);
  Future<Map<String, dynamic>> finishMapping(String password) =>
      _postJson('/api/finish_mapping', {'password': password}, _longOp);
  Future<Map<String, dynamic>> cancelMapping(String password) =>
      _postJson('/api/cancel_mapping', {'password': password}, _longOp);

  // ---- otonom sürüş (~/cryvex_araclar/otonom_gezgin.py, bkz. cafe_ui_server /api/otonom) ----
  /// {calisiyor: bool, log: [son satirlar]}
  Future<Map<String, dynamic>> otonomStatus() => _getJson('/api/otonom');

  /// mod: kesif | devriye | dur. kesif+yeni = bulundugu yerde haritayi sifirdan
  /// cikarir. Sunucu once eski sureci kapatir (25 sn'ye kadar) -> uzun zaman asimi.
  Future<Map<String, dynamic>> otonom(String mod, String password, {bool yeni = false}) =>
      _postJson('/api/otonom', {'password': password, 'mod': mod, 'yeni': yeni}, _longOp);

  /// Kamera + YOLO algilamanin son karesi (algilama.py, ayri sunucu :8081).
  /// Kamera/algilama calismiyorsa istek basarisiz olur ya da 503 doner.
  String cameraFrameUrl() {
    final u = Uri.parse(baseUrl);
    return u.replace(port: 8081, path: '/kare.jpg',
        query: '_=${DateTime.now().millisecondsSinceEpoch}').toString();
  }

  /// "Robot Burada" (2D pose estimate). yaw null = robotun bildigi son yon korunur.
  Future<Map<String, dynamic>> setPose(double x, double y, [double? yaw]) =>
      _postJson('/api/set_pose', {'x': x, 'y': y, 'yaw': ?yaw});

  // ---- harita kurulumu (uygulamanin kendi ekrani: kurulum_screen.dart) ----
  /// {resolution, origin: [x, y, theta], width, height, image}; width 0 = kayitli harita yok.
  Future<Map<String, dynamic>> mapInfo() => _getJson('/api/map_info');

  /// Kayitli haritanin PNG'si (satir 0 = ust; dunya y'si yukari).
  Future<Uint8List?> mapPng() async {
    try {
      final res = await http.get(Uri.parse(mapUrl())).timeout(const Duration(seconds: 10));
      return res.statusCode == 200 ? res.bodyBytes : null;
    } catch (_) {
      return null;
    }
  }

  /// {ok: bool, x, y, yaw, mode} - ok=false: konum sistemi kapali.
  Future<Map<String, dynamic>> robotPose() => _getJson('/api/robot_pose');

  /// {"barista": {...}|null, "door": {...}|null, "tables": [{isim, x, y, yaw}, ...]}
  Future<Map<String, dynamic>> saveWaypoints(Map<String, dynamic> cfg) => _postJson('/api/waypoints', cfg);

  /// Firca: [{"v": "free"|"occ", "r": piksel, "pts": [[fx, fy], ...]}]. Sunucu
  /// eski haritayi yedekler ve Nav2'yi yeniden baslatir -> uzun zaman asimi.
  Future<Map<String, dynamic>> mapEdit(List<Map<String, dynamic>> strokes) =>
      _postJson('/api/map_edit', {'strokes': strokes}, _longOp);

  // ---- hoparlör sesi (0-100, robotun kendi hoparlörü - JBL vb.) ----
  Future<int> getVolume() async {
    try {
      final data = await _getJson('/api/volume');
      final v = (data['volume'] as num?)?.toInt() ?? -1;
      return v;
    } catch (_) {
      return -1;
    }
  }

  Future<void> setVolume(int pct) => _postJson('/api/volume', {'volume': pct});

  // ---- siparişler (robot ekranından verilir) ----
  Future<Map<String, dynamic>> orders() => _getJson('/api/orders');
  /// Barmen: "Hazır – Robot Götürsün" -> robot barmenden alıp siparişin masasına götürür.
  Future<Map<String, dynamic>> orderReady(int id) => _postJson('/api/order_ready', {'id': id});
  Future<Map<String, dynamic>> orderCancel(int id) => _postJson('/api/order_cancel', {'id': id});
  /// Barmende / masada bekleyen robotun kocaman "Devam Et"i.
  Future<Map<String, dynamic>> continueRobot() => _postJson('/api/continue', {});

  // ---- robot yüzünün renkleri: eye / mouth / light (yürüyen ışık) / face ----
  Future<Map<String, String>?> getTheme() async {
    try {
      final data = await _getJson('/api/theme');
      return data.map((k, v) => MapEntry(k, v.toString()));
    } catch (_) {
      return null;
    }
  }

  /// changes: {'eye': '#ff3355'} gibi; {'reset': true} = varsayılan renkler.
  Future<Map<String, dynamic>> setTheme(Map<String, dynamic> changes) => _postJson('/api/theme', changes);

  /// Robotun gerçek sesi (Piper/edge-tts) - text -> ses bayt dizisi ya da
  /// null (sunucu üretemedi, ör. sessizce geç).
  Future<Uint8List?> fetchTtsAudio(String text) async {
    try {
      final res = await http
          .get(_u('/api/tts_audio?text=${Uri.encodeComponent(text)}'))
          .timeout(const Duration(seconds: 6));
      if (res.statusCode != 200) return null;
      return res.bodyBytes;
    } catch (_) {
      return null;
    }
  }

  /// Robotun KENDİ ekranı konuşur (ses robotun hoparlöründen, ağız oynar);
  /// [expr] verilirse (happy/love/alert/sad) robotun yüz ifadesi de değişir.
  Future<void> speakHere(String text, {String expr = ''}) =>
      _postJson('/api/speak_here', {'text': text, 'expr': expr});
}
