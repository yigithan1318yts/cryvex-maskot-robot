import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../api/discovery.dart';
import '../api/robot_api.dart';
import '../notify.dart';

/// "1× Latte, 2× Kurabiye"
String orderItemsText(Map<String, dynamic> order) => [
      for (final i in (order['items'] as List? ?? [])) '${i['qty']}× ${i['name']}',
    ].join(', ');

/// 125.0 -> "125", 12.5 -> "12.5"
String formatPrice(dynamic v) {
  final n = (v is num) ? v : num.tryParse('$v') ?? 0;
  return n == n.roundToDouble() ? n.round().toString() : n.toStringAsFixed(1);
}

/// Robotun "Devam" beklediği durumlar (patrol.py) -> telefonda kocaman buton.
const waitStates = {
  'at_barista': ('🔔 Sipariş barmende', 'Robot siparişi barmene iletti. Alınca devam ettirin.',
      '👍 Siparişi Aldım · Devriyeye Devam Et'),
  'pickup_wait': ('📦 Robot barmende bekliyor', 'Hazır siparişi robota yerleştirin.',
      '✅ Yüklendi · Masaya Götür'),
  'served_wait': ('🍽️ Robot masada bekliyor', 'Müşteri siparişini alınca devam ettirin.',
      '👍 Teslim Edildi · Devriyeye Devam Et'),
};

/// Uygulama genelinde paylaşılan bağlantı + robot durumu. index.html'deki
/// pollStatus()'un Flutter karşılığı - /api/status'u periyodik okur, /patrol_status
/// JSON'unu (data.status alani, string olarak geliyor) parse eder.
class RobotState extends ChangeNotifier {
  RobotApi? api;
  String? ip;
  bool connected = false;
  int statusCount = 0;
  double statusAge = -1;

  // patrol.py /patrol_status alanlari (index.html'deki 'info' nesnesiyle ayni)
  String state = 'idle';
  String waypoint = '';
  bool mapReady = true;
  bool rescueActive = false;
  bool screenOn = false;
  bool configured = false;
  String mode = 'unconfigured';

  /// "Robot Sağlığı" (sunucunun health_payload'u): anahtar -> {ok: bool|null, text}
  Map<String, dynamic> health = {};

  // ---- siparişler ----
  List<Map<String, dynamic>> orders = [];
  double serverClockOffset = 0;      // robot saati - telefon saati (sn): "x dk önce" robotun saatiyle
  String deliveryTable = '';         // robotun şu an götürdüğü siparişin masası
  int _ordersSeq = -1;
  Set<int>? _knownOrderIds;          // null = ilk yükleme (eski siparişler için bildirim yok)
  String _prevState = '';

  bool get waitingForContinue => waitStates.containsKey(state);

  /// Dev Ekran Modunda (müşteriye dönük tablet) sipariş/robot bildirimleri kapalı.
  bool notificationsEnabled = true;

  Timer? _pollTimer;
  bool searching = false;   // robot agda araniyor (IP degismis olabilir)
  int _failStreak = 0;

  bool get isLive => connected && statusCount > 0 && statusAge >= 0 && statusAge < 4.0;

  Future<void> loadSavedIp() async {
    final prefs = await SharedPreferences.getInstance();
    ip = prefs.getString('robot_ip');
    if (ip != null && ip!.isNotEmpty) {
      api = RobotApi(baseUrl: 'http://$ip:8080');
    }
    notifyListeners();
  }

  Future<void> setIp(String newIp) async {
    ip = newIp.trim();
    api = RobotApi(baseUrl: 'http://$ip:8080');
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('robot_ip', ip!);
    notifyListeners();
  }

  /// Robotu agda bulur (bkz. discovery.dart) ve adresini kaydeder. Modem
  /// robota yeni IP verdiyse ya da baska bir kafenin agindaysa kullanici
  /// hicbir sey girmeden baglanti kendiliginden duzelir.
  Future<bool> findRobot() async {
    if (searching) return false;
    searching = true;
    notifyListeners();
    try {
      final found = await discoverRobot();
      if (found == null) return false;
      if (found != ip) await setIp(found);
      _failStreak = 0;
      return true;
    } finally {
      searching = false;
      notifyListeners();
    }
  }

  // Birden fazla ekran sorgulama isteyebilir (ör. Ana Panel -> Dev Ekran geçişinde
  // eski ekranın dispose'u yenisinin sorgusunu durdurmasın): kullanıcı sayacı.
  int _pollUsers = 0;

  void startPolling() {
    _pollUsers++;
    if (_pollTimer != null) return;
    _poll();
    _pollTimer = Timer.periodic(const Duration(milliseconds: 1200), (_) => _poll());
  }

  void stopPolling() {
    if (_pollUsers > 0) _pollUsers--;
    if (_pollUsers > 0) return;
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  Future<void> _poll() async {
    if (api == null) return;
    try {
      final data = await api!.status();
      connected = true;
      statusCount = (data['count'] ?? 0) as int;
      statusAge = ((data['age'] ?? -1) as num).toDouble();
      final h = data['health'];
      health = h is Map<String, dynamic> ? h : {};
      final raw = data['status'];
      if (raw is String && raw.isNotEmpty) {
        try {
          final info = jsonDecode(raw) as Map<String, dynamic>;
          state = (info['state'] ?? state) as String;
          waypoint = (info['waypoint'] ?? '') as String;
          mapReady = (info['map_ready'] ?? true) as bool;
          rescueActive = (info['rescue_active'] ?? false) as bool;
          screenOn = (info['screen_on'] ?? false) as bool;
          deliveryTable = (info['delivery_table'] ?? '') as String;        } catch (_) {
          // henuz patrol.py'den veri gelmemis olabilir - sessiz gec
        }
      }
      _notifyWaitState();
      final seq = data['orders_seq'];
      if (seq is int && seq != _ordersSeq) {
        _ordersSeq = seq;
        await refreshOrders();
      }
      final modeData = await api!.mode();
      configured = (modeData['configured'] ?? false) as bool;
      mode = (modeData['mode'] ?? 'unconfigured') as String;
      _failStreak = 0;
    } catch (_) {
      connected = false;
      // ~4 sn ust uste cevap yok: robotun IP'si degismis olabilir -> yeniden ara
      // (bulunamazsa ~12 sn'de bir tekrar; telefonu surekli taramayla yormasin).
      if (++_failStreak % 10 == 3 && !searching) unawaited(findRobot());
    }
    notifyListeners();
  }

  /// Sipariş listesini çeker; yeni gelen siparişler için bildirim gösterir.
  Future<void> refreshOrders() async {
    if (api == null) return;
    try {
      final data = await api!.orders();
      orders = [for (final o in (data['orders'] as List? ?? [])) Map<String, dynamic>.from(o as Map)];
      serverClockOffset = ((data['now'] ?? 0) as num).toDouble() - DateTime.now().millisecondsSinceEpoch / 1000;
    } catch (_) {
      return;
    }
    final ids = {for (final o in orders) o['id'] as int};
    final known = _knownOrderIds;
    if (known != null && notificationsEnabled) {
      for (final o in orders) {
        if (!known.contains(o['id']) && o['status'] == 'preparing') {
          Notifier.show(1000 + (o['id'] as int), '🧾 Yeni sipariş · ${o['table']}',
              '${orderItemsText(o)} – ${formatPrice(o['total'])} ₺');
        }
      }
    }
    _knownOrderIds = {...?known, ...ids};
    notifyListeners();
  }

  // Robot barmende/masada "Devam" beklemeye başlayınca cebindeki garsona haber ver.
  void _notifyWaitState() {
    if (state != _prevState && waitStates.containsKey(state) && _prevState.isNotEmpty && notificationsEnabled) {
      final (title, body, _) = waitStates[state]!;
      final table = deliveryTable.isNotEmpty ? ' · $deliveryTable' : '';
      Notifier.show(1, '$title$table', body);
    }
    _prevState = state;
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    super.dispose();
  }
}
