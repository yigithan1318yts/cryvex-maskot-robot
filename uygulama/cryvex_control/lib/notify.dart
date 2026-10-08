import 'package:flutter/foundation.dart';
import 'package:flutter_local_notifications/flutter_local_notifications.dart';

/// Garsonun telefonuna bildirim: yeni sipariş, robot barmende/masada "Devam"
/// bekliyor. Uygulama açıkken (ya da arka plandayken hâlâ çalışıyorsa) robot
/// durumunu 1,2 sn'de bir sorguluyor; bu sınıf olayları sistem bildirimine çevirir.
class Notifier {
  Notifier._();
  static final _plugin = FlutterLocalNotificationsPlugin();
  static bool _ready = false;

  static const _details = NotificationDetails(
    android: AndroidNotificationDetails(
      'cryvex_orders',
      'Siparişler ve robot',
      channelDescription: 'Yeni sipariş ve robotun onay beklediği anlar',
      importance: Importance.max,
      priority: Priority.high,
      playSound: true,
      enableVibration: true,
    ),
  );

  static Future<void> init() async {
    try {
      await _plugin.initialize(
        settings: const InitializationSettings(android: AndroidInitializationSettings('@mipmap/ic_launcher')),
      );
      await _plugin
          .resolvePlatformSpecificImplementation<AndroidFlutterLocalNotificationsPlugin>()
          ?.requestNotificationsPermission();
      _ready = true;
    } catch (e) {
      debugPrint('Bildirimler açılamadı: $e');   // bildirim olmadan da uygulama çalışır
    }
  }

  static Future<void> show(int id, String title, String body) async {
    if (!_ready) return;
    try {
      await _plugin.show(id: id, title: title, body: body, notificationDetails: _details);
    } catch (e) {
      debugPrint('Bildirim gösterilemedi: $e');
    }
  }
}
