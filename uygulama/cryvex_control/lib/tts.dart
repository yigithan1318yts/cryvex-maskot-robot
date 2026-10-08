import 'api/robot_api.dart';

/// Robotu konuşturur: ses telefonda DEĞİL robotun hoparlöründen çıkar, robotun
/// ekranında ağız oynar ve [expr] (happy/love/alert/sad) verilirse ifadesi değişir.
/// Sunucuya ulaşılamazsa SESSİZCE geçer, hiçbir buton hata göstermez.
Future<void> speak(RobotApi api, String text, {String expr = ''}) async {
  if (text.isEmpty) return;
  try {
    await api.speakHere(text, expr: expr);
  } catch (_) {
    // sessiz geç
  }
}
