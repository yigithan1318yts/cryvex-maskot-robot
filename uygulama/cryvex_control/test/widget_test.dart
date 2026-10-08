// Cryvex Kontrol - başlangıç smoke test'i. Ağ/robot bağlantısı gerektiren
// asıl ekranlar (Dashboard/Joystick) manuel cihaz testiyle doğrulanıyor;
// burada uygulamanın çökmeden Bağlantı ekranını açtığını doğrularız.
//
// NOT: Bağlantı ekranı açılır açılmaz robotu ağda aramaya başlar (UDP yayını +
// HTTP taraması, bkz. lib/api/discovery.dart). Test ortamında bu gerçek ağ
// işlemleri tamamlanmaz, "aranıyor" çarkı dönmeye devam eder - bu yüzden
// pumpAndSettle() (animasyonların bitmesini bekler) burada HİÇ bitmez.
// Birkaç kare ilerletip ekranı doğrulamak yeterli.
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'package:cryvex_control/main.dart';

void main() {
  testWidgets('Uygulama açılır, Bağlantı ekranı robotu aramaya başlar', (WidgetTester tester) async {
    SharedPreferences.setMockInitialValues({});   // kayıtlı IP yok -> Bağlantı ekranı
    await tester.pumpWidget(const CryvexApp());
    await tester.pump();                                      // _Bootstrap: kayıtlı ayarları okur
    await tester.pump(const Duration(milliseconds: 100));     // Bağlantı ekranı + otomatik arama

    expect(find.textContaining('Bağlantı', findRichText: true), findsOneWidget);
    expect(find.text('Robot aranıyor…'), findsOneWidget);     // otomatik arama başladı
    expect(find.text('Bu adrese bağlan'), findsOneWidget);    // elle IP yedeği görünüyor

    // Ağaç kapatılınca bekleyen zamanlayıcılar (arama zaman aşımları) temizlensin.
    await tester.pumpWidget(const SizedBox.shrink());
    await tester.pump(const Duration(seconds: 10));
  });
}
