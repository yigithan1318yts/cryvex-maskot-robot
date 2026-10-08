import 'dart:convert';
import 'package:cryvex_control/api/robot_api.dart';
import 'package:cryvex_control/screens/kurulum_screen.dart';
import 'package:cryvex_control/state/robot_state.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';
import 'package:provider/provider.dart';

// 1x1 PNG - harita resmi yerine (Image.memory ekranı doldurmak için uzatır).
final _png = base64Decode(
    'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg==');

/// Sahte robot: 20x10 piksel, 0,1 m/piksel, orijin (-1, -0,5) -> harita
/// dünyada x -1..1, y -0,5..0,5; tam ortası (0, 0).
class _SahteRobot {
  final posts = <String, Map<String, dynamic>>{};

  MockClient client() => MockClient((req) async {
        final p = req.url.path;
        if (req.method == 'POST') {
          posts[p] = jsonDecode(req.body) as Map<String, dynamic>;
          return http.Response('{"result":"ok"}', 200);
        }
        return switch (p) {
          '/api/map_info' => http.Response(
              '{"resolution":0.1,"origin":[-1.0,-0.5,0.0],"width":20,"height":10,"image":"cafe_map.pgm"}', 200),
          '/api/map.png' => http.Response.bytes(_png, 200),
          '/api/waypoints' => http.Response(
              '{"barista":null,"door":null,"tables":[{"isim":"Masa 1","x":0.5,"y":0.2,"yaw":0.0}]}', 200),
          '/api/robot_pose' => http.Response('{"ok":true,"x":0.0,"y":0.0,"yaw":1.57,"mode":"operating"}', 200),
          _ => http.Response('{}', 404),
        };
      });
}

Future<void> _ac(WidgetTester tester) async {
  tester.view.physicalSize = const Size(1000, 1600);
  tester.view.devicePixelRatio = 1.0;
  final st = RobotState()..api = RobotApi(baseUrl: 'http://robot:8080');
  await tester.pumpWidget(ChangeNotifierProvider.value(
    value: st,
    child: const MaterialApp(home: KurulumScreen()),
  ));
  for (var i = 0; i < 5; i++) {
    await tester.pump(const Duration(milliseconds: 100));
  }
}

Future<void> _kapat(WidgetTester tester) async {
  await tester.pumpWidget(const SizedBox());   // zamanlayıcı dispose'da durur
  tester.view.resetPhysicalSize();
  tester.view.resetDevicePixelRatio();
}

void main() {
  testWidgets('Masa ekle: 1. dokunuş yer, 2. dokunuş yön; kaydedince doğru koordinat gider', (tester) async {
    final robot = _SahteRobot();
    await http.runWithClient(() async {
      await _ac(tester);
      expect(find.text('Masa 1'), findsOneWidget);

      await tester.tap(find.text('🍽️ Masa'));
      await tester.pump();
      final harita = tester.getCenter(find.byType(Image));
      await tester.tapAt(harita);                          // dünya (0, 0)
      await tester.pump(const Duration(milliseconds: 400));
      await tester.tapAt(harita + const Offset(0, -60));   // yukarı = +y yönü (90°)
      await tester.pump(const Duration(milliseconds: 400));
      expect(find.text('Masa 2'), findsOneWidget);

      await tester.tap(find.text('Kaydet'));
      await tester.pump(const Duration(milliseconds: 200));
      final masalar = robot.posts['/api/waypoints']!['tables'] as List;
      expect(masalar, hasLength(2));
      final yeni = masalar[1] as Map;
      expect(yeni['isim'], 'Masa 2');
      expect((yeni['x'] as num).toDouble(), closeTo(0.0, 0.05));
      expect((yeni['y'] as num).toDouble(), closeTo(0.0, 0.05));
      expect((yeni['yaw'] as num).toDouble(), closeTo(1.5708, 0.01));
      await _kapat(tester);
    }, robot.client);
  });

  testWidgets('Robot Burada tek dokunuşta yön göndermez', (tester) async {
    final robot = _SahteRobot();
    await http.runWithClient(() async {
      await _ac(tester);
      await tester.tap(find.text('🤖 Robot Burada'));
      await tester.pump();
      await tester.tapAt(tester.getCenter(find.byType(Image)) + const Offset(-100, 0));
      await tester.pump(const Duration(milliseconds: 400));
      final body = robot.posts['/api/set_pose']!;
      expect(body.containsKey('yaw'), isFalse);
      expect((body['x'] as num).toDouble(), lessThan(0));
      expect((body['y'] as num).toDouble(), closeTo(0.0, 0.05));
      await _kapat(tester);
    }, robot.client);
  });

  testWidgets('Beyaz fırça darbesi "free" olarak haritaya uygulanır', (tester) async {
    final robot = _SahteRobot();
    await http.runWithClient(() async {
      await _ac(tester);
      await tester.scrollUntilVisible(find.text('⬜ Beyaz Fırça'), 150, scrollable: find.byType(Scrollable).first);
      await tester.pumpAndSettle();
      await tester.tap(find.text('⬜ Beyaz Fırça'));
      await tester.pump();
      final m = tester.getCenter(find.byType(Image));
      final g = await tester.startGesture(m);
      await g.moveBy(const Offset(40, 0));
      await g.moveBy(const Offset(40, 0));
      await g.up();
      await tester.pump();
      await tester.tap(find.text('Haritaya Uygula'));
      await tester.pump(const Duration(milliseconds: 300));
      final darbeler = robot.posts['/api/map_edit']!['strokes'] as List;
      expect(darbeler, hasLength(1));
      expect((darbeler.first as Map)['v'], 'free');
      expect(((darbeler.first as Map)['pts'] as List).length, greaterThanOrEqualTo(3));
      await _kapat(tester);
    }, robot.client);
  });
}
