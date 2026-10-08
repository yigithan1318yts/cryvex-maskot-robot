import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../api/robot_api.dart';
import '../state/robot_state.dart';
import '../theme.dart';

/// Harita kurulumu - robotun web kurulum sayfasının (web/setup.html) uygulamanın
/// kendi ekranı olarak yeniden yazılmış hali:
///  * 🤖 Robot Burada (tek dokunuş, robotun bildiği yön korunur) / 🧭 Robot + Yön
///  * 🍽️ Masa, 🏠 Üs (barmen), 🚪 Kapı: 1. dokunuş yer, 2. dokunuş robotun bakacağı yön
///  * ⬜ Beyaz / ⬛ Siyah fırça: haritadaki pürüzü sil / duvar çiz
///  * Noktaları adlandır, sil, 💾 kaydet (/api/waypoints)
/// Robotun konumu /api/robot_pose ile saniyede bir alınıp haritanın üstüne çizilir.
class KurulumScreen extends StatefulWidget {
  const KurulumScreen({super.key});

  @override
  State<KurulumScreen> createState() => _KurulumScreenState();
}

enum _Arac { gezin, robot, robotYon, masa, us, kapi, beyaz, siyah }

const _aracAd = {
  _Arac.gezin: '✋ Gezin',
  _Arac.robot: '🤖 Robot Burada',
  _Arac.robotYon: '🧭 Robot + Yön',
  _Arac.masa: '🍽️ Masa',
  _Arac.us: '🏠 Üs',
  _Arac.kapi: '🚪 Kapı',
  _Arac.beyaz: '⬜ Beyaz Fırça',
  _Arac.siyah: '⬛ Siyah Fırça',
};

/// Harita karesi (x, y metre; yaw radyan, harita çerçevesi).
class _Nokta {
  _Nokta(this.isim, this.x, this.y, this.yaw);
  String isim;
  double x, y, yaw;

  Map<String, dynamic> toJson() => {'isim': isim, 'x': x, 'y': y, 'yaw': yaw};

  static _Nokta? fromJson(dynamic j, String varsayilan) {
    if (j is! Map) return null;
    return _Nokta(
      (j['isim'] ?? varsayilan).toString(),
      (j['x'] as num).toDouble(),
      (j['y'] as num).toDouble(),
      ((j['yaw'] ?? 0) as num).toDouble(),
    );
  }
}

/// Fırça darbesi: v = free|occ, r = harita pikseli, pts = resmin 0..1 kesirleri.
class _Darbe {
  _Darbe(this.v, this.r);
  final String v;
  final double r;
  final List<Offset> pts = [];

  Map<String, dynamic> toJson() => {
    'v': v,
    'r': r,
    'pts': [
      for (final p in pts) [p.dx, p.dy],
    ],
  };
}

class _KurulumScreenState extends State<KurulumScreen> {
  // harita
  Map<String, dynamic>? _info;
  Uint8List? _png;
  String? _hata;
  bool _yukleniyor = true;

  // noktalar
  _Nokta? _us, _kapi;
  List<_Nokta> _masalar = [];
  bool _kaydedilmedi = false;

  // robot
  Map<String, dynamic>? _poz;
  Timer? _pozTimer;

  // araçlar
  _Arac _arac = _Arac.gezin;
  Offset? _bekleyen; // 2 dokunuşlu araçlarda 1. dokunuş (harita kesri)
  double _fircaCm = 30;
  final List<_Darbe> _darbeler = [];
  bool _mesgul = false;

  RobotApi get _api => context.read<RobotState>().api!;

  @override
  void initState() {
    super.initState();
    _yukle();
    _pozTimer = Timer.periodic(const Duration(seconds: 1), (_) => _pozAl());
  }

  @override
  void dispose() {
    _pozTimer?.cancel();
    super.dispose();
  }

  Future<void> _yukle() async {
    setState(() {
      _yukleniyor = true;
      _hata = null;
    });
    try {
      final info = await _api.mapInfo();
      if (((info['width'] ?? 0) as num) == 0) {
        setState(() {
          _hata =
              'Robotta kayıtlı harita yok.\nÖnce "Ortamı Haritala" ya da Otonom → Keşfet ile harita çıkarıp kaydedin.';
          _yukleniyor = false;
        });
        return;
      }
      final png = await _api.mapPng();
      final wp = await _api.waypoints();
      if (!mounted) return;
      setState(() {
        _info = info;
        _png = png;
        _us = _Nokta.fromJson(wp['barista'], 'Barmen');
        _kapi = _Nokta.fromJson(wp['door'], 'Kapi');
        _masalar = [for (final (i, t) in ((wp['tables'] as List?) ?? []).indexed) ?_Nokta.fromJson(t, 'Masa ${i + 1}')];
        _kaydedilmedi = false;
        _yukleniyor = false;
        if (png == null) _hata = 'Harita resmi alınamadı';
      });
    } catch (_) {
      if (mounted) {
        setState(() {
          _hata = 'Robota ulaşılamıyor';
          _yukleniyor = false;
        });
      }
    }
  }

  Future<void> _pozAl() async {
    try {
      final p = await _api.robotPose();
      if (mounted) setState(() => _poz = p['ok'] == true ? p : null);
    } catch (_) {}
  }

  // ---- koordinat dönüşümü (setup.html worldToFrac / fracToWorld ile aynı) ----
  double get _res => (_info!['resolution'] as num).toDouble();
  double get _ox => ((_info!['origin'] as List)[0] as num).toDouble();
  double get _oy => ((_info!['origin'] as List)[1] as num).toDouble();
  double get _w => (_info!['width'] as num).toDouble();
  double get _h => (_info!['height'] as num).toDouble();

  Offset _dunyadanKesre(double x, double y) => Offset(((x - _ox) / _res) / _w, (_h - (y - _oy) / _res) / _h);

  (double, double) _kesirdenDunyaya(Offset f) => (_ox + f.dx * _w * _res, _oy + (_h - f.dy * _h) * _res);

  /// İki kesir noktası arasındaki dünya yönü (ekranda y aşağı, dünyada yukarı).
  double _yon(Offset a, Offset b) => math.atan2(-(b.dy - a.dy) * _h, (b.dx - a.dx) * _w);

  // ---- dokunma ----
  void _dokun(Offset f) {
    switch (_arac) {
      case _Arac.gezin:
      case _Arac.beyaz:
      case _Arac.siyah:
        return;
      case _Arac.robot:
        _robotKonumu(f, null);
        return;
      case _Arac.robotYon:
      case _Arac.masa:
      case _Arac.us:
      case _Arac.kapi:
        if (_bekleyen == null) {
          setState(() => _bekleyen = f);
          return;
        }
        final a = _bekleyen!;
        final yaw = _yon(a, f);
        setState(() => _bekleyen = null);
        if (_arac == _Arac.robotYon) {
          _robotKonumu(a, yaw);
          return;
        }
        final (x, y) = _kesirdenDunyaya(a);
        setState(() {
          if (_arac == _Arac.masa) _masalar.add(_Nokta('Masa ${_masalar.length + 1}', x, y, yaw));
          if (_arac == _Arac.us) _us = _Nokta('Barmen', x, y, yaw);
          if (_arac == _Arac.kapi) _kapi = _Nokta('Kapi', x, y, yaw);
          _kaydedilmedi = true;
        });
    }
  }

  Future<void> _robotKonumu(Offset f, double? yaw) async {
    final (x, y) = _kesirdenDunyaya(f);
    final res = await _api.setPose(x, y, yaw);
    _bildir(
      res['result'] == 'ok'
          ? (yaw == null ? '🤖 Konum verildi (yön korundu). Robot birkaç saniyede oturur.' : '🧭 Konum ve yön verildi.')
          : '❌ Konum verilemedi: ${res['reason'] ?? ''}',
    );
    _pozAl();
  }

  void _boyaBasla(Offset f) {
    if (_arac != _Arac.beyaz && _arac != _Arac.siyah) return;
    final r = math.max(1.0, (_fircaCm / 100) / 2 / _res);
    setState(() => _darbeler.add(_Darbe(_arac == _Arac.beyaz ? 'free' : 'occ', r)..pts.add(f)));
  }

  void _boyaDevam(Offset f) {
    if ((_arac != _Arac.beyaz && _arac != _Arac.siyah) || _darbeler.isEmpty) return;
    setState(() => _darbeler.last.pts.add(f));
  }

  Future<void> _boyayiUygula() async {
    if (_darbeler.isEmpty) return;
    setState(() => _mesgul = true);
    final res = await _api.mapEdit([for (final d in _darbeler) d.toJson()]);
    if (!mounted) return;
    setState(() => _mesgul = false);
    if (res['result'] == 'ok') {
      _darbeler.clear();
      _bildir('✅ Harita güncellendi (eskisi yedeklendi). Konum sistemi yeniden açılıyor…');
      final png = await _api.mapPng();
      if (mounted && png != null) setState(() => _png = png);
    } else {
      _bildir('❌ Harita kaydedilemedi: ${res['reason'] ?? ''}');
    }
  }

  Future<void> _kaydet() async {
    setState(() => _mesgul = true);
    final res = await _api.saveWaypoints({
      'barista': _us?.toJson(),
      'door': _kapi?.toJson(),
      'tables': [for (final t in _masalar) t.toJson()],
    });
    if (!mounted) return;
    setState(() {
      _mesgul = false;
      if (res['result'] == 'ok') _kaydedilmedi = false;
    });
    _bildir(
      res['result'] == 'ok'
          ? '💾 Kaydedildi: ${_masalar.length} masa${_us != null ? ', üs' : ''}${_kapi != null ? ', kapı' : ''}'
          : '❌ Kaydedilemedi: ${res['reason'] ?? ''}',
    );
  }

  void _bildir(String s) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(s)));
  }

  Future<void> _yenidenAdlandir(_Nokta n) async {
    final c = TextEditingController(text: n.isim);
    final yeni = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Masanın adı'),
        content: TextField(controller: c, autofocus: true, maxLength: 24),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(), child: const Text('Vazgeç')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(c.text.trim()), child: const Text('Tamam')),
        ],
      ),
    );
    if (yeni != null && yeni.isNotEmpty) {
      setState(() {
        n.isim = yeni;
        _kaydedilmedi = true;
      });
    }
  }

  void _masaSil(int i) => setState(() {
    _masalar.removeAt(i);
    for (final (j, t) in _masalar.indexed) {
      if (RegExp(r'^Masa \d+$').hasMatch(t.isim)) t.isim = 'Masa ${j + 1}';
    }
    _kaydedilmedi = true;
  });

  String get _ipucu => switch (_arac) {
    _Arac.gezin => 'İki parmakla yakınlaştır, sürükleyerek gez.',
    _Arac.robot => 'Robotun gerçekte durduğu yere bir kez dokun.',
    _Arac.beyaz => 'Parmağınla sil: yanlış görünen siyah noktaları beyaza boya.',
    _Arac.siyah => 'Parmağınla çiz: robotun girmemesi gereken yeri siyaha boya.',
    _ when _bekleyen == null => '1) Yerine dokun.',
    _ => '2) Robotun orada bakacağı yöne dokun.',
  };

  Future<bool> _cikisOnayi() async {
    if (!_kaydedilmedi && _darbeler.isEmpty) return true;
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Kaydedilmemiş değişiklik var'),
        content: const Text('Çıkarsan işaretlediğin noktalar ve fırça darbeleri kaybolur.'),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Kal')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(true), child: const Text('Kaydetmeden çık')),
        ],
      ),
    );
    return ok == true;
  }

  @override
  Widget build(BuildContext context) {
    return PopScope(
      canPop: !_kaydedilmedi && _darbeler.isEmpty,
      onPopInvokedWithResult: (didPop, _) async {
        if (didPop) return;
        if (await _cikisOnayi() && context.mounted) Navigator.of(context).pop();
      },
      child: Scaffold(
        appBar: AppBar(
          title: const Text('🛠️ Harita Kurulumu'),
          actions: [IconButton(onPressed: _yukle, icon: const Icon(Icons.refresh), tooltip: 'Yenile')],
        ),
        body: SafeArea(child: _govde()),
      ),
    );
  }

  Widget _govde() {
    if (_yukleniyor) return const Center(child: CircularProgressIndicator(color: CryvexColors.cyan));
    if (_info == null || _png == null) {
      return Center(
        child: Padding(
          padding: const EdgeInsets.all(24),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                _hata ?? 'Harita yok',
                textAlign: TextAlign.center,
                style: const TextStyle(color: CryvexColors.textMuted),
              ),
              const SizedBox(height: 12),
              TextButton(onPressed: _yukle, child: const Text('Tekrar dene')),
            ],
          ),
        ),
      );
    }
    final firca = _arac == _Arac.beyaz || _arac == _Arac.siyah;
    return Column(
      children: [
        _aracCubugu(),
        Padding(
          padding: const EdgeInsets.fromLTRB(14, 2, 14, 6),
          child: Row(
            children: [
              Expanded(
                child: Text(_ipucu, style: const TextStyle(color: CryvexColors.cyan, fontSize: 12.5)),
              ),
              if (_bekleyen != null)
                TextButton(onPressed: () => setState(() => _bekleyen = null), child: const Text('Vazgeç')),
            ],
          ),
        ),
        Expanded(child: _haritaAlani(firca)),
        if (firca || _darbeler.isNotEmpty) _fircaCubugu(),
        _altPanel(),
      ],
    );
  }

  Widget _aracCubugu() => SizedBox(
    height: 48,
    child: ListView(
      scrollDirection: Axis.horizontal,
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      children: [
        for (final a in _Arac.values)
          Padding(
            padding: const EdgeInsets.only(right: 6),
            child: ChoiceChip(
              label: Text(_aracAd[a]!),
              selected: _arac == a,
              onSelected: (_) => setState(() {
                _arac = a;
                _bekleyen = null;
              }),
            ),
          ),
      ],
    ),
  );

  Widget _haritaAlani(bool firca) {
    return LayoutBuilder(
      builder: (context, c) {
        // Haritayı kutuya oranını bozmadan sığdır.
        final olcek = math.min(c.maxWidth / _w, c.maxHeight / _h);
        final boyut = Size(_w * olcek, _h * olcek);
        Offset kesir(Offset yerel) =>
            Offset((yerel.dx / boyut.width).clamp(0.0, 1.0), (yerel.dy / boyut.height).clamp(0.0, 1.0));
        return Container(
          color: Colors.black,
          child: InteractiveViewer(
            minScale: 1,
            maxScale: 10,
            panEnabled: !firca,
            scaleEnabled: !firca,
            child: Center(
              child: SizedBox(
                width: boyut.width,
                height: boyut.height,
                child: Listener(
                  // Fırça ham parmak hareketini dinler (InteractiveViewer ile yarışmaz).
                  onPointerDown: (e) => _boyaBasla(kesir(e.localPosition)),
                  onPointerMove: (e) => _boyaDevam(kesir(e.localPosition)),
                  child: GestureDetector(
                    onTapUp: (d) => _dokun(kesir(d.localPosition)),
                    child: Stack(
                      fit: StackFit.expand,
                      children: [
                        Image.memory(_png!, fit: BoxFit.fill, filterQuality: FilterQuality.none, gaplessPlayback: true),
                        CustomPaint(painter: _Cizim(this)),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _fircaCubugu() => Padding(
    padding: const EdgeInsets.fromLTRB(10, 6, 10, 0),
    child: Row(
      children: [
        for (final cm in [15.0, 30.0, 60.0])
          Padding(
            padding: const EdgeInsets.only(right: 4),
            child: ChoiceChip(
              label: Text('${cm.round()} cm'),
              selected: _fircaCm == cm,
              onSelected: (_) => setState(() => _fircaCm = cm),
            ),
          ),
        const Spacer(),
        IconButton(
          tooltip: 'Geri al',
          onPressed: _darbeler.isEmpty ? null : () => setState(() => _darbeler.removeLast()),
          icon: const Icon(Icons.undo),
        ),
        FilledButton(
          onPressed: _darbeler.isEmpty || _mesgul ? null : _boyayiUygula,
          child: const Text('Haritaya Uygula'),
        ),
      ],
    ),
  );

  Widget _altPanel() {
    final satirlar = <Widget>[
      if (_us != null)
        _noktaSatiri(
          '🏠',
          'Üs (barmen)',
          () => setState(() {
            _us = null;
            _kaydedilmedi = true;
          }),
        ),
      if (_kapi != null)
        _noktaSatiri(
          '🚪',
          'Kapı',
          () => setState(() {
            _kapi = null;
            _kaydedilmedi = true;
          }),
        ),
      for (final (i, t) in _masalar.indexed)
        _noktaSatiri('🍽️', t.isim, () => _masaSil(i), onTap: () => _yenidenAdlandir(t)),
    ];
    // Material (Container rengi değil): ListTile dokunma efekti en yakın Material'a çizilir.
    return Material(
      color: const Color(0xFF0D0D18),
      shape: const Border(top: BorderSide(color: CryvexColors.cardLine)),
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxHeight: 210),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(14, 8, 10, 4),
              child: Row(
                children: [
                  Expanded(
                    child: Text(
                      _poz == null
                          ? '⚠ Robotun konumu bilinmiyor - "Robot Burada" ile verin'
                          : 'Robot: x ${(_poz!['x'] as num).toStringAsFixed(2)} · y ${(_poz!['y'] as num).toStringAsFixed(2)}',
                      style: TextStyle(color: _poz == null ? CryvexColors.amber : CryvexColors.textMuted, fontSize: 12),
                    ),
                  ),
                  FilledButton.icon(
                    onPressed: _kaydedilmedi && !_mesgul ? _kaydet : null,
                    icon: const Icon(Icons.save),
                    label: const Text('Kaydet'),
                  ),
                ],
              ),
            ),
            if (_mesgul) const LinearProgressIndicator(color: CryvexColors.cyan, minHeight: 2),
            Flexible(
              child: satirlar.isEmpty
                  ? const Padding(
                      padding: EdgeInsets.all(14),
                      child: Text(
                        'Henüz nokta yok. Üstten 🍽️ Masa, 🏠 Üs ya da 🚪 Kapı seçip haritaya dokunun.',
                        style: TextStyle(color: CryvexColors.textMuted, fontSize: 12.5),
                      ),
                    )
                  : ListView(shrinkWrap: true, padding: const EdgeInsets.only(bottom: 8), children: satirlar),
            ),
          ],
        ),
      ),
    );
  }

  Widget _noktaSatiri(String ikon, String ad, VoidCallback sil, {VoidCallback? onTap}) => ListTile(
    dense: true,
    visualDensity: VisualDensity.compact,
    leading: Text(ikon, style: const TextStyle(fontSize: 18)),
    title: Text(ad),
    subtitle: onTap != null ? const Text('adını değiştirmek için dokun', style: TextStyle(fontSize: 11)) : null,
    onTap: onTap,
    trailing: IconButton(
      icon: const Icon(Icons.delete_outline, color: CryvexColors.red),
      onPressed: sil,
    ),
  );
}

/// Haritanın üstüne: fırça önizlemesi, masalar/üs/kapı (yön oklarıyla), robot,
/// iki dokunuşlu araçta bekleyen ilk nokta.
class _Cizim extends CustomPainter {
  _Cizim(this.s);
  final _KurulumScreenState s;

  @override
  void paint(Canvas canvas, Size size) {
    Offset px(Offset f) => Offset(f.dx * size.width, f.dy * size.height);
    final pikselBasi = size.width / s._w; // ekran pikseli / harita pikseli

    for (final d in s._darbeler) {
      final p = Paint()
        ..color = (d.v == 'free' ? Colors.white : Colors.black).withValues(alpha: 0.85)
        ..strokeWidth = d.r * 2 * pikselBasi
        ..strokeCap = StrokeCap.round
        ..style = PaintingStyle.stroke;
      if (d.pts.length == 1) {
        canvas.drawCircle(px(d.pts.first), d.r * pikselBasi, p..style = PaintingStyle.fill);
      } else {
        final path = Path()..moveTo(px(d.pts.first).dx, px(d.pts.first).dy);
        for (final q in d.pts.skip(1)) {
          path.lineTo(px(q).dx, px(q).dy);
        }
        canvas.drawPath(path, p);
      }
    }

    // Ok uzunluğu ve işaret boyu ekranda sabit kalsın diye harita ölçeğinden bağımsız.
    void isaret(_Nokta n, Color renk, String etiket) {
      final m = px(s._dunyadanKesre(n.x, n.y));
      final uc = m + Offset(math.cos(n.yaw), -math.sin(n.yaw)) * 16;
      canvas.drawLine(
        m,
        uc,
        Paint()
          ..color = renk
          ..strokeWidth = 2.5,
      );
      canvas.drawCircle(m, 8, Paint()..color = renk);
      final tp = TextPainter(
        text: TextSpan(
          text: etiket,
          style: const TextStyle(color: Colors.black, fontSize: 9, fontWeight: FontWeight.w800),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, m - Offset(tp.width / 2, tp.height / 2));
    }

    for (final (i, t) in s._masalar.indexed) {
      isaret(t, CryvexColors.cyan, '${i + 1}');
    }
    if (s._us != null) isaret(s._us!, CryvexColors.green, 'U');
    if (s._kapi != null) isaret(s._kapi!, CryvexColors.amber, 'K');

    final poz = s._poz;
    if (poz != null) {
      final m = px(s._dunyadanKesre((poz['x'] as num).toDouble(), (poz['y'] as num).toDouble()));
      final yaw = (poz['yaw'] as num).toDouble();
      final govde = 0.30 / s._res * pikselBasi; // robot yarıçapı 0,30 m
      canvas.drawCircle(m, govde, Paint()..color = const Color(0xFF00AAFF).withValues(alpha: 0.25));
      canvas.drawCircle(
        m,
        govde,
        Paint()
          ..color = const Color(0xFF00AAFF)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.5,
      );
      canvas.drawLine(
        m,
        m + Offset(math.cos(yaw), -math.sin(yaw)) * math.max(govde, 14),
        Paint()
          ..color = const Color(0xFF00AAFF)
          ..strokeWidth = 3,
      );
    }

    final b = s._bekleyen;
    if (b != null) {
      canvas.drawCircle(
        px(b),
        9,
        Paint()
          ..color = Colors.pinkAccent
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2.5,
      );
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
