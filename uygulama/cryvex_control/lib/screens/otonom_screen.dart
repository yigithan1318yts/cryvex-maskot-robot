import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import '../api/robot_api.dart';
import '../state/robot_state.dart';
import '../theme.dart';
import '../widgets/live_map.dart';
import '../widgets/password_sheet.dart';
import 'kurulum_screen.dart';

/// Otonom sürüş: robot haritayı kendi çıkarır (Keşfet) ya da mevcut haritada
/// kendi dolaşır (Devriye). Robottaki ~/cryvex_araclar/otonom_gezgin.py'yi
/// cafe_ui_server'ın /api/otonom ucu başlatır/durdurur; web'deki /otonom
/// sayfasının uygulama karşılığı.
///
/// Keşfin çıkardığı harita robotun GEZİNME haritası olmaz: "Haritayı Kaydet"
/// (finish_mapping) ile kaydedilince masalar işaretlenebilir ve devriye onu kullanır.
class OtonomScreen extends StatefulWidget {
  const OtonomScreen({super.key});

  @override
  State<OtonomScreen> createState() => _OtonomScreenState();
}

class _OtonomScreenState extends State<OtonomScreen> {
  Timer? _timer;
  bool _running = false;
  bool _reachable = true;
  bool _busy = false;
  List<String> _log = [];
  final _logScroll = ScrollController();

  @override
  void initState() {
    super.initState();
    context.read<RobotState>().startPolling();   // mode (mapping/navigation) icin
    _refresh();
    _timer = Timer.periodic(const Duration(seconds: 2), (_) => _refresh());
  }

  @override
  void dispose() {
    _timer?.cancel();
    context.read<RobotState>().stopPolling();
    _logScroll.dispose();
    super.dispose();
  }

  RobotApi get _api => context.read<RobotState>().api!;

  Future<void> _refresh() async {
    try {
      final d = await _api.otonomStatus();
      if (!mounted) return;
      setState(() {
        _reachable = true;
        _running = d['calisiyor'] == true;
        _log = [for (final l in (d['log'] as List? ?? [])) '$l'];
      });
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (_logScroll.hasClients) _logScroll.jumpTo(_logScroll.position.maxScrollExtent);
      });
    } catch (_) {
      if (mounted) setState(() => _reachable = false);
    }
  }

  void _snack(String s) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(s)));
  }

  Future<bool> _confirm(String title, String body, String yes) async {
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text(title),
        content: Text(body),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Vazgeç')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(true), child: Text(yes)),
        ],
      ),
    );
    return ok == true;
  }

  Future<void> _start(String mod) async {
    final kesif = mod == 'kesif';
    final ok = await _confirm(
      kesif ? '🧭 Keşif başlasın mı?' : '🔁 Otonom devriye başlasın mı?',
      kesif
          ? 'Robot bulunduğu yerden haritayı SIFIRDAN çıkarır ve kendi kendine gezer.\n\n'
              'Etrafının boş olduğundan, kablonun dolanmayacağından emin olun.'
          : 'Robot mevcut haritada, yakında gitmediği yerlere öncelik vererek kendi kendine dolaşır.',
      'Başlat',
    );
    if (!ok || !mounted) return;
    if (!await askPassword(context, subtitle: 'Otonom sürüş için 4 haneli şifre')) return;
    setState(() => _busy = true);
    final res = await _api.otonom(mod, kStopPassword, yeni: kesif);
    if (!mounted) return;
    setState(() => _busy = false);
    _snack(res['result'] == 'ok' ? (kesif ? '🧭 Keşif başladı' : '🔁 Devriye başladı')
        : '❌ Başlatılamadı: ${res['reason'] ?? ''}');
    _refresh();
  }

  /// DUR şifre sormaz: durdurmak her zaman tek dokunuş olmalı.
  Future<void> _stop() async {
    final res = await _api.otonom('dur', kStopPassword);
    _snack(res['result'] == 'ok' ? '⏹ Durduruldu (harita kaydediliyor)' : '❌ Durdurulamadı: ${res['reason'] ?? ''}');
    _refresh();
  }

  Future<void> _saveMap() async {
    final ok = await _confirm('Haritayı kaydet?',
        'Keşfin çıkardığı harita robotun gezinme haritası olacak; eskisinin yerine geçer.\n\n'
            'Sonra masa/kapı noktalarını işaretleyebilirsiniz.',
        'Evet, kaydet');
    if (!ok || !mounted) return;
    setState(() => _busy = true);
    final res = await _api.finishMapping(kStopPassword);
    if (!mounted) return;
    setState(() => _busy = false);
    if (res['result'] != 'ok') {
      _snack('❌ Kaydedilemedi: ${res['reason'] ?? ''}');
      return;
    }
    _snack('✅ Harita kaydedildi. Şimdi masaları, üssü ve kapıyı işaretleyin.');
    Navigator.of(context).push(MaterialPageRoute(builder: (_) => const KurulumScreen()));
  }

  @override
  Widget build(BuildContext context) {
    final st = context.watch<RobotState>();
    final mapping = st.mode == 'mapping';
    return Scaffold(
      appBar: AppBar(title: const Text('🤖 Otonom Sürüş')),
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.all(16),
          children: [
            _statusCard(),
            const SizedBox(height: 12),
            SizedBox(
              width: double.infinity,
              child: ElevatedButton(
                style: CryvexButtonStyle.red,
                onPressed: _stop,
                child: const Text('⏹ DUR', style: TextStyle(fontSize: 20, fontWeight: FontWeight.w800)),
              ),
            ),
            const SizedBox(height: 10),
            Row(children: [
              Expanded(
                child: ElevatedButton(
                  style: CryvexButtonStyle.cyan,
                  onPressed: _busy || _running ? null : () => _start('kesif'),
                  child: const Text('🧭 Keşfet\n(yeni harita)', textAlign: TextAlign.center),
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: ElevatedButton(
                  style: CryvexButtonStyle.green,
                  onPressed: _busy || _running ? null : () => _start('devriye'),
                  child: const Text('🔁 Devriye\n(mevcut harita)', textAlign: TextAlign.center),
                ),
              ),
            ]),
            if (mapping && !_running) ...[
              const SizedBox(height: 10),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton(
                  style: CryvexButtonStyle.amber,
                  onPressed: _busy ? null : _saveMap,
                  child: const Text('💾 Keşfin Haritasını Robotun Haritası Yap'),
                ),
              ),
            ],
            if (_busy) const Padding(
              padding: EdgeInsets.only(top: 10),
              child: LinearProgressIndicator(color: CryvexColors.cyan),
            ),
            const SizedBox(height: 12),
            LiveMapCard(baseUrl: st.api?.baseUrl, statusLine: mapping ? 'Haritalama açık' : ''),
            const SizedBox(height: 12),
            CameraCard(api: _api),
            const SizedBox(height: 12),
            _logCard(),
            const SizedBox(height: 8),
            const Text(
              'Keşfet: bulunduğu yerde haritayı sıfırdan çıkarır, bitince kendiliğinden devriyeye geçer. '
              'Devriye: mevcut haritada dolaşır. DUR: hemen durur ve haritayı kaydeder.',
              style: TextStyle(color: CryvexColors.textMuted, fontSize: 12),
            ),
          ],
        ),
      ),
    );
  }

  Widget _statusCard() {
    final (text, color) = !_reachable
        ? ('Robota ulaşılamıyor', CryvexColors.amber)
        : _running
            ? ('ÇALIŞIYOR · robot kendi sürüyor', CryvexColors.green)
            : ('Duruyor', CryvexColors.textMuted);
    return _card(Row(children: [
      Container(width: 12, height: 12, decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
      const SizedBox(width: 10),
      Expanded(child: Text(text, style: TextStyle(color: color, fontWeight: FontWeight.w700, fontSize: 15))),
    ]));
  }

  Widget _logCard() => _card(Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        const Text('📜 Robotun günlüğü', style: TextStyle(fontWeight: FontWeight.w700)),
        const SizedBox(height: 6),
        SizedBox(
          height: 180,
          child: _log.isEmpty
              ? const Center(child: Text('Henüz kayıt yok', style: TextStyle(color: CryvexColors.textMuted)))
              : ListView(
                  controller: _logScroll,
                  children: [
                    for (final l in _log)
                      Text(l, style: const TextStyle(fontFamily: 'monospace', fontSize: 11.5,
                          color: CryvexColors.textPrimary)),
                  ],
                ),
        ),
      ]));
}

Widget _card(Widget child) => Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: CryvexColors.cardLine),
      ),
      child: child,
    );

/// Kamera + YOLO algılamanın canlı karesi (kutular ve etiketler robotta çizilir).
/// Açılınca saniyede ~2 kare çeker; kapalıyken ağı yormaz.
class CameraCard extends StatefulWidget {
  const CameraCard({super.key, required this.api});
  final RobotApi api;

  @override
  State<CameraCard> createState() => _CameraCardState();
}

class _CameraCardState extends State<CameraCard> {
  bool _on = false;
  Timer? _timer;
  bool _busy = false;
  Uint8List? _jpg;
  String? _error;

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  void _toggle() {
    setState(() => _on = !_on);
    _timer?.cancel();
    if (_on) {
      _fetch();
      _timer = Timer.periodic(const Duration(milliseconds: 500), (_) => _fetch());
    }
  }

  Future<void> _fetch() async {
    if (_busy) return;
    _busy = true;
    try {
      final res = await http.get(Uri.parse(widget.api.cameraFrameUrl())).timeout(const Duration(seconds: 3));
      if (!mounted) return;
      setState(() {
        if (res.statusCode == 200) {
          _jpg = res.bodyBytes;
          _error = null;
        } else {
          _error = 'Kamera henüz kare üretmedi';
        }
      });
    } catch (_) {
      if (mounted) setState(() => _error = 'Kamera/algılama çalışmıyor');
    } finally {
      _busy = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    return _card(Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
      Row(children: [
        const Text('📷 Kamera (yapay zekâ)', style: TextStyle(fontWeight: FontWeight.w700)),
        const Spacer(),
        Switch(value: _on, onChanged: (_) => _toggle(), activeThumbColor: CryvexColors.cyan),
      ]),
      if (_on)
        ClipRRect(
          borderRadius: BorderRadius.circular(10),
          child: Container(
            color: Colors.black,
            constraints: const BoxConstraints(minHeight: 140),
            child: _jpg == null
                ? Center(child: Padding(
                    padding: const EdgeInsets.all(20),
                    child: Text(_error ?? 'Kamera açılıyor…', style: const TextStyle(color: CryvexColors.textMuted))))
                : Image.memory(_jpg!, gaplessPlayback: true, fit: BoxFit.contain),
          ),
        ),
      if (_on && _error != null && _jpg != null)
        Padding(
          padding: const EdgeInsets.only(top: 6),
          child: Text('⚠ $_error · son kare gösteriliyor', style: const TextStyle(color: CryvexColors.amber, fontSize: 12)),
        ),
    ]));
  }
}
