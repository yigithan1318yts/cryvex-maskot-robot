import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../api/robot_api.dart';
import '../state/robot_state.dart';
import '../theme.dart';
import '../tts.dart';
import '../widgets/yon_tuslari.dart';
import '../widgets/password_sheet.dart';

/// index.html'deki #overlay-mapping'in (joyMode: map/rescue/control) Flutter
/// karşılığı - AYNI mantık, AYNI API uçları, tek ekranda üç görev.
enum JoyMode { map, rescue, control }

class JoystickScreen extends StatefulWidget {
  const JoystickScreen({super.key, required this.mode});
  final JoyMode mode;

  @override
  State<JoystickScreen> createState() => _JoystickScreenState();
}

class _JoystickScreenState extends State<JoystickScreen> {
  Timer? _sendTimer;
  Timer? _mapTimer;
  double _lx = 0, _az = 0;
  bool _busy = false;
  List<dynamic> _tables = [];
  bool _started = false;
  String? _mapUrlNonce;
  // GUVENLIK AGI: haritalama baslatilip "Bitir"/"Vazgeç" DISINDA bir yoldan
  // (Android geri tusu vb.) ekrandan cikilirsa slam_toolbox sonsuza kadar
  // calisir durumda kalir, map_ready hep false doner ve dashboard'daki TUM
  // devriye butonlari kilitli kalir - dispose() bu bayraga bakip gerekirse
  // otomatik cancel_mapping cagirir.
  bool _mappingConcluded = true;

  @override
  void initState() {
    super.initState();
    // _startFlow() sifre penceresi (showModalBottomSheet) acabiliyor - bunu
    // initState() TAMAMLANMADAN cagirmak "dependOnInheritedWidgetOfExactType
    // called before initState() completed" hatasi veriyor (context henuz agaca
    // tam baglanmamis). Ilk frame cizildikten SONRAYA erteliyoruz.
    WidgetsBinding.instance.addPostFrameCallback((_) => _startFlow());
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _mapTimer?.cancel();
    _leave();
    if (widget.mode == JoyMode.map && _started && !_mappingConcluded) {
      _mappingConcluded = true;
      _api.cancelMapping(kStopPassword);
    }
    super.dispose();
  }

  RobotApi get _api => context.read<RobotState>().api!;

  Map<String, String> get _text => switch (widget.mode) {
        JoyMode.map => const {
            'title': '🗺️ Ortamı Haritala',
            'sub': 'Robotu yön tuşlarıyla sürüp haritayı çıkarın',
            'tip': '💡 Sürerken robotun ARKASINDAN yürüyün ki lidar sizi engel olarak '
                'haritaya işlemesin. Bitirmeden önce robotu durdurup son konumuna getirin — '
                'o konum ve yönü otomatik kaydedilir.',
            'cancel': 'Vazgeç', 'finish': 'Bitir',
          },
        JoyMode.rescue => const {
            'title': '🆘 Kurtarma Modu',
            'sub': 'Robotu yön tuşlarıyla takıldığı yerden çıkarın',
            'tip': '💡 Robot şu an olduğu göreve devam etmeye çalışıyor ama takıldı. '
                'Elle sürüp açık bir yere getirin, "Bitti"ye basınca AYNI göreve kaldığı yerden devam edecek.',
            'cancel': 'Kapat', 'finish': 'Bitti, Devam Et',
          },
        JoyMode.control => const {
            'title': '🕹️ Kontrol Sende',
            'sub': 'Yön tuşlarıyla serbestçe sürün veya bir masaya yönlendirin',
            'tip': '💡 Bir masaya dokunursanız robot oraya gidip menüyü açar. Bitirdiğinizde '
                '"Devriyeye Devam Et"e basın — otonom devriye kaldığı yerden sürer.',
            'cancel': 'Kapat', 'finish': '▶ Devriyeye Devam Et',
          },
      };

  Future<void> _startFlow() async {
    if (widget.mode == JoyMode.map || widget.mode == JoyMode.rescue) {
      final ok = await askPassword(context,
          subtitle: widget.mode == JoyMode.map
              ? 'Haritalamayı başlatmak için 4 haneli şifre'
              : 'Kurtarma moduna geçmek için 4 haneli şifre');
      if (!ok) { if (mounted) Navigator.of(context).pop(); return; }
    }
    if (!mounted) return;
    setState(() => _busy = true);
    Map<String, dynamic> res = {'result': 'ok'};
    if (widget.mode == JoyMode.map) res = await _api.startMapping(kStopPassword);
    if (widget.mode == JoyMode.rescue) res = await _api.rescueStart(kStopPassword);
    setState(() => _busy = false);
    if (res['result'] != 'ok') {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('❌ Başlatılamadı: ${res['reason'] ?? ''}')));
        Navigator.of(context).pop();
      }
      return;
    }
    setState(() => _started = true);
    if (widget.mode == JoyMode.map) {
      _mappingConcluded = false; _startLiveMap();
      speak(_api, 'Haritalamaya başlıyorum, beni sürebilirsiniz.');
    }
    if (widget.mode == JoyMode.rescue) speak(_api, 'Yardım için hazırım, beni sürebilirsiniz.');
    if (widget.mode == JoyMode.control) { _loadTables(); speak(_api, 'Kontrol sizde.'); }
  }

  void _startLiveMap() {
    _mapTimer?.cancel();
    _mapTimer = Timer.periodic(const Duration(milliseconds: 1200), (_) {
      if (mounted) setState(() => _mapUrlNonce = DateTime.now().millisecondsSinceEpoch.toString());
    });
    _mapUrlNonce = DateTime.now().millisecondsSinceEpoch.toString();
  }

  Future<void> _loadTables() async {
    try {
      final wp = await _api.waypoints();
      if (mounted) setState(() => _tables = (wp['tables'] as List?) ?? []);
    } catch (_) {}
  }

  void _onMove(double lx, double az) {
    _lx = lx; _az = az;
    _sendTimer ??= Timer.periodic(const Duration(milliseconds: 120), (_) {
      if (widget.mode == JoyMode.rescue) {
        _api.rescueTeleop(_lx, _az);
      } else {
        _api.teleop(_lx, _az);
      }
    });
  }

  void _onRelease() {
    _sendTimer?.cancel();
    _sendTimer = null;
    _lx = 0; _az = 0;
    // NOT: rescue'da bu sadece HIZI SIFIRLAR, kurtarma modundan cikmaz -
    // parmak gecici kalksa bile gorev otomatik devam etmeye baslamasin.
    if (widget.mode == JoyMode.rescue) {
      _api.rescueTeleop(0, 0);
    } else {
      _api.teleopStop();
    }
  }

  /// Ekrandan TAM cikarken (Kapat/Bitti/geri tusu) - rescue'da bu GERCEKTEN
  /// kurtarma modunu bitirir (joyStop'tan farkli).
  void _leave() {
    _sendTimer?.cancel();
    _mapTimer?.cancel();
    if (widget.mode == JoyMode.rescue) {
      _api.rescueStop();
    } else {
      _api.teleopStop();
    }
  }

  Future<void> _onCancel() async {
    _mappingConcluded = true;
    _leave();
    if (widget.mode == JoyMode.rescue) {
      await _api.rescueStop();
      _toast('Kurtarma bitti, görev kaldığı yerden devam ediyor.');
      speak(_api, 'Görevime devam ediyorum.');
    } else if (widget.mode == JoyMode.control) {
      _toast('Kapatıldı (devriye otomatik başlamadı).');
    } else {
      await _api.cancelMapping(kStopPassword);
      _toast('Haritalamadan vazgeçildi.');
      speak(_api, 'Haritalamadan vazgeçtim.');
    }
    if (mounted) Navigator.of(context).pop();
  }

  Future<void> _onFinish() async {
    _mappingConcluded = true;
    if (widget.mode == JoyMode.rescue) {
      _leave();
      await _api.rescueStop();
      _toast('✅ Kurtarma tamam, robot görevine devam ediyor.');
      speak(_api, 'Görevime devam ediyorum.');
      if (mounted) Navigator.of(context).pop();
      return;
    }
    if (widget.mode == JoyMode.control) {
      _leave();
      await _api.startPatrol();
      _toast('▶ Devriye başlatıldı.');
      speak(_api, 'Devriyeye başlıyorum.');
      if (mounted) Navigator.of(context).pop();
      return;
    }
    // "Bitir" eski haritanin ve masalarin USTUNE yazar - once sor (sunucu
    // eskisini ayrica yedekler ama yanlislikla basilmasin).
    final sure = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Haritayı kaydet?'),
        content: const Text('Yeni harita eskisinin yerine geçecek.\n\n'
            'Masa, kapı ve barmen noktaları SİLİNİR; yeni haritada yeniden işaretlemen gerekir. '
            '(Eski harita yedeklenir.)'),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Vazgeç')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(true), child: const Text('Evet, kaydet')),
        ],
      ),
    );
    if (sure != true || !mounted) {
      _mappingConcluded = false; // haritalama suruyor, guvenlik agi yeniden etkin
      return;
    }
    setState(() => _busy = true);
    _leave();
    final res = await _api.finishMapping(kStopPassword);
    setState(() => _busy = false);
    if (res['result'] != 'ok') {
      _toast('❌ Kaydedilemedi: ${res['reason'] ?? ''}');
      _mappingConcluded = false;   // haritalama hala aktif, guvenlik agini yeniden etkinlestir
      _startLiveMap();
      return;
    }
    if (res['pose_captured'] != true) {
      _toast('⚠ Robot konumu otomatik alınamadı - Kurulum ekranında "Robot Burada" ile elle ayarlayın.');
      speak(_api, 'Harita kaydedildi, konumumu elle ayarlamanız gerekiyor.');
    } else {
      _toast('✅ Harita kaydedildi! Şimdi masaları/kapıyı işaretleyin.');
      speak(_api, 'Harita kaydedildi.');
    }
    if (mounted) Navigator.of(context).pop(true); // true = kuruluma yönlendir
  }

  void _toast(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  Future<void> _sendToTable(int index, String name) async {
    final res = await _api.goto(index + 1, action: 'welcome_menu');
    _toast(res['result'] == 'ok' ? '➡ $name\'e yönlendiriliyor…' : '❌ Gönderilemedi');
  }

  @override
  Widget build(BuildContext context) {
    final t = _text;
    return Scaffold(
      appBar: AppBar(title: Text(t['title']!)),
      body: !_started
          ? Center(
              child: _busy
                  ? Column(
                      mainAxisSize: MainAxisSize.min,
                      children: const [
                        CircularProgressIndicator(color: CryvexColors.cyan),
                        SizedBox(height: 16),
                        Padding(
                          padding: EdgeInsets.symmetric(horizontal: 32),
                          child: Text(
                            'Başlatılıyor… Nav2/harita süreci değiştiği için\nbu 10-15 saniye kadar sürebilir.',
                            style: TextStyle(color: CryvexColors.textMuted, fontSize: 13),
                            textAlign: TextAlign.center,
                          ),
                        ),
                      ],
                    )
                  : const SizedBox(),
            )
          : SafeArea(
              child: SingleChildScrollView(
                padding: const EdgeInsets.fromLTRB(18, 8, 18, 18),
                child: Column(
                  children: [
                    Text(t['sub']!, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 13.5), textAlign: TextAlign.center),
                    const SizedBox(height: 8),
                    Container(
                      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
                      child: Text(t['tip']!, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12, height: 1.45), textAlign: TextAlign.center),
                    ),
                    if (widget.mode == JoyMode.map) _liveMapView(),
                    const SizedBox(height: 12),
                    YonTuslari(onMove: _onMove, onRelease: _onRelease),   // 2026-10-08: joystick yerine yon tuslari
                    if (widget.mode == JoyMode.control) ...[
                      const SizedBox(height: 16),
                      _tablePicker(),
                    ],
                    const SizedBox(height: 18),
                    Row(children: [
                      Expanded(
                        child: ElevatedButton.icon(
                          style: CryvexButtonStyle.gray,
                          onPressed: _onCancel,
                          icon: const Icon(Icons.close, size: 18),
                          label: Text(t['cancel']!),
                        ),
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: ElevatedButton.icon(
                          style: CryvexButtonStyle.green,
                          onPressed: _busy ? null : _onFinish,
                          icon: const Icon(Icons.check, size: 18),
                          label: Text(t['finish']!, overflow: TextOverflow.ellipsis),
                        ),
                      ),
                    ]),
                  ],
                ),
              ),
            ),
    );
  }

  Widget _liveMapView() {
    final baseUrl = context.read<RobotState>().api!.baseUrl;
    final url = '$baseUrl/api/live_map.png?_=${_mapUrlNonce ?? '0'}';
    return GestureDetector(
      // Dokununca tam ekran (iki parmakla yakinlastir/kaydir) - telefonda
      // izgara etiketleri (A1, B2...) ve lidar noktalari rahat okunsun.
      onTap: () => Navigator.of(context).push(
          MaterialPageRoute(builder: (_) => _FullscreenLiveMap(baseUrl: baseUrl))),
      child: AspectRatio(
        aspectRatio: 1,
        child: Container(
          margin: const EdgeInsets.only(top: 8),
          decoration: BoxDecoration(
            color: Colors.black,
            borderRadius: BorderRadius.circular(16),
            border: Border.all(color: CryvexColors.cardLine),
          ),
          clipBehavior: Clip.antiAlias,
          child: Stack(fit: StackFit.expand, children: [
            Image.network(
              url,
              key: ValueKey(url),
              fit: BoxFit.contain,
              gaplessPlayback: true,
              loadingBuilder: (ctx, child, progress) => progress == null ? child : _mapPlaceholder(),
              errorBuilder: (ctx, err, st) => _mapPlaceholder(),
            ),
            Positioned(
              top: 8,
              right: 8,
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 9, vertical: 5),
                decoration: BoxDecoration(
                    color: Colors.black.withValues(alpha: 0.6), borderRadius: BorderRadius.circular(10)),
                child: const Row(mainAxisSize: MainAxisSize.min, children: [
                  Icon(Icons.fullscreen, size: 16, color: CryvexColors.cyan),
                  SizedBox(width: 4),
                  Text('Büyüt', style: TextStyle(color: CryvexColors.cyan, fontSize: 12, fontWeight: FontWeight.w600)),
                ]),
              ),
            ),
          ]),
        ),
      ),
    );
  }

  Widget _mapPlaceholder() => const Center(
        child: Padding(
          padding: EdgeInsets.all(20),
          child: Text('Harita oluşturuluyor… robotu sürmeye başlayın, birazdan burada görünecek.',
              style: TextStyle(color: CryvexColors.textMuted, fontSize: 12.5), textAlign: TextAlign.center),
        ),
      );

  Widget _tablePicker() {
    if (_tables.isEmpty) {
      return const Text('Henüz masa tanımlı değil.', style: TextStyle(color: CryvexColors.textMuted, fontSize: 12.5));
    }
    return Wrap(
      spacing: 8,
      runSpacing: 8,
      alignment: WrapAlignment.center,
      children: [
        for (var i = 0; i < _tables.length; i++)
          OutlinedButton(
            style: OutlinedButton.styleFrom(
              foregroundColor: CryvexColors.textPrimary,
              backgroundColor: CryvexColors.card,
              side: const BorderSide(color: CryvexColors.cardLine),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            ),
            onPressed: () => _sendToTable(i, (_tables[i]['isim'] as String?) ?? 'Masa ${i + 1}'),
            child: Text((_tables[i]['isim'] as String?) ?? 'Masa ${i + 1}'),
          ),
      ],
    );
  }
}

/// Canli haritanin tam ekran hali: iki parmakla yakinlastir, tek parmakla
/// kaydir. Haritalama arkadaki ekranda surer (geri donunce joystick oradadir).
class _FullscreenLiveMap extends StatefulWidget {
  const _FullscreenLiveMap({required this.baseUrl});
  final String baseUrl;

  @override
  State<_FullscreenLiveMap> createState() => _FullscreenLiveMapState();
}

class _FullscreenLiveMapState extends State<_FullscreenLiveMap> {
  Timer? _timer;
  String _nonce = DateTime.now().millisecondsSinceEpoch.toString();
  // Yenilemede yakinlastirma sifirlanmasin diye disarida tutuluyor.
  final _zoom = TransformationController();

  @override
  void initState() {
    super.initState();
    _timer = Timer.periodic(const Duration(milliseconds: 1200), (_) {
      if (mounted) setState(() => _nonce = DateTime.now().millisecondsSinceEpoch.toString());
    });
  }

  @override
  void dispose() {
    _timer?.cancel();
    _zoom.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final url = '${widget.baseUrl}/api/live_map.png?_=$_nonce';
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(title: const Text('Canlı Harita')),
      body: SafeArea(
        child: Column(children: [
          Expanded(
            child: InteractiveViewer(
              transformationController: _zoom,
              minScale: 1,
              maxScale: 6,
              child: Center(
                child: Image.network(
                  url,
                  key: ValueKey(url),
                  fit: BoxFit.contain,
                  gaplessPlayback: true,
                  errorBuilder: (ctx, err, st) => const SizedBox(),
                ),
              ),
            ),
          ),
          const Padding(
            padding: EdgeInsets.all(10),
            child: Text('İki parmakla yakınlaştır · kırmızı: LiDAR\'ın gördüğü · mavi ok: robotun önü',
                style: TextStyle(color: CryvexColors.textMuted, fontSize: 12), textAlign: TextAlign.center),
          ),
        ]),
      ),
    );
  }
}
