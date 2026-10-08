import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import '../theme.dart';

/// Ana ekrandaki canlı harita: robotun haritadaki yeri ve yönü (mavi), LiDAR'ın
/// o an gördüğü noktalar (kırmızı), Nav2'nin planladığı yol (sarı), masalar
/// (camgöbeği, numaralı), Üs (yeşil "U") ve Kapı (turuncu "K").
/// Görüntüyü robot çizer (/api/live_map.png?grid=1&marks=1&plan=1); burada
/// saniyede bir alınıp titremeden (gaplessPlayback) gösterilir.
class LiveMapCard extends StatefulWidget {
  const LiveMapCard({super.key, required this.baseUrl, this.statusLine = ''});

  final String? baseUrl;
  final String statusLine;

  @override
  State<LiveMapCard> createState() => _LiveMapCardState();
}

class _LiveMapCardState extends State<LiveMapCard> with WidgetsBindingObserver {
  Timer? _timer;
  bool _busy = false;
  bool _paused = false;
  Uint8List? _png;
  String? _error;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _fetch();
    _timer = Timer.periodic(const Duration(seconds: 1), (_) => _fetch());
  }

  @override
  void dispose() {
    _timer?.cancel();
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    // Uygulama arka plandayken robotu ve telefonun pilini boşuna yorma.
    _paused = state != AppLifecycleState.resumed;
  }

  Future<void> _fetch() async {
    final base = widget.baseUrl;
    if (_busy || _paused || base == null || !mounted) return;
    _busy = true;
    try {
      final uri = Uri.parse('$base/api/live_map.png?grid=1&marks=1&plan=1&crop=1&_=${DateTime.now().millisecondsSinceEpoch}');
      final res = await http.get(uri).timeout(const Duration(seconds: 4));
      if (!mounted) return;
      if (res.statusCode == 200) {
        setState(() { _png = res.bodyBytes; _error = null; });
      } else {
        setState(() => _error = res.statusCode == 503 ? 'Harita henüz yok (önce Ortamı Haritala)' : 'Harita alınamadı (${res.statusCode})');
      }
    } catch (_) {
      if (mounted) setState(() => _error = 'Robota ulaşılamıyor');
    } finally {
      _busy = false;
    }
  }

  Widget _mapImage({bool interactive = false}) {
    final img = Image.memory(_png!, gaplessPlayback: true, filterQuality: FilterQuality.medium, fit: BoxFit.contain);
    if (!interactive) return img;
    return InteractiveViewer(minScale: 1, maxScale: 6, child: Center(child: img));
  }

  void _openFull() {
    if (_png == null) return;
    Navigator.of(context).push(MaterialPageRoute(builder: (_) => _FullMapPage(state: this)));
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: CryvexColors.cardLine),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        Row(children: [
          const Text('🗺️ Canlı Harita', style: TextStyle(color: CryvexColors.textPrimary, fontWeight: FontWeight.w700)),
          const Spacer(),
          if (_png != null)
            TextButton.icon(
              onPressed: _openFull,
              icon: const Icon(Icons.fullscreen, size: 18),
              label: const Text('Büyüt'),
            ),
        ]),
        if (widget.statusLine.isNotEmpty)
          Padding(
            padding: const EdgeInsets.only(bottom: 6),
            child: Text(widget.statusLine, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12.5)),
          ),
        ClipRRect(
          borderRadius: BorderRadius.circular(10),
          child: Container(
            color: Colors.black,
            constraints: const BoxConstraints(minHeight: 160, maxHeight: 420),
            child: _png == null
                ? Center(
                    child: Padding(
                      padding: const EdgeInsets.all(24),
                      child: Text(_error ?? 'Harita yükleniyor…',
                          textAlign: TextAlign.center, style: const TextStyle(color: CryvexColors.textMuted)),
                    ),
                  )
                : GestureDetector(onTap: _openFull, child: _mapImage()),
          ),
        ),
        if (_error != null && _png != null)
          Padding(
            padding: const EdgeInsets.only(top: 6),
            child: Text('⚠ $_error · son görüntü gösteriliyor',
                style: const TextStyle(color: CryvexColors.amber, fontSize: 12)),
          ),
        const SizedBox(height: 8),
        const _Legend(),
      ]),
    );
  }
}

class _Legend extends StatelessWidget {
  const _Legend();

  @override
  Widget build(BuildContext context) {
    Widget item(Color c, String t, {bool line = false}) => Row(mainAxisSize: MainAxisSize.min, children: [
          Container(
              width: line ? 16 : 10, height: line ? 4 : 10,
              decoration: BoxDecoration(color: c, shape: line ? BoxShape.rectangle : BoxShape.circle)),
          const SizedBox(width: 5),
          Text(t, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 11.5)),
        ]);
    return Wrap(spacing: 12, runSpacing: 6, children: [
      item(const Color(0xFF00AAFF), 'Robot'),
      item(const Color(0xFFFF2D2D), 'LiDAR'),
      item(const Color(0xFFFFD500), 'Gideceği yol', line: true),
      item(CryvexColors.cyan, 'Masa'),
      item(CryvexColors.green, 'U = Üs'),
      item(CryvexColors.amber, 'K = Kapı'),
    ]);
  }
}

class _FullMapPage extends StatefulWidget {
  const _FullMapPage({required this.state});
  final _LiveMapCardState state;

  @override
  State<_FullMapPage> createState() => _FullMapPageState();
}

class _FullMapPageState extends State<_FullMapPage> {
  Timer? _t;

  @override
  void initState() {
    super.initState();
    // Kart arka planda saniyede bir yeniliyor; tam ekran sadece en son resmi gösterir.
    _t = Timer.periodic(const Duration(seconds: 1), (_) { if (mounted) setState(() {}); });
  }

  @override
  void dispose() {
    _t?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final s = widget.state;
    return Scaffold(
      appBar: AppBar(title: const Text('Canlı Harita')),
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Column(children: [
          Expanded(child: s._png == null ? const SizedBox() : s._mapImage(interactive: true)),
          const Padding(padding: EdgeInsets.all(12), child: _Legend()),
        ]),
      ),
    );
  }
}
