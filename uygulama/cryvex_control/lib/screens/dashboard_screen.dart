import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../api/robot_api.dart';
import '../state/robot_state.dart';
import '../theme.dart';
import '../tts.dart';
import '../widgets/live_map.dart';
import '../widgets/password_sheet.dart';
import 'connect_screen.dart';
import 'joystick_screen.dart';
import 'otonom_screen.dart';
import 'panel_screen.dart';
import 'kurulum_screen.dart';
import 'gorev_paneli_screen.dart';

class DashboardScreen extends StatefulWidget {
  const DashboardScreen({super.key});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

// Robot yüzü renk paletleri - göz/ağız/ışık parlak, yüz (arka plan) koyu
// tonlar: açık arka planda beyaz göz akı kaybolur.
const _brightColors = <(String, String)>[
  ('Turkuaz', '#00d4ff'), ('Mavi', '#1fa2ff'), ('Lacivert', '#3b5bff'), ('Mor', '#a855ff'),
  ('Pembe', '#ff5fc8'), ('Kırmızı', '#ff3355'), ('Turuncu', '#ff8a00'), ('Sarı', '#ffd500'),
  ('Yeşil', '#00e676'), ('Beyaz', '#f2f5ff'),
];
const _faceColors = <(String, String)>[
  ('Gece', '#06060e'), ('Lacivert', '#0a1030'), ('Mor', '#1a0a2e'),
  ('Bordo', '#2a0a14'), ('Orman', '#06201a'), ('Antrasit', '#1c1c22'),
];

class _DashboardScreenState extends State<DashboardScreen> {
  int _volume = 50;
  bool _volumeLoaded = false;
  Timer? _volumeDebounce;
  Map<String, String> _theme = {};

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => context.read<RobotState>().startPolling());
    _loadVolume();
    _loadTheme();
  }

  Future<void> _loadTheme() async {
    final t = await _api.getTheme();
    if (mounted && t != null) setState(() => _theme = t);
  }

  Future<void> _setThemeColor(String key, String hex) async {
    setState(() => _theme = {..._theme, key: hex});   // aninda secili goster
    try {
      final res = await _api.setTheme({key: hex});
      final t = res['theme'];
      if (mounted && t is Map) setState(() => _theme = t.map((k, v) => MapEntry('$k', '$v')));
    } catch (_) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('❌ Renk gönderilemedi')));
      }
      _loadTheme();
    }
  }

  Future<void> _resetTheme() async {
    try {
      final res = await _api.setTheme({'reset': true});
      final t = res['theme'];
      if (mounted && t is Map) setState(() => _theme = t.map((k, v) => MapEntry('$k', '$v')));
    } catch (_) {/* baglanti yok - bir sonraki acilista yuklenir */}
  }

  Future<void> _loadVolume() async {
    final v = await _api.getVolume();
    if (mounted && v >= 0) setState(() { _volume = v; _volumeLoaded = true; });
  }

  void _onVolumeChanged(double v) {
    setState(() => _volume = v.round());
    _volumeDebounce?.cancel();
    _volumeDebounce = Timer(const Duration(milliseconds: 200), () => _api.setVolume(_volume));
  }

  @override
  void dispose() {
    context.read<RobotState>().stopPolling();
    _volumeDebounce?.cancel();
    super.dispose();
  }

  RobotApi get _api => context.read<RobotState>().api!;

  Future<void> _requireMapReady(VoidCallback action) async {
    final st = context.read<RobotState>();
    if (!st.mapReady) {
      ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('⚠ Kurulum/haritalama sürüyor, önce "Ortamı Haritala" akışını bitirin.')));
      return;
    }
    action();
  }

  Future<void> _stopPatrol() async {
    final ok = await askPassword(context, subtitle: 'Devriyeyi durdurmak için 4 haneli şifre');
    if (!ok) return;
    await _api.stopPatrol();
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Devriye durduruldu')));
    speak(_api, 'Devriyeyi durdurdum.', expr: 'happy');
  }

  void _openJoystick(JoyMode mode) async {
    final result = await Navigator.of(context).push(MaterialPageRoute(builder: (_) => JoystickScreen(mode: mode)));
    if (result == true && mounted) {
      // finish_mapping sonrasi "true" doner - kuruluma yonlendir.
      Navigator.of(context).push(MaterialPageRoute(builder: (_) => const KurulumScreen()));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<RobotState>(
      builder: (context, st, _) {
        return Scaffold(
          appBar: AppBar(
            title: const CryvexBaslik(alt: 'TECH · KONTROL'),
            actions: [
              IconButton(
                icon: const Icon(Icons.wifi_tethering),
                tooltip: st.ip ?? '',
                onPressed: () => Navigator.of(context).push(MaterialPageRoute(builder: (_) => const ConnectScreen())),
              ),
            ],
          ),
          body: SafeArea(
            child: ListView(
              padding: const EdgeInsets.all(16),
              children: [
                if (st.waitingForContinue) ...[
                  _continueCard(st),
                  const SizedBox(height: 12),
                ],
                _statusCard(st),
                const SizedBox(height: 12),
                _gorevPaneliCard(st),
                const SizedBox(height: 12),
                LiveMapCard(
                  baseUrl: st.api?.baseUrl,
                  statusLine: st.waypoint.isNotEmpty ? 'Durum: ${st.state} · ${st.waypoint}' : 'Durum: ${st.state}',
                ),
                const SizedBox(height: 12),
                if (st.orders.isNotEmpty) ...[
                  _ordersCard(st),
                  const SizedBox(height: 12),
                ],
                if (st.health.isNotEmpty) ...[
                  _healthCard(st),
                  const SizedBox(height: 12),
                ],
                _volumeCard(),
                const SizedBox(height: 18),
                _sectionLabel('Devriye'),
                _actionButton('🚀 Devriyeyi Başlat', CryvexButtonStyle.green,
                    () => _requireMapReady(() { _api.startPatrol(); speak(_api, 'Devriyeye başlıyorum.', expr: 'alert'); })),
                _actionButton('🚪 Karşılama (Kapıda, 3 dk)', CryvexButtonStyle.cyan,
                    () => _requireMapReady(() { _api.greetDoor(); speak(_api, 'Kapıda karşılamaya gidiyorum.', expr: 'love'); })),
                _actionButton('🎈 Sosyalleşme (Gezinme)', CryvexButtonStyle.cyan,
                    () => _requireMapReady(() { _api.wander(); speak(_api, 'Kafede geziniyorum.', expr: 'happy'); })),
                _actionButton('🏠 Üsse Dön', CryvexButtonStyle.cyan,
                    () => _requireMapReady(() { _api.goHome(); speak(_api, 'Üsse dönüyorum.'); })),
                _actionButton('🔒 Devriyeyi Durdur', CryvexButtonStyle.red, _stopPatrol),
                const SizedBox(height: 18),
                _sectionLabel('Manuel'),
                _actionButton('🆘 Kurtar (Manuel Sürüş)', CryvexButtonStyle.amber,
                    () => _openJoystick(JoyMode.rescue)),
                _actionButton('🕹️ Kontrol Sende', CryvexButtonStyle.cyan,
                    () => _openJoystick(JoyMode.control)),
                const SizedBox(height: 18),
                _sectionLabel('Otonom'),
                _actionButton('🤖 Otonom Sürüş (Keşfet · Devriye · Kamera)', CryvexButtonStyle.green,
                    () => Navigator.of(context).push(MaterialPageRoute(builder: (_) => const OtonomScreen()))),
                const SizedBox(height: 18),
                _sectionLabel('Kurulum'),
                _actionButton('🗺️ Ortamı Haritala (Yön tuşlarıyla)', CryvexButtonStyle.cyan,
                    () => _openJoystick(JoyMode.map)),
                _actionButton('🛠️ Harita Kurulumu (Masa · Üs · Kapı · Konum · Fırça)', CryvexButtonStyle.gray,
                    () => Navigator.of(context).push(MaterialPageRoute(builder: (_) => const KurulumScreen()))),
                _actionButton('📺 Bu Cihazı Robotun Dev Ekranı Yap', CryvexButtonStyle.gray, _startPanelMode),
                const SizedBox(height: 18),
                _sectionLabel('🎨 Robotun Görünümü'),
                _themeCard(),
              ],
            ),
          ),
        );
      },
    );
  }

  /// Yeni Gorev Paneli (robotta :8090) + ACIL DUR. Panelde: canli kamera, HOME -> Masa 1 -> HOME, HOME'a don (lidarla),
  /// kisa devriye, hareketsiz kontrol, canli gorev logu, guvenlik ayarlari.
  Widget _gorevPaneliCard(RobotState st) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(18),
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [CryvexColors.cyanDeep.withValues(alpha: 0.35), CryvexColors.bg],
        ),
        border: Border.all(color: CryvexColors.cyan.withValues(alpha: 0.35)),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        Row(children: [
          Image.asset('assets/cryvex_x.png', width: 40, height: 40),
          const SizedBox(width: 12),
          const Expanded(
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Text('Görev Paneli',
                  style: TextStyle(fontSize: 17, fontWeight: FontWeight.w800, color: CryvexColors.textPrimary)),
              SizedBox(height: 2),
              Text('Canlı kamera · HOME → Masa 1 → HOME · HOME\'a dön · güvenlik',
                  style: TextStyle(fontSize: 12, color: CryvexColors.textMuted)),
            ]),
          ),
        ]),
        const SizedBox(height: 14),
        Row(children: [
          Expanded(
            child: ElevatedButton(
              style: CryvexButtonStyle.cyan,
              onPressed: st.ip == null
                  ? null
                  : () => Navigator.of(context).push(MaterialPageRoute(builder: (_) => const GorevPaneliScreen())),
              child: const Text('🎯 Paneli Aç'),
            ),
          ),
          const SizedBox(width: 10),
          Expanded(
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: CryvexColors.red,
                foregroundColor: Colors.white,
                padding: const EdgeInsets.symmetric(vertical: 16),
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
                textStyle: const TextStyle(fontSize: 15, fontWeight: FontWeight.w900, letterSpacing: 1),
              ),
              onPressed: st.ip == null
                  ? null
                  : () async {
                      final (ok, mesaj) = await acilDur(st.ip!);
                      if (!mounted) return;
                      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                        backgroundColor: ok ? CryvexColors.red : CryvexColors.amber,
                        content: Text('ACİL DUR: $mesaj'),
                      ));
                    },
              child: const Text('⛔ ACİL DUR'),
            ),
          ),
        ]),
      ]),
    );
  }

  Widget _sectionLabel(String s) => Padding(
        padding: const EdgeInsets.only(bottom: 8, top: 4),
        child: Text(s.toUpperCase(),
            style: const TextStyle(color: CryvexColors.textMuted, fontSize: 11, letterSpacing: 1.2, fontWeight: FontWeight.w600)),
      );

  Widget _actionButton(String label, ButtonStyle style, VoidCallback onTap) => Padding(
        padding: const EdgeInsets.only(bottom: 10),
        child: SizedBox(width: double.infinity, child: ElevatedButton(style: style, onPressed: onTap, child: Text(label))),
      );

  Widget _volumeCard() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: CryvexColors.cardLine),
      ),
      child: Row(children: [
        const Icon(Icons.volume_up, color: CryvexColors.textMuted, size: 20),
        Expanded(
          child: Slider(
            value: _volume.toDouble().clamp(0, 100),
            min: 0, max: 100,
            activeColor: CryvexColors.cyan,
            onChanged: _volumeLoaded ? _onVolumeChanged : null,
          ),
        ),
        SizedBox(width: 38, child: Text('$_volume%', textAlign: TextAlign.right,
            style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12.5))),
      ]),
    );
  }

  Future<void> _startPanelMode() async {
    final sure = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Dev Ekran Modu'),
        content: const Text(
            'Bu cihaz robotun gövdesindeki büyük dokunmatik ekran olur: menü, sipariş, mesaj ve '
            'ayarlar burada açılır, boştayken kocaman CRYVEX yazar.\n\n'
            'Uygulama bundan sonra hep bu modda açılır. Çıkmak için sol üst köşeye uzun basıp şifre girin.'),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Vazgeç')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(true), child: const Text('Başlat')),
        ],
      ),
    );
    if (sure != true || !mounted) return;
    Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => const PanelScreen()));
  }

  void _snack(String text) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(text)));
  }

  // Robot barmende/masada "Devam" bekliyor: robot ekranındakiyle aynı kocaman buton.
  Widget _continueCard(RobotState st) {
    final (title, body, button) = waitStates[st.state]!;
    final table = st.deliveryTable.isNotEmpty ? ' · ${st.deliveryTable}' : '';
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: CryvexColors.cyan.withValues(alpha: 0.08),
        borderRadius: BorderRadius.circular(18),
        border: Border.all(color: CryvexColors.cyan, width: 1.5),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        Text('$title$table', style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w800)),
        const SizedBox(height: 4),
        Text(body, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 13)),
        const SizedBox(height: 4),
        const Text('Basılmadan robot yerinden kıpırdamaz.',
            style: TextStyle(color: CryvexColors.amber, fontSize: 12)),
        const SizedBox(height: 12),
        SizedBox(
          height: 72,
          child: ElevatedButton(
            style: CryvexButtonStyle.cyan,
            onPressed: () async {
              try {
                final res = await _api.continueRobot();
                _snack(res['result'] == 'ok' ? '▶ Robot devam ediyor' : '⚠ ${res['reason'] ?? ''}');
              } catch (_) {
                _snack('❌ Robota ulaşılamadı');
              }
            },
            child: Text(button, textAlign: TextAlign.center,
                style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w800)),
          ),
        ),
      ]),
    );
  }

  // Siparişler: robot ekranından verilenler. Barmen hazırlayınca "Hazır" ->
  // robot barmenden alıp siparişin masasına götürür.
  Widget _ordersCard(RobotState st) {
    final active = st.orders
        .where((o) => o['status'] == 'preparing' || o['status'] == 'ready' || o['status'] == 'delivering')
        .toList();
    final done = st.orders.where((o) => o['status'] == 'delivered' || o['status'] == 'cancelled').take(3).toList();
    final nowServer = DateTime.now().millisecondsSinceEpoch / 1000 + st.serverClockOffset;
    String ago(Map<String, dynamic> o) {
      final min = ((nowServer - ((o['created'] ?? nowServer) as num)) / 60).floor();
      if (min < 1) return 'az önce';
      if (min < 60) return '$min dk önce';
      return '${min ~/ 60} sa önce';
    }

    return Container(
      padding: const EdgeInsets.fromLTRB(16, 14, 16, 10),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: active.isNotEmpty ? CryvexColors.amber.withValues(alpha: 0.6) : CryvexColors.cardLine),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
        Row(children: [
          const Text('🧾 Siparişler', style: TextStyle(fontWeight: FontWeight.w700, fontSize: 14.5)),
          const Spacer(),
          Text(active.isEmpty ? 'bekleyen yok' : '${active.length} bekliyor',
              style: TextStyle(color: active.isEmpty ? CryvexColors.textMuted : CryvexColors.amber, fontSize: 12.5)),
        ]),
        for (final o in active) ...[
          const SizedBox(height: 10),
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.black.withValues(alpha: 0.25),
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: CryvexColors.cardLine),
            ),
            child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
              Row(children: [
                Text('${o['table']}', style: const TextStyle(fontWeight: FontWeight.w800, fontSize: 15)),
                Text('  · #${o['id']} · ${ago(o)}', style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12)),
                const Spacer(),
                Text('${formatPrice(o['total'])} ₺',
                    style: const TextStyle(color: CryvexColors.green, fontWeight: FontWeight.w700)),
              ]),
              const SizedBox(height: 4),
              Text(orderItemsText(o), style: const TextStyle(fontSize: 13.5)),
              const SizedBox(height: 10),
              if (o['status'] == 'preparing')
                Row(children: [
                  Expanded(
                    child: ElevatedButton(
                      style: CryvexButtonStyle.green,
                      onPressed: () => _orderReady(o),
                      child: const Text('✅ Hazır – Robot Götürsün'),
                    ),
                  ),
                  TextButton(onPressed: () => _orderCancel(o), child: const Text('İptal')),
                ])
              else if (o['status'] == 'ready')
                // Robot müşteriyle / başka teslimatla ilgileniyor: sıraya aldı,
                // o işten çıkar çıkmaz gelecek (patrol.py önceliği teslimat).
                Row(children: [
                  const Expanded(
                    child: Text('⏳ Sırada – robot işini bitirince gelecek',
                        style: TextStyle(color: CryvexColors.amber, fontWeight: FontWeight.w600)),
                  ),
                  TextButton(onPressed: () => _orderCancel(o), child: const Text('İptal')),
                ])
              else
                const Text('🚚 Robot götürüyor', style: TextStyle(color: CryvexColors.cyan, fontWeight: FontWeight.w600)),
            ]),
          ),
        ],
        if (done.isNotEmpty) const SizedBox(height: 8),
        for (final o in done)
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 3),
            child: Text(
              '${o['status'] == 'delivered' ? '✓ teslim edildi' : '✕ iptal'} · ${o['table']} · ${orderItemsText(o)}',
              style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
            ),
          ),
      ]),
    );
  }

  Future<void> _orderReady(Map<String, dynamic> o) async {
    try {
      final res = await _api.orderReady(o['id'] as int);
      if (res['result'] != 'ok') {
        _snack('⚠ ${res['reason'] ?? 'gönderilemedi'}');
      } else if (res['coming_now'] == true) {
        _snack('🚚 Robot ${o['table']} siparişi için barmene geliyor');
      } else {
        _snack('⏳ Sıraya alındı: robot elindeki işi bitirince hemen gelecek');
      }
    } catch (_) {
      _snack('❌ Robota ulaşılamadı');
    }
    if (mounted) context.read<RobotState>().refreshOrders();
  }

  Future<void> _orderCancel(Map<String, dynamic> o) async {
    final sure = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text('${o['table']} siparişi iptal edilsin mi?'),
        content: Text(orderItemsText(o)),
        actions: [
          TextButton(onPressed: () => Navigator.of(ctx).pop(false), child: const Text('Vazgeç')),
          TextButton(onPressed: () => Navigator.of(ctx).pop(true), child: const Text('İptal Et')),
        ],
      ),
    );
    if (sure != true) return;
    try {
      final res = await _api.orderCancel(o['id'] as int);
      _snack(res['result'] == 'ok' ? 'Sipariş iptal edildi' : '⚠ ${res['reason'] ?? ''}');
    } catch (_) {
      _snack('❌ Robota ulaşılamadı');
    }
    if (mounted) context.read<RobotState>().refreshOrders();
  }

  // Robot Sağlığı: sunucunun ölçtüğü her parça için yeşil (iyi) / kırmızı
  // (sorun) / gri (bilgi yok) - sahada "neden gitmiyor?" sorusunun cevabı.
  static const _healthRows = [
    ('brain', '🧠', 'Devriye beyni'),
    ('stm32', '🔌', 'STM32 (motor kartı)'),
    ('lidar', '📡', 'LiDAR'),
    ('estop', '🛑', 'Acil stop'),
    ('bumper', '🧱', 'Tampon'),
    ('battery', '🔋', 'Batarya'),
    ('localization', '📍', 'Konum'),
  ];

  Widget _healthCard(RobotState st) {
    Color dot(dynamic ok) => ok == true
        ? CryvexColors.green
        : ok == false
            ? CryvexColors.red
            : CryvexColors.textMuted;
    final summary = st.health['summary'] as Map? ?? {};
    final allOk = summary['ok'] == true;
    return Container(
      padding: const EdgeInsets.fromLTRB(16, 14, 16, 10),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: allOk ? CryvexColors.cardLine : CryvexColors.red.withValues(alpha: 0.5)),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Row(children: [
          const Text('🩺 Robot Sağlığı', style: TextStyle(fontWeight: FontWeight.w700, fontSize: 14.5)),
          const Spacer(),
          Flexible(
            child: Text('${summary['text'] ?? ''}',
                textAlign: TextAlign.right,
                style: TextStyle(color: allOk ? CryvexColors.green : CryvexColors.red, fontSize: 12.5,
                    fontWeight: FontWeight.w600)),
          ),
        ]),
        const SizedBox(height: 8),
        for (final (key, icon, label) in _healthRows)
          if (st.health[key] is Map)
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 4),
              child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Padding(
                  padding: const EdgeInsets.only(top: 5),
                  child: Container(width: 9, height: 9,
                      decoration: BoxDecoration(color: dot(st.health[key]['ok']), shape: BoxShape.circle)),
                ),
                const SizedBox(width: 10),
                SizedBox(width: 150, child: Text('$icon $label', style: const TextStyle(fontSize: 13))),
                Expanded(
                  child: Text('${st.health[key]['text']}',
                      style: TextStyle(
                          fontSize: 13,
                          color: st.health[key]['ok'] == false ? CryvexColors.red : CryvexColors.textMuted)),
                ),
              ]),
            ),
      ]),
    );
  }

  Widget _themeCard() {
    Widget row(String label, String key, List<(String, String)> palette) {
      final current = (_theme[key] ?? '').toLowerCase();
      return Padding(
        padding: const EdgeInsets.only(bottom: 12),
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Text(label, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12.5)),
          const SizedBox(height: 6),
          Wrap(spacing: 8, runSpacing: 8, children: [
            for (final (name, hex) in palette)
              Tooltip(
                message: name,
                child: GestureDetector(
                  onTap: () => _setThemeColor(key, hex),
                  child: Container(
                    width: 32,
                    height: 32,
                    decoration: BoxDecoration(
                      color: Color(int.parse('FF${hex.substring(1)}', radix: 16)),
                      shape: BoxShape.circle,
                      border: Border.all(
                        color: current == hex ? Colors.white : Colors.white24,
                        width: current == hex ? 3 : 1,
                      ),
                    ),
                  ),
                ),
              ),
          ]),
        ]),
      );
    }

    return Container(
      padding: const EdgeInsets.fromLTRB(16, 14, 16, 4),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: CryvexColors.cardLine),
      ),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        row('👁 Göz', 'eye', _brightColors),
        row('👄 Ağız', 'mouth', _brightColors),
        row('✨ Yürüyen ışık', 'light', _brightColors),
        row('🙂 Yüz (arka plan)', 'face', _faceColors),
        Align(
          alignment: Alignment.centerRight,
          child: TextButton(onPressed: _resetTheme, child: const Text('↺ Varsayılan renkler')),
        ),
      ]),
    );
  }

  Widget _statusCard(RobotState st) {
    final dotColor = st.isLive ? CryvexColors.green : CryvexColors.amber;
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: CryvexColors.card,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: CryvexColors.cardLine),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(children: [
            Container(width: 10, height: 10, decoration: BoxDecoration(color: dotColor, shape: BoxShape.circle)),
            const SizedBox(width: 8),
            Text(
                st.isLive
                    ? 'Robot bağlı · ${st.ip}'
                    : (st.searching ? 'Robot ağda aranıyor…' : 'Robota ulaşılamıyor…'),
                style: const TextStyle(color: CryvexColors.textMuted, fontSize: 13)),
          ]),
          const SizedBox(height: 8),
          Row(children: [
            Container(
                width: 10, height: 10,
                decoration: BoxDecoration(color: st.configured ? CryvexColors.green : CryvexColors.amber, shape: BoxShape.circle)),
            const SizedBox(width: 8),
            Text(st.configured ? 'Kurulum tamam · masalar tanımlı' : 'Kurulum eksik · henüz masa/harita yok',
                style: const TextStyle(color: CryvexColors.textMuted, fontSize: 13)),
          ]),
          const Divider(height: 24, color: CryvexColors.cardLine),
          Text('Durum: ${st.state}', style: const TextStyle(color: CryvexColors.textPrimary, fontWeight: FontWeight.w600)),
          if (st.waypoint.isNotEmpty) Text(st.waypoint, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 12.5)),
          if (st.rescueActive)
            const Padding(
              padding: EdgeInsets.only(top: 6),
              child: Text('🆘 Kurtarma modu aktif', style: TextStyle(color: CryvexColors.amber, fontWeight: FontWeight.w600)),
            ),
        ],
      ),
    );
  }
}
