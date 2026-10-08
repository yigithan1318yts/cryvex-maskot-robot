import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:webview_flutter/webview_flutter.dart';
import '../state/robot_state.dart';
import '../theme.dart';
import '../widgets/password_sheet.dart';
import 'dashboard_screen.dart';

/// "Dev Ekran Modu": bu cihaz (tablet) robotun gövdesindeki büyük dokunmatik
/// ekran olur - robotun /panel sayfası (menü, sipariş, mesaj, "Devam Et",
/// ayarlar; boştayken kocaman CRYVEX) tam ekran. Robotun IP'si değişirse
/// uygulama onu yeniden bulup sayfayı yeniler; ekran kararmaz. Müşteri
/// yanlışlıkla çıkamasın: geri tuşu kapalı, çıkış = sol üst köşeye uzun bas + şifre.
const panelModePrefKey = 'panel_mode';
const _screenChannel = MethodChannel('cryvex/screen');

Future<bool> isPanelModeSaved() async =>
    (await SharedPreferences.getInstance()).getBool(panelModePrefKey) ?? false;

class PanelScreen extends StatefulWidget {
  const PanelScreen({super.key});

  @override
  State<PanelScreen> createState() => _PanelScreenState();
}

class _PanelScreenState extends State<PanelScreen> {
  late final RobotState _st;
  WebViewController? _controller;
  String? _loadedIp;
  bool _loadFailed = false;

  @override
  void initState() {
    super.initState();
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
    _screenChannel.invokeMethod('keepOn', true);
    SharedPreferences.getInstance().then((p) => p.setBool(panelModePrefKey, true));
    _st = context.read<RobotState>();
    _st.notificationsEnabled = false;   // müşteriye dönük ekranda sipariş bildirimi çıkmasın
    _st.addListener(_onRobotState);
    _st.startPolling();                 // bağlantı koparsa robotu yeniden arar (findRobot)
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_st.ip == null) _st.findRobot();
      _onRobotState();
    });
  }

  void _onRobotState() {
    final ip = _st.ip;
    if (ip == null || !mounted) return;
    // Yeni IP (robot yeniden bulundu) ya da sayfa yüklenemedi ve robot geri geldi -> yükle
    if (ip != _loadedIp || (_loadFailed && _st.isLive)) _load(ip);
  }

  void _load(String ip) {
    _loadedIp = ip;
    _loadFailed = false;
    _controller ??= WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(Colors.black)
      ..setNavigationDelegate(NavigationDelegate(
        onWebResourceError: (err) {
          if ((err.isForMainFrame ?? true) && mounted) setState(() => _loadFailed = true);
        },
      ));
    _controller!.loadRequest(Uri.parse('http://$ip:8080/panel'));
    setState(() {});
  }

  Future<void> _exit() async {
    final ok = await askPassword(context, subtitle: 'Dev Ekran Modundan çıkmak için şifre');
    if (!ok || !mounted) return;
    (await SharedPreferences.getInstance()).setBool(panelModePrefKey, false);
    if (!mounted) return;
    Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => const DashboardScreen()));
  }

  @override
  void dispose() {
    _st.removeListener(_onRobotState);
    _st.stopPolling();
    _st.notificationsEnabled = true;
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    _screenChannel.invokeMethod('keepOn', false);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final searching = context.watch<RobotState>().searching;
    return PopScope(
      canPop: false,   // müşteri geri tuşuyla çıkamasın
      child: Scaffold(
        backgroundColor: Colors.black,
        body: Stack(children: [
          if (_controller != null && !_loadFailed)
            WebViewWidget(controller: _controller!)
          else
            Center(
              child: Column(mainAxisSize: MainAxisSize.min, children: [
                const Text('CRYVEX',
                    style: TextStyle(fontSize: 64, fontWeight: FontWeight.w900, letterSpacing: 6, color: CryvexColors.cyan)),
                const SizedBox(height: 16),
                Text(searching ? 'Robot aranıyor…' : 'Robota bağlanılıyor…',
                    style: const TextStyle(color: CryvexColors.textMuted, fontSize: 16)),
              ]),
            ),
          // Gizli çıkış: sol üst köşeye uzun bas -> şifre
          Positioned(
            left: 0,
            top: 0,
            width: 72,
            height: 72,
            child: GestureDetector(behavior: HitTestBehavior.opaque, onLongPress: _exit),
          ),
        ]),
      ),
    );
  }
}
