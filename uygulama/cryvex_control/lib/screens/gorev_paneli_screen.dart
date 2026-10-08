import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import 'package:webview_flutter/webview_flutter.dart';
import '../state/robot_state.dart';
import '../theme.dart';

/// Robottaki yeni "Gorev Paneli" (cryvex_araclar/panel, port 8090): canli kamera + harita, HOME -> Masa 1 -> HOME,
/// HOME'a don (lidarla), kisa devriye, hareketsiz kontrol, DUR, canli gorev logu, guvenlik ayarlari.
/// Panel robotun kendi sayfasi oldugu icin robottaki her guncelleme uygulama guncellenmeden buraya gelir.
const gorevPaneliPort = 8090;

String gorevPaneliUrl(String ip) => 'http://$ip:$gorevPaneliPort/';

/// ACIL DUR: robottaki TUM hareket programlarini kapatir + sifir hiz (sifresiz). Basarili mi + mesaj.
Future<(bool, String)> acilDur(String ip) async {
  try {
    final r = await http.post(Uri.parse('http://$ip:$gorevPaneliPort/api/dur')).timeout(const Duration(seconds: 5));
    final m = jsonDecode(r.body) as Map<String, dynamic>;
    return (m['ok'] == true, (m['mesaj'] ?? 'durduruldu').toString());
  } catch (e) {
    return (false, 'Robota ulaşılamadı: $e');
  }
}

class GorevPaneliScreen extends StatefulWidget {
  const GorevPaneliScreen({super.key});

  @override
  State<GorevPaneliScreen> createState() => _GorevPaneliScreenState();
}

class _GorevPaneliScreenState extends State<GorevPaneliScreen> {
  WebViewController? _web;
  String? _ip;
  bool _yukleniyor = true;
  String? _hata;

  @override
  void initState() {
    super.initState();
    _ip = context.read<RobotState>().ip;
    if (_ip == null) {
      _hata = 'Robot bağlı değil. Önce robota bağlan.';
      _yukleniyor = false;
      return;
    }
    _web = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(CryvexColors.bg)
      ..setNavigationDelegate(NavigationDelegate(
        onPageStarted: (_) => mounted ? setState(() => _yukleniyor = true) : null,
        onPageFinished: (_) => mounted ? setState(() => _yukleniyor = false) : null,
        onWebResourceError: (e) {
          if ((e.isForMainFrame ?? true) && mounted) {
            setState(() {
              _yukleniyor = false;
              _hata = 'Görev Paneli açılamadı (${gorevPaneliUrl(_ip!)}).\n'
                  'Robot açık mı, telefon aynı Wi-Fi ağında mı?';
            });
          }
        },
      ))
      ..loadRequest(Uri.parse(gorevPaneliUrl(_ip!)));
  }

  void _yenile() {
    if (_web == null) return;
    setState(() => _hata = null);
    _web!.loadRequest(Uri.parse(gorevPaneliUrl(_ip!)));
  }

  Future<void> _dur() async {
    if (_ip == null) return;
    final (ok, mesaj) = await acilDur(_ip!);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      backgroundColor: ok ? CryvexColors.red : CryvexColors.amber,
      content: Text('DUR: $mesaj'),
    ));
  }

  @override
  Widget build(BuildContext context) {
    return PopScope(
      canPop: false,
      onPopInvokedWithResult: (didPop, _) async {
        if (didPop) return;
        if (_web != null && _hata == null && await _web!.canGoBack()) {
          await _web!.goBack();
        } else if (context.mounted) {
          Navigator.of(context).pop();
        }
      },
      child: Scaffold(
        appBar: AppBar(
          title: const CryvexBaslik(alt: 'GÖREV PANELİ'),
          actions: [
            IconButton(icon: const Icon(Icons.refresh), tooltip: 'Yenile', onPressed: _yenile),
            Padding(
              padding: const EdgeInsets.only(right: 8),
              child: FilledButton(
                style: FilledButton.styleFrom(backgroundColor: CryvexColors.red, foregroundColor: Colors.white),
                onPressed: _dur,
                child: const Text('DUR', style: TextStyle(fontWeight: FontWeight.w900, letterSpacing: 1.5)),
              ),
            ),
          ],
          bottom: _yukleniyor
              ? const PreferredSize(
                  preferredSize: Size.fromHeight(2),
                  child: LinearProgressIndicator(minHeight: 2, color: CryvexColors.cyan))
              : null,
        ),
        body: _hata == null && _web != null
            ? WebViewWidget(controller: _web!)
            : Center(
                child: Padding(
                  padding: const EdgeInsets.all(28),
                  child: Column(mainAxisSize: MainAxisSize.min, children: [
                    Image.asset('assets/cryvex_x.png', width: 84, height: 84),
                    const SizedBox(height: 20),
                    Text(_hata ?? '', textAlign: TextAlign.center,
                        style: const TextStyle(color: CryvexColors.red, height: 1.5)),
                    const SizedBox(height: 20),
                    if (_web != null)
                      ElevatedButton.icon(
                          style: CryvexButtonStyle.cyan,
                          onPressed: _yenile,
                          icon: const Icon(Icons.refresh),
                          label: const Text('Tekrar dene')),
                  ]),
                ),
              ),
      ),
    );
  }
}
