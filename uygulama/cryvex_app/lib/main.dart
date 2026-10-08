// CryvexTech kafe robotu - telefon uygulamasi.
// Robottaki Gorev Paneli'ni (http://<robot>:8090) tam ekran acar: canli kamera, gorevler, DUR.
// Ilk acilista robotun IP adresini sorar, baglanabildigini kontrol eder ve hatirlar.
import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';
import 'package:webview_flutter/webview_flutter.dart';

const Color kZemin = Color(0xFF0B0F15);
const Color kKart = Color(0xFF15212B);
const Color kCizgi = Color(0xFF1D2630);
const Color kVurgu = Color(0xFF26CCEA);
const Color kVurgu2 = Color(0xFF0D7F96);
const Color kMetin = Color(0xFFE9EFF6);
const Color kSoluk = Color(0xFF9FB0C2);
const Color kKirmizi = Color(0xFFFF6B6B);
const int kPort = 8090;
const String kVarsayilanIp = '192.168.1.8';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
    statusBarColor: kZemin,
    statusBarIconBrightness: Brightness.light,
    systemNavigationBarColor: kZemin,
  ));
  runApp(const CryvexUygulama());
}

class CryvexUygulama extends StatelessWidget {
  const CryvexUygulama({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Cryvex',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        brightness: Brightness.dark,
        scaffoldBackgroundColor: kZemin,
        colorScheme: const ColorScheme.dark(primary: kVurgu, secondary: kVurgu2, surface: kKart, error: kKirmizi),
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: kKart,
          border: OutlineInputBorder(borderRadius: BorderRadius.circular(14), borderSide: const BorderSide(color: kCizgi)),
          enabledBorder:
              OutlineInputBorder(borderRadius: BorderRadius.circular(14), borderSide: const BorderSide(color: kCizgi)),
          focusedBorder:
              OutlineInputBorder(borderRadius: BorderRadius.circular(14), borderSide: const BorderSide(color: kVurgu)),
        ),
      ),
      home: const Baslangic(),
    );
  }
}

/// Robota ulasilabiliyor mu? (panelin durum adresi)
Future<bool> robotaUlas(String ip) async {
  try {
    final r = await http.get(Uri.parse('http://$ip:$kPort/api/durum')).timeout(const Duration(seconds: 4));
    return r.statusCode == 200;
  } catch (_) {
    return false;
  }
}

/// Acilis: kayitli IP varsa dogrudan baglanmayi dener, yoksa IP ekranini acar.
class Baslangic extends StatefulWidget {
  const Baslangic({super.key});

  @override
  State<Baslangic> createState() => _BaslangicState();
}

class _BaslangicState extends State<Baslangic> {
  @override
  void initState() {
    super.initState();
    _ac();
  }

  Future<void> _ac() async {
    final tercih = await SharedPreferences.getInstance();
    final ip = tercih.getString('robot_ip');
    if (!mounted) return;
    if (ip != null && await robotaUlas(ip)) {
      if (!mounted) return;
      Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => PanelEkrani(ip: ip)));
    } else {
      if (!mounted) return;
      Navigator.of(context).pushReplacement(MaterialPageRoute(
          builder: (_) => IpEkrani(baslangicIp: ip ?? kVarsayilanIp, hata: ip == null ? null : 'Robota ulaşılamadı ($ip)')));
    }
  }

  @override
  Widget build(BuildContext context) {
    return const Scaffold(
      body: Center(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Logo(boyut: 120),
          SizedBox(height: 28),
          SizedBox(width: 26, height: 26, child: CircularProgressIndicator(strokeWidth: 2.5, color: kVurgu)),
          SizedBox(height: 14),
          Text('Robota bağlanılıyor…', style: TextStyle(color: kSoluk)),
        ]),
      ),
    );
  }
}

class Logo extends StatelessWidget {
  const Logo({super.key, this.boyut = 96});
  final double boyut;

  @override
  Widget build(BuildContext context) {
    return Column(mainAxisSize: MainAxisSize.min, children: [
      Image.asset('assets/x.png', width: boyut, height: boyut),
      const SizedBox(height: 10),
      const Text('CRYVEX',
          style: TextStyle(fontSize: 26, fontWeight: FontWeight.w800, letterSpacing: 9, color: kMetin)),
      const Text('TECH', style: TextStyle(fontSize: 12, letterSpacing: 8, color: kSoluk)),
    ]);
  }
}

/// Robot IP adresi ekrani
class IpEkrani extends StatefulWidget {
  const IpEkrani({super.key, required this.baslangicIp, this.hata});
  final String baslangicIp;
  final String? hata;

  @override
  State<IpEkrani> createState() => _IpEkraniState();
}

class _IpEkraniState extends State<IpEkrani> {
  late final TextEditingController _ip = TextEditingController(text: widget.baslangicIp);
  bool _deniyor = false;
  String? _hata;

  @override
  void initState() {
    super.initState();
    _hata = widget.hata;
  }

  Future<void> _baglan() async {
    final ip = _ip.text.trim();
    if (!RegExp(r'^\d{1,3}(\.\d{1,3}){3}$').hasMatch(ip)) {
      setState(() => _hata = 'Geçerli bir IP adresi yaz (örn. 192.168.1.8)');
      return;
    }
    setState(() {
      _deniyor = true;
      _hata = null;
    });
    final ok = await robotaUlas(ip);
    if (!mounted) return;
    if (ok) {
      final tercih = await SharedPreferences.getInstance();
      await tercih.setString('robot_ip', ip);
      if (!mounted) return;
      Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => PanelEkrani(ip: ip)));
    } else {
      setState(() {
        _deniyor = false;
        _hata = 'Robota ulaşılamadı ($ip:$kPort). Telefon robotla aynı Wi-Fi ağında mı? Robot açık mı?';
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Center(
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(24),
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 420),
              child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
                const Logo(),
                const SizedBox(height: 36),
                const Text('Robotun IP adresi', style: TextStyle(color: kSoluk, fontSize: 13, letterSpacing: 1.2)),
                const SizedBox(height: 8),
                TextField(
                  controller: _ip,
                  keyboardType: const TextInputType.numberWithOptions(decimal: true),
                  style: const TextStyle(fontSize: 20, letterSpacing: 1.5),
                  decoration: const InputDecoration(hintText: '192.168.1.8', suffixText: ':$kPort'),
                  onSubmitted: (_) => _baglan(),
                ),
                if (_hata != null) ...[
                  const SizedBox(height: 12),
                  Text(_hata!, style: const TextStyle(color: kKirmizi, height: 1.4)),
                ],
                const SizedBox(height: 20),
                SizedBox(
                  height: 54,
                  child: FilledButton(
                    style: FilledButton.styleFrom(
                        backgroundColor: kVurgu,
                        foregroundColor: kZemin,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14))),
                    onPressed: _deniyor ? null : _baglan,
                    child: _deniyor
                        ? const SizedBox(
                            width: 22, height: 22, child: CircularProgressIndicator(strokeWidth: 2.5, color: kZemin))
                        : const Text('BAĞLAN', style: TextStyle(fontWeight: FontWeight.w800, letterSpacing: 2)),
                  ),
                ),
                const SizedBox(height: 18),
                const Text(
                  'Telefon robotla aynı Wi-Fi ağında olmalı. Robotun IP adresi değişirse buradan güncelleyebilirsin '
                  '(panelde sol üstteki logoya uzun bas).',
                  textAlign: TextAlign.center,
                  style: TextStyle(color: kSoluk, fontSize: 12.5, height: 1.5),
                ),
              ]),
            ),
          ),
        ),
      ),
    );
  }
}

/// Gorev Paneli (WebView, tam ekran)
class PanelEkrani extends StatefulWidget {
  const PanelEkrani({super.key, required this.ip});
  final String ip;

  @override
  State<PanelEkrani> createState() => _PanelEkraniState();
}

class _PanelEkraniState extends State<PanelEkrani> {
  late final WebViewController _web;
  bool _yukleniyor = true;
  String? _hata;

  String get _adres => 'http://${widget.ip}:$kPort/';

  @override
  void initState() {
    super.initState();
    _web = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..setBackgroundColor(kZemin)
      ..setNavigationDelegate(NavigationDelegate(
        onPageStarted: (_) => setState(() => _yukleniyor = true),
        onPageFinished: (_) => setState(() => _yukleniyor = false),
        onWebResourceError: (e) {
          if (e.isForMainFrame ?? true) {
            setState(() {
              _yukleniyor = false;
              _hata = 'Robota bağlanılamadı.\n${e.description}';
            });
          }
        },
      ))
      ..loadRequest(Uri.parse(_adres));
  }

  void _yenile() {
    setState(() => _hata = null);
    _web.loadRequest(Uri.parse(_adres));
  }

  void _ipDegistir() {
    Navigator.of(context)
        .pushReplacement(MaterialPageRoute(builder: (_) => IpEkrani(baslangicIp: widget.ip)));
  }

  @override
  Widget build(BuildContext context) {
    return PopScope(
      canPop: false,
      onPopInvokedWithResult: (didPop, _) async {
        if (didPop) return;
        if (await _web.canGoBack()) {
          await _web.goBack();
        } else {
          SystemNavigator.pop();
        }
      },
      child: Scaffold(
        body: SafeArea(
          child: Stack(children: [
            if (_hata == null) WebViewWidget(controller: _web),
            // logo alanina uzun basinca IP degistirme (panelin kendi logosu sol ustte)
            Positioned(
              left: 0,
              top: 0,
              width: 72,
              height: 56,
              child: GestureDetector(behavior: HitTestBehavior.translucent, onLongPress: _ipDegistir),
            ),
            if (_yukleniyor && _hata == null)
              const Positioned(top: 0, left: 0, right: 0, child: LinearProgressIndicator(color: kVurgu, minHeight: 2)),
            if (_hata != null)
              Center(
                child: Padding(
                  padding: const EdgeInsets.all(28),
                  child: Column(mainAxisSize: MainAxisSize.min, children: [
                    const Logo(boyut: 84),
                    const SizedBox(height: 24),
                    Text(_hata!, textAlign: TextAlign.center, style: const TextStyle(color: kKirmizi, height: 1.5)),
                    const SizedBox(height: 22),
                    Wrap(spacing: 12, runSpacing: 12, alignment: WrapAlignment.center, children: [
                      FilledButton.icon(
                        style: FilledButton.styleFrom(backgroundColor: kVurgu, foregroundColor: kZemin),
                        onPressed: _yenile,
                        icon: const Icon(Icons.refresh),
                        label: const Text('Tekrar dene'),
                      ),
                      OutlinedButton.icon(
                        onPressed: _ipDegistir,
                        icon: const Icon(Icons.settings_ethernet),
                        label: const Text('IP değiştir'),
                      ),
                    ]),
                  ]),
                ),
              ),
          ]),
        ),
      ),
    );
  }
}
