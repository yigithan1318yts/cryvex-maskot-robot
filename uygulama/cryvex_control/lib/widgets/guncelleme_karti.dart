import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:package_info_plus/package_info_plus.dart';
import 'package:url_launcher/url_launcher.dart';
import '../theme.dart';

/// Uygulama ici guncelleme: robottaki Gorev Paneli (:8090/api/surum) en son surumu soyler. Telefondaki surumden yeniyse
/// "Guncelle" karti cikar; basinca yeni APK tarayicida iner, Android kurulum ekranini acar.
/// Yeni surum yayinlamak: APK'yi robota ~/cryvex_araclar/panel/cryvex.apk olarak koy, panel/surum.json'da versionCode'u artir.
class GuncellemeKarti extends StatefulWidget {
  const GuncellemeKarti({super.key, required this.ip});
  final String? ip;

  @override
  State<GuncellemeKarti> createState() => _GuncellemeKartiState();
}

class _GuncellemeKartiState extends State<GuncellemeKarti> {
  Map<String, dynamic>? _yeni;
  String? _bakilanIp;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _kontrol();
  }

  @override
  void didUpdateWidget(covariant GuncellemeKarti old) {
    super.didUpdateWidget(old);
    _kontrol();
  }

  Future<void> _kontrol() async {
    final ip = widget.ip;
    if (ip == null || ip == _bakilanIp) return;
    _bakilanIp = ip;
    try {
      final r = await http.get(Uri.parse('http://$ip:8090/api/surum')).timeout(const Duration(seconds: 5));
      final s = jsonDecode(r.body) as Map<String, dynamic>;
      final bu = int.tryParse((await PackageInfo.fromPlatform()).buildNumber) ?? 0;
      if (((s['versionCode'] ?? 0) as num) > bu && mounted) setState(() => _yeni = s);
    } catch (_) {}
  }

  Future<void> _guncelle() async {
    final url = Uri.parse('http://${widget.ip}:8090${_yeni!['apk'] ?? '/cryvex.apk'}');
    final ok = await launchUrl(url, mode: LaunchMode.externalApplication);
    if (!ok && mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Tarayıcı açılamadı. Elle aç: $url')));
    }
  }

  @override
  Widget build(BuildContext context) {
    final y = _yeni;
    if (y == null) return const SizedBox.shrink();
    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        color: CryvexColors.green.withValues(alpha: 0.10),
        border: Border.all(color: CryvexColors.green.withValues(alpha: 0.45)),
      ),
      child: Row(children: [
        const Icon(Icons.system_update, color: CryvexColors.green, size: 30),
        const SizedBox(width: 12),
        Expanded(
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text('Yeni sürüm hazır: ${y['versionName'] ?? ''}',
                style: const TextStyle(fontWeight: FontWeight.w800, color: CryvexColors.textPrimary)),
            if (y['notlar'] != null)
              Text(y['notlar'].toString(), style: const TextStyle(fontSize: 12, color: CryvexColors.textMuted)),
          ]),
        ),
        const SizedBox(width: 8),
        ElevatedButton(style: CryvexButtonStyle.green, onPressed: _guncelle, child: const Text('Güncelle')),
      ]),
    );
  }
}
