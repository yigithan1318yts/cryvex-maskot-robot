import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../state/robot_state.dart';
import '../theme.dart';
import 'dashboard_screen.dart';

/// Robotu ağda KENDİSİ bulur (discovery.dart) - IP girmek sadece yedek:
/// açılır açılmaz arar, bulursa doğrudan Ana Panel'e geçer.
class ConnectScreen extends StatefulWidget {
  const ConnectScreen({super.key});

  @override
  State<ConnectScreen> createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> {
  final _ctrl = TextEditingController();
  bool _checking = false;
  String? _error;

  @override
  void initState() {
    super.initState();
    final st = context.read<RobotState>();
    if (st.ip != null) _ctrl.text = st.ip!;
    WidgetsBinding.instance.addPostFrameCallback((_) => _search());
  }

  Future<void> _search() async {
    setState(() => _error = null);
    final st = context.read<RobotState>();
    final ok = await st.findRobot();
    if (!mounted) return;
    if (ok) {
      Navigator.of(context).pushReplacement(MaterialPageRoute(builder: (_) => const DashboardScreen()));
    } else {
      setState(() => _error = 'Robot bulunamadı: robot açık mı, telefon robotla aynı WiFi\'de mi?');
    }
  }

  Future<void> _connect() async {
    final ip = _ctrl.text.trim();
    if (ip.isEmpty) return;
    setState(() { _checking = true; _error = null; });
    final st = context.read<RobotState>();
    await st.setIp(ip);
    try {
      await st.api!.status();
      if (!mounted) return;
      Navigator.of(context).pushReplacement(
          MaterialPageRoute(builder: (_) => const DashboardScreen()));
    } catch (e) {
      setState(() => _error = 'Bağlanılamadı: robot açık mı, aynı WiFi\'de misiniz?');
    } finally {
      if (mounted) setState(() => _checking = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final searching = context.watch<RobotState>().searching;
    return Scaffold(
      body: SafeArea(
        child: Center(
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(28),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Text('📡', style: TextStyle(fontSize: 48)),
                const SizedBox(height: 12),
                RichText(
                  text: const TextSpan(
                    style: TextStyle(fontSize: 22, fontWeight: FontWeight.w700, letterSpacing: 1, color: CryvexColors.textPrimary),
                    children: [
                      TextSpan(text: 'CRYVEX '),
                      TextSpan(text: '· Bağlantı', style: TextStyle(color: CryvexColors.cyan)),
                    ],
                  ),
                ),
                const SizedBox(height: 28),
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton(
                    style: CryvexButtonStyle.cyan,
                    onPressed: searching ? null : _search,
                    child: searching
                        ? const Row(mainAxisSize: MainAxisSize.min, children: [
                            SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2, color: CryvexColors.cyan)),
                            SizedBox(width: 10),
                            Text('Robot aranıyor…'),
                          ])
                        : const Text('🔍 Robotu Bul'),
                  ),
                ),
                const SizedBox(height: 26),
                const Text('Bulunamazsa: robotun IP adresi', style: TextStyle(color: CryvexColors.textMuted, fontSize: 13)),
                const SizedBox(height: 8),
                TextField(
                  controller: _ctrl,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.center,
                  style: const TextStyle(fontSize: 20, color: CryvexColors.textPrimary, letterSpacing: 1),
                  decoration: InputDecoration(
                    hintText: '192.168.1.9',
                    hintStyle: const TextStyle(color: Colors.white24),
                    filled: true,
                    fillColor: CryvexColors.card,
                    border: OutlineInputBorder(borderRadius: BorderRadius.circular(16), borderSide: const BorderSide(color: CryvexColors.cardLine)),
                    enabledBorder: OutlineInputBorder(borderRadius: BorderRadius.circular(16), borderSide: const BorderSide(color: CryvexColors.cardLine)),
                    focusedBorder: OutlineInputBorder(borderRadius: BorderRadius.circular(16), borderSide: const BorderSide(color: CryvexColors.cyan)),
                    contentPadding: const EdgeInsets.symmetric(vertical: 16, horizontal: 16),
                  ),
                  onSubmitted: (_) => _connect(),
                ),
                if (_error != null) ...[
                  const SizedBox(height: 10),
                  Text(_error!, style: const TextStyle(color: CryvexColors.red, fontSize: 12.5), textAlign: TextAlign.center),
                ],
                const SizedBox(height: 20),
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton(
                    style: CryvexButtonStyle.gray,
                    onPressed: _checking || searching ? null : _connect,
                    child: _checking
                        ? const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(strokeWidth: 2, color: CryvexColors.cyan))
                        : const Text('Bu adrese bağlan'),
                  ),
                ),
                const SizedBox(height: 14),
                const Text(
                  'Telefonun robotla AYNI WiFi ağında olması gerekir. Uygulama robotu\n'
                  'kendisi bulur; adres değişse bile tekrar arar.',
                  style: TextStyle(color: CryvexColors.textMuted, fontSize: 11.5, height: 1.5),
                  textAlign: TextAlign.center,
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
