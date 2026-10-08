import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../theme.dart';

/// Joystick yerine YON TUSLARI (sahibin istegi 2026-10-08). Joystick ile ayni arayuz: onMove(lx, az) / onRelease.
///   ▲ dumduz ileri · ▼ dumduz geri · ◀ ▶ yerinde yavas donus · ■ dur
/// Tus BASILI TUTULDUGU surece hareket, birakinca (ya da parmak kayinca) DUR. Ayni anda tek tus.
/// Hizlar kasitli dusuk (robot guc kablosuyla calisiyor; yavas, ongorulebilir hareket).
class YonTuslari extends StatefulWidget {
  const YonTuslari({super.key, required this.onMove, required this.onRelease});

  final void Function(double lx, double az) onMove;
  final VoidCallback onRelease;

  @override
  State<YonTuslari> createState() => _YonTuslariState();
}

enum _Yon { ileri, geri, sol, sag }

class _YonTuslariState extends State<YonTuslari> {
  _Yon? _basili;
  bool _hizli = false;

  // (lx m/s, az rad/s): Yavas / Normal
  (double, double) _hiz(_Yon y) {
    final lin = _hizli ? 0.20 : 0.10;
    final ang = _hizli ? 0.50 : 0.30;
    switch (y) {
      case _Yon.ileri:
        return (lin, 0);
      case _Yon.geri:
        return (-lin * 0.8, 0);   // geri biraz daha yavas
      case _Yon.sol:
        return (0, ang);
      case _Yon.sag:
        return (0, -ang);
    }
  }

  void _bas(_Yon y) {
    if (_basili != null && _basili != y) return;   // ayni anda tek tus
    HapticFeedback.selectionClick();
    setState(() => _basili = y);
    final (lx, az) = _hiz(y);
    widget.onMove(lx, az);
  }

  void _birak(_Yon y) {
    if (_basili != y) return;
    setState(() => _basili = null);
    widget.onRelease();
  }

  void _dur() {
    HapticFeedback.mediumImpact();
    setState(() => _basili = null);
    widget.onRelease();
  }

  Widget _tus(_Yon y, IconData ikon, String ad) {
    final aktif = _basili == y;
    return Listener(
      onPointerDown: (_) => _bas(y),
      onPointerUp: (_) => _birak(y),
      onPointerCancel: (_) => _birak(y),
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 90),
        width: 84,
        height: 84,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(22),
          color: aktif ? CryvexColors.cyan : CryvexColors.cyan.withValues(alpha: 0.10),
          border: Border.all(color: CryvexColors.cyan.withValues(alpha: aktif ? 1 : 0.35), width: 1.5),
          boxShadow: aktif ? [BoxShadow(color: CryvexColors.cyan.withValues(alpha: 0.45), blurRadius: 18)] : null,
        ),
        child: Semantics(
          button: true,
          label: ad,
          child: Icon(ikon, size: 46, color: aktif ? CryvexColors.bg : CryvexColors.cyan),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    const bosluk = SizedBox(width: 84, height: 84);
    return Column(mainAxisSize: MainAxisSize.min, children: [
      Row(mainAxisSize: MainAxisSize.min, children: [bosluk, const SizedBox(width: 10), _tus(_Yon.ileri, Icons.keyboard_arrow_up_rounded, 'İleri'), const SizedBox(width: 10), bosluk]),
      const SizedBox(height: 10),
      Row(mainAxisSize: MainAxisSize.min, children: [
        _tus(_Yon.sol, Icons.rotate_left_rounded, 'Sola dön'),
        const SizedBox(width: 10),
        GestureDetector(
          onTap: _dur,
          child: Container(
            width: 84,
            height: 84,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(22),
              color: CryvexColors.red.withValues(alpha: 0.14),
              border: Border.all(color: CryvexColors.red.withValues(alpha: 0.6), width: 1.5),
            ),
            child: const Column(mainAxisAlignment: MainAxisAlignment.center, children: [
              Icon(Icons.stop_rounded, size: 36, color: CryvexColors.red),
              Text('DUR', style: TextStyle(color: CryvexColors.red, fontWeight: FontWeight.w900, letterSpacing: 1.5, fontSize: 12)),
            ]),
          ),
        ),
        const SizedBox(width: 10),
        _tus(_Yon.sag, Icons.rotate_right_rounded, 'Sağa dön'),
      ]),
      const SizedBox(height: 10),
      Row(mainAxisSize: MainAxisSize.min, children: [bosluk, const SizedBox(width: 10), _tus(_Yon.geri, Icons.keyboard_arrow_down_rounded, 'Geri'), const SizedBox(width: 10), bosluk]),
      const SizedBox(height: 14),
      SegmentedButton<bool>(
        segments: const [
          ButtonSegment(value: false, label: Text('Yavaş'), icon: Icon(Icons.speed, size: 16)),
          ButtonSegment(value: true, label: Text('Normal'), icon: Icon(Icons.speed, size: 16)),
        ],
        selected: {_hizli},
        onSelectionChanged: (s) => setState(() => _hizli = s.first),
        style: SegmentedButton.styleFrom(
          selectedBackgroundColor: CryvexColors.cyan.withValues(alpha: 0.2),
          selectedForegroundColor: CryvexColors.cyan,
        ),
      ),
      const SizedBox(height: 6),
      const Text('Basılı tut: hareket · bırak: dur', style: TextStyle(color: CryvexColors.textMuted, fontSize: 12)),
    ]);
  }
}
