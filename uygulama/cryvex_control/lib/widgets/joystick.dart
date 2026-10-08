import 'package:flutter/material.dart';
import '../theme.dart';

/// Web arayüzündeki #joy-base/#joy-knob'un Flutter karşılığı. Sürüklerken
/// onMove(lx, az) callback'ini ~120ms'de bir değil, her frame'de çağırır -
/// çağıran taraf (ekranlar) kendi hız sınırlama/gönderme aralığını yönetir.
class Joystick extends StatefulWidget {
  const Joystick({
    super.key,
    required this.onMove,
    required this.onRelease,
    this.size = 190,
    this.maxLin = 0.30,
    this.maxAng = 1.00,
  });

  /// lx: -maxLin..maxLin (ileri/geri), az: -maxAng..maxAng (dönüş)
  final void Function(double lx, double az) onMove;
  final VoidCallback onRelease;
  final double size;
  final double maxLin;
  final double maxAng;

  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _knobOffset = Offset.zero;
  double get _radius => widget.size / 2 - 38; // 78px knob ile ayni oranti

  void _handle(Offset localPos) {
    final center = Offset(widget.size / 2, widget.size / 2);
    var delta = localPos - center;
    final dist = delta.distance;
    if (dist > _radius) {
      delta = Offset(delta.dx / dist * _radius, delta.dy / dist * _radius);
    }
    setState(() => _knobOffset = delta);
    final lx = (-delta.dy / _radius) * widget.maxLin;
    final az = (-delta.dx / _radius) * widget.maxAng;
    widget.onMove(lx, az);
  }

  void _release() {
    setState(() => _knobOffset = Offset.zero);
    widget.onRelease();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanStart: (d) => _handle(d.localPosition),
      onPanUpdate: (d) => _handle(d.localPosition),
      onPanEnd: (_) => _release(),
      onPanCancel: _release,
      child: Container(
        width: widget.size,
        height: widget.size,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          border: Border.all(color: CryvexColors.cardLine),
          gradient: RadialGradient(colors: [
            CryvexColors.cyan.withValues(alpha: 0.08),
            Colors.white.withValues(alpha: 0.03),
          ]),
        ),
        child: Center(
          child: Transform.translate(
            offset: _knobOffset,
            child: Container(
              width: 78,
              height: 78,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                border: Border.all(color: CryvexColors.cyan, width: 2),
                gradient: const RadialGradient(
                  center: Alignment(-0.3, -0.4),
                  colors: [Color(0xFF1A2540), Color(0xFF0A1020)],
                ),
                boxShadow: [
                  BoxShadow(color: CryvexColors.cyan.withValues(alpha: 0.35), blurRadius: 18),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}
