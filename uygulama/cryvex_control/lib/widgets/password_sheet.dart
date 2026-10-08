import 'package:flutter/material.dart';
import '../theme.dart';

const String kStopPassword = '1234'; // web arayüzüyle AYNI (STOP_PASSWORD)

/// Web'deki #overlay-pass'ın karşılığı: 4 haneli şifre iste, doğruysa true
/// döndür. Yanlış girişte titretip sıfırlar, iptalde null/false döner.
Future<bool> askPassword(BuildContext context, {String subtitle = 'Devam etmek için 4 haneli şifre'}) async {
  final result = await showModalBottomSheet<bool>(
    context: context,
    backgroundColor: CryvexColors.bg,
    isScrollControlled: true,
    shape: const RoundedRectangleBorder(
      borderRadius: BorderRadius.vertical(top: Radius.circular(28)),
    ),
    builder: (ctx) => _PasswordSheet(subtitle: subtitle),
  );
  return result ?? false;
}

class _PasswordSheet extends StatefulWidget {
  const _PasswordSheet({required this.subtitle});
  final String subtitle;

  @override
  State<_PasswordSheet> createState() => _PasswordSheetState();
}

class _PasswordSheetState extends State<_PasswordSheet> {
  String buf = '';
  bool bad = false;

  void _tap(String k) {
    setState(() {
      if (k == 'del') {
        buf = buf.isEmpty ? buf : buf.substring(0, buf.length - 1);
      } else if (k == 'clear') {
        buf = '';
      } else if (buf.length < 4) {
        buf += k;
      }
    });
    if (buf.length == 4) {
      if (buf == kStopPassword) {
        Navigator.of(context).pop(true);
      } else {
        setState(() => bad = true);
        Future.delayed(const Duration(milliseconds: 450), () {
          if (mounted) setState(() { bad = false; buf = ''; });
        });
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: EdgeInsets.only(
          left: 24, right: 24, top: 20, bottom: MediaQuery.of(context).viewInsets.bottom + 24),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(width: 40, height: 4, decoration: BoxDecoration(
              color: Colors.white.withValues(alpha: 0.15), borderRadius: BorderRadius.circular(4))),
          const SizedBox(height: 16),
          const Text('🔒 Şifre Gerekli', style: TextStyle(fontSize: 20, fontWeight: FontWeight.w600, color: CryvexColors.textPrimary)),
          const SizedBox(height: 6),
          Text(widget.subtitle, style: const TextStyle(color: CryvexColors.textMuted, fontSize: 13), textAlign: TextAlign.center),
          const SizedBox(height: 18),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: List.generate(4, (i) {
              final on = i < buf.length;
              return AnimatedContainer(
                duration: const Duration(milliseconds: 120),
                margin: const EdgeInsets.symmetric(horizontal: 8),
                width: 14, height: 14,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: on ? (bad ? CryvexColors.red : CryvexColors.cyan) : Colors.transparent,
                  border: Border.all(color: bad ? CryvexColors.red : (on ? CryvexColors.cyan : Colors.white24), width: 2),
                ),
              );
            }),
          ),
          const SizedBox(height: 22),
          GridView.count(
            crossAxisCount: 3,
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            mainAxisSpacing: 12,
            crossAxisSpacing: 12,
            childAspectRatio: 1.6,
            children: [
              for (final k in ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'clear', '0', 'del'])
                _KeypadButton(label: k == 'clear' ? '✕' : (k == 'del' ? '⌫' : k), onTap: () => _tap(k)),
            ],
          ),
          const SizedBox(height: 8),
          TextButton(onPressed: () => Navigator.of(context).pop(false), child: const Text('Vazgeç', style: TextStyle(color: CryvexColors.textMuted))),
        ],
      ),
    );
  }
}

class _KeypadButton extends StatelessWidget {
  const _KeypadButton({required this.label, required this.onTap});
  final String label;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return Material(
      color: CryvexColors.card,
      borderRadius: BorderRadius.circular(16),
      child: InkWell(
        borderRadius: BorderRadius.circular(16),
        onTap: onTap,
        child: Container(
          decoration: BoxDecoration(borderRadius: BorderRadius.circular(16), border: Border.all(color: CryvexColors.cardLine)),
          alignment: Alignment.center,
          child: Text(label, style: const TextStyle(fontSize: 20, fontWeight: FontWeight.w600, color: CryvexColors.textPrimary)),
        ),
      ),
    );
  }
}
