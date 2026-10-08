import 'package:flutter/material.dart';

/// CryvexTech kurumsal paleti (cryvextech.com.tr/css/site.css): accent #26ccea, accent2 #0d7f96, koyu antrasit zemin.
/// Robottaki Gorev Paneli (:8090) de ayni renkleri kullanir.
class CryvexColors {
  static const bg = Color(0xFF0B0F15);
  static const card = Color(0x0BFFFFFF);
  static const cardLine = Color(0x12FFFFFF);
  static const cyan = Color(0xFF26CCEA);
  static const cyanDeep = Color(0xFF0D7F96);
  static const green = Color(0xFF2BD47B);
  static const red = Color(0xFFFF6B6B);
  static const amber = Color(0xFFFEBC2E);
  static const textPrimary = Color(0xFFE9EFF6);
  static const textMuted = Color(0xFF9FB0C2);
}

/// Ust bar basligi: CryvexTech "X" logosu + CRYVEX yazisi (+ istege bagli alt baslik).
class CryvexBaslik extends StatelessWidget {
  const CryvexBaslik({super.key, this.alt});
  final String? alt;

  @override
  Widget build(BuildContext context) {
    return Row(mainAxisSize: MainAxisSize.min, children: [
      Image.asset('assets/cryvex_x.png', width: 30, height: 30),
      const SizedBox(width: 10),
      Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.start, children: [
        const Text('CRYVEX',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w800, letterSpacing: 5, color: CryvexColors.textPrimary)),
        if (alt != null)
          Text(alt!, style: const TextStyle(fontSize: 9.5, letterSpacing: 2.2, color: CryvexColors.cyan, fontWeight: FontWeight.w600)),
      ]),
    ]);
  }
}

ThemeData buildCryvexTheme() {
  return ThemeData(
    useMaterial3: true,
    brightness: Brightness.dark,
    scaffoldBackgroundColor: CryvexColors.bg,
    colorScheme: const ColorScheme.dark(
      primary: CryvexColors.cyan,
      secondary: CryvexColors.green,
      error: CryvexColors.red,
      surface: CryvexColors.bg,
    ),
    fontFamily: 'Roboto',
    appBarTheme: const AppBarTheme(
      backgroundColor: CryvexColors.bg,
      foregroundColor: CryvexColors.textPrimary,
      elevation: 0,
      centerTitle: true,
    ),
    textTheme: const TextTheme(
      bodyMedium: TextStyle(color: CryvexColors.textPrimary),
      bodyLarge: TextStyle(color: CryvexColors.textPrimary),
    ),
    cardColor: CryvexColors.card,
  );
}

/// Manager panelindeki .btn-* sınıflarıyla eşleşen buton renk kalıpları.
class CryvexButtonStyle {
  static ButtonStyle _make(Color fg, Color bg, Color border) {
    return ElevatedButton.styleFrom(
      foregroundColor: fg,
      backgroundColor: bg,
      side: BorderSide(color: border, width: 1),
      elevation: 0,
      padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 18),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      textStyle: const TextStyle(fontSize: 15, fontWeight: FontWeight.w600),
    );
  }

  static ButtonStyle green = _make(CryvexColors.green, CryvexColors.green.withValues(alpha: 0.10),
      CryvexColors.green.withValues(alpha: 0.25));
  static ButtonStyle cyan = _make(CryvexColors.cyan, CryvexColors.cyan.withValues(alpha: 0.10),
      CryvexColors.cyan.withValues(alpha: 0.22));
  static ButtonStyle red = _make(CryvexColors.red, CryvexColors.red.withValues(alpha: 0.10),
      CryvexColors.red.withValues(alpha: 0.22));
  static ButtonStyle amber = _make(CryvexColors.amber, CryvexColors.amber.withValues(alpha: 0.10),
      CryvexColors.amber.withValues(alpha: 0.25));
  static ButtonStyle gray = _make(CryvexColors.textMuted, Colors.white.withValues(alpha: 0.04),
      Colors.white.withValues(alpha: 0.08));
}
