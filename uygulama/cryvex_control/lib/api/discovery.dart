import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:http/http.dart' as http;

/// Robotu IP bilmeden bulur - modem robota her acilista baska adres
/// verebiliyor, robot baska bir kafenin agina da tasinabilir; kullanici
/// adres girmek zorunda kalmasin.
///
/// 1) UDP yayini: "CRYVEX?" -> robot (cafe_ui_server.py, DISCOVERY_PORT)
///    JSON ile cevap verir, cevabin geldigi adres robotun adresidir.
/// 2) Yayin engelli aglar icin yedek: telefonun kendi alt agindaki (x.y.z.1-254)
///    adreslerde /api/mode'u dener (sim'deki tablet_server da buna cevap verir).
const int discoveryPort = 47474;
const int robotHttpPort = 8080;

Future<String?> discoverRobot() async {
  final byBroadcast = await _discoverByBroadcast();
  if (byBroadcast != null) return byBroadcast;
  return _discoverBySweep();
}

/// Telefonun WiFi'deki ozel (yerel) IPv4 adresleri.
Future<List<InternetAddress>> _localAddresses() async {
  final result = <InternetAddress>[];
  for (final iface in await NetworkInterface.list(type: InternetAddressType.IPv4)) {
    for (final addr in iface.addresses) {
      final p = addr.rawAddress;
      final private = p[0] == 10 || (p[0] == 192 && p[1] == 168) || (p[0] == 172 && p[1] >= 16 && p[1] <= 31);
      if (private && !addr.isLoopback) result.add(addr);
    }
  }
  return result;
}

Future<String?> _discoverByBroadcast() async {
  RawDatagramSocket? socket;
  try {
    socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    socket.broadcastEnabled = true;
    // Genel yayin + her yerel agin kendi yayin adresi (bazi modemler yalniz birini iletir).
    final targets = <InternetAddress>[InternetAddress('255.255.255.255')];
    for (final a in await _localAddresses()) {
      final p = a.rawAddress;
      targets.add(InternetAddress('${p[0]}.${p[1]}.${p[2]}.255'));
    }
    final found = Completer<String?>();
    final sub = socket.listen((event) {
      if (event != RawSocketEvent.read) return;
      final dg = socket!.receive();
      if (dg == null) return;
      try {
        final info = jsonDecode(utf8.decode(dg.data));
        if (info is Map && info['cryvex'] == 'robot' && !found.isCompleted) {
          found.complete(dg.address.address);
        }
      } catch (_) {/* baska bir cihazin paketi */}
    });
    for (var attempt = 0; attempt < 3 && !found.isCompleted; attempt++) {
      for (final t in targets) {
        socket.send(utf8.encode('CRYVEX?'), t, discoveryPort);
      }
      await Future.any([found.future, Future.delayed(const Duration(milliseconds: 700))]);
    }
    await sub.cancel();
    return found.isCompleted ? await found.future : null;
  } catch (_) {
    return null;
  } finally {
    socket?.close();
  }
}

Future<String?> _discoverBySweep() async {
  final client = http.Client();
  try {
    for (final a in await _localAddresses()) {
      final p = a.rawAddress;
      final prefix = '${p[0]}.${p[1]}.${p[2]}.';
      // 32'serli gruplar: telefonu 254 esanli baglantiyla bogmadan ~3-4 sn.
      for (var start = 1; start < 255; start += 32) {
        final batch = [
          for (var i = start; i < start + 32 && i < 255; i++)
            if (i != p[3]) _probe(client, '$prefix$i'),
        ];
        final hits = (await Future.wait(batch)).whereType<String>();
        if (hits.isNotEmpty) return hits.first;
      }
    }
    return null;
  } finally {
    client.close();
  }
}

Future<String?> _probe(http.Client client, String ip) async {
  try {
    final res = await client
        .get(Uri.parse('http://$ip:$robotHttpPort/api/mode'))
        .timeout(const Duration(milliseconds: 800));
    if (res.statusCode == 200 && (jsonDecode(res.body) as Map).containsKey('mode')) return ip;
  } catch (_) {/* bu adreste robot yok */}
  return null;
}
