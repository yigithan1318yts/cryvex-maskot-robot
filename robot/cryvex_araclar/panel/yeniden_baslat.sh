#!/bin/bash
# Cryvex gorev panelini (yeniden) baslatir: bash ~/cryvex_araclar/panel/yeniden_baslat.sh
cd ~/cryvex_araclar/panel || exit 1
python3 -m py_compile panel_sunucu.py || exit 1
for p in $(pgrep -f 'python3 -u panel_sunucu.py'); do kill "$p"; done
sleep 1
nohup python3 -u panel_sunucu.py > /tmp/panel.log 2>&1 < /dev/null &
sleep 2
for u in / /x.png /simge-192.png /manifest.webmanifest /api/durum; do
  curl -s -o /dev/null -w "$u %{http_code} %{content_type}\n" "localhost:8090$u"
done
