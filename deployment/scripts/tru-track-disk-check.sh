#!/bin/bash
THRESHOLD=80
USAGE=$(df / | awk 'NR==2 {print $5}' | tr -d '%')
DATE=$(date "+%Y-%m-%d %H:%M:%S")

if [ "$USAGE" -ge "$THRESHOLD" ]; then
  journalctl --vacuum-size=100M
  echo "[${DATE}] ALERT: Disk at ${USAGE}% — journal vacuum triggered" >> /var/log/tru-track/health.log
  systemd-cat -t tru-track-disk-alert -p warning echo "Disk at ${USAGE}%"
else
  echo "[${DATE}] Disk OK: ${USAGE}%" >> /var/log/tru-track/health.log
fi
