#!/bin/bash
SERVICES=(mongod mosquitto nginx tru-track-ingest tru-track-backend)
LOG=/var/log/tru-track/health.log
TS=$(date '+%Y-%m-%d %H:%M:%S')
RESTARTED=()
FAILED=0

for svc in "${SERVICES[@]}"; do
  if ! systemctl is-active --quiet "$svc"; then
    echo "[$TS] DOWN: $svc — restarting" >> "$LOG"
    if systemctl restart "$svc" 2>/dev/null; then
      echo "[$TS] RESTARTED: $svc" >> "$LOG"
      RESTARTED+=("$svc")
    else
      echo "[$TS] RESTART FAILED: $svc" >> "$LOG"
      ((FAILED++)) || true
    fi
  fi
done

DISK=$(df / | awk 'NR==2{print $5}' | tr -d '%')
[ "$DISK" -gt 80 ] && echo "[$TS] DISK WARNING: ${DISK}%" >> "$LOG" || true

if [ ${#RESTARTED[@]} -gt 0 ]; then
  echo "[$TS] Restarted: ${RESTARTED[*]}" >> "$LOG"
elif [ $FAILED -eq 0 ]; then
  echo "[$TS] All OK. Disk: ${DISK}%" >> "$LOG"
fi

