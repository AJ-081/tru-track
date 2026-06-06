#!/bin/bash
set -e

source /etc/tru-track/secrets.env

DATE=$(date "+%Y%m%d-%H%M%S")
BACKUP_DIR="/backup/tru-track"

mkdir -p "$BACKUP_DIR/mongodb" "$BACKUP_DIR/code"

mongodump \
  --uri "$MONGO_URI" \
  --db nav \
  --out "$BACKUP_DIR/mongodb/nav_${DATE}" \
  --gzip

tar -czf "$BACKUP_DIR/code/code_${DATE}.tar.gz" \
  /opt/tru-track/ingest/mqtt_ingest.py \
  /opt/tru-track/backend/nav_backend.py \
  /opt/tru-track/deployment \
  /etc/systemd/system/tru-track-*.service \
  /etc/systemd/system/tru-track-*.service.d \
  /etc/mosquitto/conf.d/tru-track.conf \
  2>/dev/null || true

find "$BACKUP_DIR/mongodb/" -mindepth 1 -maxdepth 1 -type d -mtime +7 -exec rm -rf {} + 2>/dev/null || true
find "$BACKUP_DIR/code/" -type f -mtime +7 -delete 2>/dev/null || true

echo "[${DATE}] Backup complete" >> /var/log/tru-track/backup.log
