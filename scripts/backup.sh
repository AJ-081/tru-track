#!/bin/bash
set -e
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR=/backup/tru-track
LOG=/var/log/tru-track/backup.log

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOG"; }

log "--- Backup started ---"

# MongoDB
mongodump \
  --uri "mongodb://mongoadmin:TruTrackMongo2026@127.0.0.1:27017/?authSource=admin&directConnection=true" \
  --db nav \
  --out "${BACKUP_DIR}/mongodb/nav_${DATE}" \
  --gzip 2>> "$LOG" \
  && log "MongoDB backup OK" \
  || log "MongoDB backup FAILED"

# Code snapshot
tar -czf "${BACKUP_DIR}/code/code_${DATE}.tar.gz" \
  /opt/tru-track/ingest/mqtt_ingest.py \
  /opt/tru-track/backend/nav_backend.py \
  /opt/tru-track/backend/init_db.py \
  /opt/tru-track/backend/create_user.py \
  /opt/tru-track/dashboard/ \
  /opt/tru-track/scripts/ \
  /etc/systemd/system/tru-track-*.service \
  /etc/nginx/sites-available/tru-track \
  /etc/mosquitto/conf.d/tru-track.conf \
  2>> "$LOG" \
  && log "Code backup OK" \
  || log "Code backup FAILED"

# Delete backups older than 30 days
find "${BACKUP_DIR}/mongodb/" -maxdepth 1 -type d -mtime +30 -exec rm -rf {} + 2>/dev/null || true
find "${BACKUP_DIR}/code/"    -type f -mtime +30 -delete 2>/dev/null || true
log "Old backups cleaned (>30 days)"

# Disk usage warning
DISK=$(df / | awk 'NR==2{print $5}' | tr -d '%')
[ "$DISK" -gt 80 ] && log "WARNING: Disk at ${DISK}%" || true

log "--- Backup complete. Disk: ${DISK}% ---"
