# TRU-TRACK Deployment Guide
Last updated: May 2026

## Server
- OS: Ubuntu Server 22.04.5 LTS
- CPU: Intel i7-7700
- RAM: 8 GB
- Disk: 256 GB SATA SSD
- LAN IP: 172.21.129.185
- Public IP: 14.139.121.53
- Domain: tru-track.bittest.in (HTTPS via Let's Encrypt)

## Directory Structure
/opt/tru-track/
├── ingest/          mqtt_ingest.py + venv
├── backend/         nav_backend.py + venv
├── ai/              ai_service.py + venv (future)
├── dashboard/       index.html, app.js, auth.js, style.css
├── scripts/         backup.sh, health_check.sh
DEPLOYMENT.md
/etc/tru-track/secrets.env     All credentials (chmod 600)
/var/log/tru-track/            ingest.log, backend.log, backup.log, health.log
/backup/tru-track/             mongodb/ + code/ (30-day rotation)

## Services
| Service | Command | Log |
|---------|---------|-----|
| MongoDB | systemctl {start\|stop\|restart} mongod | /var/log/mongodb/mongod.log |
| Mosquitto | systemctl {start\|stop\|restart} mosquitto | /var/log/mosquitto/mosquitto.log |
| Nginx | systemctl {start\|stop\|restart} nginx | /var/log/nginx/ |
| Ingest | systemctl {start\|stop\|restart} tru-track-ingest | /var/log/tru-track/ingest.log |
| Backend | systemctl {start\|stop\|restart} tru-track-backend | /var/log/tru-track/backend.log |

## MongoDB
- Auth: enabled
- Admin user: mongoadmin (authSource: admin)
- App user: navapp (authSource: admin)
- Database: nav
- 9 collections: gnss_raw, eskf_state, imu_raw, sessions, device_latest,
  device_registry, ai_sequences, users, matlab_exports
- TTL: 90 days on raw collections

## MQTT
- Broker: Mosquitto 2.0.11 on port 1883
- Auth: required (no anonymous)
- Ingest account: trutrack_ingest
- Device account: esp32_device (shared, all ESP32 devices)
- ACL: ingest reads nav/#, devices write nav/#

## Add a New User
```bash
cd /opt/tru-track/backend
source venv/bin/activate
python create_user.py
deactivate
```
Roles: superadmin / admin / analyst / viewer

## Add a New ESP32 Device
1. Flash Provision_NVS.ino with:
   - wifi_ssid, wifi_pass: network credentials
   - mqtt_host: 14.139.121.53
   - mqtt_port: 1883
   - mqtt_user: esp32_device
   - mqtt_pass: TruTrackDevice2026
2. Flash TruTrack_v2.ino
3. Device auto-registers on first connection
4. Appears in dashboard device dropdown immediately

## Restore from Backup
```bash
# List available backups
ls -la /backup/tru-track/mongodb/

# Restore a specific backup
mongorestore \
  --uri "mongodb://mongoadmin:TruTrackMongo2026@127.0.0.1:27017/?authSource=admin" \
  --db nav \
  --drop \
  /backup/tru-track/mongodb/nav_YYYYMMDD_HHMMSS/nav/ \
  --gzip
```

## Cron Jobs
- 02:00 daily: /opt/tru-track/scripts/backup.sh
- Every 5 min: /opt/tru-track/scripts/health_check.sh

## API Base URL
https://tru-track.bittest.in/api/v1/

## Key Endpoints
- POST /auth/login — get JWT token
- GET  /devices — list registered devices
- GET  /sessions/{device_id} — list sessions
- GET  /device/latest/{device_id} — real-time health
- GET  /gnss/track/{device_id}?session_id= — GNSS polyline
- GET  /eskf/track/{device_id}?session_id= — ESKF polyline
- GET  /session/export?device_id=&session_id= — CSV download
- GET  /session/geojson?device_id=&session_id= — GeoJSON download
- GET  /server/health — server + service status

## Pending / Future
- Day 14: AI service (infrastructure ready, model pending MATLAB tuning)
- Fuel gauge IC: MAX17043 or BQ27441 on ESP32 I2C
- MQTT per-device credentials (currently shared esp32_device account)
- Google Maps API key (currently using Leaflet/Esri satellite)
- MATLAB ESKF offline tuning workflow
