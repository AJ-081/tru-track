#!/usr/bin/env python3
"""
TRU-TRACK Flask Backend — v2.1
Changes from v2.0:
  - Export base changed from gnss_raw → imu_raw (continuous, no gaps)
  - t_epoch_ms column in CSV export
  - fix_valid, init_valid, alignment_valid, calibrated columns in CSV
  - GeoJSON uses fix_valid=True filter for GNSS track
  - GeoJSON uses init_valid=True filter for ESKF track
  - ESKF track endpoint filters init_valid=True
"""
import csv
import io
import logging
import os
import subprocess
import time
from collections import defaultdict
from datetime import datetime, timedelta

import bcrypt
import psutil
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_jwt_extended import (
    JWTManager, create_access_token, create_refresh_token,
    get_jwt_identity, jwt_required,
)
from flask_socketio import SocketIO, join_room
from pymongo import MongoClient

load_dotenv("/etc/tru-track/secrets.env")

# ── Config ────────────────────────────────────────────────────────────────────
MONGO_URI      = os.getenv("MONGO_URI", "mongodb://127.0.0.1:27017/?directConnection=true")
MONGO_DB       = os.getenv("MONGO_DB", "nav")
FLASK_SECRET   = os.getenv("FLASK_SECRET_KEY", "changeme")
JWT_SECRET     = os.getenv("JWT_SECRET_KEY", "changeme")
DASHBOARD_DIR  = os.getenv("DASHBOARD_DIR", "/opt/tru-track/dashboard")
SERVER_HOST    = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT    = int(os.getenv("SERVER_PORT", 9000))
GOOGLE_MAPS_KEY = os.getenv("GOOGLE_MAPS_KEY", "")

logging.getLogger("werkzeug").setLevel(logging.ERROR)

# ── Flask + SocketIO ──────────────────────────────────────────────────────────
app = Flask(__name__)
app.config["SECRET_KEY"]                = FLASK_SECRET
app.config["JWT_SECRET_KEY"]            = JWT_SECRET
app.config["JWT_ACCESS_TOKEN_EXPIRES"]  = timedelta(hours=8)
app.config["JWT_REFRESH_TOKEN_EXPIRES"] = timedelta(days=30)

jwt      = JWTManager(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── MongoDB ───────────────────────────────────────────────────────────────────
mongo = MongoClient(MONGO_URI)
db    = mongo[MONGO_DB]

DEFAULT_HISTORY_DAYS = 30
SESSION_GAP_SECONDS  = 120

# ── Role levels ───────────────────────────────────────────────────────────────
ROLE_LEVELS = {"viewer": 1, "analyst": 2, "admin": 3, "superadmin": 4}

def role_required(min_role: str):
    def decorator(fn):
        from functools import wraps
        @wraps(fn)
        @jwt_required()
        def wrapper(*args, **kwargs):
            identity = get_jwt_identity()
            user = db.users.find_one({"email": identity}, {"role": 1})
            if not user:
                return jsonify({"error": "User not found"}), 401
            if ROLE_LEVELS.get(user.get("role", "viewer"), 0) < ROLE_LEVELS.get(min_role, 99):
                return jsonify({"error": "Insufficient role"}), 403
            return fn(*args, **kwargs)
        return wrapper
    return decorator


# ─────────────────────────────────────────────────────────────────────────────
# AUTH ROUTES
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/auth/login", methods=["POST"])
def login():
    data     = request.get_json() or {}
    email    = (data.get("email") or "").strip().lower()
    password = data.get("password") or ""

    if not email or not password:
        return jsonify({"error": "Email and password required"}), 400

    user = db.users.find_one({"email": email})
    if not user:
        return jsonify({"error": "Invalid credentials"}), 401

    pw_hash = user.get("password_hash") or user.get("password") or ""
    if not bcrypt.checkpw(password.encode(), pw_hash.encode()):
        return jsonify({"error": "Invalid credentials"}), 401

    access  = create_access_token(identity=email)
    refresh = create_refresh_token(identity=email)
    return jsonify({
        "access_token":  access,
        "refresh_token": refresh,
        "role":          user.get("role", "viewer"),
        "name":          user.get("name", ""),
    })


@app.route("/api/v1/auth/refresh", methods=["POST"])
@jwt_required(refresh=True)
def refresh():
    identity = get_jwt_identity()
    return jsonify({"access_token": create_access_token(identity=identity)})


@app.route("/api/v1/auth/me")
@jwt_required()
def me():
    identity = get_jwt_identity()
    user = db.users.find_one({"email": identity}, {"_id": 0, "password_hash": 0, "password": 0})
    return jsonify(user or {})


@app.route("/api/v1/maps/key")
@jwt_required()
def maps_key():
    return jsonify({"key": GOOGLE_MAPS_KEY})


# ─────────────────────────────────────────────────────────────────────────────
# DASHBOARD STATIC FILES
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/")
def dashboard():
    return send_from_directory(DASHBOARD_DIR, "index.html")

@app.route("/login")
def login_page():
    return send_from_directory(DASHBOARD_DIR, "login.html")

@app.route("/<path:filename>")
def static_files(filename):
    return send_from_directory(DASHBOARD_DIR, filename)


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def parse_iso_utc(value):
    if not value:
        return None
    try:
        if value.endswith("Z"):
            value = value[:-1] + "+00:00"
        dt = datetime.fromisoformat(value)
        if dt.tzinfo is not None:
            return dt.astimezone().replace(tzinfo=None)
        return dt
    except Exception:
        return None


def request_session_filter(device_id):
    q          = {"device_id": device_id}
    session_id = request.args.get("session_id")
    start      = parse_iso_utc(request.args.get("start"))
    end        = parse_iso_utc(request.args.get("end"))

    if session_id:
        q["session_id"] = session_id
        return q
    if start or end:
        ts = {}
        if start: ts["$gte"] = start
        if end:   ts["$lte"] = end
        if ts:    q["t_server"] = ts
        return q
    q["t_server"] = {"$gte": datetime.utcnow() - timedelta(days=DEFAULT_HISTORY_DAYS)}
    return q


# ─────────────────────────────────────────────────────────────────────────────
# DEVICE + SESSION ROUTES
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/devices")
@jwt_required()
def devices():
    docs = list(db.device_registry.find(
        {}, {"_id": 0, "device_id": 1, "first_seen": 1, "last_seen": 1,
             "fw_version": 1, "total_packets": 1}
    ).sort("last_seen", -1))
    for d in docs:
        d["label"] = d.get("device_id", "")
        for k in ("first_seen", "last_seen"):
            if isinstance(d.get(k), datetime):
                d[k] = d[k].isoformat()
    return jsonify(docs)


@app.route("/api/v1/sessions/<device_id>")
@jwt_required()
def sessions(device_id):
    docs = list(db.sessions.find(
        {"device_id": device_id},
        {"_id": 0, "session_id": 1, "started_at_server": 1,
         "last_seen": 1, "packet_count": 1, "boot_count": 1,
         "fw_version": 1, "started_t_epoch_ms": 1}
    ).sort("started_at_server", -1))

    result = []
    for d in docs:
        started = d.get("started_at_server")
        last    = d.get("last_seen")
        dur_s   = None
        if isinstance(started, datetime) and isinstance(last, datetime):
            dur_s = max(0, int((last - started).total_seconds()))
        result.append({
            "session_id":       d.get("session_id"),
            "started_at_server": started.isoformat() if isinstance(started, datetime) else None,
            "duration_s":       dur_s,
            "packet_count":     d.get("packet_count"),
            "boot_count":       d.get("boot_count"),
            "fw_version":       d.get("fw_version"),
            "started_t_epoch_ms": d.get("started_t_epoch_ms", 0),
        })
    return jsonify(result)


@app.route("/api/v1/device/latest/<device_id>")
@jwt_required()
def device_latest(device_id):
    doc = db.device_latest.find_one({"device_id": device_id}, {"_id": 0})
    if doc and isinstance(doc.get("last_seen"), datetime):
        doc["last_seen"] = doc["last_seen"].isoformat()
    return jsonify(doc or {})


# ─────────────────────────────────────────────────────────────────────────────
# TRACK ROUTES
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/gnss/track/<device_id>")
@jwt_required()
def gnss_track(device_id):
    q = request_session_filter(device_id)
    # Only return rows where GNSS fix was valid and position is non-zero
    q["fix_valid"] = True
    q["lat"]       = {"$nin": [None, 0]}
    q["lon"]       = {"$nin": [None, 0]}
    docs = list(db.gnss_raw.find(
        q,
        {"_id": 0, "lat": 1, "lon": 1, "alt": 1, "speed": 1,
         "course": 1, "sats": 1, "hdop": 1, "t_ms": 1,
         "t_server": 1, "t_epoch_ms": 1}
    ).sort("t_server", 1).limit(50000))
    for d in docs:
        if isinstance(d.get("t_server"), datetime):
            d["t_server"] = d["t_server"].isoformat()
    return jsonify(docs)


@app.route("/api/v1/eskf/track/<device_id>")
@jwt_required()
def eskf_track(device_id):
    q = request_session_filter(device_id)
    # Only return rows where ESKF was initialized and aligned
    q["init_valid"]      = True
    q["alignment_valid"] = True
    q["lat"]             = {"$nin": [None, 0]}
    q["lon"]             = {"$nin": [None, 0]}
    docs = list(db.eskf_state.find(
        q,
        {"_id": 0, "lat": 1, "lon": 1, "alt": 1, "vE": 1, "vN": 1,
         "vU": 1, "roll": 1, "pitch": 1, "yaw": 1, "t_ms": 1,
         "t_server": 1, "t_epoch_ms": 1}
    ).sort("t_server", 1).limit(50000))
    for d in docs:
        if isinstance(d.get("t_server"), datetime):
            d["t_server"] = d["t_server"].isoformat()
    return jsonify(docs)


@app.route("/api/v1/imu/latest/<device_id>")
@jwt_required()
def imu_latest(device_id):
    doc = db.imu_raw.find_one(
        {"device_id": device_id},
        sort=[("t_server", -1)],
        projection={"_id": 0}
    )
    if doc and isinstance(doc.get("t_server"), datetime):
        doc["t_server"] = doc["t_server"].isoformat()
    return jsonify(doc or {})


# ─────────────────────────────────────────────────────────────────────────────
# ALERTS
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/alerts/<device_id>")
@jwt_required()
def alerts(device_id):
    doc    = db.device_latest.find_one({"device_id": device_id}) or {}
    status = doc.get("status") or {}
    result = []

    if not status.get("gnss_fix"):
        result.append({"level": "error",   "code": "GNSS_NO_FIX",
                        "msg": "No GNSS fix"})
    elif (status.get("sats") or 0) < 4:
        result.append({"level": "warning",  "code": "GNSS_FEW_SATS",
                        "msg": f"Only {status.get('sats', 0)} satellites"})

    rssi = status.get("wifi_rssi_dbm")
    if rssi is not None and rssi < -80:
        result.append({"level": "warning",  "code": "SIGNAL_WEAK",
                        "msg": f"RSSI {rssi} dBm"})

    last_seen = doc.get("last_seen")
    if isinstance(last_seen, datetime):
        age = (datetime.utcnow() - last_seen).total_seconds()
        if age > 30:
            result.append({"level": "error", "code": "STALE_DATA",
                            "msg": f"No packet for {int(age)}s"})

    bpct = status.get("battery_pct", -1)
    if bpct is not None and 0 <= bpct < 10:
        result.append({"level": "error",   "code": "BATTERY_CRITICAL",
                        "msg": f"Battery {bpct}%"})
    elif bpct is not None and 10 <= bpct < 20:
        result.append({"level": "warning", "code": "BATTERY_LOW",
                        "msg": f"Battery {bpct}%"})

    return jsonify(result)


# ─────────────────────────────────────────────────────────────────────────────
# AI (FUTURE)
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/ai/latest/<device_id>")
@jwt_required()
def ai_latest(device_id):
    doc = db.ai_sequences.find_one(
        {"device_id": device_id},
        sort=[("t_start", -1)],
        projection={"_id": 0}
    )
    return jsonify(doc or {"driver_score": None, "anomaly": None})


# ─────────────────────────────────────────────────────────────────────────────
# CSV EXPORT — imu_raw as base (continuous anchor)
# Left-joins gnss_raw and eskf_state by t_ms proximity
# All rows present regardless of GNSS fix or ESKF state
# ─────────────────────────────────────────────────────────────────────────────
def docs_to_csv_rows(device_id):
    base_query = request_session_filter(device_id)

    # imu_raw is the anchor — every packet writes here, no conditions
    imu_docs  = list(db.imu_raw.find(
        base_query,
        {"_id": 0, "device_id": 1, "session_id": 1, "boot_count": 1,
         "t_ms": 1, "t_server": 1, "t_epoch_ms": 1, "ntp_synced": 1,
         "calibrated": 1,
         "accel_raw": 1, "gyro_raw": 1, "accel_mps2": 1,
         "gyro_radps": 1, "gyro_bias_radps": 1}
    ).sort("t_server", 1))

    gnss_docs = list(db.gnss_raw.find(
        base_query,
        {"_id": 0, "t_ms": 1, "t_server": 1,
         "fix_valid": 1, "lat": 1, "lon": 1, "alt": 1,
         "speed": 1, "course": 1, "sats": 1, "hdop": 1,
         "fix_type": 1, "age_ms": 1}
    ).sort("t_server", 1))

    eskf_docs = list(db.eskf_state.find(
        base_query,
        {"_id": 0, "t_ms": 1, "t_server": 1,
         "init_valid": 1, "alignment_valid": 1,
         "lat": 1, "lon": 1, "alt": 1,
         "vE": 1, "vN": 1, "vU": 1,
         "roll": 1, "pitch": 1, "yaw": 1, "innov": 1}
    ).sort("t_server", 1))

    # Build lookup by t_ms for fast join
    def tms_lookup(docs):
        d = {}
        for doc in docs:
            k = doc.get("t_ms")
            if k is not None:
                d[k] = doc
        return d

    gnss_by_tms = tms_lookup(gnss_docs)
    eskf_by_tms = tms_lookup(eskf_docs)

    rows = []
    for imu in imu_docs:
        tms = imu.get("t_ms")
        ts  = imu.get("t_server")
        gnss = gnss_by_tms.get(tms, {})
        eskf = eskf_by_tms.get(tms, {})

        accel_raw  = imu.get("accel_raw")  or [None, None, None]
        gyro_raw   = imu.get("gyro_raw")   or [None, None, None]
        accel_mps2 = imu.get("accel_mps2") or [None, None, None]
        gyro_radps = imu.get("gyro_radps") or [None, None, None]

        innov = eskf.get("innov") or {}

        row = {
            "device_id":      device_id,
            "session_id":     imu.get("session_id"),
            "boot_count":     imu.get("boot_count"),
            "t_ms":           tms,
            "t_server":       ts.isoformat() if isinstance(ts, datetime) else ts,
            "t_epoch_ms_ist": imu.get("t_epoch_ms", 0),
            "ntp_synced":     imu.get("ntp_synced", False),

            # GNSS — empty when fix_valid=False
            "gnss_fix_valid": gnss.get("fix_valid", ""),
            "gnss_lat":       gnss.get("lat", ""),
            "gnss_lon":       gnss.get("lon", ""),
            "gnss_alt":       gnss.get("alt", ""),
            "gnss_speed":     gnss.get("speed", ""),
            "gnss_course":    gnss.get("course", ""),
            "gnss_sats":      gnss.get("sats", ""),
            "gnss_hdop":      gnss.get("hdop", ""),
            "gnss_fix_type":  gnss.get("fix_type", ""),
            "gnss_age_ms":    gnss.get("age_ms", ""),

            # ESKF — empty when init_valid=False
            "eskf_init_valid":      eskf.get("init_valid", ""),
            "eskf_alignment_valid": eskf.get("alignment_valid", ""),
            "eskf_lat":             eskf.get("lat", ""),
            "eskf_lon":             eskf.get("lon", ""),
            "eskf_alt":             eskf.get("alt", ""),
            "eskf_vE":              eskf.get("vE", ""),
            "eskf_vN":              eskf.get("vN", ""),
            "eskf_vU":              eskf.get("vU", ""),
            "eskf_roll":            eskf.get("roll", ""),
            "eskf_pitch":           eskf.get("pitch", ""),
            "eskf_yaw":             eskf.get("yaw", ""),
            "eskf_innov_pos_norm":  innov.get("pos_norm", ""),
            "eskf_innov_vel_norm":  innov.get("vel_norm", ""),

            # IMU — always present
            "imu_calibrated":    imu.get("calibrated", False),
            "imu_accel_raw_x":   accel_raw[0]  if len(accel_raw)  > 0 else "",
            "imu_accel_raw_y":   accel_raw[1]  if len(accel_raw)  > 1 else "",
            "imu_accel_raw_z":   accel_raw[2]  if len(accel_raw)  > 2 else "",
            "imu_gyro_raw_x":    gyro_raw[0]   if len(gyro_raw)   > 0 else "",
            "imu_gyro_raw_y":    gyro_raw[1]   if len(gyro_raw)   > 1 else "",
            "imu_gyro_raw_z":    gyro_raw[2]   if len(gyro_raw)   > 2 else "",
            "imu_accel_mps2_x":  accel_mps2[0] if len(accel_mps2) > 0 else "",
            "imu_accel_mps2_y":  accel_mps2[1] if len(accel_mps2) > 1 else "",
            "imu_accel_mps2_z":  accel_mps2[2] if len(accel_mps2) > 2 else "",
            "imu_gyro_radps_x":  gyro_radps[0] if len(gyro_radps) > 0 else "",
            "imu_gyro_radps_y":  gyro_radps[1] if len(gyro_radps) > 1 else "",
            "imu_gyro_radps_z":  gyro_radps[2] if len(gyro_radps) > 2 else "",
        }
        rows.append(row)

    return rows


@app.route("/api/v1/session/export")
@role_required("analyst")
def session_export():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id:
        return jsonify({"error": "device_id required"}), 400

    rows = docs_to_csv_rows(device_id)
    if not rows:
        return jsonify({"error": "No data for requested session"}), 404

    fieldnames = [
        "device_id", "session_id", "boot_count",
        "t_ms", "t_server", "t_epoch_ms_ist", "ntp_synced",
        "gnss_fix_valid", "gnss_lat", "gnss_lon", "gnss_alt",
        "gnss_speed", "gnss_course", "gnss_sats", "gnss_hdop",
        "gnss_fix_type", "gnss_age_ms",
        "eskf_init_valid", "eskf_alignment_valid",
        "eskf_lat", "eskf_lon", "eskf_alt",
        "eskf_vE", "eskf_vN", "eskf_vU",
        "eskf_roll", "eskf_pitch", "eskf_yaw",
        "eskf_innov_pos_norm", "eskf_innov_vel_norm",
        "imu_calibrated",
        "imu_accel_raw_x", "imu_accel_raw_y", "imu_accel_raw_z",
        "imu_gyro_raw_x",  "imu_gyro_raw_y",  "imu_gyro_raw_z",
        "imu_accel_mps2_x","imu_accel_mps2_y","imu_accel_mps2_z",
        "imu_gyro_radps_x","imu_gyro_radps_y","imu_gyro_radps_z",
    ]

    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=fieldnames, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)

    suffix = (session_id or "").replace(":", "-")[-8:] or datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    fname  = f"tru-track-{device_id.replace(':', '-')}-{suffix}.csv"
    return Response(
        buf.getvalue(),
        mimetype="text/csv",
        headers={"Content-Disposition": f'attachment; filename="{fname}"'},
    )


@app.route("/api/v1/session/geojson")
@jwt_required()
def session_geojson():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id:
        return jsonify({"error": "device_id required"}), 400

    q = request_session_filter(device_id)

    # GNSS track — fix_valid=True only
    gnss_q = {**q, "fix_valid": True,
               "lat": {"$nin": [None, 0]},
               "lon": {"$nin": [None, 0]}}
    gnss_docs = list(db.gnss_raw.find(
        gnss_q, {"_id": 0, "lat": 1, "lon": 1, "alt": 1}
    ).sort("t_server", 1))

    # ESKF track — init_valid + alignment_valid
    eskf_q = {**q, "init_valid": True, "alignment_valid": True,
               "lat": {"$nin": [None, 0]},
               "lon": {"$nin": [None, 0]}}
    eskf_docs = list(db.eskf_state.find(
        eskf_q, {"_id": 0, "lat": 1, "lon": 1, "alt": 1}
    ).sort("t_server", 1))

    features = []
    if gnss_docs:
        features.append({
            "type": "Feature",
            "properties": {"name": "GNSS track", "source": "gnss_raw"},
            "geometry": {
                "type": "LineString",
                "coordinates": [[d["lon"], d["lat"], d.get("alt", 0)] for d in gnss_docs]
            }
        })
    if eskf_docs:
        features.append({
            "type": "Feature",
            "properties": {"name": "ESKF track", "source": "eskf_state"},
            "geometry": {
                "type": "LineString",
                "coordinates": [[d["lon"], d["lat"], d.get("alt", 0)] for d in eskf_docs]
            }
        })

    suffix = (session_id or "").replace(":", "-")[-8:] or datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    fname  = f"tru-track-{device_id.replace(':', '-')}-{suffix}.geojson"
    return Response(
        __import__("json").dumps({"type": "FeatureCollection", "features": features}, indent=2),
        mimetype="application/geo+json",
        headers={"Content-Disposition": f'attachment; filename="{fname}"'},
    )


# ─────────────────────────────────────────────────────────────────────────────
# SERVER HEALTH
# ─────────────────────────────────────────────────────────────────────────────
@app.route("/api/v1/server/health")
def server_health():
    import subprocess as _sp
    mem  = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    cpu  = psutil.cpu_percent(interval=0.2)

    svcs = ["mongod", "mosquitto", "nginx",
            "tru-track-ingest", "tru-track-backend"]
    svc_status = {}
    for svc in svcs:
        try:
            r = _sp.run(["systemctl", "is-active", svc],
                        capture_output=True, text=True, timeout=2)
            svc_status[svc] = (r.stdout.strip() == "active")
        except Exception:
            svc_status[svc] = False

    core      = ["mongod", "tru-track-ingest", "tru-track-backend"]
    core_down = [s for s in core if not svc_status.get(s)]
    any_down  = [s for s in svcs  if not svc_status.get(s)]

    if core_down or disk.percent > 90:
        level = "critical"
    elif cpu > 85 or mem.percent > 90 or disk.percent > 85 or (any_down and not core_down):
        level = "major_warning"
    elif cpu > 70 or mem.percent > 80 or disk.percent > 75:
        level = "warning"
    else:
        level = "healthy"

    return jsonify({
        "status":         "ok",
        "status_level":   level,
        "cpu_percent":    cpu,
        "memory": {
            "used_gb":    round(mem.used  / 1e9, 2),
            "total_gb":   round(mem.total / 1e9, 2),
            "percent":    mem.percent,
        },
        "disk": {
            "used_gb":    round(disk.used  / 1e9, 2),
            "total_gb":   round(disk.total / 1e9, 2),
            "percent":    disk.percent,
        },
        "uptime_hours":   round((time.time() - psutil.boot_time()) / 3600, 1),
        "services":       svc_status,
        "services_up":    sum(svc_status.values()),
        "services_total": len(svcs),
        "timestamp":      datetime.utcnow().isoformat(),
    })


# ─────────────────────────────────────────────────────────────────────────────
# WEBSOCKET
# ─────────────────────────────────────────────────────────────────────────────
@socketio.on("join_device")
def on_join_device(data):
    device_id = data.get("device_id")
    if device_id:
        join_room(device_id)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    socketio.run(app, host=SERVER_HOST, port=SERVER_PORT, allow_unsafe_werkzeug=True)
