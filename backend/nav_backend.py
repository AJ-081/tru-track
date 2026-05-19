#!/usr/bin/env python3
"""TRU-TRACK Navigation Backend API.
Flask + Flask-JWT-Extended + Flask-SocketIO.
Day 6: auth core, maps key, static serving, WebSocket skeleton.
Day 7: all /api/v1/ data routes added below the marked section.
"""
import csv
import io
import json
import logging
import os
import time
from datetime import datetime, timedelta, timezone
from functools import wraps

import bcrypt
import psutil
from bson import ObjectId
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_cors import CORS
from flask_jwt_extended import (
    JWTManager,
    create_access_token,
    create_refresh_token,
    get_jwt,
    get_jwt_identity,
    jwt_required,
    verify_jwt_in_request,
)
from flask_socketio import SocketIO, join_room
from pymongo import MongoClient, ASCENDING, DESCENDING

load_dotenv("/etc/tru-track/secrets.env")

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")
logging.getLogger("werkzeug").setLevel(logging.ERROR)

# ── App ───────────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.config["SECRET_KEY"]                = os.environ["FLASK_SECRET_KEY"]
app.config["JWT_SECRET_KEY"]            = os.environ["JWT_SECRET_KEY"]
app.config["JWT_ACCESS_TOKEN_EXPIRES"]  = timedelta(hours=8)
app.config["JWT_REFRESH_TOKEN_EXPIRES"] = timedelta(days=30)

CORS(app, resources={r"/api/*": {"origins": "*"}})
jwt = JWTManager(app)
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

DASHBOARD_DIR = os.environ.get("DASHBOARD_DIR", "/opt/tru-track/dashboard")
SERVER_PORT   = int(os.environ.get("SERVER_PORT", 9000))

# ── MongoDB ───────────────────────────────────────────────────────────────────
MONGO_URI = os.environ.get("MONGO_URI", "mongodb://127.0.0.1:27017/?directConnection=true")
MONGO_DB  = os.environ.get("MONGO_DB", "nav")
mongo     = MongoClient(MONGO_URI)
db        = mongo[MONGO_DB]

# ── Role system ───────────────────────────────────────────────────────────────
ROLE_LEVELS = {"viewer": 1, "analyst": 2, "admin": 3, "superadmin": 4}

def role_required(min_role: str):
    def decorator(fn):
        @wraps(fn)
        def wrapper(*args, **kwargs):
            verify_jwt_in_request()
            role = get_jwt().get("role", "viewer")
            if ROLE_LEVELS.get(role, 0) < ROLE_LEVELS.get(min_role, 99):
                return jsonify({"error": "Insufficient role"}), 403
            return fn(*args, **kwargs)
        return wrapper
    return decorator

# ── Auth routes ───────────────────────────────────────────────────────────────
@app.route("/api/v1/auth/login", methods=["POST"])
def auth_login():
    data     = request.get_json(silent=True) or {}
    email    = (data.get("email") or "").lower().strip()
    password = (data.get("password") or "").encode()
    if not email or not password:
        return jsonify({"error": "email and password required"}), 400
    user = db.users.find_one({"email": email})
    if not user or not bcrypt.checkpw(password, user["password_hash"].encode()):
        return jsonify({"error": "Invalid credentials"}), 401
    db.users.update_one({"_id": user["_id"]},
                        {"$set": {"last_login": datetime.utcnow()}})
    extra = {"role": user["role"], "name": user.get("name", "")}
    return jsonify({
        "access_token":  create_access_token(identity=email,
                                              additional_claims=extra),
        "refresh_token": create_refresh_token(identity=email,
                                              additional_claims=extra),
        "role": user["role"],
        "name": user.get("name", ""),
    })

@app.route("/api/v1/auth/refresh", methods=["POST"])
@jwt_required(refresh=True)
def auth_refresh():
    identity = get_jwt_identity()
    claims   = get_jwt()
    extra    = {"role": claims.get("role"), "name": claims.get("name", "")}
    return jsonify({
        "access_token": create_access_token(identity=identity,
                                             additional_claims=extra)
    })

@app.route("/api/v1/auth/me")
@jwt_required()
def auth_me():
    claims = get_jwt()
    return jsonify({
        "email": get_jwt_identity(),
        "role":  claims.get("role"),
        "name":  claims.get("name"),
    })

# ── Maps key (auth-gated, never in HTML) ─────────────────────────────────────
@app.route("/api/v1/maps/key")
@jwt_required()
def maps_key():
    return jsonify({"key": os.environ.get("GOOGLE_MAPS_KEY", "")})

# ── Server health (public — used by monitoring) ───────────────────────────────

@app.route("/api/v1/server/health")
def server_health():
    import subprocess as _sp
    mem  = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    cpu  = psutil.cpu_percent(interval=0.2)

    # Check all services
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

    core = ["mongod", "tru-track-ingest", "tru-track-backend"]
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
        "status":          "ok",
        "status_level":    level,
        "cpu_percent":     cpu,
        "memory": {
            "used_gb":     round(mem.used  / 1e9, 2),
            "total_gb":    round(mem.total / 1e9, 2),
            "percent":     mem.percent,
        },
        "disk": {
            "used_gb":     round(disk.used  / 1e9, 2),
            "total_gb":    round(disk.total / 1e9, 2),
            "percent":     disk.percent,
        },
        "uptime_hours":    round((time.time() - psutil.boot_time()) / 3600, 1),
        "services":        svc_status,
        "services_up":     sum(svc_status.values()),
        "services_total":  len(svcs),
        "timestamp":       datetime.utcnow().isoformat(),
    })

# ── Static dashboard ──────────────────────────────────────────────────────────
@app.route("/")
def dashboard():
    return send_from_directory(DASHBOARD_DIR, "index.html")

@app.route("/login")
def login_page():
    return send_from_directory(DASHBOARD_DIR, "login.html")

@app.route("/<path:filename>")
def static_files(filename):
    return send_from_directory(DASHBOARD_DIR, filename)

# ── WebSocket ─────────────────────────────────────────────────────────────────
@sio.on("join_device")
def on_join_device(data):
    device_id = data.get("device_id", "all")
    join_room(device_id)

# ═══════════════════════════════════════════════════════════════════════════════
# DAY 7 — API DATA ROUTES GO HERE
# ═══════════════════════════════════════════════════════════════════════════════

# ── Devices ───────────────────────────────────────────────────────────────────
@app.route("/api/v1/devices")
@jwt_required()
def api_devices():
    docs = list(db.device_registry.find({}, {"_id": 0}).sort("last_seen", DESCENDING))
    for d in docs:
        if isinstance(d.get("last_seen"), datetime):
            d["last_seen"] = d["last_seen"].isoformat()
        if isinstance(d.get("first_seen"), datetime):
            d["first_seen"] = d["first_seen"].isoformat()
    return jsonify(docs)


# ── Sessions ──────────────────────────────────────────────────────────────────
@app.route("/api/v1/sessions/<device_id>")
@jwt_required()
def api_sessions(device_id):
    cursor = db.sessions.find(
        {"device_id": device_id}, {"_id": 0}
    ).sort("started_at_server", DESCENDING).limit(200)
    docs = []
    for d in cursor:
        for k in ("started_at_server", "last_packet_at"):
            if isinstance(d.get(k), datetime):
                d[k] = d[k].isoformat()
        if isinstance(d.get("started_at_server"), str) and isinstance(d.get("last_packet_at"), str):
            try:
                t0 = datetime.fromisoformat(d["started_at_server"])
                t1 = datetime.fromisoformat(d["last_packet_at"])
                d["duration_s"] = round((t1 - t0).total_seconds())
            except Exception:
                d["duration_s"] = 0
        docs.append(d)
    return jsonify(docs)


# ── Device latest state ───────────────────────────────────────────────────────
@app.route("/api/v1/device/latest/<device_id>")
@jwt_required()
def api_device_latest(device_id):
    doc = db.device_latest.find_one({"device_id": device_id}, {"_id": 0})
    if not doc:
        return jsonify({"error": "device not found"}), 404
    if isinstance(doc.get("last_seen"), datetime):
        doc["last_seen"] = doc["last_seen"].isoformat()
    return jsonify(doc)


# ── GNSS track ────────────────────────────────────────────────────────────────
@app.route("/api/v1/gnss/track/<device_id>")
@jwt_required()
def api_gnss_track(device_id):
    session_id = request.args.get("session_id")
    query = {"device_id": device_id}
    if session_id:
        query["session_id"] = session_id
    cursor = db.gnss_raw.find(
        query,
        {"_id": 0, "lat": 1, "lon": 1, "alt": 1, "speed": 1,
         "sats": 1, "hdop": 1, "fix_type": 1, "t_ms": 1, "t_server": 1}
    ).sort("t_server", ASCENDING).limit(50000)
    points = []
    for d in cursor:
        if not d.get("lat") or not d.get("lon"):
            continue
        if d["lat"] == 0 and d["lon"] == 0:
            continue
        if isinstance(d.get("t_server"), datetime):
            d["t_server"] = d["t_server"].isoformat()
        points.append(d)
    return jsonify(points)


# ── ESKF track ────────────────────────────────────────────────────────────────
@app.route("/api/v1/eskf/track/<device_id>")
@jwt_required()
def api_eskf_track(device_id):
    session_id = request.args.get("session_id")
    query = {"device_id": device_id}
    if session_id:
        query["session_id"] = session_id
    cursor = db.eskf_state.find(
        query,
        {"_id": 0, "lat": 1, "lon": 1, "alt": 1,
         "vE": 1, "vN": 1, "vU": 1, "roll": 1, "pitch": 1, "yaw": 1,
         "t_ms": 1, "t_server": 1}
    ).sort("t_server", ASCENDING).limit(50000)
    points = []
    for d in cursor:
        if not d.get("lat") or not d.get("lon"):
            continue
        if isinstance(d.get("t_server"), datetime):
            d["t_server"] = d["t_server"].isoformat()
        points.append(d)
    return jsonify(points)


# ── IMU latest ────────────────────────────────────────────────────────────────
@app.route("/api/v1/imu/latest/<device_id>")
@jwt_required()
def api_imu_latest(device_id):
    doc = db.imu_raw.find_one(
        {"device_id": device_id},
        {"_id": 0},
        sort=[("t_server", DESCENDING)]
    )
    if not doc:
        return jsonify({"error": "no IMU data"}), 404
    if isinstance(doc.get("t_server"), datetime):
        doc["t_server"] = doc["t_server"].isoformat()
    return jsonify(doc)


# ── Alerts ────────────────────────────────────────────────────────────────────
@app.route("/api/v1/alerts/<device_id>")
@jwt_required()
def api_alerts(device_id):
    doc = db.device_latest.find_one({"device_id": device_id}, {"_id": 0})
    if not doc:
        return jsonify([])
    st     = doc.get("status", {})
    alerts = []
    last_seen = doc.get("last_seen")
    if isinstance(last_seen, datetime):
        age_s = (datetime.utcnow() - last_seen).total_seconds()
        if age_s > 10:
            alerts.append({"code": "STALE_DATA", "msg": f"No packet for {int(age_s)}s", "level": "error"})
    if not st.get("gnss_fix"):
        alerts.append({"code": "GNSS_NO_FIX", "msg": "No GNSS fix", "level": "warning"})
    if (st.get("sats") or 0) < 6:
        alerts.append({"code": "GNSS_WEAK", "msg": f"Only {st.get('sats',0)} satellites", "level": "warning"})
    if (st.get("hdop") or 99) > 2.0:
        alerts.append({"code": "GNSS_ACCURACY_LOW", "msg": f"HDOP {st.get('hdop',99):.1f}", "level": "warning"})
    if (st.get("wifi_rssi_dbm") or -127) < -80:
        alerts.append({"code": "SIGNAL_WEAK", "msg": f"RSSI {st.get('wifi_rssi_dbm')} dBm", "level": "warning"})
    if not st.get("eskf_init"):
        alerts.append({"code": "ESKF_NOT_INIT", "msg": "ESKF not initialized", "level": "info"})
    bpct = st.get("battery_pct", -1)
    if bpct is not None and 0 <= bpct < 20:
        alerts.append({"code": "LOW_BATTERY", "msg": f"Battery {bpct}%", "level": "error"})
    return jsonify(alerts)


# ── AI latest ─────────────────────────────────────────────────────────────────
@app.route("/api/v1/ai/latest/<device_id>")
@jwt_required()
def api_ai_latest(device_id):
    doc = db.ai_sequences.find_one(
        {"device_id": device_id},
        {"_id": 0, "features": 0},
        sort=[("t_end", DESCENDING)]
    )
    if not doc:
        return jsonify({"driver_score": None, "anomaly_flag": False, "model_version": "none"})
    for k in ("t_start", "t_end"):
        if isinstance(doc.get(k), datetime):
            doc[k] = doc[k].isoformat()
    return jsonify(doc)


# ── CSV export ────────────────────────────────────────────────────────────────
@app.route("/api/v1/session/export")
@role_required("analyst")
def api_session_export():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id or not session_id:
        return jsonify({"error": "device_id and session_id required"}), 400

    query = {"device_id": device_id, "session_id": session_id}
    gnss_docs = list(db.gnss_raw.find(query, {"_id": 0}).sort("t_ms", ASCENDING))
    eskf_docs = list(db.eskf_state.find(query, {"_id": 0}).sort("t_ms", ASCENDING))
    imu_docs  = list(db.imu_raw.find(query, {"_id": 0}).sort("t_ms", ASCENDING))

    # Index by t_ms for alignment
    eskf_idx = {d["t_ms"]: d for d in eskf_docs if d.get("t_ms")}
    imu_idx  = {d["t_ms"]: d for d in imu_docs  if d.get("t_ms")}

    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow([
        "t_ms", "t_server",
        "gnss_lat", "gnss_lon", "gnss_alt", "gnss_speed", "gnss_course",
        "gnss_sats", "gnss_hdop", "gnss_fix_type",
        "eskf_lat", "eskf_lon", "eskf_alt",
        "eskf_vE", "eskf_vN", "eskf_vU",
        "eskf_roll", "eskf_pitch", "eskf_yaw",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
    ])
    for g in gnss_docs:
        t  = g.get("t_ms")
        e  = eskf_idx.get(t, {})
        im = imu_idx.get(t, {})
        am = im.get("accel_mps2") or []
        gm = im.get("gyro_radps") or []
        ts = g.get("t_server")
        writer.writerow([
            t, ts.isoformat() if isinstance(ts, datetime) else ts,
            g.get("lat"), g.get("lon"), g.get("alt"),
            g.get("speed"), g.get("course"),
            g.get("sats"), g.get("hdop"), g.get("fix_type"),
            e.get("lat"), e.get("lon"), e.get("alt"),
            e.get("vE"), e.get("vN"), e.get("vU"),
            e.get("roll"), e.get("pitch"), e.get("yaw"),
            am[0] if len(am) > 0 else None,
            am[1] if len(am) > 1 else None,
            am[2] if len(am) > 2 else None,
            gm[0] if len(gm) > 0 else None,
            gm[1] if len(gm) > 1 else None,
            gm[2] if len(gm) > 2 else None,
        ])

    fname = f"tru-track-{device_id.replace(':','-')}-{session_id[-8:]}.csv"
    return Response(
        output.getvalue(),
        mimetype="text/csv",
        headers={"Content-Disposition": f"attachment; filename={fname}"}
    )


# ── GeoJSON export ────────────────────────────────────────────────────────────
@app.route("/api/v1/session/geojson")
@jwt_required()
def api_session_geojson():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id or not session_id:
        return jsonify({"error": "device_id and session_id required"}), 400

    query = {"device_id": device_id, "session_id": session_id}
    gnss_pts = [
        [d["lon"], d["lat"], d.get("alt", 0)]
        for d in db.gnss_raw.find(query, {"_id": 0, "lat": 1, "lon": 1, "alt": 1})
                             .sort("t_ms", ASCENDING)
        if d.get("lat") and d.get("lon") and not (d["lat"] == 0 and d["lon"] == 0)
    ]
    eskf_pts = [
        [d["lon"], d["lat"], d.get("alt", 0)]
        for d in db.eskf_state.find(query, {"_id": 0, "lat": 1, "lon": 1, "alt": 1})
                               .sort("t_ms", ASCENDING)
        if d.get("lat") and d.get("lon")
    ]
    fc = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "geometry": {"type": "LineString", "coordinates": gnss_pts},
             "properties": {"name": "GNSS track", "color": "#FF4444"}},
            {"type": "Feature", "geometry": {"type": "LineString", "coordinates": eskf_pts},
             "properties": {"name": "ESKF track", "color": "#4488FF"}},
        ]
    }
    fname = f"tru-track-{device_id.replace(':','-')}-{session_id[-8:]}.geojson"
    return Response(
        json.dumps(fc),
        mimetype="application/geo+json",
        headers={"Content-Disposition": f"attachment; filename={fname}"}
    )

# ── Start ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.info(f"TRU-TRACK backend starting on port {SERVER_PORT}")
    sio.run(app, host="127.0.0.1", port=SERVER_PORT,
            debug=False, allow_unsafe_werkzeug=True)
