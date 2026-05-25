#!/usr/bin/env python3
"""
TRU-TRACK Flask Backend — v2.1
- All timestamps returned as IST (UTC+5:30)
- Session labels in IST format
- Both /api/ (dashboard compat) and /api/v1/ (JWT) routes
- Backward-compatible track filters for pre-v2.1 data
- imu_raw as CSV export base (continuous, no gaps)
"""
import csv
import io
import json
import logging
import os
import subprocess
import time
from datetime import datetime, timedelta, timezone

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

MONGO_URI       = os.getenv("MONGO_URI", "mongodb://127.0.0.1:27017/?directConnection=true")
MONGO_DB        = os.getenv("MONGO_DB", "nav")
FLASK_SECRET    = os.getenv("FLASK_SECRET_KEY", "changeme")
JWT_SECRET      = os.getenv("JWT_SECRET_KEY", "changeme")
DASHBOARD_DIR   = os.getenv("DASHBOARD_DIR", "/opt/tru-track/dashboard")
SERVER_HOST     = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT     = int(os.getenv("SERVER_PORT", 9000))
GOOGLE_MAPS_KEY = os.getenv("GOOGLE_MAPS_KEY", "")

logging.getLogger("werkzeug").setLevel(logging.ERROR)

IST = timezone(timedelta(hours=5, minutes=30))

def to_ist_iso(dt):
    if not isinstance(dt, datetime): return None
    if dt.tzinfo is None: dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(IST).isoformat()

def to_ist_display(dt):
    if not isinstance(dt, datetime): return None
    if dt.tzinfo is None: dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(IST).strftime("%d/%m/%Y %H:%M:%S")

def to_ist_short(dt):
    if not isinstance(dt, datetime): return None
    if dt.tzinfo is None: dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(IST).strftime("%d/%m %H:%M")

def _serialize(obj):
    if isinstance(obj, datetime): return to_ist_iso(obj)
    if isinstance(obj, dict): return {k: _serialize(v) for k, v in obj.items()}
    if isinstance(obj, list): return [_serialize(i) for i in obj]
    return obj

app = Flask(__name__)
app.config["SECRET_KEY"]                = FLASK_SECRET
app.config["JWT_SECRET_KEY"]            = JWT_SECRET
app.config["JWT_ACCESS_TOKEN_EXPIRES"]  = timedelta(hours=8)
app.config["JWT_REFRESH_TOKEN_EXPIRES"] = timedelta(days=30)

jwt      = JWTManager(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

mongo = MongoClient(MONGO_URI)
db    = mongo[MONGO_DB]

DEFAULT_HISTORY_DAYS = 30

ROLE_LEVELS = {"viewer": 1, "analyst": 2, "admin": 3, "superadmin": 4}

def role_required(min_role):
    def decorator(fn):
        from functools import wraps
        @wraps(fn)
        @jwt_required()
        def wrapper(*args, **kwargs):
            user = db.users.find_one({"email": get_jwt_identity()}, {"role": 1})
            if not user: return jsonify({"error": "User not found"}), 401
            if ROLE_LEVELS.get(user.get("role","viewer"),0) < ROLE_LEVELS.get(min_role,99):
                return jsonify({"error": "Insufficient role"}), 403
            return fn(*args, **kwargs)
        return wrapper
    return decorator

def parse_iso(value):
    if not value: return None
    try:
        if value.endswith("Z"): value = value[:-1] + "+00:00"
        dt = datetime.fromisoformat(value)
        if dt.tzinfo is not None: return dt.astimezone(timezone.utc).replace(tzinfo=None)
        return dt
    except: return None

def request_session_filter(device_id):
    q = {"device_id": device_id}
    sid   = request.args.get("session_id")
    start = parse_iso(request.args.get("start"))
    end   = parse_iso(request.args.get("end"))
    if sid:   q["session_id"] = sid; return q
    if start or end:
        ts = {}
        if start: ts["$gte"] = start
        if end:   ts["$lte"] = end
        if ts:    q["t_server"] = ts
        return q
    q["t_server"] = {"$gte": datetime.utcnow() - timedelta(days=DEFAULT_HISTORY_DAYS)}
    return q

def check_password(user, password):
    pw = user.get("password_hash") or user.get("password") or ""
    if not pw: return False
    try: return bcrypt.checkpw(password.encode(), pw.encode())
    except: return False

# ── AUTH ──────────────────────────────────────────────────────────────────────
def _do_login():
    data = request.get_json() or {}
    email = (data.get("email") or "").strip().lower()
    password = data.get("password") or ""
    if not email or not password:
        return jsonify({"error": "Email and password required"}), 400
    user = db.users.find_one({"email": email})
    if not user or not check_password(user, password):
        return jsonify({"error": "Invalid credentials"}), 401
    return jsonify({
        "access_token":  create_access_token(identity=email),
        "refresh_token": create_refresh_token(identity=email),
        "role":          user.get("role", "viewer"),
        "name":          user.get("name", ""),
    })

@app.route("/api/auth/login",    methods=["POST"])
@app.route("/api/v1/auth/login", methods=["POST"])
def login(): return _do_login()

@app.route("/api/auth/refresh",    methods=["POST"])
@app.route("/api/v1/auth/refresh", methods=["POST"])
@jwt_required(refresh=True)
def refresh_token():
    return jsonify({"access_token": create_access_token(identity=get_jwt_identity())})

@app.route("/api/auth/me")
@app.route("/api/v1/auth/me")
@jwt_required()
def me():
    u = db.users.find_one({"email": get_jwt_identity()},
                          {"_id":0,"password_hash":0,"password":0})
    return jsonify(u or {})

@app.route("/api/maps/key")
@app.route("/api/v1/maps/key")
@jwt_required()
def maps_key(): return jsonify({"key": GOOGLE_MAPS_KEY})

# ── STATIC ────────────────────────────────────────────────────────────────────
@app.route("/")
def dashboard(): return send_from_directory(DASHBOARD_DIR, "index.html")
@app.route("/login")
def login_page(): return send_from_directory(DASHBOARD_DIR, "login.html")
@app.route("/fleet")
def fleet_page(): return send_from_directory(DASHBOARD_DIR, "fleet.html")

@app.route("/<path:filename>")
def static_files(filename): return send_from_directory(DASHBOARD_DIR, filename)

# ── DEVICES ───────────────────────────────────────────────────────────────────
def _get_devices():
    docs = list(db.device_registry.find(
        {}, {"_id":0,"device_id":1,"first_seen":1,"last_seen":1,
             "fw_version":1,"total_packets":1}
    ).sort("last_seen", -1))
    if not docs:
        ids = (set(db.gnss_raw.distinct("device_id")) |
               set(db.eskf_state.distinct("device_id")) |
               set(db.imu_raw.distinct("device_id")))
        return jsonify([{"device_id":x,"label":x} for x in sorted(ids) if x])
    result = [{
        "device_id":     d.get("device_id",""),
        "label":         d.get("device_id",""),
        "fw_version":    d.get("fw_version"),
        "total_packets": d.get("total_packets"),
        "first_seen":    to_ist_iso(d.get("first_seen")),
        "last_seen":     to_ist_iso(d.get("last_seen")),
    } for d in docs]
    # Also set id field = device_id string for old dashboard compat
    for r in result:
        r["id"] = r["device_id"]
    return jsonify(result)

@app.route("/api/devices")
@app.route("/api/v1/devices")
@jwt_required(optional=True)
def devices(): return _get_devices()

# ── SESSIONS ──────────────────────────────────────────────────────────────────
def _get_sessions(device_id):
    docs = list(db.sessions.find(
        {"device_id": device_id},
        {"_id":0,"session_id":1,"started_at_server":1,
         "last_seen":1,"last_packet_at":1,
         "packet_count":1,"boot_count":1,"fw_version":1,"started_t_epoch_ms":1}
    ).sort("started_at_server", -1))
    result = []
    for d in docs:
        started = d.get("started_at_server")
        last    = d.get("last_seen") or d.get("last_packet_at")
        dur_s, dur_min = None, 0
        if isinstance(started, datetime) and isinstance(last, datetime):
            dur_s   = max(0, int((last - started).total_seconds()))
            dur_min = dur_s // 60
        ist_label = to_ist_short(started) or "Unknown"
        result.append({
            "session_id":         d.get("session_id"),
            "started_at_server":  to_ist_iso(started),
            "last_seen":          to_ist_iso(last),
            "duration_s":         dur_s,
            "packet_count":       d.get("packet_count"),
            "boot_count":         d.get("boot_count"),
            "fw_version":         d.get("fw_version"),
            "started_t_epoch_ms": d.get("started_t_epoch_ms", 0),
            "label":              f"{ist_label} IST ({dur_min}m)",
            "gnss_points":        d.get("packet_count", 1),
            "eskf_points":        d.get("packet_count", 1),
        })
    return jsonify(result)

@app.route("/api/sessions/<device_id>")
@app.route("/api/v1/sessions/<device_id>")
@jwt_required(optional=True)
def sessions(device_id): return _get_sessions(device_id)

# ── DEVICE LATEST ─────────────────────────────────────────────────────────────
def _get_latest(device_id):
    doc = db.device_latest.find_one({"device_id": device_id}, {"_id": 0})
    return jsonify(_serialize(doc) if doc else {})

@app.route("/api/device/latest/<device_id>")
@app.route("/api/v1/device/latest/<device_id>")
@jwt_required(optional=True)
def device_latest(device_id): return _get_latest(device_id)

# ── TRACKS ────────────────────────────────────────────────────────────────────
def _gnss_track(device_id):
    q = request_session_filter(device_id)
    q["fix_valid"] = {"$ne": False}
    q["lat"] = {"$nin": [None, 0]}
    q["lon"] = {"$nin": [None, 0]}
    docs = list(db.gnss_raw.find(
        q, {"_id":0,"lat":1,"lon":1,"alt":1,"speed":1,"course":1,
            "sats":1,"hdop":1,"t_ms":1,"t_server":1,"t_epoch_ms":1}
    ).sort("t_server", 1).limit(50000))
    for d in docs: d["t_server"] = to_ist_iso(d.get("t_server"))
    return jsonify(docs)

def _eskf_track(device_id):
    q = request_session_filter(device_id)
    q["init_valid"]      = {"$ne": False}
    q["alignment_valid"] = {"$ne": False}
    q["lat"] = {"$nin": [None, 0]}
    q["lon"] = {"$nin": [None, 0]}
    docs = list(db.eskf_state.find(
        q, {"_id":0,"lat":1,"lon":1,"alt":1,"vE":1,"vN":1,"vU":1,
            "roll":1,"pitch":1,"yaw":1,"t_ms":1,"t_server":1,"t_epoch_ms":1}
    ).sort("t_server", 1).limit(50000))
    for d in docs: d["t_server"] = to_ist_iso(d.get("t_server"))
    return jsonify(docs)

@app.route("/api/gnss/track/<device_id>")
@app.route("/api/v1/gnss/track/<device_id>")
@jwt_required(optional=True)
def gnss_track(device_id): return _gnss_track(device_id)

@app.route("/api/eskf/track/<device_id>")
@app.route("/api/v1/eskf/track/<device_id>")
@jwt_required(optional=True)
def eskf_track(device_id): return _eskf_track(device_id)

@app.route("/api/imu/latest/<device_id>")
@app.route("/api/v1/imu/latest/<device_id>")
@jwt_required(optional=True)
def imu_latest(device_id):
    doc = db.imu_raw.find_one({"device_id":device_id},
                              sort=[("t_server",-1)], projection={"_id":0})
    return jsonify(_serialize(doc) if doc else {})

# ── ALERTS ────────────────────────────────────────────────────────────────────
def _get_alerts(device_id):
    doc    = db.device_latest.find_one({"device_id": device_id}) or {}
    status = doc.get("status") or {}
    result = []
    if not status.get("gnss_fix"):
        result.append({"level":"error","code":"GNSS_NO_FIX","msg":"No GNSS fix"})
    elif (status.get("sats") or 0) < 4:
        result.append({"level":"warning","code":"GNSS_FEW_SATS",
                        "msg":f"Only {status.get('sats',0)} satellites"})
    rssi = status.get("wifi_rssi_dbm")
    if rssi is not None and rssi < -80:
        result.append({"level":"warning","code":"SIGNAL_WEAK","msg":f"WiFi RSSI {rssi} dBm"})
    last_seen = doc.get("last_seen")
    if isinstance(last_seen, datetime):
        age = (datetime.utcnow() - last_seen).total_seconds()
        if age > 30:
            result.append({"level":"error","code":"STALE_DATA","msg":f"No packet for {int(age)}s"})
    bpct = status.get("battery_pct", -1)
    if bpct is not None and 0 <= bpct < 10:
        result.append({"level":"error","code":"BATTERY_CRITICAL","msg":f"Battery {bpct}%"})
    elif bpct is not None and 10 <= bpct < 20:
        result.append({"level":"warning","code":"BATTERY_LOW","msg":f"Battery {bpct}%"})
    return jsonify(result)

@app.route("/api/alerts/<device_id>")
@app.route("/api/v1/alerts/<device_id>")
@jwt_required(optional=True)
def alerts(device_id): return _get_alerts(device_id)

@app.route("/api/ai/latest/<device_id>")
@app.route("/api/v1/ai/latest/<device_id>")
@jwt_required(optional=True)
def ai_latest(device_id):
    doc = db.ai_sequences.find_one({"device_id":device_id},
                                   sort=[("t_start",-1)], projection={"_id":0})
    return jsonify(doc or {"driver_score":None,"anomaly":None})

# ── CSV EXPORT ────────────────────────────────────────────────────────────────
def docs_to_csv_rows(device_id):
    bq = request_session_filter(device_id)
    imu_docs  = list(db.imu_raw.find(bq,
        {"_id":0,"device_id":1,"session_id":1,"boot_count":1,
         "t_ms":1,"t_server":1,"t_epoch_ms":1,"ntp_synced":1,
         "calibrated":1,"accel_raw":1,"gyro_raw":1,"accel_mps2":1,
         "gyro_radps":1,"gyro_bias_radps":1}).sort("t_server",1))
    gnss_docs = list(db.gnss_raw.find(bq,
        {"_id":0,"t_ms":1,"t_server":1,"fix_valid":1,"lat":1,"lon":1,"alt":1,
         "speed":1,"course":1,"sats":1,"hdop":1,"fix_type":1,"age_ms":1}).sort("t_server",1))
    eskf_docs = list(db.eskf_state.find(bq,
        {"_id":0,"t_ms":1,"t_server":1,"init_valid":1,"alignment_valid":1,
         "lat":1,"lon":1,"alt":1,"vE":1,"vN":1,"vU":1,
         "roll":1,"pitch":1,"yaw":1,"innov":1}).sort("t_server",1))

    def idx(docs):
        d = {}
        for doc in docs:
            k = doc.get("t_ms")
            if k is not None: d[k] = doc
        return d

    gi, ei = idx(gnss_docs), idx(eskf_docs)
    rows = []
    def _e(lst, i): return lst[i] if lst and len(lst) > i else ""
    for imu in imu_docs:
        tms  = imu.get("t_ms")
        ts   = imu.get("t_server")
        gnss = gi.get(tms, {})
        eskf = ei.get(tms, {})
        innov = eskf.get("innov") or {}
        ar = imu.get("accel_raw") or [None]*3
        gr = imu.get("gyro_raw")  or [None]*3
        am = imu.get("accel_mps2") or [None]*3
        gm = imu.get("gyro_radps") or [None]*3
        rows.append({
            "device_id":device_id,"session_id":imu.get("session_id"),
            "boot_count":imu.get("boot_count"),"t_ms":tms,
            "t_server_utc":ts.isoformat() if isinstance(ts,datetime) else ts,
            "t_server_ist":to_ist_display(ts) if isinstance(ts,datetime) else "",
            "t_epoch_ms_ist":imu.get("t_epoch_ms",0),"ntp_synced":imu.get("ntp_synced",False),
            "gnss_fix_valid":gnss.get("fix_valid",""),"gnss_lat":gnss.get("lat",""),
            "gnss_lon":gnss.get("lon",""),"gnss_alt":gnss.get("alt",""),
            "gnss_speed":gnss.get("speed",""),"gnss_course":gnss.get("course",""),
            "gnss_sats":gnss.get("sats",""),"gnss_hdop":gnss.get("hdop",""),
            "gnss_fix_type":gnss.get("fix_type",""),"gnss_age_ms":gnss.get("age_ms",""),
            "eskf_init_valid":eskf.get("init_valid",""),
            "eskf_alignment_valid":eskf.get("alignment_valid",""),
            "eskf_lat":eskf.get("lat",""),"eskf_lon":eskf.get("lon",""),
            "eskf_alt":eskf.get("alt",""),"eskf_vE":eskf.get("vE",""),
            "eskf_vN":eskf.get("vN",""),"eskf_vU":eskf.get("vU",""),
            "eskf_roll":eskf.get("roll",""),"eskf_pitch":eskf.get("pitch",""),
            "eskf_yaw":eskf.get("yaw",""),
            "eskf_innov_pos_norm":innov.get("pos_norm",""),
            "eskf_innov_vel_norm":innov.get("vel_norm",""),
            "imu_calibrated":imu.get("calibrated",False),
            "imu_accel_raw_x":_e(ar,0),"imu_accel_raw_y":_e(ar,1),"imu_accel_raw_z":_e(ar,2),
            "imu_gyro_raw_x":_e(gr,0),"imu_gyro_raw_y":_e(gr,1),"imu_gyro_raw_z":_e(gr,2),
            "imu_accel_mps2_x":_e(am,0),"imu_accel_mps2_y":_e(am,1),"imu_accel_mps2_z":_e(am,2),
            "imu_gyro_radps_x":_e(gm,0),"imu_gyro_radps_y":_e(gm,1),"imu_gyro_radps_z":_e(gm,2),
        })
    return rows

def _session_export():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id: return jsonify({"error":"device_id required"}), 400
    rows = docs_to_csv_rows(device_id)
    if not rows: return jsonify({"error":"No data"}), 404
    fieldnames = [
        "device_id","session_id","boot_count","t_ms",
        "t_server_utc","t_server_ist","t_epoch_ms_ist","ntp_synced",
        "gnss_fix_valid","gnss_lat","gnss_lon","gnss_alt",
        "gnss_speed","gnss_course","gnss_sats","gnss_hdop","gnss_fix_type","gnss_age_ms",
        "eskf_init_valid","eskf_alignment_valid",
        "eskf_lat","eskf_lon","eskf_alt","eskf_vE","eskf_vN","eskf_vU",
        "eskf_roll","eskf_pitch","eskf_yaw","eskf_innov_pos_norm","eskf_innov_vel_norm",
        "imu_calibrated",
        "imu_accel_raw_x","imu_accel_raw_y","imu_accel_raw_z",
        "imu_gyro_raw_x","imu_gyro_raw_y","imu_gyro_raw_z",
        "imu_accel_mps2_x","imu_accel_mps2_y","imu_accel_mps2_z",
        "imu_gyro_radps_x","imu_gyro_radps_y","imu_gyro_radps_z",
    ]
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=fieldnames, extrasaction="ignore")
    w.writeheader(); w.writerows(rows)
    suf = (session_id or "").replace(":","-")[-8:] or \
          datetime.now(IST).strftime("%Y%m%d_%H%M%S")
    fname = f"tru-track-{device_id.replace(':','-')}-{suf}.csv"
    return Response(buf.getvalue(), mimetype="text/csv",
        headers={"Content-Disposition":f'attachment; filename="{fname}"'})

@app.route("/api/session/export")
@app.route("/api/v1/session/export")
@jwt_required(optional=True)
def session_export(): return _session_export()

# ── GEOJSON ───────────────────────────────────────────────────────────────────
def _geojson_export():
    device_id  = request.args.get("device_id")
    session_id = request.args.get("session_id")
    if not device_id: return jsonify({"error":"device_id required"}), 400
    q = request_session_filter(device_id)
    gq = {**q,"fix_valid":{"$ne":False},"lat":{"$nin":[None,0]},"lon":{"$nin":[None,0]}}
    eq = {**q,"init_valid":{"$ne":False},"alignment_valid":{"$ne":False},
          "lat":{"$nin":[None,0]},"lon":{"$nin":[None,0]}}
    gd = list(db.gnss_raw.find(gq,{"_id":0,"lat":1,"lon":1,"alt":1}).sort("t_server",1))
    ed = list(db.eskf_state.find(eq,{"_id":0,"lat":1,"lon":1,"alt":1}).sort("t_server",1))
    features = []
    if gd: features.append({"type":"Feature","properties":{"name":"GNSS track","source":"gnss_raw"},
        "geometry":{"type":"LineString","coordinates":[[d["lon"],d["lat"],d.get("alt",0)] for d in gd]}})
    if ed: features.append({"type":"Feature","properties":{"name":"ESKF track","source":"eskf_state"},
        "geometry":{"type":"LineString","coordinates":[[d["lon"],d["lat"],d.get("alt",0)] for d in ed]}})
    suf = (session_id or "").replace(":","-")[-8:] or datetime.now(IST).strftime("%Y%m%d_%H%M%S")
    fname = f"tru-track-{device_id.replace(':','-')}-{suf}.geojson"
    return Response(json.dumps({"type":"FeatureCollection","features":features},indent=2),
        mimetype="application/geo+json",
        headers={"Content-Disposition":f'attachment; filename="{fname}"'})

@app.route("/api/session/geojson")
@app.route("/api/v1/session/geojson")
@jwt_required(optional=True)
def session_geojson(): return _geojson_export()

# ── SERVER HEALTH ─────────────────────────────────────────────────────────────
def _server_health():
    mem  = psutil.virtual_memory()
    disk = psutil.disk_usage("/")
    cpu  = psutil.cpu_percent(interval=0.2)
    try:
        ping = subprocess.check_output(["ping","-c","1","-W","1","1.1.1.1"],
                                       stderr=subprocess.DEVNULL).decode()
        latency = ping.split("time=")[1].split(" ms")[0] + " ms"
    except: latency = "unreachable"
    svcs = ["mongod","mosquitto","nginx","tru-track-ingest","tru-track-backend"]
    svc_status = {}
    for s in svcs:
        try:
            r = subprocess.run(["systemctl","is-active",s],
                               capture_output=True,text=True,timeout=2)
            svc_status[s] = (r.stdout.strip() == "active")
        except: svc_status[s] = False
    core_down = [s for s in ["mongod","tru-track-ingest","tru-track-backend"] if not svc_status.get(s)]
    any_down  = [s for s in svcs if not svc_status.get(s)]
    if core_down or disk.percent > 90: level = "critical"
    elif cpu > 85 or mem.percent > 90 or disk.percent > 85 or (any_down and not core_down): level = "major_warning"
    elif cpu > 70 or mem.percent > 80 or disk.percent > 75: level = "warning"
    else: level = "healthy"
    now_ist = datetime.now(IST).strftime("%d/%m/%Y %H:%M:%S IST")
    return jsonify({
        "status":"ok","status_level":level,"cpu_percent":cpu,"latency":latency,
        "memory":{"used_gb":round(mem.used/1e9,2),"total_gb":round(mem.total/1e9,2),"percent":mem.percent},
        "disk":{"used_gb":round(disk.used/1e9,2),"total_gb":round(disk.total/1e9,2),"percent":disk.percent},
        "uptime_hours":round((time.time()-psutil.boot_time())/3600,1),
        "services":svc_status,"services_up":sum(svc_status.values()),
        "services_total":len(svcs),"timestamp_ist":now_ist,"timestamp":now_ist,
    })

@app.route("/api/server/health")
@app.route("/api/v1/server/health")
def server_health(): return _server_health()

# ── WEBSOCKET ─────────────────────────────────────────────────────────────────
@socketio.on("join_device")
def on_join_device(data):
    if data.get("device_id"): join_room(data["device_id"])


# ── Live ESKF emitter (background thread) ─────────────────────────────────────
# Polls eskf_state every 500ms for new docs and emits live_eskf via Socket.IO.
# This bridges the gap between the separate ingest process and the socketio instance.
import threading as _threading

def _live_emitter():
    from datetime import datetime, timedelta
    last_seen = {}   # device_id -> last t_server seen
    while True:
        try:
            cutoff = datetime.utcnow() - timedelta(seconds=3)
            docs = list(db.eskf_state.find(
                {
                    "t_server":        {"$gte": cutoff},
                    "init_valid":      {"$ne": False},
                    "alignment_valid": {"$ne": False},
                    "lat":             {"$nin": [None, 0]},
                    "lon":             {"$nin": [None, 0]},
                },
                {"_id":0,"device_id":1,"lat":1,"lon":1,"alt":1,
                 "vE":1,"vN":1,"vU":1,"t_server":1,"t_ms":1}
            ).sort("t_server", 1))

            for doc in docs:
                did  = doc.get("device_id")
                tsrv = doc.get("t_server")
                if not did or not tsrv:
                    continue
                if last_seen.get(did) and tsrv <= last_seen[did]:
                    continue
                last_seen[did] = tsrv
                socketio.emit("live_eskf", {
                    "device_id": did,
                    "lat":  doc.get("lat"),
                    "lon":  doc.get("lon"),
                    "alt":  doc.get("alt"),
                    "vE":   doc.get("vE"),
                    "vN":   doc.get("vN"),
                    "vU":   doc.get("vU"),
                    "t_ms": doc.get("t_ms"),
                }, room=did)
        except Exception:
            pass
        import time as _t; _t.sleep(0.5)

_threading.Thread(target=_live_emitter, daemon=True).start()

if __name__ == "__main__":
    socketio.run(app, host=SERVER_HOST, port=SERVER_PORT, allow_unsafe_werkzeug=True)

# ── FLEET ─────────────────────────────────────────────────────────────────────
def _fleet():
    docs = list(db.device_latest.find({}, {"_id": 0}))
    result = []
    for d in docs:
        device_id = d.get("device_id")
        if not device_id:
            continue
        status    = d.get("status") or {}
        gnss      = d.get("gnss")   or {}
        eskf      = d.get("eskf")   or {}
        last_seen = d.get("last_seen")

        # Position — prefer ESKF if initialized, fall back to GNSS
        lat, lon = None, None
        eskf_ok = eskf.get("init_valid") or eskf.get("alignment_valid") or status.get("eskf_init")
        if eskf_ok:
            lat = eskf.get("lat")
            lon = eskf.get("lon")
        if not lat or not lon:
            if gnss.get("fix_valid", status.get("gnss_fix")):
                lat = gnss.get("lat")
                lon = gnss.get("lon")

        fix_valid = bool(gnss.get("fix_valid", status.get("gnss_fix", False)))

        age_s = None
        if isinstance(last_seen, datetime):
            age_s = (datetime.utcnow() - last_seen).total_seconds()

        if age_s is None or age_s > 120:
            online = "offline"
        elif age_s > 30:
            online = "stale"
        else:
            online = "live"

        result.append({
            "device_id":         device_id,
            "lat":               lat,
            "lon":               lon,
            "fix_valid":         fix_valid,
            "online":            online,
            "age_s":             round(age_s, 1) if age_s is not None else None,
            "last_seen_iso":     to_ist_iso(last_seen),
            "last_seen_display": to_ist_display(last_seen) or "—",
            "sats":              status.get("sats") or gnss.get("sats"),
            "hdop":              status.get("hdop") or gnss.get("hdop"),
            "speed_mps":         gnss.get("speed"),
            "fw_version":        d.get("fw_version"),
            "eskf_init":         eskf_ok,
            "wifi_rssi_dbm":     status.get("wifi_rssi_dbm"),
        })
    return jsonify(result)

@app.route("/api/fleet")
@app.route("/api/v1/fleet")
@jwt_required(optional=True)
def fleet():
    return _fleet()
