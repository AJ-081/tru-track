#!/usr/bin/env python3
"""
TRU-TRACK ESKF Tuning Backend
Isolated Flask app on port 9001.
Read-only MongoDB access — no inserts, updates, or deletes.
Remove: kill this process + delete /opt/tru-track/tuning/
"""

import io
import csv
import json
import os
import traceback
from datetime import datetime, timedelta

from flask import Flask, jsonify, request, send_from_directory, Response
from pymongo import MongoClient

from eskf_python import replay_session, generate_eskf_config_h, DEFAULT_CFG

# ── Config ────────────────────────────────────────────────────────────────────
MONGO_URI  = os.environ.get(
    "MONGO_URI",
    "mongodb://navapp:TruTrackNav2026@127.0.0.1:27017/nav?authSource=admin&directConnection=true"
)
TUNING_PORT = int(os.environ.get("TUNING_PORT", 9001))
TUNING_DIR    = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dashboard")

# ── MongoDB — READ ONLY (only .find() used below) ─────────────────────────────
mongo = MongoClient(MONGO_URI)
db    = mongo["nav"]

# ── Flask ─────────────────────────────────────────────────────────────────────
app = Flask(__name__)

def cors(resp):
    resp.headers["Access-Control-Allow-Origin"]  = "*"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp

@app.after_request
def add_cors(resp):
    return cors(resp)

@app.errorhandler(Exception)
def handle_err(e):
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


# ── Serve tuning.html ─────────────────────────────────────────────────────────
@app.route("/tuning", strict_slashes=False)
@app.route("/tuning/", strict_slashes=False)
def tuning_page():
    import os as _os
    p = _os.path.join(DASHBOARD_DIR, "tuning.html")
    if _os.path.exists(p):
        return send_from_directory(DASHBOARD_DIR, "tuning.html")
    return send_from_directory(TUNING_DIR, "tuning.html")


# ── GET /tuning/api/sessions ──────────────────────────────────────────────────
@app.route("/tuning/api/sessions")
def get_sessions():
    """List sessions from sessions collection — fast, pre-computed."""
    cutoff = datetime.utcnow() - timedelta(days=90)

    # Read directly from sessions collection (already has all metadata)
    sess_docs = list(db.sessions.find(
        {"started_at_server": {"$gte": cutoff}},
        {"_id":0, "session_id":1, "device_id":1, "fw_version":1,
         "packet_count":1, "started_at_server":1, "duration_s":1,
         "gnss_points":1, "eskf_points":1}
    ).sort("started_at_server", -1).limit(50))

    if not sess_docs:
        # Fallback: get distinct session_ids from imu_raw (just IDs, fast with index)
        sids = db.imu_raw.distinct("session_id",
            {"t_server": {"$gte": cutoff}})[:30]
        sess_docs = [{"session_id": s, "device_id": "?", "packet_count": 0}
                     for s in sids if s]

    result = []
    for s in sess_docs:
        sid = s.get("session_id")
        if not sid:
            continue
        ft    = s.get("started_at_server")
        dur_s = s.get("duration_s") or 0
        pkts  = s.get("packet_count") or 0
        # Duration from packet count (5Hz = 200ms per packet)
        dur_s = pkts * 0.2
        mins  = int(dur_s) // 60
        did_short = s.get("device_id","?").split(":")[-3:]
        dev_label = ":".join(did_short)
        result.append({
            "session_id":  sid,
            "device_id":   s.get("device_id", "?"),
            "fw_version":  s.get("fw_version", "?"),
            "imu_count":   pkts,
            "has_gnss":    bool(s.get("gnss_points", 1)),
            "duration_s":  int(dur_s),
            "started_ist": _to_ist(ft),
            "label": f"{_to_ist(ft)} ({mins}m) — {dev_label} [{pkts}pkt]",
        })

    return jsonify(result)


# ── POST /tuning/api/run ──────────────────────────────────────────────────────
@app.route("/tuning/api/run", methods=["POST"])
def run_tuning():
    """
    Load session data from MongoDB (read-only) and replay ESKF with custom params.
    Body: {
        session_id: str,
        params: { sigma_acc, sigma_gyro, ... },
        gnss_deny_ranges: [[t_start_ms, t_end_ms], ...],   optional
        downsample_imu: int  (keep 1 in N IMU samples, default 1)
    }
    """
    body       = request.get_json(force=True) or {}
    session_id = body.get("session_id")
    if not session_id:
        return jsonify({"error": "session_id required"}), 400

    params = {**DEFAULT_CFG, **(body.get("params") or {})}
    deny   = body.get("gnss_deny_ranges") or []
    csv_gnss = body.get("csv_gnss")  # optional GNSS override from CSV
    ds     = max(1, int(body.get("downsample_imu", 1)))  # default 20Hz

    # ── Load IMU (read-only find) ─────────────────────────────────────────────
    # Prefer imu_hf (100Hz, firmware v2.1+) over imu_raw (5Hz)
    has_hf = db.imu_hf.count_documents({"session_id": session_id}, limit=1) > 0
    _imu_col = db.imu_hf if has_hf else db.imu_raw
    imu_cursor = _imu_col.find(
        {"session_id": session_id},
        {"_id":0, "t_ms":1, "accel_mps2":1, "gyro_radps":1, "calibrated":1}
    ).sort("t_ms", 1)

    imu_data = []
    for i, doc in enumerate(imu_cursor):
        if i % ds != 0:
            continue
        if len(imu_data) >= 60000:
            break
        a = doc.get("accel_mps2") or [0, 0, 0]
        g = doc.get("gyro_radps") or [0, 0, 0]
        # NOTE: accel_mps2 and gyro_radps are already bias-corrected by firmware.
        # Do NOT subtract gyro_bias_radps again — that causes double correction
        # and attitude drift of hundreds of radians over a long session.
        imu_data.append({
            "t_ms": doc["t_ms"],
            "ax": a[0], "ay": a[1], "az": a[2],
            "gx": g[0], "gy": g[1], "gz": g[2],
        })

    if not imu_data:
        return jsonify({"error": "No IMU data for this session"}), 404

    # ── Load GNSS (read-only find) ────────────────────────────────────────────
    gnss_cursor = db.gnss_raw.find(
        {"session_id": session_id},
        {"_id":0, "t_ms":1, "lat":1, "lon":1, "alt":1,
         "speed":1, "course":1, "sats":1, "fix_type":1, "sats":1, "hdop":1}
    ).sort("t_ms", 1).limit(20000)

    gnss_data = [
        {
            "t_ms":      doc["t_ms"],
            "lat":       doc.get("lat", 0),
            "lon":       doc.get("lon", 0),
            "alt":       doc.get("alt", 0),
            "speed":     doc.get("speed", 0),
            "course":    doc.get("course", 0),
            "fix_valid": (doc.get("sats") or 0) >= 5,
            "sats":      doc.get("sats", 0),
            "hdop":      doc.get("hdop", 99),
        }
        for doc in gnss_cursor
    ]

    # ── Load stored ESKF track (sampled — display only) ────────────────────────
    _all_eskf = list(db.eskf_state.find(
        {"session_id": session_id},
        {"_id":0, "t_ms":1, "lat":1, "lon":1}
    ).sort("t_ms", 1).limit(6000))
    _step = max(1, len(_all_eskf) // 300)
    stored_eskf = [
        {"t_ms": d["t_ms"], "lat": d.get("lat"), "lon": d.get("lon")}
        for d in _all_eskf[::_step]
        if d.get("lat") and d.get("lon") and d.get("lat") != 0
    ]

    if csv_gnss:
        gnss_data = csv_gnss  # CSV override (GNSS-denied simulation)
    if not gnss_data:
        return jsonify({"error": "No GNSS data for this session"}), 404

    # ── Replay ────────────────────────────────────────────────────────────────
    track, nis_seq, cov_seq, summary = replay_session(
        imu_data, gnss_data, params, deny
    )

    # Thin the track to max 2000 points for browser transfer
    step = max(1, len(track) // 500)
    track_thin = track[::step]

    return jsonify({
        "track":       track_thin,
        "nis_seq":     nis_seq,
        "cov_seq":     cov_seq,
        "summary":     summary,
        "gnss_raw":    gnss_data,
        "stored_eskf": stored_eskf,
        "imu_count":   len(imu_data),
        "gnss_count":  len(gnss_data),
    })


# ── POST /tuning/api/upload-csv ───────────────────────────────────────────────
@app.route("/tuning/api/upload-csv", methods=["POST"])
def upload_csv():
    """
    Accept a CSV file and parse as GNSS or IMU data.
    CSV format (GNSS):  t_ms, lat, lon, alt, speed, course
    CSV format (IMU):   t_ms, ax, ay, az, gx, gy, gz
    Returns parsed data as JSON for the browser to use.
    """
    if "file" not in request.files:
        return jsonify({"error": "No file uploaded"}), 400

    f    = request.files["file"]
    text = f.read().decode("utf-8", errors="replace")
    reader = csv.DictReader(io.StringIO(text))
    rows   = list(reader)

    if not rows:
        return jsonify({"error": "Empty CSV"}), 400

    headers = [h.strip().lower() for h in rows[0].keys()]

    # Detect type
    is_gnss = "lat" in headers and "lon" in headers
    is_imu  = "ax" in headers and "gx" in headers

    if not is_gnss and not is_imu:
        return jsonify({"error": f"Unknown CSV format. Headers: {headers}"}), 400

    parsed = []
    for row in rows:
        row = {k.strip().lower(): v.strip() for k, v in row.items()}
        try:
            if is_gnss:
                parsed.append({
                    "t_ms":   float(row.get("t_ms", 0)),
                    "lat":    float(row.get("lat",  0)),
                    "lon":    float(row.get("lon",  0)),
                    "alt":    float(row.get("alt",  0)),
                    "speed":  float(row.get("speed",  0)),
                    "course": float(row.get("course", 0)),
                    "fix_valid": True,
                })
            else:
                parsed.append({
                    "t_ms": float(row.get("t_ms", 0)),
                    "ax":   float(row.get("ax", 0)),
                    "ay":   float(row.get("ay", 0)),
                    "az":   float(row.get("az", 0)),
                    "gx":   float(row.get("gx", 0)),
                    "gy":   float(row.get("gy", 0)),
                    "gz":   float(row.get("gz", 0)),
                })
        except ValueError:
            continue

    return jsonify({
        "type":  "gnss" if is_gnss else "imu",
        "count": len(parsed),
        "data":  parsed,
    })


# ── GET /tuning/api/config-h ──────────────────────────────────────────────────
@app.route("/tuning/api/config-h")
def download_config_h():
    """Generate and download EskfConfig.h from query params."""
    params = {}
    for key in DEFAULT_CFG:
        val = request.args.get(key)
        if val is not None:
            try:
                params[key] = float(val)
            except ValueError:
                pass
    # Booleans
    for bk in ("use_gnss_vel", "use_altitude", "use_nhc", "use_zupt"):
        val = request.args.get(bk)
        if val is not None:
            params[bk] = val.lower() in ("1", "true", "yes")

    merged = {**DEFAULT_CFG, **params}
    content = generate_eskf_config_h(merged)

    return Response(
        content,
        mimetype="text/plain",
        headers={"Content-Disposition": "attachment; filename=EskfConfig.h"}
    )


# ── GET /tuning/api/health ────────────────────────────────────────────────────
@app.route("/tuning/api/health")
def health():
    try:
        mongo.admin.command("ping")
        db_ok = True
    except Exception:
        db_ok = False
    return jsonify({"status": "ok", "db": db_ok, "port": TUNING_PORT})


# ── Helpers ───────────────────────────────────────────────────────────────────
def _to_ist(dt):
    if dt is None:
        return "—"
    ist = dt + timedelta(hours=5, minutes=30)
    return ist.strftime("%d/%m %H:%M IST")


if __name__ == "__main__":
    print(f"TRU-TRACK Tuning Backend starting on port {TUNING_PORT}")
    print("MongoDB: READ-ONLY (find() only — no writes)")
    app.run(host="127.0.0.1", port=TUNING_PORT, debug=False)
