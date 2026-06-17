#!/usr/bin/env python3
"""
TRU-TRACK ESKF Tuning Backend — FIXED v10

Key fixes compared with the uploaded version:
  • Properly expands v2.6.1 imu_hf compact raw batches into 100 Hz samples.
  • Falls back to legacy imu_hf/imu_raw converted samples when raw batches are absent.
  • Cleans GNSS points using lat/lon, fix_valid, sats, hdop/age where available.
  • Returns debug counters so the tuning page can reveal whether GNSS updates are
    actually being applied/gated.
  • Keeps MongoDB read-only.
"""

import io
import csv
import os
import traceback
from datetime import datetime, timedelta

import numpy as np
from flask import Flask, jsonify, request, send_from_directory, Response
from pymongo import MongoClient

from eskf_python import replay_session, generate_eskf_config_h, DEFAULT_CFG, GRAVITY

# ── Config ────────────────────────────────────────────────────────────────────
# Do not hard-code production credentials here. Set MONGO_URI in the service env.
MONGO_URI = os.environ.get("MONGO_URI", "mongodb://127.0.0.1:27017/nav?directConnection=true")
TUNING_PORT = int(os.environ.get("TUNING_PORT", 9001))
TUNING_DIR = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dashboard")

mongo = MongoClient(MONGO_URI)
db = mongo["nav"]

app = Flask(__name__)


def cors(resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return resp


@app.after_request
def add_cors(resp):
    return cors(resp)


@app.errorhandler(Exception)
def handle_err(e):
    traceback.print_exc()
    return jsonify({"error": str(e)}), 500


@app.route("/tuning", strict_slashes=False)
@app.route("/tuning/", strict_slashes=False)
def tuning_page():
    p = os.path.join(DASHBOARD_DIR, "tuning.html")
    if os.path.exists(p):
        return send_from_directory(DASHBOARD_DIR, "tuning.html")
    return send_from_directory(TUNING_DIR, "tuning.html")


def _to_ist(dt):
    if dt is None:
        return "—"
    ist = dt + timedelta(hours=5, minutes=30)
    return ist.strftime("%d/%m %H:%M IST")


def _num(x, default=0.0):
    try:
        if x is None:
            return default
        v = float(x)
        if not np.isfinite(v):
            return default
        return v
    except Exception:
        return default


def _arr3(x, default=(0.0, 0.0, 0.0)):
    if isinstance(x, (list, tuple)) and len(x) >= 3:
        return [_num(x[0]), _num(x[1]), _num(x[2])]
    return list(default)


def _convert_raw(row, fs=16384.0, gs=131.0):
    # row: [t_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw]
    t, axr, ayr, azr, gxr, gyr, gzr = row[:7]
    fs = _num(fs, 16384.0) or 16384.0
    gs = _num(gs, 131.0) or 131.0
    return {
        "t_ms": _num(t),
        "ax": (_num(axr) / fs) * GRAVITY,
        "ay": (_num(ayr) / fs) * GRAVITY,
        "az": (_num(azr) / fs) * GRAVITY,
        "gx": (_num(gxr) / gs) * (np.pi / 180.0),
        "gy": (_num(gyr) / gs) * (np.pi / 180.0),
        "gz": (_num(gzr) / gs) * (np.pi / 180.0),
    }


def _load_imu_for_session(session_id, downsample=1):
    """Return (imu_data, debug). Supports v2.6.1 imu_hf raw batches and legacy samples."""
    downsample = max(1, int(downsample or 1))
    imu_data = []
    debug = {"source": None, "raw_batch_docs": 0, "legacy_docs": 0, "expanded_samples": 0, "zero_samples": 0}

    # v2.6.1 compact raw batches: fields are imu, fs, gs, t0, n, s/session_id.
    hf_query = {"$or": [{"session_id": session_id}, {"s": session_id}]}
    hf_docs = list(db.imu_hf.find(
        hf_query,
        {"_id": 0, "t_ms": 1, "t0": 1, "n": 1, "fs": 1, "gs": 1, "imu": 1,
         "accel_mps2": 1, "gyro_radps": 1, "accel_raw": 1, "gyro_raw": 1, "calibrated": 1}
    ).sort([("t0", 1), ("t_ms", 1)]).limit(20000))

    if hf_docs:
        for doc in hf_docs:
            if isinstance(doc.get("imu"), list) and doc["imu"]:
                debug["source"] = "imu_hf_raw_batch"
                debug["raw_batch_docs"] += 1
                fs = doc.get("fs", 16384.0)
                gs = doc.get("gs", 131.0)
                for row in doc["imu"]:
                    if not isinstance(row, (list, tuple)) or len(row) < 7:
                        continue
                    imu_data.append(_convert_raw(row, fs, gs))
            elif doc.get("accel_mps2") is not None and doc.get("gyro_radps") is not None:
                debug["source"] = debug["source"] or "imu_hf_legacy_converted"
                debug["legacy_docs"] += 1
                a = _arr3(doc.get("accel_mps2"))
                g = _arr3(doc.get("gyro_radps"))
                imu_data.append({"t_ms": _num(doc.get("t_ms", doc.get("t0", 0))),
                                 "ax": a[0], "ay": a[1], "az": a[2],
                                 "gx": g[0], "gy": g[1], "gz": g[2]})
            elif doc.get("accel_raw") is not None and doc.get("gyro_raw") is not None:
                debug["source"] = debug["source"] or "imu_hf_legacy_raw"
                debug["legacy_docs"] += 1
                ar = _arr3(doc.get("accel_raw")); gr = _arr3(doc.get("gyro_raw"))
                imu_data.append(_convert_raw([doc.get("t_ms", doc.get("t0", 0)), *ar, *gr], doc.get("fs", 16384.0), doc.get("gs", 131.0)))

    # Fallback to 5 Hz imu_raw latest-sample collection.
    if not imu_data:
        cursor = db.imu_raw.find(
            {"session_id": session_id},
            {"_id": 0, "t_ms": 1, "accel_mps2": 1, "gyro_radps": 1,
             "accel_raw": 1, "gyro_raw": 1, "calibrated": 1}
        ).sort("t_ms", 1).limit(60000)
        for doc in cursor:
            if doc.get("accel_mps2") is not None and doc.get("gyro_radps") is not None:
                debug["source"] = debug["source"] or "imu_raw_converted"
                a = _arr3(doc.get("accel_mps2")); g = _arr3(doc.get("gyro_radps"))
                imu_data.append({"t_ms": _num(doc.get("t_ms")),
                                 "ax": a[0], "ay": a[1], "az": a[2],
                                 "gx": g[0], "gy": g[1], "gz": g[2]})
            elif doc.get("accel_raw") is not None and doc.get("gyro_raw") is not None:
                debug["source"] = debug["source"] or "imu_raw_raw"
                ar = _arr3(doc.get("accel_raw")); gr = _arr3(doc.get("gyro_raw"))
                imu_data.append(_convert_raw([doc.get("t_ms", 0), *ar, *gr]))

    # Clean + sort + downsample.
    clean = []
    for d in sorted(imu_data, key=lambda x: x.get("t_ms", 0)):
        vals = [d.get(k, 0) for k in ("t_ms", "ax", "ay", "az", "gx", "gy", "gz")]
        if not np.isfinite(vals).all():
            continue
        if abs(d["ax"]) + abs(d["ay"]) + abs(d["az"]) + abs(d["gx"]) + abs(d["gy"]) + abs(d["gz"]) == 0:
            debug["zero_samples"] += 1
            continue
        clean.append(d)

    clean = clean[::downsample]
    debug["expanded_samples"] = len(clean)
    if debug["source"] is None:
        debug["source"] = "none"
    return clean, debug


def _load_gnss_for_session(session_id):
    cursor = db.gnss_raw.find(
        {"session_id": session_id},
        {"_id": 0, "t_ms": 1, "lat": 1, "lon": 1, "alt": 1,
         "speed": 1, "speed_mps": 1, "course": 1, "sats": 1,
         "fix_type": 1, "hdop": 1, "age_ms": 1, "fix_valid": 1}
    ).sort("t_ms", 1).limit(50000)

    out = []
    skipped = 0
    for doc in cursor:
        lat = _num(doc.get("lat"), 0); lon = _num(doc.get("lon"), 0)
        if lat == 0 or lon == 0:
            skipped += 1
            continue
        sats = int(_num(doc.get("sats"), 0))
        age = _num(doc.get("age_ms"), 0)
        explicit = doc.get("fix_valid", None)
        if explicit is not None:
            fix_valid = bool(explicit)
        else:
            # Old DB may not have fix_valid/age_ms; use sats when that is all we have.
            fix_valid = sats >= 5 and (age == 0 or age < 2500)
        if not fix_valid:
            skipped += 1
            continue
        out.append({
            "t_ms": _num(doc.get("t_ms")),
            "lat": lat,
            "lon": lon,
            "alt": _num(doc.get("alt"), 0),
            "speed": _num(doc.get("speed", doc.get("speed_mps", 0)), 0),
            "course": _num(doc.get("course"), 0),
            "fix_valid": True,
            "sats": sats,
            "hdop": _num(doc.get("hdop"), 99),
        })
    return out, {"gnss_valid": len(out), "gnss_skipped": skipped}


@app.route("/tuning/api/sessions")
def get_sessions():
    cutoff = datetime.utcnow() - timedelta(days=90)
    sess_docs = list(db.sessions.find(
        {"started_at_server": {"$gte": cutoff}},
        {"_id": 0, "session_id": 1, "device_id": 1, "fw_version": 1,
         "packet_count": 1, "started_at_server": 1, "duration_s": 1,
         "gnss_points": 1, "eskf_points": 1}
    ).sort("started_at_server", -1).limit(50))

    if not sess_docs:
        sids = db.imu_raw.distinct("session_id", {"t_server": {"$gte": cutoff}})[:30]
        sess_docs = [{"session_id": s, "device_id": "?", "packet_count": 0} for s in sids if s]

    result = []
    for s in sess_docs:
        sid = s.get("session_id")
        if not sid:
            continue
        pkts = s.get("packet_count") or 0
        dur_s = s.get("duration_s") or (pkts * 0.2)
        mins = int(dur_s) // 60
        did_short = s.get("device_id", "?").split(":")[-3:]
        dev_label = ":".join(did_short)
        result.append({
            "session_id": sid,
            "device_id": s.get("device_id", "?"),
            "fw_version": s.get("fw_version", "?"),
            "imu_count": pkts,
            "has_gnss": bool(s.get("gnss_points", 1)),
            "duration_s": int(dur_s),
            "started_ist": _to_ist(s.get("started_at_server")),
            "label": f"{_to_ist(s.get('started_at_server'))} ({mins}m) — {dev_label} [{pkts}pkt]",
        })
    return jsonify(result)


@app.route("/tuning/api/run", methods=["POST"])
def run_tuning():
    body = request.get_json(force=True) or {}
    session_id = body.get("session_id")
    if not session_id:
        return jsonify({"error": "session_id required"}), 400

    params = {**DEFAULT_CFG, **(body.get("params") or {})}
    deny = body.get("gnss_deny_ranges") or []
    ds = max(1, int(body.get("downsample_imu", 1)))

    imu_data, imu_debug = _load_imu_for_session(session_id, ds)
    if not imu_data:
        return jsonify({"error": "No usable IMU data for this session", "imu_debug": imu_debug}), 404

    csv_gnss = body.get("csv_gnss")
    if csv_gnss:
        gnss_data = csv_gnss
        gnss_debug = {"gnss_valid": len(gnss_data), "gnss_skipped": 0, "source": "csv_override"}
    else:
        gnss_data, gnss_debug = _load_gnss_for_session(session_id)
        gnss_debug["source"] = "mongodb_gnss_raw"
    if not gnss_data:
        return jsonify({"error": "No usable GNSS data for this session", "gnss_debug": gnss_debug}), 404

    _all_eskf = list(db.eskf_state.find(
        {"session_id": session_id},
        {"_id": 0, "t_ms": 1, "lat": 1, "lon": 1}
    ).sort("t_ms", 1).limit(10000))
    _step = max(1, len(_all_eskf) // 500)
    stored_eskf = [
        {"t_ms": d.get("t_ms"), "lat": d.get("lat"), "lon": d.get("lon")}
        for d in _all_eskf[::_step]
        if d.get("lat") and d.get("lon") and d.get("lat") != 0 and d.get("lon") != 0
    ]

    track, nis_seq, cov_seq, summary = replay_session(imu_data, gnss_data, params, deny)
    if summary.get("error"):
        return jsonify({"error": summary["error"], "imu_debug": imu_debug, "gnss_debug": gnss_debug}), 400

    step = max(1, len(track) // 1200)
    track_thin = track[::step]
    summary["imu_debug"] = imu_debug
    summary["gnss_debug"] = gnss_debug
    summary["stored_eskf_count"] = len(stored_eskf)

    return jsonify({
        "track": track_thin,
        "nis_seq": nis_seq,
        "cov_seq": cov_seq,
        "summary": summary,
        "gnss_raw": gnss_data,
        "stored_eskf": stored_eskf,
        "imu_count": len(imu_data),
        "gnss_count": len(gnss_data),
    })


@app.route("/tuning/api/upload-csv", methods=["POST"])
def upload_csv():
    if "file" not in request.files:
        return jsonify({"error": "No file uploaded"}), 400
    text = request.files["file"].read().decode("utf-8", errors="replace")
    reader = csv.DictReader(io.StringIO(text))
    rows = list(reader)
    if not rows:
        return jsonify({"error": "Empty CSV"}), 400
    headers = [h.strip().lower() for h in rows[0].keys()]

    # Accept simple GNSS CSV or your merged TRU-TRACK CSV columns.
    has_simple = "lat" in headers and "lon" in headers
    has_merged = "gnss_lat" in headers and "gnss_lon" in headers
    has_imu = ("ax" in headers and "gx" in headers) or ("imu_accel_mps2_x" in headers and "imu_gyro_radps_x" in headers)

    parsed = []
    for row in rows:
        r = {k.strip().lower(): (v.strip() if isinstance(v, str) else v) for k, v in row.items()}
        try:
            if has_merged or has_simple:
                lat_key = "gnss_lat" if has_merged else "lat"
                lon_key = "gnss_lon" if has_merged else "lon"
                alt_key = "gnss_alt" if has_merged else "alt"
                spd_key = "gnss_speed" if "gnss_speed" in r else ("gnss_speed_mps" if "gnss_speed_mps" in r else "speed")
                crs_key = "gnss_course" if "gnss_course" in r else "course"
                lat = _num(r.get(lat_key), 0); lon = _num(r.get(lon_key), 0)
                if lat == 0 or lon == 0:
                    continue
                parsed.append({
                    "t_ms": _num(r.get("t_ms", r.get("time_ms", 0))),
                    "lat": lat,
                    "lon": lon,
                    "alt": _num(r.get(alt_key), 0),
                    "speed": _num(r.get(spd_key), 0),
                    "course": _num(r.get(crs_key), 0),
                    "fix_valid": True,
                })
        except Exception:
            continue

    if parsed:
        return jsonify({"type": "gnss", "count": len(parsed), "data": parsed})
    if has_imu:
        return jsonify({"error": "IMU CSV upload is not wired in the frontend yet. Upload GNSS/merged CSV only, or use MongoDB session replay."}), 400
    return jsonify({"error": f"Unknown CSV format. Headers: {headers}"}), 400


@app.route("/tuning/api/config-h")
def download_config_h():
    params = {}
    for key in DEFAULT_CFG:
        val = request.args.get(key)
        if val is not None:
            try:
                params[key] = float(val)
            except ValueError:
                pass
    for bk in ("use_gnss_vel", "use_altitude", "use_nhc", "use_zupt"):
        val = request.args.get(bk)
        if val is not None:
            params[bk] = val.lower() in ("1", "true", "yes")
    merged = {**DEFAULT_CFG, **params}
    content = generate_eskf_config_h(merged)
    return Response(content, mimetype="text/plain", headers={"Content-Disposition": "attachment; filename=EskfConfig.h"})


@app.route("/tuning/api/health")
def health():
    try:
        mongo.admin.command("ping")
        db_ok = True
    except Exception:
        db_ok = False
    return jsonify({"status": "ok", "db": db_ok, "port": TUNING_PORT})


if __name__ == "__main__":
    print(f"TRU-TRACK Tuning Backend FIXED v10 starting on port {TUNING_PORT}")
    print("MongoDB: READ-ONLY (find() only — no writes)")
    print("MONGO_URI source:", "env" if os.environ.get("MONGO_URI") else "default localhost")
    app.run(host="127.0.0.1", port=TUNING_PORT, debug=False)
