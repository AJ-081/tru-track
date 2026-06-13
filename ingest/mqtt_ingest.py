#!/usr/bin/env python3
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime

import paho.mqtt.client as mqtt
from pymongo import MongoClient, ASCENDING, DESCENDING
from pymongo.errors import DuplicateKeyError, PyMongoError

MQTT_HOST = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")

MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017")
MONGO_DB = os.getenv("MONGO_DB", "nav")

TOPICS = [
    ("nav/+/eskf", 0),
    ("nav/eskf/debug", 0),
    ("nav/+/imu_hf", 0),
]

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger("tru-track-ingest")

mongo = MongoClient(MONGO_URI)
db = mongo[MONGO_DB]

gnss_col = db.gnss_raw
eskf_col = db.eskf_state
imu_col = db.imu_raw
imu_hf_col = db.imu_hf
latest_col = db.device_latest
sessions_col = db.sessions
registry_col = db.device_registry

packet_count = 0
insert_count = 0
duplicate_count = 0
error_count = 0


def now_utc():
    return datetime.utcnow()


def clean_doc(doc):
    # Never allow reused/carry-over Mongo _id into insert.
    doc = dict(doc)
    doc.pop("_id", None)
    return doc


def safe_insert(col, doc, label):
    global insert_count, duplicate_count, error_count
    try:
        col.insert_one(clean_doc(doc))
        insert_count += 1
    except DuplicateKeyError:
        duplicate_count += 1
        # Do not print the full document; that caused GB-scale logs.
        if duplicate_count % 1000 == 1:
            log.warning("DuplicateKey ignored in %s; total_duplicates=%s", label, duplicate_count)
    except PyMongoError as exc:
        error_count += 1
        log.error("Mongo insert failed in %s: %s", label, exc.__class__.__name__)


def build_common(payload, t_server):
    return {
        "device_id": payload.get("device_id"),
        "session_id": payload.get("session_id"),
        "boot_id": payload.get("boot_id"),
        "boot_count": payload.get("boot_count"),
        "fw_version": payload.get("fw_version"),
        "t_ms": payload.get("t_ms"),
        "t_epoch_ms": payload.get("t_epoch_ms"),
        "ntp_synced": payload.get("ntp_synced"),
        "calibrated": payload.get("calibrated"),
        "t_server": t_server,
        "status": payload.get("status", {}),
    }


def handle_imu_hf(payload):
    device_id = payload.get("d")
    session_id = payload.get("s")
    fw = payload.get("v")
    samples = payload.get("imu", [])
    t_server = now_utc()
    docs=[]
    for s in samples[:200]:
        if not isinstance(s, list) or len(s) < 7:
            continue
        docs.append({
            "device_id": device_id, "session_id": session_id,
            "fw_version": fw, "t_ms": int(s[0]), "t_server": t_server,
            "accel_mps2": [float(s[1]),float(s[2]),float(s[3])],
            "gyro_radps": [float(s[4]),float(s[5]),float(s[6])],
        })
    if docs:
        try: imu_hf_col.insert_many(docs, ordered=False)
        except PyMongoError: pass

def handle_payload(payload):
    global packet_count

    device_id = payload.get("device_id")
    if not device_id:
        return

    packet_count += 1
    t_server = now_utc()
    common = build_common(payload, t_server)

    gnss = payload.get("gnss") or {}
    if isinstance(gnss, dict) and gnss:
        safe_insert(gnss_col, {
            **common,
            "lat": gnss.get("lat"),
            "lon": gnss.get("lon"),
            "alt": gnss.get("alt"),
            "speed": gnss.get("speed"),
            "course": gnss.get("course"),
            "sats": gnss.get("sats"),
            "hdop": gnss.get("hdop"),
            "fix_type": gnss.get("fix_type"),
            "age_ms": gnss.get("age_ms"),
        }, "gnss_raw")

    eskf = payload.get("eskf") or {}
    if isinstance(eskf, dict) and eskf:
        safe_insert(eskf_col, {
            **common,
            "lat": eskf.get("lat"),
            "lon": eskf.get("lon"),
            "alt": eskf.get("alt"),
            "vE": eskf.get("vE"),
            "vN": eskf.get("vN"),
            "vU": eskf.get("vU"),
            "roll": eskf.get("roll"),
            "pitch": eskf.get("pitch"),
            "yaw": eskf.get("yaw"),
            "innov": eskf.get("innov"),
            "P": eskf.get("P"),
        }, "eskf_state")

    imu = payload.get("imu") or {}
    imu_samples = []

    if isinstance(imu, dict) and imu:
        imu_samples.append(imu)
    elif isinstance(imu, list):
        imu_samples.extend(imu)

    for sample in imu_samples[:500]:
        if not isinstance(sample, dict):
            continue
        safe_insert(imu_col, {
            **common,
            "t_ms": sample.get("t_ms", common.get("t_ms")),
            "accel_raw": sample.get("accel_raw"),
            "gyro_raw": sample.get("gyro_raw"),
            "accel_mps2": sample.get("accel_mps2"),
            "gyro_radps": sample.get("gyro_radps"),
            "gyro_bias_radps": sample.get("gyro_bias_radps"),
        }, "imu_raw")

    # Lightweight latest/session metadata.
    try:
        latest_col.update_one(
            {"device_id": device_id},
            {"$set": {
                "device_id": device_id,
                "session_id": payload.get("session_id"),
                "last_seen": t_server,
                "fw_version": payload.get("fw_version"),
                "status": payload.get("status", {}),
                "gnss": gnss if isinstance(gnss, dict) else {},
                "eskf": eskf if isinstance(eskf, dict) else {},
            }},
            upsert=True,
        )

        registry_col.update_one(
            {"device_id": device_id},
            {
                "$setOnInsert": {"first_seen": t_server},
                "$set": {"last_seen": t_server, "fw_version": payload.get("fw_version")},
                "$inc": {"total_packets": 1},
            },
            upsert=True,
        )

        sid = payload.get("session_id")
        if sid:
            sessions_col.update_one(
                {"session_id": sid},
                {
                    "$setOnInsert": {
                        "session_id": sid,
                        "device_id": device_id,
                        "started_at_server": t_server,
                    },
                    "$set": {"last_packet_at": t_server, "open": True},
                    "$inc": {"packet_count": 1},
                },
                upsert=True,
            )
    except PyMongoError as exc:
        log.error("Mongo metadata update failed: %s", exc.__class__.__name__)

    if packet_count % 100 == 1:
        log.info(
            "packets=%s inserts=%s duplicates=%s errors=%s last_device=%s",
            packet_count, insert_count, duplicate_count, error_count, device_id
        )


def on_connect(client, userdata, flags, rc, properties=None):
    log.info("MQTT connected → %s:%s rc=%s", MQTT_HOST, MQTT_PORT, rc)
    for topic in TOPICS:
        client.subscribe(topic)
    log.info("Subscribed: nav/+/eskf + nav/eskf/debug")


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8", errors="replace"))
        if msg.topic.endswith("/imu_hf"):
            handle_imu_hf(payload)
        else:
            handle_payload(payload)
    except Exception as exc:
        log.error("Message dropped: %s", exc.__class__.__name__)


def shutdown(signum, frame):
    log.info("Shutting down ingest")
    sys.exit(0)


signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)

log.info("TRU-TRACK Ingest SAFE starting...")

try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tru-track-ingest-safe")
except Exception:
    client = mqtt.Client(client_id="tru-track-ingest-safe")

if MQTT_USER:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

client.on_connect = on_connect
client.on_message = on_message

while True:
    try:
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        client.loop_forever()
    except Exception as exc:
        log.error("MQTT loop failed: %s; retrying in 5s", exc.__class__.__name__)
        time.sleep(5)
