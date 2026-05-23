#!/usr/bin/env python3
"""
TRU-TRACK Async MQTT Ingest Service — v2.1
Changes from v2.0:
  - t_epoch_ms stored in all collections (device-side IST epoch from NTP)
  - gnss_raw written on EVERY packet (fix_valid flag inside — no more gaps)
  - eskf_state written on EVERY packet (init_valid, alignment_valid flags)
  - imu_raw has calibrated flag
  - ntp_synced flag stored in sessions + device_latest
"""
import asyncio
import json
import logging
import os
from datetime import datetime

import aiomqtt
import motor.motor_asyncio
from dotenv import load_dotenv

load_dotenv("/etc/tru-track/secrets.env")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_HOST   = os.getenv("MQTT_HOST", "127.0.0.1")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
MQTT_USER   = os.getenv("MQTT_USER", "")
MQTT_PASS   = os.getenv("MQTT_PASS", "")
MONGO_URI   = os.getenv("MONGO_URI", "mongodb://127.0.0.1:27017/?directConnection=true")
MONGO_DB    = os.getenv("MONGO_DB", "nav")

BATCH_SIZE  = 50
FLUSH_MS    = 200

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    handlers=[
        logging.FileHandler("/var/log/tru-track/ingest.log"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("ingest")

# ── MongoDB ───────────────────────────────────────────────────────────────────
mongo  = motor.motor_asyncio.AsyncIOMotorClient(MONGO_URI)
db     = mongo[MONGO_DB]

gnss_col  = db.gnss_raw
eskf_col  = db.eskf_state
imu_col   = db.imu_raw
sess_col  = db.sessions
dlat_col  = db.device_latest
dreg_col  = db.device_registry

# ── Write buffers ─────────────────────────────────────────────────────────────
gnss_buf: list = []
eskf_buf: list = []
imu_buf:  list = []


# ── Flush buffers to MongoDB ──────────────────────────────────────────────────
async def flush_buffers(force: bool = False):
    global gnss_buf, eskf_buf, imu_buf
    tasks = []

    if gnss_buf and (force or len(gnss_buf) >= BATCH_SIZE):
        tasks.append(gnss_col.insert_many(gnss_buf.copy(), ordered=False))
        gnss_buf.clear()

    if eskf_buf and (force or len(eskf_buf) >= BATCH_SIZE):
        tasks.append(eskf_col.insert_many(eskf_buf.copy(), ordered=False))
        eskf_buf.clear()

    if imu_buf and (force or len(imu_buf) >= BATCH_SIZE):
        tasks.append(imu_col.insert_many(imu_buf.copy(), ordered=False))
        imu_buf.clear()

    if tasks:
        try:
            await asyncio.gather(*tasks)
        except Exception as e:
            log.error(f"Buffer flush error: {e}")


async def periodic_flush():
    while True:
        await asyncio.sleep(FLUSH_MS / 1000)
        await flush_buffers(force=True)


# ── Process one MQTT message ──────────────────────────────────────────────────
async def process_message(payload_bytes: bytes):
    try:
        data = json.loads(payload_bytes.decode("utf-8"))
    except Exception as e:
        log.warning(f"JSON decode error: {e}")
        return

    device_id  = data.get("device_id")
    session_id = data.get("session_id")

    if not device_id:
        log.warning("Dropped: missing device_id")
        return

    t_server   = datetime.utcnow()
    t_epoch_ms = data.get("t_epoch_ms", 0)   # ← device-side IST epoch ms (0 before NTP sync)
    ntp_synced = data.get("ntp_synced", False)

    # Common fields — present in every collection document
    common = {
        "device_id":   device_id,
        "session_id":  session_id,
        "boot_id":     data.get("boot_id"),
        "boot_count":  data.get("boot_count"),
        "fw_version":  data.get("fw_version"),
        "t_ms":        data.get("t_ms"),
        "t_server":    t_server,
        "t_epoch_ms":  t_epoch_ms,    # ← IST epoch ms from device NTP
        "ntp_synced":  ntp_synced,
    }

    status = data.get("status", {})
    gnss   = data.get("gnss", {})
    eskf   = data.get("eskf", {})
    imu    = data.get("imu", {})

    # ── gnss_raw — written on EVERY packet ───────────────────────────────────
    # fix_valid=False rows mark gaps where GNSS signal was lost
    # Export can filter on fix_valid=True for position data only
    if gnss:
        gnss_buf.append({
            **common,
            "fix_valid":  gnss.get("fix_valid", False),
            "lat":        gnss.get("lat"),
            "lon":        gnss.get("lon"),
            "alt":        gnss.get("alt"),
            "speed":      gnss.get("speed"),
            "course":     gnss.get("course"),
            "sats":       gnss.get("sats"),
            "hdop":       gnss.get("hdop"),
            "fix_type":   gnss.get("fix_type"),
            "age_ms":     gnss.get("age_ms"),
        })

    # ── eskf_state — written on EVERY packet ─────────────────────────────────
    # init_valid=False rows exist before ESKF initialization
    # alignment_valid=False rows exist during alignment phase
    if eskf:
        eskf_buf.append({
            **common,
            "init_valid":      eskf.get("init_valid", False),
            "alignment_valid": eskf.get("alignment_valid", False),
            "lat":             eskf.get("lat"),
            "lon":             eskf.get("lon"),
            "alt":             eskf.get("alt"),
            "vE":              eskf.get("vE"),
            "vN":              eskf.get("vN"),
            "vU":              eskf.get("vU"),
            "roll":            eskf.get("roll"),
            "pitch":           eskf.get("pitch"),
            "yaw":             eskf.get("yaw"),
            "innov":           eskf.get("innov"),
            "P":               eskf.get("P"),
        })

    # ── imu_raw — always present ──────────────────────────────────────────────
    if imu:
        imu_buf.append({
            **common,
            "calibrated":      imu.get("calibrated", False),
            "accel_raw":       imu.get("accel_raw"),
            "gyro_raw":        imu.get("gyro_raw"),
            "accel_mps2":      imu.get("accel_mps2"),
            "gyro_radps":      imu.get("gyro_radps"),
            "gyro_bias_radps": imu.get("gyro_bias_radps"),
        })

    # ── sessions upsert ───────────────────────────────────────────────────────
    try:
        await sess_col.update_one(
            {"session_id": session_id},
            {
                "$setOnInsert": {
                    "device_id":           device_id,
                    "session_id":          session_id,
                    "started_at_server":   t_server,
                    "started_t_epoch_ms":  t_epoch_ms,
                    "boot_id":             data.get("boot_id"),
                    "boot_count":          data.get("boot_count"),
                    "reboot_reason":       data.get("reboot_reason"),
                    "fw_version":          data.get("fw_version"),
                },
                "$set": {
                    "last_seen":          t_server,
                    "last_t_epoch_ms":    t_epoch_ms,
                    "ntp_synced":         ntp_synced,
                },
                "$inc": {"packet_count": 1},
            },
            upsert=True,
        )
    except Exception as e:
        log.error(f"sessions upsert error: {e}")

    # ── device_latest upsert ──────────────────────────────────────────────────
    try:
        await dlat_col.update_one(
            {"device_id": device_id},
            {"$set": {
                **data,
                "t_server":     t_server,
                "t_epoch_ms":   t_epoch_ms,
                "last_seen":    t_server,
                "ntp_synced":   ntp_synced,
            }},
            upsert=True,
        )
    except Exception as e:
        log.error(f"device_latest upsert error: {e}")

    # ── device_registry upsert ────────────────────────────────────────────────
    try:
        await dreg_col.update_one(
            {"device_id": device_id},
            {
                "$setOnInsert": {
                    "device_id":          device_id,
                    "first_seen":         t_server,
                    "first_t_epoch_ms":   t_epoch_ms,
                },
                "$set": {
                    "last_seen":          t_server,
                    "last_t_epoch_ms":    t_epoch_ms,
                    "fw_version":         data.get("fw_version"),
                    "ntp_synced":         ntp_synced,
                },
                "$inc": {"total_packets": 1},
            },
            upsert=True,
        )
    except Exception as e:
        log.error(f"device_registry upsert error: {e}")

    await flush_buffers()

    log.info(
        f"{device_id} | gnss={'FIX' if gnss.get('fix_valid') else 'NO_FIX'}"
        f" | eskf={'READY' if eskf.get('alignment_valid') else ('INIT' if eskf.get('init_valid') else 'WAIT')}"
        f" | ntp={'OK' if ntp_synced else '--'}"
        f" | t_epoch_ms={t_epoch_ms}"
    )


# ── Main MQTT loop ────────────────────────────────────────────────────────────
async def main():
    log.info("TRU-TRACK Ingest v2.1 starting...")

    asyncio.create_task(periodic_flush())

    mqtt_kwargs = {
        "hostname": MQTT_HOST,
        "port":     MQTT_PORT,
    }
    if MQTT_USER:
        mqtt_kwargs["username"] = MQTT_USER
        mqtt_kwargs["password"] = MQTT_PASS

    while True:
        try:
            async with aiomqtt.Client(**mqtt_kwargs) as client:
                log.info(f"MQTT connected → {MQTT_HOST}:{MQTT_PORT}")
                await client.subscribe("nav/+/eskf")
                await client.subscribe("nav/eskf/debug")   # legacy topic
                log.info("Subscribed: nav/+/eskf  +  nav/eskf/debug (legacy)")

                async for message in client.messages:
                    await process_message(message.payload)

        except aiomqtt.MqttError as e:
            log.warning(f"MQTT error: {e} — reconnecting in 5s")
            await asyncio.sleep(5)
        except Exception as e:
            log.error(f"Unexpected error: {e} — reconnecting in 5s")
            await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())
