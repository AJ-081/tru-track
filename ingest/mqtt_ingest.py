#!/usr/bin/env python3
"""TRU-TRACK MQTT Async Ingest Service.
asyncio + aiomqtt + motor.
Subscribes to nav/+/eskf (per-device) and nav/eskf/debug (legacy).
Writes gnss_raw, eskf_state, imu_raw.
Upserts sessions, device_latest, device_registry on every packet.
Uses buffered insert_many for HDD write performance.
"""
import asyncio
import json
import logging
import os
import time
from datetime import datetime, timezone

import aiomqtt
import motor.motor_asyncio
from dotenv import load_dotenv

load_dotenv("/etc/tru-track/secrets.env")

# ── Config ────────────────────────────────────────────────────────────────────
MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017")
MONGO_DB  = os.getenv("MONGO_DB", "nav")

BATCH_SIZE = 50
FLUSH_MS   = 200

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
g_col  = db.gnss_raw
e_col  = db.eskf_state
i_col  = db.imu_raw
s_col  = db.sessions
dl_col = db.device_latest
dr_col = db.device_registry

# ── Write buffers ─────────────────────────────────────────────────────────────
gnss_buf: list = []
eskf_buf: list = []
imu_buf:  list = []
last_flush: float = time.monotonic()


def utcnow() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def flush_buffers(force: bool = False) -> None:
    global last_flush
    now = time.monotonic()
    elapsed_ms = (now - last_flush) * 1000

    if not force and len(gnss_buf) < BATCH_SIZE and elapsed_ms < FLUSH_MS:
        return

    tasks = []
    if gnss_buf:
        tasks.append(g_col.insert_many(gnss_buf.copy(), ordered=False))
        gnss_buf.clear()
    if eskf_buf:
        tasks.append(e_col.insert_many(eskf_buf.copy(), ordered=False))
        eskf_buf.clear()
    if imu_buf:
        tasks.append(i_col.insert_many(imu_buf.copy(), ordered=False))
        imu_buf.clear()

    if tasks:
        results = await asyncio.gather(*tasks, return_exceptions=True)
        for r in results:
            if isinstance(r, Exception):
                log.error(f"insert_many error: {r}")

    last_flush = time.monotonic()


async def periodic_flush() -> None:
    while True:
        await asyncio.sleep(FLUSH_MS / 1000)
        await flush_buffers(force=True)


async def upsert_session(p: dict, t: datetime) -> None:
    sid = p.get("session_id")
    if not sid:
        return
    await s_col.update_one(
        {"session_id": sid},
        {"$setOnInsert": {
            "session_id":          sid,
            "device_id":           p.get("device_id"),
            "boot_id":             p.get("boot_id"),
            "boot_count":          p.get("boot_count"),
            "reboot_reason":       p.get("reboot_reason"),
            "fw_version":          p.get("fw_version"),
            "eskf_config_version": p.get("status", {}).get("eskf_config_version"),
            "started_at_server":   t,
        },
         "$set": {"last_packet_at": t, "open": True},
         "$inc": {"packet_count": 1},
        },
        upsert=True,
    )


async def upsert_device_latest(p: dict, t: datetime) -> None:
    did = p.get("device_id")
    if not did:
        return
    await dl_col.update_one(
        {"device_id": did},
        {"$set": {
            "device_id":  did,
            "session_id": p.get("session_id"),
            "boot_count": p.get("boot_count"),
            "fw_version": p.get("fw_version"),
            "last_seen":  t,
            "status":     p.get("status", {}),
            "gnss":       p.get("gnss", {}),
            "eskf":       p.get("eskf", {}),
        }},
        upsert=True,
    )


async def upsert_device_registry(p: dict, t: datetime) -> None:
    did = p.get("device_id")
    if not did:
        return
    await dr_col.update_one(
        {"device_id": did},
        {"$setOnInsert": {"first_seen": t},
         "$set": {
            "last_seen":  t,
            "fw_version": p.get("fw_version"),
         },
         "$inc": {"total_packets": 1},
        },
        upsert=True,
    )


async def process_message(p: dict, t: datetime) -> None:
    did = p.get("device_id")
    if not did:
        log.warning("Dropped packet: missing device_id")
        return

    common = {
        "device_id":     did,
        "session_id":    p.get("session_id"),
        "boot_id":       p.get("boot_id"),
        "boot_count":    p.get("boot_count"),
        "reboot_reason": p.get("reboot_reason"),
        "fw_version":    p.get("fw_version"),
        "t_ms":          p.get("t_ms"),
        "t_server":      t,
        "status":        p.get("status", {}),
    }

    gnss = p.get("gnss", {})
    eskf = p.get("eskf", {})
    imu  = p.get("imu",  {})

    if gnss:
        gnss_buf.append({**common,
            "lat":      gnss.get("lat"),
            "lon":      gnss.get("lon"),
            "alt":      gnss.get("alt"),
            "speed":    gnss.get("speed"),
            "course":   gnss.get("course"),
            "sats":     gnss.get("sats"),
            "hdop":     gnss.get("hdop"),
            "fix_type": gnss.get("fix_type"),
            "age_ms":   gnss.get("age_ms"),
        })

    if eskf:
        eskf_buf.append({**common,
            "lat":   eskf.get("lat"),
            "lon":   eskf.get("lon"),
            "alt":   eskf.get("alt"),
            "vE":    eskf.get("vE"),
            "vN":    eskf.get("vN"),
            "vU":    eskf.get("vU"),
            "roll":  eskf.get("roll"),
            "pitch": eskf.get("pitch"),
            "yaw":   eskf.get("yaw"),
            "innov": eskf.get("innov"),
            "P":     eskf.get("P"),
        })

    if imu:
        imu_buf.append({**common,
            "accel_raw":       imu.get("accel_raw"),
            "gyro_raw":        imu.get("gyro_raw"),
            "accel_mps2":      imu.get("accel_mps2"),
            "gyro_radps":      imu.get("gyro_radps"),
            "gyro_bias_radps": imu.get("gyro_bias_radps"),
        })

    await asyncio.gather(
        upsert_session(p, t),
        upsert_device_latest(p, t),
        upsert_device_registry(p, t),
        return_exceptions=True,
    )

    await flush_buffers()


async def main() -> None:
    log.info("TRU-TRACK Ingest starting...")
    asyncio.create_task(periodic_flush())

    reconnect_delay = 5
    while True:
        try:
            async with aiomqtt.Client(
                hostname=MQTT_HOST,
                port=MQTT_PORT,
                username=MQTT_USER or None,
                password=MQTT_PASS or None,
            ) as client:
                log.info(f"MQTT connected → {MQTT_HOST}:{MQTT_PORT}")
                reconnect_delay = 5

                await client.subscribe("nav/+/eskf")
                await client.subscribe("nav/eskf/debug")
                log.info("Subscribed: nav/+/eskf  +  nav/eskf/debug (legacy)")

                async for message in client.messages:
                    try:
                        p = json.loads(message.payload.decode())
                        await process_message(p, utcnow())
                    except json.JSONDecodeError as ex:
                        log.warning(f"Bad JSON on {message.topic}: {ex}")
                    except Exception as ex:
                        log.error(f"Message error: {ex}")

        except aiomqtt.MqttError as ex:
            log.warning(f"MQTT disconnected: {ex} — retry in {reconnect_delay}s")
            await flush_buffers(force=True)
            await asyncio.sleep(reconnect_delay)
            reconnect_delay = min(reconnect_delay * 2, 60)
        except Exception as ex:
            log.error(f"Unexpected error: {ex} — retry in {reconnect_delay}s")
            await flush_buffers(force=True)
            await asyncio.sleep(reconnect_delay)


if __name__ == "__main__":
    asyncio.run(main())
