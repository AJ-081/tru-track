#!/usr/bin/env python3
"""TRU-TRACK MongoDB Schema Initializer.
Run once on fresh install. Safe to re-run — createIndex is idempotent."""

from pymongo import MongoClient, ASCENDING, DESCENDING
from datetime import datetime

MONGO_URI = "mongodb://localhost:27017"
DB_NAME   = "nav"
TTL_90D   = 60 * 60 * 24 * 90   # 7,776,000 seconds

client = MongoClient(MONGO_URI)
db     = client[DB_NAME]

print(f"Connected to MongoDB — initializing database '{DB_NAME}'")

# ── 1. gnss_raw ───────────────────────────────────────────────────────────────
col = db.gnss_raw
col.create_index([("device_id",ASCENDING),("session_id",ASCENDING),("t_server",ASCENDING)], name="dev_sess_ts")
col.create_index([("session_id",ASCENDING),("t_server",ASCENDING)],                         name="sess_ts")
col.create_index([("device_id",ASCENDING),("t_server",DESCENDING)],                         name="dev_ts_desc")
col.create_index([("t_server",ASCENDING)], expireAfterSeconds=TTL_90D,                       name="ttl_90d")
print("  gnss_raw        : 4 indexes")

# ── 2. eskf_state ─────────────────────────────────────────────────────────────
col = db.eskf_state
col.create_index([("device_id",ASCENDING),("session_id",ASCENDING),("t_server",ASCENDING)], name="dev_sess_ts")
col.create_index([("session_id",ASCENDING),("t_server",ASCENDING)],                         name="sess_ts")
col.create_index([("device_id",ASCENDING),("t_server",DESCENDING)],                         name="dev_ts_desc")
col.create_index([("t_server",ASCENDING)], expireAfterSeconds=TTL_90D,                       name="ttl_90d")
print("  eskf_state      : 4 indexes")

# ── 3. imu_raw ────────────────────────────────────────────────────────────────
col = db.imu_raw
col.create_index([("device_id",ASCENDING),("session_id",ASCENDING),("t_server",ASCENDING)], name="dev_sess_ts")
col.create_index([("session_id",ASCENDING),("t_server",ASCENDING)],                         name="sess_ts")
col.create_index([("device_id",ASCENDING),("t_server",DESCENDING)],                         name="dev_ts_desc")
col.create_index([("t_server",ASCENDING)], expireAfterSeconds=TTL_90D,                       name="ttl_90d")
print("  imu_raw         : 4 indexes")

# ── 4. sessions ───────────────────────────────────────────────────────────────
col = db.sessions
col.create_index([("session_id",ASCENDING)], unique=True,                                    name="session_id_unique")
col.create_index([("device_id",ASCENDING),("started_at_server",DESCENDING)],                 name="dev_started")
col.create_index([("device_id",ASCENDING),("open",ASCENDING),("last_packet_at",DESCENDING)], name="dev_open_last")
print("  sessions        : 3 indexes")

# ── 5. device_latest ──────────────────────────────────────────────────────────
col = db.device_latest
col.create_index([("device_id",ASCENDING)], unique=True,                                     name="device_id_unique")
col.create_index([("last_seen",DESCENDING)],                                                 name="last_seen_desc")
print("  device_latest   : 2 indexes")

# ── 6. device_registry ────────────────────────────────────────────────────────
col = db.device_registry
col.create_index([("device_id",ASCENDING)], unique=True,                                     name="device_id_unique")
col.create_index([("last_seen",DESCENDING)],                                                 name="last_seen_desc")
print("  device_registry : 2 indexes")

# ── 7. ai_sequences ───────────────────────────────────────────────────────────
col = db.ai_sequences
col.create_index([("device_id",ASCENDING),("session_id",ASCENDING),("t_start",ASCENDING)],  name="dev_sess_t")
print("  ai_sequences    : 1 index")

# ── 8. users ──────────────────────────────────────────────────────────────────
col = db.users
col.create_index([("email",ASCENDING)], unique=True,                                         name="email_unique")
print("  users           : 1 index")

# ── 9. matlab_exports ─────────────────────────────────────────────────────────
col = db.matlab_exports
col.create_index([("device_id",ASCENDING),("session_id",ASCENDING)],                        name="dev_sess")
col.create_index([("created_at",DESCENDING)],                                                name="created_desc")
print("  matlab_exports  : 2 indexes")

# ── Summary ───────────────────────────────────────────────────────────────────
print("\n── Collection Summary ───────────────────────────────────────")
total_idx = 0
for name in sorted(db.list_collection_names()):
    idx_count = len(db[name].index_information())
    doc_count = db[name].count_documents({})
    total_idx += idx_count
    print(f"  {name:<20} {doc_count:>6} docs   {idx_count} indexes")

print(f"\n  Total indexes (including _id): {total_idx}")
print("✓ Schema initialization complete — ready for Day 5")
client.close()
