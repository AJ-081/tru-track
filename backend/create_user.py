#!/usr/bin/env python3
"""TRU-TRACK — Create dashboard user. Run once per user."""
import os, getpass
from pathlib import Path
from dotenv import load_dotenv
from pymongo import MongoClient
import bcrypt
from datetime import datetime

# Load secrets
load_dotenv("/etc/tru-track/secrets.env")
MONGO_URI = os.getenv("MONGO_URI")
MONGO_DB  = os.getenv("MONGO_DB", "nav")

if not MONGO_URI:
    print("ERROR: MONGO_URI not found in /etc/tru-track/secrets.env")
    exit(1)

client = MongoClient(MONGO_URI)
db     = client[MONGO_DB]

VALID_ROLES = ["superadmin", "admin", "analyst", "viewer"]

print("\n=== TRU-TRACK User Creation ===\n")
name  = input("Full name : ").strip()
email = input("Email     : ").strip().lower()

if not name or not email or "@" not in email:
    print("ERROR: Invalid name or email.")
    exit(1)

if db.users.find_one({"email": email}):
    print(f"ERROR: User {email} already exists.")
    exit(1)

password = getpass.getpass("Password  : ")
if len(password) < 6:
    print("ERROR: Password must be at least 6 characters.")
    exit(1)

role = input(f"Role {VALID_ROLES} (default: viewer): ").strip().lower()
if role not in VALID_ROLES:
    role = "viewer"

hashed = bcrypt.hashpw(password.encode(), bcrypt.gensalt()).decode()

db.users.insert_one({
    "name":       name,
    "email":      email,
    "password":   hashed,
    "role":       role,
    "created_at": datetime.utcnow(),
})

print(f"\n✓ User created: {name} <{email}> [{role}]")
client.close()
