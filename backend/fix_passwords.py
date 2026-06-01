#!/usr/bin/env python3
"""
Fix missing password_hash for test users.
Run once: python3 fix_passwords.py
"""
import os, bcrypt
from pymongo import MongoClient
from dotenv import load_dotenv

load_dotenv("/etc/tru-track/secrets.env")
db = MongoClient(os.getenv("MONGO_URI"))["nav"]

users = [
    ("analyst@gmail.com",    "analyst"),
    ("admin@gmail.com",      "admin123"),
    ("viewer@gmail.com",     "viewer"),
    ("superadmin@gmail.com", "superadmin"),
]

for email, password in users:
    h = bcrypt.hashpw(password.encode(), bcrypt.gensalt()).decode()
    r = db.users.update_one({"email": email}, {"$set": {"password_hash": h}})
    if r.matched_count:
        print(f"✓ Fixed: {email}")
    else:
        print(f"✗ Not found: {email}")

print("\nDone. All users can now log in.")
