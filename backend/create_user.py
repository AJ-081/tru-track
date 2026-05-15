#!/usr/bin/env python3
"""Create a TRU-TRACK user. Run once to create the first superadmin."""
import getpass
import sys
from datetime import datetime

import bcrypt
from pymongo import MongoClient

db = MongoClient("mongodb://127.0.0.1:27017/?directConnection=true")["nav"]

name     = input("Full name: ").strip()
email    = input("Email: ").strip().lower()
password = getpass.getpass("Password: ")
role     = input("Role [superadmin/admin/analyst/viewer] (default: superadmin): ").strip()
if role not in ("superadmin", "admin", "analyst", "viewer"):
    role = "superadmin"

if db.users.find_one({"email": email}):
    print(f"ERROR: {email} already exists.")
    sys.exit(1)

pw_hash = bcrypt.hashpw(password.encode(), bcrypt.gensalt()).decode()
db.users.insert_one({
    "name":          name,
    "email":         email,
    "password_hash": pw_hash,
    "role":          role,
    "created_at":    datetime.utcnow(),
    "last_login":    None,
})
print(f"✓ User created: {email} ({role})")
