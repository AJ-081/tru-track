#!/bin/bash
set -e
mongosh admin --host localhost -u root -p "$MONGO_INITDB_ROOT_PASSWORD" --eval "
db.getSiblingDB('admin').createUser({
  user: 'navapp',
  pwd: '$NAVAPP_PASSWORD',
  roles: [{ role: 'readWrite', db: 'nav' }]
})"
