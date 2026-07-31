# TRU-TRACK

Real-time vehicle tracking and navigation system with GNSS/IMU sensor fusion.
Developed at **TruVibe Technologies**, incubated at ASHINE, SVNIT Surat.

## System Overview

TRU-TRACK fuses a multi-constellation GNSS receiver (Quectel L89HA, NavIC/dual-band) with a 6-axis IMU (MPU6050) using a 15-state Error-State Kalman Filter (ESKF) running on an ESP32. Telemetry is transmitted at 5 Hz over 4G LTE (SIMCom A7672S) via MQTT to a cloud backend, and visualized on a real-time web dashboard.

## Hardware

| Component | Role | Interface |
|---|---|---|
| ESP32 | Main controller | — |
| SIMCom A7672S | 4G LTE modem + MQTT | UART2 RX17/TX16 |
| Quectel L89HA | Dual-band NavIC GNSS | UART1 RX26/TX27 |
| MPU6050 | 6-axis IMU | I2C SDA21/SCL22 (0x68) |
| MAX17048 | Battery fuel gauge | I2C SDA21/SCL22 (0x36), ALRT->GPIO4 |
| LM2596S | 2S LiPo -> 5V buck | — |

## Repository Structure

    firmware/4g/TruTrack_v274/   <- Current firmware (tt-v2.7.4)
    firmware/libraries/Eskf3D/   <- ESKF library
    firmware/Provision_NVS/      <- NVS provisioning sketch
    backend/                     <- Flask API (port 9000)
    ingest/                      <- MQTT ingest service
    tuning/                      <- ESKF tuning service (port 9001)
    dashboard/                   <- Web dashboard
    deployment/                  <- nginx, systemd, scripts
    secrets.env                  <- Server credentials

## Server

- URL: https://tru-track.bittest.in
- SSH: crl@14.139.121.53
- Services: tru-track-backend (9000), tru-track-ingest, tru-track-tuning (9001), mongod, mosquitto, nginx

## Health Check

    sudo systemctl status tru-track-backend tru-track-ingest tru-track-tuning --no-pager | grep Active
    curl -s http://127.0.0.1:9000/api/server/health | python3 -m json.tool

## Firmware Flash

1. Open firmware/4g/TruTrack_v274/TruTrack_v274.ino in Arduino IDE
2. Copy firmware/libraries/Eskf3D/ to Arduino libraries folder
3. Flash Provision_NVS first to set MQTT host/APN in NVS
4. Flash TruTrack_v274.ino

## Team

Devaam Dalal, Abha Jadav, Vrushti Javeri, Dhruv Shah
Faculty Advisor: Dr. Shweta Shah, Electronics Engineering, SVNIT Surat


ESKF and Website by Abha Jadav 
