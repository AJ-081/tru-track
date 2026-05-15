// TRU-TRACK Dashboard — app.js
// All real data. Zero simulation. Zero random values.

// ── Auth guard ────────────────────────────────────────────────────────────────
AUTH.requireLogin();
AUTH.populateUserUI();

// ── State ─────────────────────────────────────────────────────────────────────
let selectedDevice   = null;
let selectedSession  = null;
let socket           = null;
let healthTimer      = null;
let alertTimer       = null;
let staleTimer       = null;
let lastPacketAt     = null;
let packetCount      = 0;
let mapReady         = false;

// Map objects (set after Google Maps loads)
let gmap         = null;
let gnssPolyline = null;
let eskfPolyline = null;

// ── Init ─────────────────────────────────────────────────────────────────────
window.addEventListener("DOMContentLoaded", async () => {
  await tryLoadMap();
  await loadDevices();
  connectSocket();
  startStaleChecker();
});

// ── Google Maps ──────────────────────────────────────────────────────────────
async function tryLoadMap() {
  try {
    const res = await AUTH.apiFetch("/api/v1/maps/key");
    if (!res || !res.ok) return;
    const { key } = await res.json();
    if (!key) return;  // no key configured — placeholder stays visible

    // Hide placeholder, load Maps script dynamically
    document.getElementById("map-placeholder").style.display = "none";
    const script = document.createElement("script");
    script.src = `https://maps.googleapis.com/maps/api/js?key=${key}&callback=onMapReady`;
    script.async = true;
    document.head.appendChild(script);
  } catch { /* map stays as placeholder */ }
}

window.onMapReady = function() {
  mapReady = true;
  gmap = new google.maps.Map(document.getElementById("map"), {
    zoom: 15,
    center: { lat: 21.1642, lng: 72.7862 },
    mapTypeId: "satellite",
    disableDefaultUI: false,
  });
  gnssPolyline = new google.maps.Polyline({
    map: gmap, strokeColor: "#ef4444", strokeWeight: 2, strokeOpacity: 0.9,
  });
  eskfPolyline = new google.maps.Polyline({
    map: gmap, strokeColor: "#3b82f6", strokeWeight: 2, strokeOpacity: 0.9,
  });
};

// ── Device dropdown ──────────────────────────────────────────────────────────
async function loadDevices() {
  const sel = document.getElementById("device-select");
  try {
    const res = await AUTH.apiFetch("/api/v1/devices");
    if (!res || !res.ok) return;
    const devices = await res.json();

    if (!devices.length) {
      sel.innerHTML = "<option value=''>No devices registered yet</option>";
      return;
    }

    sel.innerHTML = "<option value=''>Select device...</option>";
    devices.forEach(d => {
      const o = document.createElement("option");
      o.value       = d.device_id;
      o.textContent = d.label || d.device_id;
      sel.appendChild(o);
    });

    // Auto-select if only one device
    if (devices.length === 1) {
      sel.value = devices[0].device_id;
      await onDeviceChange(devices[0].device_id);
    }
  } catch {
    sel.innerHTML = "<option value=''>Failed to load devices</option>";
  }
}

// ── On device select ─────────────────────────────────────────────────────────
async function onDeviceChange(deviceId) {
  selectedDevice  = deviceId || null;
  selectedSession = null;

  // Reset health + alerts
  resetHealth();
  setAlerts([]);
  clearMapTracks();
  setConnectionStatus("offline", "No session selected");
  document.getElementById("session-info").textContent = "No session selected";
  document.getElementById("pkt-counter").textContent  = "";
  packetCount  = 0;
  lastPacketAt = null;

  if (!deviceId) {
    document.getElementById("session-select").innerHTML = "<option value=''>Select a device first</option>";
    document.getElementById("session-select").disabled  = true;
    return;
  }

  // Load sessions for this device
  const ssel = document.getElementById("session-select");
  ssel.innerHTML  = "<option value=''>Loading...</option>";
  ssel.disabled   = true;

  try {
    const res = await AUTH.apiFetch(`/api/v1/sessions/${deviceId}`);
    if (!res || !res.ok) return;
    const sessions = await res.json();

    ssel.innerHTML = "<option value='live'>▶ LIVE</option>";
    sessions.forEach(s => {
      const o   = document.createElement("option");
      o.value   = s.session_id;
      const dur = s.duration_s ? Math.round(s.duration_s / 60) + "m" : "open";
      const ts  = s.started_at_server
        ? new Date(s.started_at_server).toLocaleString()
        : s.session_id.slice(-12);
      o.textContent = `${ts}  (${dur})`;
      ssel.appendChild(o);
    });

    ssel.disabled = false;

    // Auto-select LIVE
    ssel.value = "live";
    await onSessionChange("live");
  } catch {
    ssel.innerHTML = "<option value=''>Failed to load sessions</option>";
  }
}

// ── On session select ────────────────────────────────────────────────────────
async function onSessionChange(sessionId) {
  selectedSession = sessionId || null;
  clearMapTracks();

  stopPolling();

  if (!sessionId || !selectedDevice) return;

  if (sessionId === "live") {
    // Live mode
    document.getElementById("session-info").textContent = `LIVE — ${selectedDevice}`;
    joinDeviceRoom(selectedDevice);
    startPolling(selectedDevice);
  } else {
    // Historical replay mode — load stored track
    document.getElementById("session-info").textContent = `Session: ${sessionId.slice(-16)}`;
    setConnectionStatus("offline", "Historical session");
    await loadHistoricalTrack(sessionId);
    startPolling(selectedDevice);  // still poll health for device status
  }
}

// ── Historical track loader ──────────────────────────────────────────────────
async function loadHistoricalTrack(sessionId) {
  if (!mapReady) return;

  try {
    const [gr, er] = await Promise.all([
      AUTH.apiFetch(`/api/v1/gnss/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
      AUTH.apiFetch(`/api/v1/eskf/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
    ]);

    const gnss = gr && gr.ok ? await gr.json() : [];
    const eskf = er && er.ok ? await er.json() : [];

    if (gnss.length) {
      gnssPolyline.setPath(gnss.map(p => ({ lat: p.lat, lng: p.lon })));
    }
    if (eskf.length) {
      eskfPolyline.setPath(eskf.map(p => ({ lat: p.lat, lng: p.lon })));
    }

    // Auto-fit map bounds to track
    if (gnss.length || eskf.length) {
      const bounds = new google.maps.LatLngBounds();
      [...gnss, ...eskf].forEach(p => bounds.extend({ lat: p.lat, lng: p.lon }));
      gmap.fitBounds(bounds);
    }

    document.getElementById("pkt-counter").textContent =
      `${gnss.length} GNSS pts · ${eskf.length} ESKF pts`;
  } catch { /* silent */ }
}

// ── WebSocket ────────────────────────────────────────────────────────────────
function connectSocket() {
  socket = io(window.location.origin, {
    auth: { token: AUTH.getToken() },
    transports: ["websocket", "polling"],
  });

  socket.on("connect",    () => { /* connected */ });
  socket.on("disconnect", () => {
    if (selectedSession === "live")
      setConnectionStatus("offline", "WebSocket disconnected");
  });

  socket.on("live_eskf", (doc) => {
    if (doc.device_id !== selectedDevice) return;
    lastPacketAt = Date.now();
    packetCount++;

    setConnectionStatus("live", "Live");
    document.getElementById("stale-badge").style.display = "none";
    document.getElementById("pkt-counter").textContent   = `${packetCount} pkts`;

    // Append ESKF point to map
    if (mapReady && eskfPolyline && doc.lat && doc.lon) {
      const path = eskfPolyline.getPath();
      path.push(new google.maps.LatLng(doc.lat, doc.lon));
      gmap.panTo({ lat: doc.lat, lng: doc.lon });
    }
  });
}

function joinDeviceRoom(deviceId) {
  if (socket) socket.emit("join_device", { device_id: deviceId });
}

// ── Health polling ────────────────────────────────────────────────────────────
function startPolling(deviceId) {
  updateHealth(deviceId);
  updateAlerts(deviceId);
  healthTimer = setInterval(() => updateHealth(deviceId), 3000);
  alertTimer  = setInterval(() => updateAlerts(deviceId), 5000);
}

function stopPolling() {
  clearInterval(healthTimer);
  clearInterval(alertTimer);
  healthTimer = null;
  alertTimer  = null;
}

async function updateHealth(deviceId) {
  try {
    const res = await AUTH.apiFetch(`/api/v1/device/latest/${deviceId}`);
    if (!res || !res.ok) { resetHealth(); return; }
    const doc = await res.json();
    const st  = doc.status || {};

    // ── GNSS ──
    const sats   = st.sats  !== undefined ? st.sats  : null;
    const hdop   = st.hdop  !== undefined ? st.hdop  : null;
    const gnssQ  = sats !== null ? Math.min(100, Math.round(sats / 12 * 100)) : 0;
    const gnssColor = sats === null ? "#555"
                    : sats >= 8 ? "#10b981"
                    : sats >= 4 ? "#eab308"
                    : "#ef4444";
    setBar("h-gnss", gnssQ, gnssColor,
      sats !== null ? `${sats} sats` : "—",
      sats !== null ? `HDOP ${hdop !== null ? hdop.toFixed(1) : "—"}` : "No data");

    // ── WiFi signal ──
    const rssi   = st.wifi_rssi_dbm !== undefined ? st.wifi_rssi_dbm : null;
    const wifiQ  = rssi !== null ? Math.max(0, Math.min(100, Math.round((rssi + 90) / 60 * 100))) : 0;
    const wifiColor = rssi === null ? "#555"
                    : rssi >= -65 ? "#10b981"
                    : rssi >= -80 ? "#eab308"
                    : "#ef4444";
    setBar("h-wifi", wifiQ, wifiColor,
      rssi !== null ? `${rssi} dBm` : "—",
      rssi !== null ? (rssi >= -65 ? "Strong" : rssi >= -80 ? "Weak" : "Poor") : "No data");

    // ── ESKF ──
    const eskfInit = st.eskf_init !== undefined ? st.eskf_init : null;
    const eskfQ    = eskfInit === true ? 100 : 0;
    const eskfColor = eskfInit === true ? "#10b981"
                    : eskfInit === false ? "#ef4444"
                    : "#555";
    setBar("h-eskf", eskfQ, eskfColor,
      eskfInit === null ? "—" : eskfInit ? "OK" : "Not init",
      eskfInit === null ? "No data"
                       : eskfInit ? "Filter initialized" : "Awaiting GPS fix");

    // ── Battery ──
    const bpct = st.battery_pct !== undefined ? st.battery_pct : null;
    const bv   = st.battery_v   !== undefined ? st.battery_v   : null;
    if (bpct === null || bpct < 0) {
      setBar("h-batt", 0, "#555", "—", "N/A — IC pending");
    } else {
      const battColor = bpct >= 50 ? "#10b981" : bpct >= 20 ? "#eab308" : "#ef4444";
      setBar("h-batt", bpct, battColor,
        `${bpct}%`,
        bv !== null ? `${bv.toFixed(2)} V` : "");
    }

  } catch { resetHealth(); }
}

function setBar(id, pct, color, val, sub) {
  const fill = document.getElementById(`${id}-bar`);
  const valEl = document.getElementById(`${id}-val`);
  const subEl = document.getElementById(`${id}-sub`);
  if (fill)  { fill.style.width = pct + "%"; fill.style.background = color; }
  if (valEl) valEl.textContent = val;
  if (subEl) subEl.textContent = sub;
}

function resetHealth() {
  ["h-gnss", "h-wifi", "h-eskf", "h-batt"].forEach(id => {
    setBar(id, 0, "#555", "—", "No data");
  });
  document.getElementById("h-batt-sub").textContent = "N/A — IC pending";
}

// ── Alerts ────────────────────────────────────────────────────────────────────
async function updateAlerts(deviceId) {
  try {
    const res = await AUTH.apiFetch(`/api/v1/alerts/${deviceId}`);
    if (!res || !res.ok) return;
    setAlerts(await res.json());
  } catch { /* silent */ }
}

function setAlerts(alerts) {
  const list = document.getElementById("alert-list");
  if (!alerts.length) {
    list.innerHTML = "<div class='no-alerts'>No active alerts</div>";
    return;
  }
  list.innerHTML = alerts.map(a => `
    <div class="alert-item ${a.level}">
      <div class="alert-dot"></div>
      <div>
        <div class="alert-msg">${a.msg}</div>
        <div class="alert-code">${a.code}</div>
      </div>
    </div>
  `).join("");
}

// ── Stale data checker ────────────────────────────────────────────────────────
function startStaleChecker() {
  staleTimer = setInterval(() => {
    if (!lastPacketAt || selectedSession !== "live") return;
    const stale = Date.now() - lastPacketAt > 10000;
    const badge = document.getElementById("stale-badge");
    const dot   = document.getElementById("conn-dot");
    badge.style.display = stale ? "block" : "none";
    if (stale) {
      dot.className  = "status-dot stale";
      document.getElementById("conn-label").textContent = "Stale";
    }
  }, 1000);
}

// ── Helpers ───────────────────────────────────────────────────────────────────
function setConnectionStatus(state, label) {
  const dot = document.getElementById("conn-dot");
  dot.className = "status-dot " + state;
  document.getElementById("conn-label").textContent = label;
}

function clearMapTracks() {
  if (gnssPolyline) gnssPolyline.setPath([]);
  if (eskfPolyline) eskfPolyline.setPath([]);
}
