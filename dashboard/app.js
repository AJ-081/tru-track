// TRU-TRACK Dashboard — app.js
// Leaflet maps. All real data. Zero simulation.

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

// Leaflet objects
let gmap         = null;
let gnssPolyline = null;
let eskfPolyline = null;

// ── Init ──────────────────────────────────────────────────────────────────────
window.addEventListener("DOMContentLoaded", async () => {
  initLeafletMap();
  await loadDevices();
  connectSocket();
  startStaleChecker();
});

// ── Leaflet Map ───────────────────────────────────────────────────────────────
function initLeafletMap() {
  const ph = document.getElementById("map-placeholder");
  if (ph) ph.style.display = "none";

  gmap = L.map("map", { zoomControl: true }).setView([21.1642, 72.786], 15);

  // Satellite base layer (Esri World Imagery — free, no key)
  L.tileLayer(
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { attribution: "Tiles © Esri", maxZoom: 22, maxNativeZoom: 19 }
  ).addTo(gmap);

  // Labels overlay
  L.tileLayer(
    "https://{s}.basemaps.cartocdn.com/light_only_labels/{z}/{x}/{y}{r}.png",
    { attribution: "© CartoDB", maxZoom: 22, maxNativeZoom: 19, opacity: 0.7 }
  ).addTo(gmap);

  gnssPolyline = L.polyline([], { color: "#ef4444", weight: 2.5, opacity: 0.9 }).addTo(gmap);
  eskfPolyline = L.polyline([], { color: "#3b82f6", weight: 2.5, opacity: 0.9 }).addTo(gmap);
  L.control.layers(null, {"GNSS (red)": gnssPolyline, "ESKF (blue)": eskfPolyline}, {collapsed:false, position:"topright"}).addTo(gmap);

  mapReady = true;

  // Ensure map fills container after layout settles
  setTimeout(() => gmap.invalidateSize(), 300);
}

// ── Device dropdown ───────────────────────────────────────────────────────────
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

    if (devices.length === 1) {
      sel.value = devices[0].device_id;
      await onDeviceChange(devices[0].device_id);
    }
  } catch {
    sel.innerHTML = "<option value=''>Failed to load devices</option>";
  }
}

// ── On device select ──────────────────────────────────────────────────────────
async function onDeviceChange(deviceId) {
  selectedDevice  = deviceId || null;
  selectedSession = null;

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

  const ssel = document.getElementById("session-select");
  ssel.innerHTML = "<option value=''>Loading...</option>";
  ssel.disabled  = true;

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
    ssel.value    = "live";
    await onSessionChange("live");
  } catch {
    ssel.innerHTML = "<option value=''>Failed to load sessions</option>";
  }
}

// ── On session select ─────────────────────────────────────────────────────────
async function onSessionChange(sessionId) {
  selectedSession = sessionId || null;
  clearMapTracks();
  stopPolling();
  stopReplay();
  { const ts=document.getElementById("trip-section"); if(ts) ts.style.display="none"; }
  { const ts=document.getElementById('trip-section'); if(ts) ts.style.display='none'; }

  if (!sessionId || !selectedDevice) return;

  if (sessionId === "live") {
    setExportButtons(false);
    document.getElementById("session-info").textContent = `LIVE — ${selectedDevice}`;
    setConnectionStatus("live", "Awaiting packets…");
    if (!socket) initSocket();
    joinDeviceRoom(selectedDevice);
    startPolling(selectedDevice);
  } else {
    setExportButtons(true);
    document.getElementById("session-info").textContent = `Session: ${sessionId.slice(-16)}`;
    setConnectionStatus("offline", "Historical session");
    await loadHistoricalTrack(sessionId);
    await loadReplayData(sessionId);
    loadTripReport(sessionId);
    startPolling(selectedDevice);
  }
}

// ── Historical track ──────────────────────────────────────────────────────────
async function loadHistoricalTrack(sessionId) {
  if (!mapReady) return;
  try {
    const [gr, er] = await Promise.all([
      AUTH.apiFetch(`/api/v1/gnss/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
      AUTH.apiFetch(`/api/v1/eskf/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
    ]);

    const gnss = gr && gr.ok ? await gr.json() : [];
    const eskf = er && er.ok ? await er.json() : [];

    if (gnss.length) gnssPolyline.setLatLngs(gnss.map(p => [p.lat, p.lon]));
    if (eskf.length) eskfPolyline.setLatLngs(eskf.map(p => [p.lat, p.lon]));

    const all = [...gnss, ...eskf];
    if (all.length) {
      gmap.fitBounds(L.latLngBounds(all.map(p => [p.lat, p.lon])), { padding: [20, 20] });
    }

    document.getElementById("pkt-counter").textContent =
      `${gnss.length} GNSS · ${eskf.length} ESKF`;
  } catch { /* silent */ }
}

// ── WebSocket ─────────────────────────────────────────────────────────────────
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
    if (selectedSession !== "live") return;
    if (doc.device_id !== selectedDevice) return;
    lastPacketAt = Date.now();
    packetCount++;

    setConnectionStatus("live", "Live");
    document.getElementById("stale-badge").style.display = "none";
    document.getElementById("pkt-counter").textContent   = `${packetCount} pkts`;

    if (mapReady && eskfPolyline && doc.lat && doc.lon) {
      eskfPolyline.addLatLng([doc.lat, doc.lon]);
      gmap.panTo([doc.lat, doc.lon]);
    }
    if (mapReady && gnssPolyline && doc.gnss_valid && doc.gnss_lat && doc.gnss_lon) {
      gnssPolyline.addLatLng([doc.gnss_lat, doc.gnss_lon]);
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

    // GNSS
    const sats  = st.sats  !== undefined ? st.sats  : null;
    const hdop  = st.hdop  !== undefined ? st.hdop  : null;
    const gnssQ = sats !== null ? Math.min(100, Math.round(sats / 12 * 100)) : 0;
    const gnssColor = sats === null ? "#555" : sats >= 8 ? "#10b981" : sats >= 4 ? "#eab308" : "#ef4444";
    setBar("h-gnss", gnssQ, gnssColor,
      sats !== null ? `${sats} sats` : "—",
      sats !== null ? `HDOP ${hdop !== null ? hdop.toFixed(1) : "—"}` : "No data");

    // WiFi
    const rssi  = st.lte_rssi_dbm !== undefined ? st.lte_rssi_dbm : null;
    const wifiQ = rssi !== null ? Math.max(0, Math.min(100, Math.round((rssi + 90) / 60 * 100))) : 0;
    const wifiColor = rssi === null ? "#555" : rssi >= -65 ? "#10b981" : rssi >= -80 ? "#eab308" : "#ef4444";
    setBar("h-wifi", wifiQ, wifiColor,
      rssi !== null ? `${rssi} dBm` : "—",
      rssi !== null ? (rssi >= -65 ? "Strong" : rssi >= -80 ? "Weak" : "Poor") : "No data");

    // ESKF
    const eskfInit  = st.eskf_init !== undefined ? st.eskf_init : null;
    const eskfQ     = eskfInit === true ? 100 : 0;
    const eskfColor = eskfInit === true ? "#10b981" : eskfInit === false ? "#ef4444" : "#555";
    setBar("h-eskf", eskfQ, eskfColor,
      eskfInit === null ? "—" : eskfInit ? "OK" : "Not init",
      eskfInit === null ? "No data" : eskfInit ? "Filter initialized" : "Awaiting GPS fix");

    // Battery
    const bpct = st.battery_pct !== undefined ? st.battery_pct : null;
    const bv   = st.battery_v   !== undefined ? st.battery_v   : null;
    if (bpct === null || bpct < 0) {
      setBar("h-batt", 0, "#555", "—", "N/A — IC pending");
    } else {
      const battColor = bpct >= 50 ? "#10b981" : bpct >= 20 ? "#eab308" : "#ef4444";
      setBar("h-batt", bpct, battColor, `${bpct}%`, bv !== null ? `${bv.toFixed(2)} V` : "");
    }
  } catch { resetHealth(); }
}

function setBar(id, pct, color, val, sub) {
  const fill  = document.getElementById(`${id}-bar`);
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

// ── Stale checker ─────────────────────────────────────────────────────────────
function startStaleChecker() {
  staleTimer = setInterval(() => {
    if (!lastPacketAt || selectedSession !== "live") return;
    const stale = Date.now() - lastPacketAt > 10000;
    const badge = document.getElementById("stale-badge");
    const dot   = document.getElementById("conn-dot");
    badge.style.display = stale ? "block" : "none";
    if (stale) {
      dot.className = "status-dot stale";
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
  if (gnssPolyline) gnssPolyline.setLatLngs([]);
  if (eskfPolyline) eskfPolyline.setLatLngs([]);
}

// ══════════════════════════════════════════════════════════════════════════════
// EXPORT + REPLAY
// ══════════════════════════════════════════════════════════════════════════════

async function exportCSV() {
  if (!selectedDevice || !selectedSession || selectedSession === "live") return;
  const url = `/api/v1/session/export?device_id=${encodeURIComponent(selectedDevice)}&session_id=${encodeURIComponent(selectedSession)}`;
  await triggerDownload(url);
}

async function exportGeoJSON() {
  if (!selectedDevice || !selectedSession || selectedSession === "live") return;
  const url = `/api/v1/session/geojson?device_id=${encodeURIComponent(selectedDevice)}&session_id=${encodeURIComponent(selectedSession)}`;
  await triggerDownload(url);
}

async function exportImuHf() {
  if (!selectedDevice || !selectedSession || selectedSession === "live") return;
  const url = `/api/v1/session/export-imu-hf?device_id=${encodeURIComponent(selectedDevice)}&session_id=${encodeURIComponent(selectedSession)}`;
  await triggerDownload(url);
}

async function triggerDownload(url) {
  try {
    const res = await AUTH.apiFetch(url);
    if (!res || !res.ok) { alert("Export failed — no data for this session."); return; }
    const blob = await res.blob();
    const disp = res.headers.get("Content-Disposition") || "";
    const match = disp.match(/filename="?([^";]+)"?/);
    const fname = match ? match[1].trim() : "tru-track-export";
    const a = document.createElement("a");
    a.href     = URL.createObjectURL(blob);
    a.download = fname;
    a.click();
    URL.revokeObjectURL(a.href);
  } catch { alert("Export failed — check connection."); }
}

// ── Replay ────────────────────────────────────────────────────────────────────
let replayGnss   = [];
let replayEskf   = [];
let replayIdx    = 0;
let replayTimer  = null;
let replayActive = false;

async function loadReplayData(sessionId) {
  if (!mapReady) return;
  try {
    const [gr, er] = await Promise.all([
      AUTH.apiFetch(`/api/v1/gnss/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
      AUTH.apiFetch(`/api/v1/eskf/track/${selectedDevice}?session_id=${encodeURIComponent(sessionId)}`),
    ]);
    replayGnss = gr && gr.ok ? await gr.json() : [];
    replayEskf = er && er.ok ? await er.json() : [];
    replayIdx  = 0;

    const slider = document.getElementById("replay-slider");
    slider.max   = Math.max(replayGnss.length, replayEskf.length);
    slider.value = 0;
    updateReplayTime(0);

    if (replayGnss.length) gnssPolyline.setLatLngs(replayGnss.map(p => [p.lat, p.lon]));
    if (replayEskf.length) eskfPolyline.setLatLngs(replayEskf.map(p => [p.lat, p.lon]));

    const all = [...replayGnss, ...replayEskf];
    if (all.length) {
      gmap.fitBounds(L.latLngBounds(all.map(p => [p.lat, p.lon])), { padding: [20, 20] });
    }

    document.getElementById("pkt-counter").textContent =
      `${replayGnss.length} GNSS · ${replayEskf.length} ESKF`;
  } catch { /* silent */ }
}

function toggleReplay() {
  if (!replayGnss.length && !replayEskf.length) return;
  replayActive = !replayActive;
  const btn = document.getElementById("btn-replay");
  btn.classList.toggle("active", replayActive);
  btn.textContent = replayActive ? "⏹ Stop" : "▶ Replay";

  if (replayActive) {
    replayIdx = 0;
    document.getElementById("replay-bar").classList.add("visible");
    replayStep();
  } else {
    stopReplay();
  }
}

function replayStep() {
  const max = Math.max(replayGnss.length, replayEskf.length);
  if (replayIdx >= max) { stopReplay(); return; }

  if (mapReady) {
    gnssPolyline.setLatLngs(replayGnss.slice(0, replayIdx + 1).map(p => [p.lat, p.lon]));
    eskfPolyline.setLatLngs(replayEskf.slice(0, replayIdx + 1).map(p => [p.lat, p.lon]));
  }

  document.getElementById("replay-slider").value = replayIdx;
  updateReplayTime(replayIdx);
  replayIdx++;
  replayTimer = setTimeout(replayStep, 80);
}

function replaySeek(val) {
  replayIdx = parseInt(val);
  if (mapReady) {
    gnssPolyline.setLatLngs(replayGnss.slice(0, replayIdx + 1).map(p => [p.lat, p.lon]));
    eskfPolyline.setLatLngs(replayEskf.slice(0, replayIdx + 1).map(p => [p.lat, p.lon]));
  }
  updateReplayTime(replayIdx);
}

function replayPlayPause() {
  if (replayTimer) {
    clearTimeout(replayTimer);
    replayTimer = null;
    document.getElementById("replay-play-btn").textContent = "▶";
  } else {
    document.getElementById("replay-play-btn").textContent = "⏸";
    replayStep();
  }
}

function stopReplay() {
  clearTimeout(replayTimer);
  replayTimer  = null;
  replayActive = false;
  const btn = document.getElementById("btn-replay");
  if (btn) { btn.classList.remove("active"); btn.textContent = "▶ Replay"; }
  const bar = document.getElementById("replay-bar");
  if (bar) bar.classList.remove("visible");
  const pbtn = document.getElementById("replay-play-btn");
  if (pbtn) pbtn.textContent = "▶";
}

function updateReplayTime(idx) {
  const max = Math.max(replayGnss.length, replayEskf.length);
  document.getElementById("replay-time").textContent = `${idx} / ${max}`;
}

function setExportButtons(enabled) {
  ["btn-csv", "btn-geojson", "btn-imuhf", "btn-replay"].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.disabled = !enabled;
  });
}

// ── Server status panel ───────────────────────────────────────────────────────
const SRV_LABELS = {
  healthy:       "Healthy",
  warning:       "Warning",
  major_warning: "Major Warning",
  critical:      "Critical",
};

async function updateServerStatus() {
  try {
    const res = await AUTH.apiFetch("/api/v1/server/health");
    if (!res || !res.ok) return;
    const d = await res.json();

    const dot   = document.getElementById("srv-dot");
    const label = document.getElementById("srv-label");
    dot.className   = "srv-dot " + (d.status_level || "");
    label.textContent = SRV_LABELS[d.status_level] || d.status_level;

    document.getElementById("srv-cpu").textContent  = d.cpu_percent.toFixed(1) + "%";
    document.getElementById("srv-ram").textContent  = d.memory.percent.toFixed(1) + "%  (" + d.memory.used_gb + "/" + d.memory.total_gb + " GB)";
    document.getElementById("srv-disk").textContent = d.disk.percent.toFixed(1) + "%  (" + d.disk.used_gb + "/" + d.disk.total_gb + " GB)";
    document.getElementById("srv-svcs").textContent = d.services_up + "/" + d.services_total + " running";
  } catch { /* silent */ }
}

// Poll server status every 15 seconds
updateServerStatus();
setInterval(updateServerStatus, 15000);

async function loadTripReport(sessionId){
  const sec=document.getElementById("trip-section");
  if(!sec) return;
  try{
    const r=await AUTH.apiFetch("/api/v1/trip/"+selectedDevice+"?session_id="+encodeURIComponent(sessionId));
    if(!r||!r.ok){sec.style.display="none";return;}
    const t=await r.json();
    if(t.error){sec.style.display="none";return;}
    document.getElementById("trip-dist").textContent=t.distance_km+" km";
    document.getElementById("trip-dur").textContent=t.duration_min+" min";
    document.getElementById("trip-max").textContent=t.max_speed_kmh+" km/h";
    document.getElementById("trip-avg").textContent=t.avg_speed_kmh+" km/h";
    document.getElementById("trip-stops").textContent=t.stops;
    document.getElementById("trip-deny").textContent=t.gnss_deny_windows;
    sec.style.display="block";
  }catch(e){sec.style.display="none";}
}
