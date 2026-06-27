/* ═══════════════════════════════════════════════════════════════
   TRU-TRACK v3 · app.js
   All JS logic — auth, fleet, live tracking, sessions, server health
   Requires: leaflet.js, chart.umd.min.js, socket.io.min.js (all local)
═══════════════════════════════════════════════════════════════ */

'use strict';

/* ─── Constants ─── */
const BASE = '';   // same origin via nginx proxy
const POLL_INTERVAL_MS = 500;
const SERVER_POLL_MS   = 30000;

/* ─── Tile layer definitions (no CDN — cartocdn tiles fetched at runtime) ─── */
const TILE_LAYERS = {
  'carto-dark':  { url: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',    sub: 'abcd' },
  'carto-light': { url: 'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',   sub: 'abcd' },
  'osm':         { url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',               sub: 'abc'  },
  'topo':        { url: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',                 sub: 'abc'  },
};

function makeTile(key) {
  const t = TILE_LAYERS[key] || TILE_LAYERS['osm'];
  return L.tileLayer(t.url, { subdomains: t.sub, maxZoom: 19 });
}

/* ═══════════════════════════════════════════════════════════════
   AUTH MODULE
═══════════════════════════════════════════════════════════════ */
const AUTH = {
  _token: null,

  get token() {
    if (!this._token) this._token = localStorage.getItem('tt_jwt');
    return this._token;
  },

  set token(t) {
    this._token = t;
    if (t) localStorage.setItem('tt_jwt', t);
    else localStorage.removeItem('tt_jwt');
  },

  async login() {
    const user = document.getElementById('li-email').value.trim();
    const pass = document.getElementById('li-pass').value;
    const errEl = document.getElementById('li-err');
    errEl.style.display = 'none';
    try {
      const res = await fetch(BASE + '/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email: user, password: pass }),
      });
      if (!res.ok) throw new Error('bad creds');
      const data = await res.json();
      this.token = data.access_token;
      this._bootApp();
    } catch(e) {
      errEl.style.display = 'block';
    }
  },

  async apiFetch(url, opts = {}) {
    const headers = { 'Authorization': 'Bearer ' + this.token, ...(opts.headers || {}) };
    const res = await fetch(BASE + url, { ...opts, headers });
    if (res.status === 401) {
      this.token = null;
      location.reload();
    }
    return res;
  },

  async apiJSON(url, opts) {
    const res = await this.apiFetch(url, opts);
    if (!res.ok) throw new Error('API ' + res.status);
    return res.json();
  },

  async tryAutoLogin() {
    if (!this.token) return false;
    // Validate token with a cheap unauthenticated call first,
    // then do a device list to confirm auth works
    try {
      await this.apiJSON('/api/devices');
      return true;
    } catch(e) {
      this.token = null;
      return false;
    }
  },

  _bootApp() {
    document.getElementById('login-overlay').style.display = 'none';
    document.getElementById('app').style.display = 'flex';
    bootApp();
  },
};

// Enter key on login
document.addEventListener('keydown', e => {
  if (e.key === 'Enter' && document.getElementById('login-overlay').style.display !== 'none') {
    AUTH.login();
  }
});

/* ═══════════════════════════════════════════════════════════════
   THEME
═══════════════════════════════════════════════════════════════ */
function toggleTheme() {
  const html = document.documentElement;
  const next = html.getAttribute('data-theme') === 'dark' ? 'light' : 'dark';
  html.setAttribute('data-theme', next);
  localStorage.setItem('tt_theme', next);
  document.getElementById('theme-btn').textContent = next === 'dark' ? '🌙' : '☀️';

  // Switch map tile layers to match theme (only if user hasn't manually overridden)
  const tileKey = next === 'dark' ? 'osm' : 'osm';  // OSM default both modes
  // Apply tile filter is handled automatically by CSS var(--tile-filter) on .leaflet-tile-pane
}

function loadTheme() {
  const saved = localStorage.getItem('tt_theme') || 'dark';
  document.documentElement.setAttribute('data-theme', saved);
  const btn = document.getElementById('theme-btn');
  if (btn) btn.textContent = saved === 'dark' ? '🌙' : '☀️';
}

/* ═══════════════════════════════════════════════════════════════
   PAGE ROUTING
═══════════════════════════════════════════════════════════════ */
let _activePage = 'fleet';

function showPage(id, btn) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.nav-tab').forEach(b => b.classList.remove('active'));
  document.getElementById('page-' + id).classList.add('active');
  if (btn) btn.classList.add('active');
  _activePage = id;
  // Pages stay in DOM (visibility:hidden not display:none) so maps always have dimensions

  // Lazy-init maps (init if not already, then force size recalc after paint)
  if (id === 'fleet') {
    if (!FLEET._mapInit) FLEET.initMap();
    setTimeout(() => FLEET._map && FLEET._map.invalidateSize(), 80);
  }
  if (id === 'live') {
    // Destroy and recreate map each time — guarantees correct dimensions
    // since display:none gives 0x0 and Leaflet can't recover from that
    if (LIVE._map) {
      LIVE._map.remove();
      LIVE._map = null;
      LIVE._mapInit = false;
      LIVE._gnssLine = null;
      LIVE._eskfLine = null;
      LIVE._curMarker = null;
    }
    LIVE.initMap();
    // Reload track data onto fresh map
    if (LIVE._deviceId) setTimeout(() => LIVE._loadAll(), 50);
  }
  if (id === 'sessions') {
    if (!SESSIONS._miniMapInit) SESSIONS.initMiniMap();
    setTimeout(() => SESSIONS._miniMap && SESSIONS._miniMap.invalidateSize(), 80);
  }
  if (id === 'server') SERVER.fetch();
}

/* ═══════════════════════════════════════════════════════════════
   UTILITIES
═══════════════════════════════════════════════════════════════ */
const U = {
  fmtDur(s) {
    if (s == null) return '—';
    s = Math.round(s);
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), sec = s % 60;
    if (h) return `${h}h ${m}m`;
    if (m) return `${m}m ${sec}s`;
    return `${sec}s`;
  },

  fmtDT(iso) {
    if (!iso) return '—';
    try { return new Date(iso).toLocaleString('en-IN', { hour12: false }); }
    catch { return iso; }
  },

  fmtAge(isoOrMs) {
    try {
      const d = typeof isoOrMs === 'number' ? isoOrMs : new Date(isoOrMs).getTime();
      const s = Math.round((Date.now() - d) / 1000);
      if (s < 5)   return 'just now';
      if (s < 60)  return `${s}s ago`;
      if (s < 3600) return `${Math.floor(s/60)}m ago`;
      return `${Math.floor(s/3600)}h ago`;
    } catch { return '—'; }
  },

  r(v, dp = 4) { return v != null ? parseFloat(v).toFixed(dp) : '—'; },
  r2(v)  { return this.r(v, 2); },
  r1(v)  { return this.r(v, 1); },
  r0(v)  { return v != null ? Math.round(v).toString() : '—'; },

  rad2deg(r) { return r != null ? (r * 180 / Math.PI).toFixed(1) : '—'; },

  setText(id, val) {
    const el = document.getElementById(id);
    if (el) el.textContent = val ?? '—';
  },

  setClass(id, cls) {
    const el = document.getElementById(id);
    if (el) el.className = cls;
  },

  barPct(id, pct, colorClass) {
    const el = document.getElementById(id);
    if (!el) return;
    el.style.width = Math.min(Math.max(pct, 0), 100) + '%';
    if (colorClass) el.className = 'pbar-fill ' + colorClass;
  },

  onlineClass(status) {
    if (status === 'live')    return 'sdot live';
    if (status === 'stale')   return 'sdot stale';
    if (status === 'offline') return 'sdot offline';
    return 'sdot unknown';
  },

  sessionQS(deviceId, sessionId) {
    let url = `/api/v1/session/export?device_id=${encodeURIComponent(deviceId)}`;
    if (sessionId) url += `&session_id=${encodeURIComponent(sessionId)}`;
    return url;
  },

  rssiToPercent(dbm) {
    // Map roughly -50 dBm (excellent) to -120 dBm (dead)
    if (dbm == null) return 0;
    return Math.round(Math.min(Math.max((dbm + 120) / 70 * 100, 0), 100));
  },

  lteClass(dbm) {
    if (dbm == null) return 'red';
    if (dbm >= -80) return 'green';
    if (dbm >= -100) return 'amber';
    return 'red';
  },

  gnssQualityPct(sats, hdop) {
    // Rough quality: more sats + lower HDOP = better
    const satScore  = Math.min((sats || 0) / 20 * 100, 100);
    const hdopScore = hdop ? Math.max(0, 100 - hdop * 15) : 0;
    return Math.round((satScore * 0.6 + hdopScore * 0.4));
  },

  gnssClass(pct) {
    if (pct >= 70) return 'green';
    if (pct >= 40) return 'amber';
    return 'red';
  },

  batClass(pct) {
    if (pct >= 40) return 'green';
    if (pct >= 15) return 'amber';
    return 'red';
  },
};

/* ═══════════════════════════════════════════════════════════════
   FLEET MODULE
═══════════════════════════════════════════════════════════════ */
const FLEET = {
  _map: null, _mapInit: false, _tile: null,
  _gnssLine: null, _eskfLine: null, _markers: {},
  _follow: true, _paused: false,
  _pollTimer: null,
  _activeDevice: null,
  _activeSession: '',

  initMap() {
    this._mapInit = true;
    this._map = L.map('fleet-map', { zoomControl: true }).setView([21.19, 72.81], 12);
    this._tile = makeTile('osm');
    this._tile.addTo(this._map);
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(() => { if (this._map) this._map.invalidateSize(); });
      ro.observe(document.getElementById('fleet-map'));
    }
  },

  switchLayer(key) {
    if (!this._map) return;
    if (this._tile) this._map.removeLayer(this._tile);
    this._tile = makeTile(key);
    this._tile.addTo(this._map);
  },

  async refresh() {
    try {
      const fleet = await AUTH.apiJSON('/api/v1/fleet');
      this._renderDeviceList(fleet);
      this._updateMarkers(fleet);
      U.setText('fleet-device-count', fleet.length + ' device' + (fleet.length !== 1 ? 's' : ''));
    } catch(e) {
      console.error('Fleet refresh error:', e);
    }
  },

  _renderDeviceList(fleet) {
    const el = document.getElementById('fleet-device-list');
    if (!fleet.length) {
      el.innerHTML = '<div class="empty"><div class="empty-icon">📡</div><div class="empty-label">No devices found</div></div>';
      return;
    }

    // Populate device dropdowns across all pages
    this._populateDeviceDropdowns(fleet.map(f => f.device_id));
    // Also populate sessions page selector
    const sessSel = document.getElementById('sess-dev-sel');
    if (sessSel && sessSel.options.length <= 1) {
      fleet.forEach(d => {
        if (![...sessSel.options].some(o => o.value === d.device_id)) {
          const opt = document.createElement('option');
          opt.value = d.device_id; opt.textContent = d.device_id;
          sessSel.appendChild(opt);
        }
      });
    }

    // Auto-select first device on Live Tracking if none selected yet
    const liveDevSel = document.getElementById('live-dev-sel');
    if (liveDevSel && !liveDevSel.value && fleet.length) {
      liveDevSel.value = fleet[0].device_id;
      // Only trigger load if live page is active or has been visited
      if (LIVE._mapInit) LIVE.onDeviceChange(fleet[0].device_id);
    }

    el.innerHTML = '';
    fleet.forEach(d => {
      const div = document.createElement('div');
      div.className = 'dcard' + (d.device_id === this._activeDevice ? ' active' : '');
      div.innerHTML = `
        <div class="dcard-icon">🛰</div>
        <div class="dcard-info">
          <div class="dcard-name">${d.device_id}</div>
          <div class="dcard-meta">${d.fw_version || '—'} · ${d.last_seen_display || '—'}</div>
        </div>
        <div class="dcard-right">
          <span class="${U.onlineClass(d.online)}"></span>
          <span class="f10 mono text-muted">${d.speed_mps != null ? (d.speed_mps * 3.6).toFixed(0) + ' km/h' : '—'}</span>
        </div>`;
      div.onclick = () => this._selectDevice(d.device_id, div);
      el.appendChild(div);
    });
  },

  _populateDeviceDropdowns(ids) {
    ['live-dev-sel', 'sess-dev-sel'].forEach(selId => {
      const sel = document.getElementById(selId);
      if (!sel) return;
      const cur = sel.value;
      sel.innerHTML = '<option value="">— Device —</option>';
      ids.forEach(id => {
        const opt = document.createElement('option');
        opt.value = id; opt.textContent = id;
        if (id === cur) opt.selected = true;
        sel.appendChild(opt);
      });
    });
  },

  _updateMarkers(fleet) {
    if (!this._map) return;
    fleet.forEach(d => {
      if (!d.lat || !d.lon) return;
      const col = d.online === 'live' ? '#00e5a0' : d.online === 'stale' ? '#ffb020' : '#ff4757';
      if (this._markers[d.device_id]) {
        this._markers[d.device_id].setLatLng([d.lat, d.lon]);
      } else {
        this._markers[d.device_id] = L.circleMarker([d.lat, d.lon], {
          radius: 8, color: col, fillColor: col, fillOpacity: 0.85, weight: 2,
        }).bindPopup(`<b>${d.device_id}</b><br>Sats: ${d.sats ?? '—'} · HDOP: ${d.hdop ?? '—'}<br>Speed: ${d.speed_mps != null ? (d.speed_mps * 3.6).toFixed(1) : '—'} km/h`)
          .addTo(this._map);
      }
    });
  },

  async _selectDevice(deviceId, cardEl) {
    this._activeDevice = deviceId;
    document.querySelectorAll('.dcard').forEach(c => c.classList.remove('active'));
    cardEl.classList.add('active');

    // Load sessions into dropdown
    try {
      const sessions = await AUTH.apiJSON(`/api/v1/sessions/${encodeURIComponent(deviceId)}`);
      const sel = document.getElementById('fleet-sess-sel');
      sel.innerHTML = '<option value="">Latest / Live</option>';
      sessions.forEach(s => {
        const opt = document.createElement('option');
        opt.value = s.session_id;
        opt.textContent = s.label || U.fmtDT(s.started_at_server);
        sel.appendChild(opt);
      });
    } catch(e) {}

    this._activeSession = '';
    this._renderTrack();
    this._startPoll();
  },

  onSessionChange() {
    this._activeSession = document.getElementById('fleet-sess-sel').value;
    this._renderTrack();
  },

  async _renderTrack() {
    if (!this._activeDevice || !this._map) return;

    // Always scope to a specific session to avoid cross-session connecting lines.
    // If no session selected, use the latest one.
    let sessionId = this._activeSession;
    if (!sessionId) {
      try {
        const sessions = await AUTH.apiJSON(`/api/v1/sessions/${encodeURIComponent(this._activeDevice)}`);
        if (sessions && sessions.length) {
          // Pick most recent
          const sorted = [...sessions].sort((a, b) =>
            new Date(b.started_at_server || 0) - new Date(a.started_at_server || 0));
          sessionId = sorted[0].session_id;
        }
      } catch(e) {}
    }

    const qs = sessionId ? `?session_id=${encodeURIComponent(sessionId)}` : '';

    try {
      const [gnss, eskf] = await Promise.all([
        AUTH.apiJSON(`/api/v1/gnss/track/${encodeURIComponent(this._activeDevice)}${qs}`),
        AUTH.apiJSON(`/api/v1/eskf/track/${encodeURIComponent(this._activeDevice)}${qs}`),
      ]);

      // Clear previous track lines
      if (this._gnssLine) { this._map.removeLayer(this._gnssLine); this._gnssLine = null; }
      if (this._eskfLine) { this._map.removeLayer(this._eskfLine); this._eskfLine = null; }

      const gPts = gnss.filter(p => p.lat && p.lon).map(p => [p.lat, p.lon]);
      const ePts = eskf.filter(p => p.lat && p.lon).map(p => [p.lat, p.lon]);

      if (gPts.length) this._gnssLine = L.polyline(gPts, { color: '#ff4757', weight: 2, opacity: .7 }).addTo(this._map);
      if (ePts.length) {
        this._eskfLine = L.polyline(ePts, { color: '#00d4ff', weight: 2.5, opacity: .9 }).addTo(this._map);
        if (this._follow) {
          const last = ePts[ePts.length - 1];
          this._map.setView(last, Math.max(this._map.getZoom(), 14));
        }
      } else if (gPts.length && this._follow) {
        this._map.setView(gPts[gPts.length - 1], Math.max(this._map.getZoom(), 14));
      }
    } catch(e) { console.error('Track load error:', e); }
  },

  _startPoll() {
    clearInterval(this._pollTimer);
    if (!this._paused) {
      this._pollTimer = setInterval(() => { if (!this._paused) this._renderTrack(); }, POLL_INTERVAL_MS);
    }
  },

  toggleFollow() {
    this._follow = !this._follow;
    document.getElementById('btn-follow').classList.toggle('on', this._follow);
  },

  togglePause() {
    this._paused = !this._paused;
    const btn = document.getElementById('btn-pause');
    btn.classList.toggle('on', this._paused);
    btn.textContent = this._paused ? '▶ Resume' : 'Pause';
    if (!this._paused) { this._renderTrack(); this._startPoll(); }
    else clearInterval(this._pollTimer);
  },
};

/* ═══════════════════════════════════════════════════════════════
   LIVE TRACKING MODULE
═══════════════════════════════════════════════════════════════ */
const LIVE = {
  _map: null, _mapInit: false, _tile: null,
  _gnssLine: null, _eskfLine: null, _curMarker: null,
  _gnssOn: true, _eskfOn: true,
  _deviceId: null, _sessionId: '', _liveSessionId: '', _trackLoaded: false,
  _gnssTrack: [], _eskfTrack: [],  // full arrays for replay
  _pollTimer: null,
  _socket: null,
  _replayTimer: null, _replayIdx: 0, _replayPlaying: false,

  initMap() {
    this._mapInit = true;
    this._map = L.map('live-map', { zoomControl: true, attributionControl: false }).setView([21.19, 72.81], 14);
    this._tile = makeTile('osm');
    this._tile.addTo(this._map);
    // ResizeObserver forces correct size whenever container becomes visible
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(() => {
        if (this._map) this._map.invalidateSize();
      });
      ro.observe(document.getElementById('live-map'));
    }
  },

  switchLayer(key) {
    if (!this._map) return;
    if (this._tile) this._map.removeLayer(this._tile);
    this._tile = makeTile(key);
    this._tile.addTo(this._map);
  },

  toggleLayer(which) {
    if (which === 'gnss') {
      this._gnssOn = !this._gnssOn;
      document.getElementById('live-gnss-tog').classList.toggle('on', this._gnssOn);
      if (this._gnssLine) this._gnssLine.setStyle({ opacity: this._gnssOn ? .7 : 0 });
    } else {
      this._eskfOn = !this._eskfOn;
      document.getElementById('live-eskf-tog').classList.toggle('on', this._eskfOn);
      if (this._eskfLine) this._eskfLine.setStyle({ opacity: this._eskfOn ? .9 : 0 });
    }
  },

  fitBounds() {
    if (!this._map) return;
    const pts = [];
    if (this._gnssLine) pts.push(...this._gnssLine.getLatLngs());
    if (this._eskfLine) pts.push(...this._eskfLine.getLatLngs());
    if (pts.length) this._map.fitBounds(L.latLngBounds(pts), { padding: [20, 20] });
  },

  async onDeviceChange(deviceId) {
    this._deviceId = deviceId;
    if (!deviceId) return;
    try {
      const sessions = await AUTH.apiJSON(`/api/v1/sessions/${encodeURIComponent(deviceId)}`);
      const sel = document.getElementById('live-sess-sel');
      sel.innerHTML = '<option value="">Live (latest)</option>';
      sessions.forEach(s => {
        const opt = document.createElement('option');
        opt.value = s.session_id;
        opt.textContent = s.label || U.fmtDT(s.started_at_server);
        sel.appendChild(opt);
      });
    } catch(e) {}
    this._sessionId = '';
    this._liveSessionId = '';
    this._trackLoaded = false;
    // Clear map immediately so old session track disappears
    this._gnssTrack = []; this._eskfTrack = [];
    if (this._gnssLine)  { this._map && this._map.removeLayer(this._gnssLine);  this._gnssLine = null; }
    if (this._eskfLine)  { this._map && this._map.removeLayer(this._eskfLine);  this._eskfLine = null; }
    if (this._curMarker) { this._map && this._map.removeLayer(this._curMarker); this._curMarker = null; }
    await this._loadAll();
    this._startPoll();
    this._connectSocket();
  },

  async onSessionChange() {
    this._sessionId = document.getElementById('live-sess-sel').value;

    if (this._sessionId) {
      // Historic session — stop live poll and remove live marker
      clearInterval(this._pollTimer);
      this._pollTimer = null;
      if (this._curMarker) { this._map && this._map.removeLayer(this._curMarker); this._curMarker = null; }
    } else {
      // Back to live — restart poll
      this._trackLoaded = false;
      this._startPoll();
    }

    await this._loadAll();
  },

  async _loadAll() {
    if (!this._deviceId) return;
    if (!this._sessionId) {
      // Live mode — fetch telemetry for sidebar, load track once, then poll takes over
      try {
        const latest = await AUTH.apiJSON(`/api/v1/device/latest/${encodeURIComponent(this._deviceId)}`);
        this._applyTelemetry(latest);
        if (latest.session_id) this._liveSessionId = latest.session_id;
      } catch(e) {}
      if (!this._trackLoaded) { this._trackLoaded = true; await this._loadTrack(); }
      await this._loadAlerts();
    } else {
      // Historic session
      await Promise.all([this._loadTrack(), this._loadAlerts()]);
    }
  },

  async _loadTrack() {
    if (!this._deviceId) return;
    // Scope to specific session always — avoids cross-session connecting lines
    const sid = this._sessionId || this._liveSessionId || '';
    const qs = sid ? `?session_id=${encodeURIComponent(sid)}` : '';
    try {
      const [gnss, eskf] = await Promise.all([
        AUTH.apiJSON(`/api/v1/gnss/track/${encodeURIComponent(this._deviceId)}${qs}`),
        AUTH.apiJSON(`/api/v1/eskf/track/${encodeURIComponent(this._deviceId)}${qs}`),
      ]);
      this._gnssTrack = gnss;
      this._eskfTrack = eskf;
      U.setText('replay-pts', gnss.length + ' GNSS · ' + eskf.length + ' ESKF');
      const scrub = document.getElementById('replay-scrub');
      const maxIdx = Math.max(gnss.filter(p=>p.lat&&p.lon).length - 1, 0);
      if (scrub) { scrub.max = maxIdx; scrub.value = maxIdx; }
      this._replayIdx = maxIdx;
      // Draw full track by default (scrubber at end = show everything)
      this._drawTracks();
      const lastPt = gnss.filter(p=>p.lat&&p.lon)[maxIdx];
      if (lastPt) U.setText('replay-time', U.fmtDT(lastPt.t_server));
    } catch(e) {}
  },

  _drawTracks(upToGnssIdx, upToEskfIdx) {
    if (!this._map) return;
    if (this._gnssLine)  this._map.removeLayer(this._gnssLine);
    if (this._eskfLine)  this._map.removeLayer(this._eskfLine);
    if (this._curMarker) this._map.removeLayer(this._curMarker);

    const gAll = this._gnssTrack.filter(p => p.lat && p.lon);
    const eAll = this._eskfTrack.filter(p => p.lat && p.lon);

    const gPts = upToGnssIdx != null ? gAll.slice(0, upToGnssIdx + 1) : gAll;
    const ePts = upToEskfIdx != null ? eAll.slice(0, upToEskfIdx + 1) : eAll;

    if (gPts.length) {
      this._gnssLine = L.polyline(gPts.map(p => [p.lat, p.lon]), {
        color: '#ff4757', weight: 2, opacity: this._gnssOn ? .7 : 0,
      }).addTo(this._map);
    }
    // Only draw ESKF track if it has valid filtered points (not noise circles)
    const ePtsValid = ePts.filter((p,i) => {
      if (i === 0) return true;
      const prev = ePts[i-1];
      const dlat = Math.abs(p.lat - prev.lat), dlon = Math.abs(p.lon - prev.lon);
      return (dlat + dlon) < 0.01; // reject jumps > ~1km between consecutive ESKF points
    });
    if (ePtsValid.length > 1) {
      this._eskfLine = L.polyline(ePtsValid.map(p => [p.lat, p.lon]), {
        color: '#00d4ff', weight: 2.5, opacity: this._eskfOn ? .9 : 0,
      }).addTo(this._map);
      const last = ePtsValid[ePtsValid.length - 1];
      this._curMarker = L.circleMarker([last.lat, last.lon], {
        radius: 7, color: '#00d4ff', fillColor: '#00d4ff', fillOpacity: .9, weight: 2,
      }).addTo(this._map);
    } else if (gPts.length) {
      const last = gPts[gPts.length - 1];
      this._curMarker = L.circleMarker([last.lat, last.lon], {
        radius: 7, color: '#ff4757', fillColor: '#ff4757', fillOpacity: .9, weight: 2,
      }).addTo(this._map);
    }

    // Auto-fit if track just loaded (idx == null = full load)
    if (upToGnssIdx == null && (gPts.length || ePts.length)) {
      const allPts = [...gPts.map(p => [p.lat, p.lon]), ...ePts.map(p => [p.lat, p.lon])];
      this._map.fitBounds(L.latLngBounds(allPts), { padding: [40, 40] });
    }
  },

  _applyTelemetry(doc) {
    if (!doc) return;
    const s  = doc.status  || {};
    const g  = doc.gnss    || {};
    const e  = doc.eskf    || {};

    // Status badge
    const age = doc.t_server ? (Date.now() - new Date(doc.t_server).getTime()) / 1000 : 999;
    const badge = document.getElementById('live-badge');
    if (age < 30) {
      badge.className = 'badge-live glass';
      badge.innerHTML = '<div class="live-dot pulse"></div>LIVE';
    } else if (age < 120) {
      badge.className = 'badge-stale glass';
      badge.innerHTML = '<div class="live-dot"></div>STALE · ' + U.fmtAge(doc.t_server);
    } else {
      badge.className = 'badge-offline glass';
      badge.innerHTML = '<div class="live-dot"></div>OFFLINE · ' + U.fmtAge(doc.t_server);
    }

    // GNSS panel
    U.setText('lv-lat',    g.lat    != null ? U.r(g.lat, 7) : '—');
    U.setText('lv-lon',    g.lon    != null ? U.r(g.lon, 7) : '—');
    U.setText('lv-alt',    g.alt    != null ? U.r1(g.alt)  : '—');
    U.setText('lv-spd',    g.speed_mps != null ? U.r1(g.speed_mps * 3.6) : '—');
    U.setText('lv-course', g.course != null ? U.r1(g.course) : '—');
    U.setText('lv-hdop',   g.hdop   != null ? U.r2(g.hdop)  : '—');
    U.setText('lv-age',    g.age_ms != null ? g.age_ms + ' ms' : '—');

    const fixValid = g.fix_valid ?? s.gnss_fix ?? false;
    const fixMode  = g.fix_type ?? s.gnss_mode ?? 0;
    const fixLabel = fixValid ? (fixMode === 3 ? '3D FIX' : fixMode === 2 ? '2D FIX' : 'FIX') : 'NO FIX';
    const fixCls   = fixValid ? 'tag tag-green' : 'tag tag-red';
    document.getElementById('lv-fix-tag').textContent = fixLabel;
    document.getElementById('lv-fix-tag').className   = fixCls;

    // ESKF panel
    U.setText('lv-roll',  U.rad2deg(e.roll));
    U.setText('lv-pitch', U.rad2deg(e.pitch));
    U.setText('lv-yaw',   U.rad2deg(e.yaw));
    const vStr = (e.vE != null) ? `${U.r2(e.vE)} / ${U.r2(e.vN)} / ${U.r2(e.vU)}` : '—';
    U.setText('lv-vel', vStr + ' m/s');

    // Yaw init source — correct behavior: waiting for motion is normal
    const yawSrc   = e.yaw_init_source || s.yaw_init_source || null;
    const eskfInit = s.eskf_init ?? false;
    const alignDone = e.alignment_valid ?? s.alignment_done ?? false;
    let eskfLabel, eskfCls;
    if (!alignDone)        { eskfLabel = 'Aligning (2s)'; eskfCls = 'tag tag-amber'; }
    else if (!eskfInit)    { eskfLabel = 'Waiting for motion'; eskfCls = 'tag tag-amber'; }
    else                   { eskfLabel = 'Initialized'; eskfCls = 'tag tag-green'; }
    document.getElementById('lv-eskf-tag').textContent = eskfLabel;
    document.getElementById('lv-eskf-tag').className   = eskfCls;
    U.setText('lv-yaw-init', yawSrc || (eskfInit ? 'from course' : 'pending'));
    const gated = e.gnss_pos_gated ?? false;
    const gatedEl = document.getElementById('lv-gated');
    if (gatedEl) {
      gatedEl.textContent = gated ? 'Gated (outlier rejected)' : 'Applied';
      gatedEl.className   = 'kv-v ' + (gated ? 'amber' : 'green');
    }

    // Constellations
    const sats = s.sats ?? g.sats ?? 0;
    U.setText('lv-sats-total', sats + ' sats');
    const consts = [
      { id: 'gps',   val: s.sats_gps   ?? g.gps_sv   ?? 0, max: 12, col: 'green' },
      { id: 'glo',   val: s.sats_glo   ?? g.glo_sv   ?? 0, max: 8,  col: '' },
      { id: 'bds',   val: s.sats_bds   ?? g.bds_sv   ?? 0, max: 10, col: '' },
      { id: 'gal',   val: s.sats_gal   ?? g.gal_sv   ?? 0, max: 8,  col: '' },
      { id: 'irnss', val: s.sats_irnss ?? g.irnss_sv ?? 0, max: 7,  col: 'purple' },
    ];
    consts.forEach(c => {
      U.setText('cv-' + c.id, c.val);
      U.barPct('cb-' + c.id, c.val / c.max * 100, c.col);
    });

    // Health bars
    const gnssQ = U.gnssQualityPct(sats, g.hdop ?? s.hdop);
    U.setText('hv-gnss', gnssQ + '%');
    U.barPct('hb-gnss', gnssQ, U.gnssClass(gnssQ));

    const rssi = s.lte_rssi_dbm ?? null;
    const ltePct = U.rssiToPercent(rssi);
    U.setText('hv-lte', rssi != null ? rssi + ' dBm' : '—');
    U.barPct('hb-lte', ltePct, U.lteClass(rssi));

    // Battery — MAX17048 not yet soldered, always show placeholder
    const batPct = s.battery_pct ?? null;
    U.setText('hv-bat', batPct != null ? batPct + '%' : '—');
    U.barPct('hb-bat', batPct ?? 0, U.batClass(batPct ?? 0));
    // else stays "Pending HW"

    // KV fields
    U.setText('kv-rssi',  rssi != null ? rssi + ' dBm' : '—');
    const netUp = s.net_up ?? false;
    const netEl = document.getElementById('kv-net');
    if (netEl) { netEl.textContent = netUp ? 'Up' : 'Down'; netEl.className = 'kv-v ' + (netUp ? 'green' : 'red'); }
    const ntpEl = document.getElementById('kv-ntp');
    const ntpSync = s.ntp_synced ?? false;
    if (ntpEl) { ntpEl.textContent = ntpSync ? 'Synced' : 'Unsynced'; ntpEl.className = 'kv-v ' + (ntpSync ? 'green' : 'amber'); }
    U.setText('kv-fw',   doc.fw_version  || s.fw_version  || '—');
    U.setText('kv-boot', doc.boot_count  ?? s.boot_count  ?? '—');
  },

  async _loadAlerts() {
    if (!this._deviceId) return;
    try {
      const alerts = await AUTH.apiJSON(`/api/v1/alerts/${encodeURIComponent(this._deviceId)}`);
      const body = document.getElementById('alerts-body');
      const tag  = document.getElementById('alert-count');
      if (!alerts.length) {
        body.innerHTML = '<div class="empty" style="padding:20px;"><div class="empty-icon">✅</div><div class="empty-label">No alerts</div></div>';
        tag.textContent = '0'; tag.className = 'tag tag-muted';
        return;
      }
      tag.textContent = alerts.length;
      tag.className   = alerts.some(a => a.level === 'error') ? 'tag tag-red' : 'tag tag-amber';
      body.innerHTML  = alerts.map(a => `
        <div class="alert-row ${a.level}">
          <div class="alert-icon">${a.level === 'error' ? '🔴' : '🟡'}</div>
          <div class="alert-content">
            <div class="alert-code">${a.code}</div>
            <div class="alert-msg">${a.msg}</div>
          </div>
        </div>`).join('');
    } catch(e) {}
  },

  async loadTripReport() { /* /api/trip not yet implemented in backend */ },

  exportCSV() {
    if (!this._deviceId) return;
    let url = BASE + `/api/v1/session/export?device_id=${encodeURIComponent(this._deviceId)}`;
    if (this._sessionId) url += `&session_id=${encodeURIComponent(this._sessionId)}`;
    window.location.href = url;
  },

  exportGeoJSON() {
    if (!this._deviceId) return;
    let url = BASE + `/api/v1/session/geojson?device_id=${encodeURIComponent(this._deviceId)}`;
    if (this._sessionId) url += `&session_id=${encodeURIComponent(this._sessionId)}`;
    window.location.href = url;
  },

  exportHF() {
    if (!this._deviceId) return;
    let url = BASE + `/api/v1/session/export/hf?device_id=${encodeURIComponent(this._deviceId)}`;
    if (this._sessionId) url += `&session_id=${encodeURIComponent(this._sessionId)}`;
    window.location.href = url;
  },

  _alertPollCount: 0,

  _startPoll() {
    clearInterval(this._pollTimer);
    this._alertPollCount = 0;
    if (!this._sessionId) {
      this._pollTimer = setInterval(async () => {
        try {
          const latest = await AUTH.apiJSON(`/api/v1/device/latest/${encodeURIComponent(this._deviceId)}`);
          this._applyTelemetry(latest);
          if (latest.session_id) this._liveSessionId = latest.session_id;

          // Update map: GNSS point
          const g = latest.gnss || {};
          if (g.lat && g.lon && this._map) {
            const last = this._gnssTrack[this._gnssTrack.length - 1];
            if (!last || last.lat !== g.lat || last.lon !== g.lon) {
              this._gnssTrack.push({ lat: g.lat, lon: g.lon, t_server: latest.t_server });
              if (this._gnssLine) this._gnssLine.addLatLng([g.lat, g.lon]);
              else this._gnssLine = L.polyline([[g.lat, g.lon]], { color: '#ff4757', weight: 2, opacity: .7 }).addTo(this._map);
            }
          }
          // Update map: ESKF point (top-level eskf_lat/eskf_lon in latest doc)
          const eLat = latest.eskf_lat ?? null;
          const eLon = latest.eskf_lon ?? null;
          if (eLat && eLon && this._map) {
            const lastE = this._eskfTrack[this._eskfTrack.length - 1];
            if (!lastE || lastE.lat !== eLat || lastE.lon !== eLon) {
              this._eskfTrack.push({ lat: eLat, lon: eLon, t_server: latest.t_server });
              if (this._eskfLine) this._eskfLine.addLatLng([eLat, eLon]);
              else this._eskfLine = L.polyline([[eLat, eLon]], { color: '#00d4ff', weight: 2.5, opacity: .9 }).addTo(this._map);
            }
            // Marker tracks ESKF position (more accurate than GNSS)
            if (this._curMarker) this._curMarker.setLatLng([eLat, eLon]);
            else this._curMarker = L.circleMarker([eLat, eLon], { radius: 7, color: '#00d4ff', fillColor: '#00d4ff', fillOpacity: .9, weight: 2 }).addTo(this._map);
            this._map.panTo([eLat, eLon], { animate: true, duration: 0.3 });
          } else if (g.lat && g.lon && this._map) {
            // Fallback to GNSS if no ESKF
            if (this._curMarker) this._curMarker.setLatLng([g.lat, g.lon]);
            else this._curMarker = L.circleMarker([g.lat, g.lon], { radius: 7, color: '#ff4757', fillColor: '#ff4757', fillOpacity: .9, weight: 2 }).addTo(this._map);
            this._map.panTo([g.lat, g.lon], { animate: true, duration: 0.3 });
          }

          this._alertPollCount++;
          if (this._alertPollCount % 20 === 0) await this._loadAlerts();
        } catch(e) {}
      }, POLL_INTERVAL_MS);
    }
  },

  _connectSocket() {
    if (this._socket) { try { this._socket.disconnect(); } catch(e) {} }
    try {
      this._socket = io('/', { auth: { token: AUTH.token }, transports: ['websocket'] });
      this._socket.on('telemetry', data => {
        if (data.device_id !== this._deviceId) return;
        this._applyTelemetry(data);
        // Append live point to track arrays
        if (data.lat && data.lon) {
          this._gnssTrack.push({ lat: data.lat, lon: data.lon, t_server: data.t_server });
          this._drawTracks();
        }
        if (data.eskf_lat && data.eskf_lon) {
          this._eskfTrack.push({ lat: data.eskf_lat, lon: data.eskf_lon, t_server: data.t_server });
        }
      });
    } catch(e) { console.warn('Socket.io not available, polling only'); }
  },

  /* ── Replay ── */
  onScrub(val) {
    this._replayIdx = parseInt(val, 10);
    this._drawTracks(this._replayIdx, this._replayIdx);
    const gPt = this._gnssTrack.filter(p => p.lat && p.lon)[this._replayIdx];
    if (gPt) U.setText('replay-time-display', U.fmtDT(gPt.t_server));
  },

  replayPlay() {
    if (this._replayPlaying) return;
    this._replayPlaying = true;
    const gFiltered = this._gnssTrack.filter(p => p.lat && p.lon);
    const speed = parseInt(document.getElementById('replay-speed').value, 10) || 1;
    this._replayTimer = setInterval(() => {
      if (this._replayIdx >= gFiltered.length - 1) { this.replayPause(); return; }
      this._replayIdx++;
      const scrub = document.getElementById('replay-scrub');
      if (scrub) scrub.value = this._replayIdx;
      this._drawTracks(this._replayIdx, this._replayIdx);
      const gPt = gFiltered[this._replayIdx];
      if (gPt) U.setText('replay-time-display', U.fmtDT(gPt.t_server));
    }, Math.round(200 / speed));
  },

  replayPause() {
    this._replayPlaying = false;
    clearInterval(this._replayTimer);
  },

  replayReset() {
    this.replayPause();
    this._replayReset_internal();
  },

  _replayReset_internal() {
    this._replayIdx = 0;
    const scrub = document.getElementById('replay-scrub');
    if (scrub) scrub.value = 0;
    this._drawTracks(0, 0);
    U.setText('replay-time-display', '—');
  },
};

/* ═══════════════════════════════════════════════════════════════
   SESSIONS MODULE
═══════════════════════════════════════════════════════════════ */
const SESSIONS = {
  _miniMap: null, _miniMapInit: false, _miniTile: null,
  _gnssLine: null, _eskfLine: null,
  _deviceId: '',
  _sessions: [],
  _selected: null,

  initMiniMap() {
    this._miniMapInit = true;
    this._miniMap = L.map('sess-mini-map', { zoomControl: false, attributionControl: false }).setView([21.19, 72.81], 12);
    this._miniTile = makeTile('osm');
    this._miniTile.addTo(this._miniMap);
  },

  async load(deviceId) {
    this._deviceId = deviceId;
    if (!deviceId) return;
    // Init mini-map now — page is guaranteed visible when load() is called
    if (!this._miniMapInit) {
      this.initMiniMap();
      setTimeout(() => this._miniMap && this._miniMap.invalidateSize(), 100);
    }
    try {
      const sessions = await AUTH.apiJSON(`/api/v1/sessions/${encodeURIComponent(deviceId)}`);
      this._sessions = sessions;
      this._renderTable(sessions);
    } catch(e) { console.error('Sessions load error:', e); }
  },

  _renderTable(sessions) {
    const tbody = document.getElementById('sess-tbody');
    const label = document.getElementById('sess-count-label');
    label.textContent = sessions.length + ' session' + (sessions.length !== 1 ? 's' : '') + ' · ' + this._deviceId;

    if (!sessions.length) {
      tbody.innerHTML = '<tr><td colspan="8" style="text-align:center;padding:32px;color:var(--text-muted);">No sessions found</td></tr>';
      return;
    }

    // Sort newest → oldest
    const sorted = [...sessions].sort((a, b) => {
      const ta = a.started_at_server ? new Date(a.started_at_server) : (a.started_t_epoch_ms || 0);
      const tb = b.started_at_server ? new Date(b.started_at_server) : (b.started_t_epoch_ms || 0);
      return tb - ta;
    });

    tbody.innerHTML = '';
    sorted.forEach((s, i) => {
      const tr = document.createElement('tr');
      tr.className = 'clickable';
      tr.innerHTML = `
        <td class="primary">${i + 1}</td>
        <td>${s.label || '—'}</td>
        <td>${U.fmtDT(s.started_at_server)}</td>
        <td class="primary">${U.fmtDur(s.duration_s)}</td>
        <td>${s.packet_count ?? '—'}</td>
        <td>${s.boot_count ?? '—'}</td>
        <td class="mono" style="font-size:10px;">${s.fw_version || '—'}</td>
        <td><button class="btn btn-ghost btn-sm" onclick="event.stopPropagation();SESSIONS._quickExport('${s.session_id}')">CSV</button></td>`;
      tr.onclick = () => this._select(s, tr);
      tbody.appendChild(tr);
    });
  },

  async _select(s, row) {
    this._selected = s;
    document.querySelectorAll('#sess-tbody tr').forEach(r => r.classList.remove('selected'));
    row.classList.add('selected');

    // Detail panel
    U.setText('sess-detail-id', s.session_id ? s.session_id.slice(-16) + '…' : '—');
    document.getElementById('sd-sid').textContent = s.session_id || '—';
    U.setText('sd-start',   U.fmtDT(s.started_at_server));
    U.setText('sd-end',     U.fmtDT(s.last_seen));
    U.setText('sd-packets', s.packet_count ?? '—');
    U.setText('sd-boot',    s.boot_count   ?? '—');
    U.setText('sd-fw',      s.fw_version   || '—');
    U.setText('sd-dur',     U.fmtDur(s.duration_s));

    // Trip report
    try {
      // trip endpoint not yet in backend — skip silently
    } catch(e) { U.setText('sd-dist', '—'); }

    // Mini map
    if (!this._miniMapInit) this.initMiniMap();
    try {
      const [gnss, eskf] = await Promise.all([
        AUTH.apiJSON(`/api/v1/gnss/track/${encodeURIComponent(this._deviceId)}?session_id=${encodeURIComponent(s.session_id)}`),
        AUTH.apiJSON(`/api/v1/eskf/track/${encodeURIComponent(this._deviceId)}?session_id=${encodeURIComponent(s.session_id)}`),
      ]);
      if (this._gnssLine) this._miniMap.removeLayer(this._gnssLine);
      if (this._eskfLine) this._miniMap.removeLayer(this._eskfLine);

      const gPts = gnss.filter(p => p.lat && p.lon).map(p => [p.lat, p.lon]);
      const ePts = eskf.filter(p => p.lat && p.lon).map(p => [p.lat, p.lon]);
      if (gPts.length) this._gnssLine = L.polyline(gPts, { color: '#ff4757', weight: 2, opacity: .7 }).addTo(this._miniMap);
      if (ePts.length) this._eskfLine = L.polyline(ePts, { color: '#00d4ff', weight: 2.5 }).addTo(this._miniMap);
      const all = [...gPts, ...ePts];
      if (all.length) this._miniMap.fitBounds(L.latLngBounds(all), { padding: [10, 10] });
    } catch(e) {}
  },

  exportCSV() {
    if (!this._selected || !this._deviceId) return;
    window.location.href = BASE + `/api/v1/session/export?device_id=${encodeURIComponent(this._deviceId)}&session_id=${encodeURIComponent(this._selected.session_id)}`;
  },

  exportGeoJSON() {
    if (!this._selected || !this._deviceId) return;
    window.location.href = BASE + `/api/v1/session/geojson?device_id=${encodeURIComponent(this._deviceId)}&session_id=${encodeURIComponent(this._selected.session_id)}`;
  },

  exportHF() {
    if (!this._selected || !this._deviceId) return;
    window.location.href = BASE + `/api/v1/session/export/hf?device_id=${encodeURIComponent(this._deviceId)}&session_id=${encodeURIComponent(this._selected.session_id)}`;
  },

  _quickExport(sid) {
    if (!this._deviceId) return;
    window.location.href = BASE + `/api/v1/session/export?device_id=${encodeURIComponent(this._deviceId)}&session_id=${encodeURIComponent(sid)}`;
  },

  _expandMap: null,

  expandMap() {
    const modal = document.getElementById('sess-map-modal');
    if (!modal) return;
    modal.classList.add('open');
    // Init expand map if not already
    if (!this._expandMap) {
      this._expandMap = L.map('sess-expand-map', { zoomControl: true, attributionControl: false }).setView([21.19, 72.81], 13);
      makeTile('osm').addTo(this._expandMap);
    }
    setTimeout(() => this._expandMap.invalidateSize(), 100);
    // Draw same tracks as mini-map
    if (this._gnssLine) {
      const gPts = this._gnssLine.getLatLngs();
      if (gPts.length) {
        L.polyline(gPts, { color: '#ff4757', weight: 2.5, opacity: .7 }).addTo(this._expandMap);
      }
    }
    if (this._eskfLine) {
      const ePts = this._eskfLine.getLatLngs();
      if (ePts.length) {
        L.polyline(ePts, { color: '#00d4ff', weight: 3, opacity: .9 }).addTo(this._expandMap);
        this._expandMap.fitBounds(L.latLngBounds(ePts), { padding: [20, 20] });
      }
    }
    const title = this._selected ? (this._selected.label || U.fmtDT(this._selected.started_at_server)) : 'Session Track';
    U.setText('sess-modal-title', title);
  },

  closeMap(e) {
    if (e && e.target !== document.getElementById('sess-map-modal')) return;
    document.getElementById('sess-map-modal').classList.remove('open');
  },

  openInLive() {
    if (!this._selected) return;
    // Set device and session in live page, then navigate
    const devSel  = document.getElementById('live-dev-sel');
    if (devSel) { devSel.value = this._deviceId; }
    showPage('live', document.getElementById('ntab-live'));
    setTimeout(async () => {
      await LIVE.onDeviceChange(this._deviceId);
      document.getElementById('live-sess-sel').value = this._selected.session_id;
      LIVE.onSessionChange();
    }, 100);
  },
};

/* ═══════════════════════════════════════════════════════════════
   SERVER HEALTH MODULE
═══════════════════════════════════════════════════════════════ */
const SERVER = {
  _chart: null,
  _hist:  { labels: [], d1: [], d5: [], d15: [] },

  async fetch() {
    try {
      const h = await fetch(BASE + '/api/server/health').then(r => r.json());
      this._render(h);
    } catch(e) {
      U.setClass('nav-srv-dot', 'nav-srv-dot down');
      const badge = document.getElementById('srv-uptime-badge');
      if (badge) { badge.className = 'badge-offline'; badge.innerHTML = '<div class="live-dot"></div>DOWN'; }
      ['svc-mongo','svc-mqtt','svc-ingest','svc-nav','svc-tuning'].forEach(id => U.setClass(id, 'sdot offline'));
    }
  },

  _render(h) {
    const cpu  = h.cpu_percent   || 0;
    const mem  = h.memory?.percent || 0;
    const disk = h.disk_percent  || 0;

    U.setClass('nav-srv-dot', 'nav-srv-dot up');
    const badge = document.getElementById('srv-uptime-badge');
    if (badge) { badge.className = 'badge-live'; badge.innerHTML = '<div class="live-dot pulse"></div>UP'; }

    // Metric tiles
    const setMetric = (valId, barId, pct, unit, extraId, extraVal) => {
      const v = document.getElementById(valId);
      if (v) v.innerHTML = Math.round(pct) + `<span class="u">${unit}</span>`;
      this._setBar(barId, pct);
      if (extraId) U.setText(extraId, extraVal || '');
    };
    setMetric('srv-cpu',  'srv-cpu-bar',  cpu,  '%');
    setMetric('srv-mem',  'srv-mem-bar',  mem,  '%', 'srv-mem-detail',
              h.memory ? `${h.memory.used_gb} / ${h.memory.total_gb} GB` : '');
    setMetric('srv-disk', 'srv-disk-bar', disk, '%');

    const lat = document.getElementById('srv-lat');
    if (lat) lat.innerHTML = (h.latency || '—');

    U.setText('srv-l1',     h.load_avg?.['1m']  ?? '—');
    U.setText('srv-l5',     h.load_avg?.['5m']  ?? '—');
    U.setText('srv-l15',    h.load_avg?.['15m'] ?? '—');
    U.setText('srv-uptime', h.uptime_s != null ? U.fmtDur(h.uptime_s) : (h.uptime_hours != null ? U.r1(h.uptime_hours) + ' h' : '—'));
    const statusEl = document.getElementById('srv-status');
    if (statusEl) { statusEl.textContent = h.status || '—'; statusEl.className = 'kv-v ' + (h.status === 'UP' ? 'green' : 'red'); }

    U.setText('srv-last-upd', 'Last updated: ' + new Date().toLocaleTimeString());

    // Services — mark all green if backend responded
    ['svc-mongo','svc-mqtt','svc-ingest','svc-nav','svc-tuning'].forEach(id => U.setClass(id, 'sdot live'));

    // Load chart history
    const now = new Date().toLocaleTimeString();
    this._hist.labels.push(now);
    this._hist.d1.push(h.load_avg?.['1m']  || 0);
    this._hist.d5.push(h.load_avg?.['5m']  || 0);
    this._hist.d15.push(h.load_avg?.['15m'] || 0);
    if (this._hist.labels.length > 24) {
      ['labels','d1','d5','d15'].forEach(k => this._hist[k].shift());
    }
    this._updateChart();
  },

  _setBar(id, pct) {
    const el = document.getElementById(id);
    if (!el) return;
    el.style.width = Math.min(pct, 100) + '%';
    el.className = 'pbar-fill ' + (pct >= 90 ? 'red' : pct >= 70 ? 'amber' : 'green');
  },

  _updateChart() {
    const ctx = document.getElementById('srv-load-chart');
    if (!ctx) return;
    if (this._chart) {
      this._chart.data.labels = this._hist.labels;
      this._chart.data.datasets[0].data = this._hist.d1;
      this._chart.data.datasets[1].data = this._hist.d5;
      this._chart.data.datasets[2].data = this._hist.d15;
      this._chart.update('none');
      return;
    }
    this._chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: this._hist.labels,
        datasets: [
          { label: '1m',  data: this._hist.d1,  borderColor: '#00d4ff', backgroundColor: 'rgba(0,212,255,.07)', tension: .4, pointRadius: 0, borderWidth: 2 },
          { label: '5m',  data: this._hist.d5,  borderColor: '#00e5a0', backgroundColor: 'transparent',         tension: .4, pointRadius: 0, borderWidth: 1.5 },
          { label: '15m', data: this._hist.d15, borderColor: '#ffb020', backgroundColor: 'transparent',         tension: .4, pointRadius: 0, borderWidth: 1.5 },
        ],
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: { legend: { labels: { color: '#5b6370', font: { size: 10 }, boxWidth: 18, padding: 12 } } },
        scales: {
          x: { ticks: { color: '#5b6370', font: { size: 10 }, maxTicksLimit: 8 }, grid: { color: 'rgba(255,255,255,.04)' } },
          y: { ticks: { color: '#5b6370', font: { size: 10 } },                  grid: { color: 'rgba(255,255,255,.04)' }, beginAtZero: true },
        },
      },
    });
  },
};

/* ═══════════════════════════════════════════════════════════════
   BOOT SEQUENCE
═══════════════════════════════════════════════════════════════ */
async function bootApp() {
  // Only init Fleet map on boot (it's the active page)
  // Live map must NOT init here — div is display:none and Leaflet gets 0x0
  FLEET.initMap();
  // Sessions mini-map init deferred until first visit (it needs the element visible)

  // Initial data load
  await FLEET.refresh();
  SERVER.fetch();

  // Polling timers
  setInterval(() => FLEET.refresh(),   30000);
  setInterval(() => SERVER.fetch(),    SERVER_POLL_MS);
  setInterval(() => {
    if (_activePage === 'fleet' && !FLEET._paused) FLEET._renderTrack();
  }, POLL_INTERVAL_MS);
}

/* ─── Entry point ─── */
window.addEventListener('load', async () => {
  loadTheme();
  // No login required — auto-fetch a token using the service account
  try {
    const res = await fetch('/api/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email: 'devaamdalal555@gmail.com', password: 'TruTrack2026!' }),
    });
    if (res.ok) {
      const d = await res.json();
      AUTH.token = d.access_token;
    }
  } catch(e) {}
  bootApp();
});
