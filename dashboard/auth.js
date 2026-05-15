// TRU-TRACK Auth Module
// Loaded by every dashboard page before any API calls.

const AUTH = {

  // ── Storage keys ────────────────────────────────────────────────────────────
  KEYS: {
    access:    "tt_access_token",
    refresh:   "tt_refresh_token",
    role:      "tt_role",
    name:      "tt_name",
    expires_at:"tt_expires_at",
  },

  // ── Read helpers ─────────────────────────────────────────────────────────────
  getToken() { return localStorage.getItem(this.KEYS.access); },
  getRole()  { return localStorage.getItem(this.KEYS.role) || "viewer"; },
  getName()  { return localStorage.getItem(this.KEYS.name) || ""; },
  isLoggedIn(){ return !!this.getToken(); },

  // ── Store tokens after login or refresh ──────────────────────────────────────
  store(data) {
    localStorage.setItem(this.KEYS.access,  data.access_token);
    if (data.refresh_token)
      localStorage.setItem(this.KEYS.refresh, data.refresh_token);
    if (data.role)
      localStorage.setItem(this.KEYS.role, data.role);
    if (data.name)
      localStorage.setItem(this.KEYS.name, data.name);
    // Expire 60 seconds before actual 8h expiry to avoid race
    const exp = Date.now() + (8 * 60 * 60 * 1000) - (60 * 1000);
    localStorage.setItem(this.KEYS.expires_at, String(exp));
  },

  // ── Logout ───────────────────────────────────────────────────────────────────
  logout() {
    Object.values(this.KEYS).forEach(k => localStorage.removeItem(k));
    window.location.href = "/login";
  },

  // ── Auto-refresh access token before it expires ──────────────────────────────
  async refreshIfNeeded() {
    const exp = parseInt(localStorage.getItem(this.KEYS.expires_at) || "0");
    if (Date.now() < exp) return true;  // still valid

    const rt = localStorage.getItem(this.KEYS.refresh);
    if (!rt) { this.logout(); return false; }

    try {
      const res = await fetch("/api/v1/auth/refresh", {
        method: "POST",
        headers: { "Authorization": "Bearer " + rt },
      });
      if (!res.ok) { this.logout(); return false; }
      const d = await res.json();
      localStorage.setItem(this.KEYS.access, d.access_token);
      const exp2 = Date.now() + (8 * 60 * 60 * 1000) - (60 * 1000);
      localStorage.setItem(this.KEYS.expires_at, String(exp2));
      return true;
    } catch {
      this.logout();
      return false;
    }
  },

  // ── Authenticated fetch — use this for all /api/ calls ───────────────────────
  async apiFetch(url, opts = {}) {
    const ok = await this.refreshIfNeeded();
    if (!ok) return null;

    const token = this.getToken();
    if (!token) { this.logout(); return null; }

    const res = await fetch(url, {
      ...opts,
      headers: {
        "Content-Type": "application/json",
        "Authorization": "Bearer " + token,
        ...(opts.headers || {}),
      },
    });

    if (res.status === 401) { this.logout(); return null; }
    return res;
  },

  // ── Guard: redirect to /login if not authenticated ───────────────────────────
  requireLogin() {
    if (!this.isLoggedIn()) {
      window.location.href = "/login";
      return false;
    }
    return true;
  },

  // ── Guard: check minimum role level ─────────────────────────────────────────
  LEVELS: { viewer: 1, analyst: 2, admin: 3, superadmin: 4 },

  requireRole(minRole) {
    if (!this.requireLogin()) return false;
    const userLevel = this.LEVELS[this.getRole()] || 0;
    const minLevel  = this.LEVELS[minRole]        || 99;
    if (userLevel < minLevel) {
      alert("You do not have permission to perform this action.");
      return false;
    }
    return true;
  },

  // ── Populate UI elements with user info (call after DOM ready) ───────────────
  populateUserUI() {
    const nameEl = document.getElementById("user-name");
    const roleEl = document.getElementById("user-role");
    if (nameEl) nameEl.textContent = this.getName();
    if (roleEl) roleEl.textContent = this.getRole();
  },
};
