#include "GeoUtils.h"
#include "EskfConfig.h"
#include <math.h>
namespace GeoUtils {

    // ---------- Internal reference ----------
    static float lat0_rad = 0.0f;
    static float lon0_rad = 0.0f;
    static float alt0_m   = 0.0f;
    static bool  ref_set  = false;

    // ---------- Angle helpers ----------
    float deg2rad(float deg) {
        return deg * (float)ESKF_PI / 180.0f;
    }

    float rad2deg(float rad) {
        return rad * 180.0f / (float)ESKF_PI;
    }

    // ---------- Set ENU reference ----------
    void setReferenceLLA(
        float lat0_deg,
        float lon0_deg,
        float alt0_m_in
    ) {
        lat0_rad = deg2rad(lat0_deg);
        lon0_rad = deg2rad(lon0_deg);
        alt0_m   = alt0_m_in;
        ref_set  = true;
    }

    // ---------- LLA -> ENU ----------
    void llaToEnu(
        float lat_deg,
        float lon_deg,
        float alt_m,
        float& E,
        float& N,
        float& U
    ) {
        if (!ref_set) {
            E = N = U = 0.0f;
            return;
        }

        float lat_rad = deg2rad(lat_deg);
        float lon_rad = deg2rad(lon_deg);

        float d_lat = lat_rad - lat0_rad;
        float d_lon = lon_rad - lon0_rad;

        float cos_lat0 = cosf(lat0_rad);

        // Local tangent plane approximation
        N = d_lat * ESKF_EARTH_RADIUS;
        E = d_lon * ESKF_EARTH_RADIUS * cos_lat0;
        U = alt_m - alt0_m;
    }

    // ---------- ENU -> LLA ----------
    void enuToLla(
        float E,
        float N,
        float U,
        float& lat_deg,
        float& lon_deg,
        float& alt_m
    ) {
        if (!ref_set) {
            lat_deg = lon_deg = alt_m = 0.0f;
            return;
        }

        float cos_lat0 = cosf(lat0_rad);

        float d_lat = N / ESKF_EARTH_RADIUS;
        float d_lon = E / (ESKF_EARTH_RADIUS * cos_lat0);

        float lat_rad = lat0_rad + d_lat;
        float lon_rad = lon0_rad + d_lon;

        lat_deg = rad2deg(lat_rad);
        lon_deg = rad2deg(lon_rad);
        alt_m   = alt0_m + U;
    }

} // namespace GeoUtils
