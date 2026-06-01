#ifndef GEO_UTILS_H
#define GEO_UTILS_H

#include <stdint.h>

// ======================================================
// Geographic utility functions
// LLA <-> ENU conversions (WGS-84)
// No filter logic here
// ======================================================

namespace GeoUtils {

    // ---------- Angle helpers ----------
    float deg2rad(float deg);
    float rad2deg(float rad);

    // ---------- Reference origin ----------
    // Must be called once to lock ENU frame
    void setReferenceLLA(
        float lat0_deg,
        float lon0_deg,
        float alt0_m
    );

    // ---------- LLA -> ENU ----------
    // Input:
    //   lat, lon : degrees
    //   alt      : meters
    // Output:
    //   E, N, U  : meters
    void llaToEnu(
        float lat_deg,
        float lon_deg,
        float alt_m,
        float& E,
        float& N,
        float& U
    );

    // ---------- ENU -> LLA ----------
    // Input:
    //   E, N, U  : meters
    // Output:
    //   lat, lon : degrees
    //   alt      : meters
    void enuToLla(
        float E,
        float N,
        float U,
        float& lat_deg,
        float& lon_deg,
        float& alt_m
    );

}

#endif // GEO_UTILS_H
