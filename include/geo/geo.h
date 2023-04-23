#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

static constexpr float CONSTANTS_ONE_G = 9.80665f;						// m/s^2

static constexpr float CONSTANTS_STD_PRESSURE_PA = 101325.0f;					// pascals (Pa)
static constexpr float CONSTANTS_STD_PRESSURE_KPA = CONSTANTS_STD_PRESSURE_PA / 1000.0f;	// kilopascals (kPa)
static constexpr float CONSTANTS_STD_PRESSURE_MBAR = CONSTANTS_STD_PRESSURE_PA / 100.0f;	// Millibar (mbar) (1 mbar = 100 Pa)

static constexpr float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225f;				// kg/m^3
static constexpr float CONSTANTS_AIR_GAS_CONST = 287.1f;					// J/(kg * K)
static constexpr float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15f;				// °C

static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
static constexpr float  CONSTANTS_RADIUS_OF_EARTH_F = CONSTANTS_RADIUS_OF_EARTH;		// meters (m)

static constexpr float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;				// radians/second (rad/s)

namespace geo {
class GlobalLocalTransformer {
public:
    /* lat/lon are in radians */
    struct map_projection_reference_s {
        uint64_t timestamp;
        double lat_rad;
        double lon_rad;
        double sin_lat;
        double cos_lat;
        bool init_done = false;
    };

    bool initialized() {
        return mp_ref_.init_done;
    }

    uint64_t timestamp() {
        return mp_ref_.timestamp;
    }

    // lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
    void init(double lat_0, double lon_0, uint64_t timestamp) {
        mp_ref_.lat_rad = radians(lat_0);
        mp_ref_.lon_rad = radians(lon_0);
        mp_ref_.sin_lat = std::sin(mp_ref_.lat_rad);
        mp_ref_.cos_lat = std::cos(mp_ref_.lat_rad);
        mp_ref_.timestamp = timestamp;
        mp_ref_.init_done = true;
    }

    double radians(double degrees) {
        return (degrees / 180.0) * M_PI;
    }

    double degrees(double radians) {
        return (radians * 180.0) / M_PI;
    }

    double constrain(double val, double min, double max) {
        return (val < min) ? min : ((val > max) ? max : val);
    }

    int map_projection_project(double lat, double lon, double *x, double *y) {
        if (!initialized())
            return -1;
        
        const double lat_rad = radians(lat);
        const double lon_rad = radians(lon);

        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);

        const double cos_d_lon = std::cos(lon_rad - mp_ref_.lon_rad);

        const double arg = constrain(mp_ref_.sin_lat * sin_lat + mp_ref_.cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
        const double c = std::acos(arg);

        double k = 1.0;

        if (std::abs(c) > 0.0) {
            k = (c / std::sin(c));
        }

        *x = static_cast<double>(k * (mp_ref_.cos_lat * sin_lat - mp_ref_.sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
        *y = static_cast<double>(k * cos_lat * std::sin(lon_rad - mp_ref_.lon_rad) * CONSTANTS_RADIUS_OF_EARTH);

        return 0;
    }

    int map_projection_reproject(double x, double y, double *lat, double *lon) {
        if (!initialized())
            return -1;

        const double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
        const double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
        const double c = std::sqrt(x_rad * x_rad + y_rad * y_rad);

        if (std::abs(c) > 0.0) {
            const double sin_c = std::sin(c);
            const double cos_c = std::cos(c);

            const double lat_rad = std::asin(cos_c * mp_ref_.sin_lat + (x_rad * sin_c * mp_ref_.cos_lat) / c);
            const double lon_rad = (mp_ref_.lon_rad + std::atan2(y_rad * sin_c, c * mp_ref_.cos_lat * cos_c - x_rad * mp_ref_.sin_lat * sin_c));

            *lat = degrees(lat_rad);
            *lon = degrees(lon_rad);

        } else {
            *lat = degrees(mp_ref_.lat_rad);
            *lon = degrees(mp_ref_.lon_rad);
        }

        return 0;
    }

private:
    map_projection_reference_s mp_ref_;
};
}