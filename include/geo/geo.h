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
        mp_ref_.lat_rad = 
    }

private:
    map_projection_reference_s mp_ref_;
};
}