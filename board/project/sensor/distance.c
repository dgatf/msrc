#include "distance.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#define INIT_DELAY_MS 1000
#define INTERVAL_MS 500
#define MIN_SATS 5
#define MAX_HDOP 2.5f

static float degrees_to_radians(float degrees);
static float get_distance_to_home_2d(float lat, float lon, float lat_init, float lon_init);

static inline bool coord_valid(float lat, float lon) {
    /* Reject NaN/Inf and the common invalid (0,0) case */
    if (!isfinite(lat) || !isfinite(lon)) return false;
    if (lat == 0.0f && lon == 0.0f) return false;
    return true;
}

void distance_task(void *parameters) {
    distance_parameters_t parameter = *(distance_parameters_t *)parameters;
    *parameter.distance = 0;
    float latitude_init = 0, longitude_init = 0;

    while (1) {
        if (*parameter.fix && *parameter.sat >= MIN_SATS && *parameter.hdop <= MAX_HDOP && *parameter.hdop > 0.0f &&
            coord_valid(*parameter.latitude, *parameter.longitude)) {
            latitude_init = *parameter.latitude;
            longitude_init = *parameter.longitude;
            debug("\nDistance (%u). Set home: Lat: %.6f, Lon: %.6f, Sats: %.0f, HDOP: %.2f",
                  uxTaskGetStackHighWaterMark(NULL), latitude_init, longitude_init, *parameter.sat, *parameter.hdop);
            break;
        } else {
            debug("\nDistance (%u): Invalid GPS data (Fix: %u, Sats: %.0f, HDOP: %.2f, Lat: %.6f, Lon: %.6f)",
                  uxTaskGetStackHighWaterMark(NULL), (unsigned int)*parameter.fix, *parameter.sat, *parameter.hdop,
                  *parameter.latitude, *parameter.longitude);
        }
        vTaskDelay(INTERVAL_MS / portTICK_PERIOD_MS);
    }
    while (1) {
        if (*parameter.fix && *parameter.sat >= MIN_SATS && *parameter.hdop <= MAX_HDOP && *parameter.hdop > 0.0f &&
            coord_valid(*parameter.latitude, *parameter.longitude)) {
            *parameter.distance =
                get_distance_to_home_2d(*parameter.latitude, *parameter.longitude, latitude_init, longitude_init);
#ifdef SIM_SENSORS
            *parameter.distance = 1234.56f;
#endif
            debug(
                "\nDistance (%u). Distance: %.2f, Lat: %.6f, Lon: %.6f, Sats: %.0f, HDOP: %.2f, Home Lat: %.6f, Home "
                "Lon: %.6f",
                uxTaskGetStackHighWaterMark(NULL), *parameter.distance, *parameter.latitude, *parameter.longitude,
                *parameter.sat, *parameter.hdop, latitude_init, longitude_init);
        } else {
            debug("\nDistance (%u): GPS invalid during distance (Fix:%u Sats:%.0f HDOP:%.2f Lat:%.6f Lon:%.6f)",
                  uxTaskGetStackHighWaterMark(NULL), (unsigned)*parameter.fix, *parameter.sat, *parameter.hdop,
                  *parameter.latitude, *parameter.longitude);
        }
        vTaskDelay(INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static float degrees_to_radians(float degrees) { return degrees * (float)PI / 180.0f; }

static float get_distance_to_home_2d(float lat, float lon, float lat_init, float lon_init) {
    const float earth_radius_m = 6371000.0f;

    float lat1 = degrees_to_radians(lat_init);
    float lat2 = degrees_to_radians(lat);
    float dlat = degrees_to_radians(lat - lat_init);
    float dlon = degrees_to_radians(lon - lon_init);

    float s1 = sinf(dlat * 0.5f);
    float s2 = sinf(dlon * 0.5f);

    float a = s1 * s1 + cosf(lat1) * cosf(lat2) * s2 * s2;
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return earth_radius_m * c;
}