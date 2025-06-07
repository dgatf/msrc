#include "distance.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#define INIT_DELAY_MS 1000
#define INTERVAL_MS 500

static double degrees_to_radians(double degrees);
static float get_distance_to_home(double lat, double lon, float alt, float lat_init, float lon_init, float alt_init);

void distance_task(void *parameters) {
    distance_parameters_t parameter = *(distance_parameters_t *)parameters;
    *parameter.distance = 0;
    while (*parameter.sat < 4) {
        vTaskDelay(INIT_DELAY_MS / portTICK_PERIOD_MS);
    }
    float latitude_init = *parameter.latitude;
    float longitude_init = *parameter.longitude;
    float altitude_init = *parameter.altitude;
    while (1) {
        if (*parameter.sat >= 6)
            *parameter.distance = get_distance_to_home(*parameter.latitude, *parameter.longitude, *parameter.altitude,
                                                       latitude_init, longitude_init, altitude_init);
#ifdef SIM_SENSORS
        *parameter.distance = 1234.56;
#endif
        debug("\nDistance (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter.distance);
        vTaskDelay(INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static double degrees_to_radians(double degrees) {
    return degrees * 3.14159265359 / 180;  // TODO: fix PI
}

static float get_distance_to_home(double lat, double lon, float alt, float lat_init, float lon_init, float alt_init) {
    uint16_t earth_radius_km = 6371;
    double rad_lat_init = degrees_to_radians(lat_init);
    double rad_lat_delta = degrees_to_radians(lat_init - lat);
    double rad_lon_delta = degrees_to_radians(lon_init - lon);
    double rad_lat = degrees_to_radians(lat);
    double a = sin(rad_lat_delta / 2) * sin(rad_lat_delta / 2) +
               sin(rad_lon_delta / 2) * sin(rad_lon_delta / 2) * cos(rad_lat) * cos(rad_lat_init);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance_on_ground = earth_radius_km * c * 1000;
    return sqrt(distance_on_ground * distance_on_ground + (alt - alt_init) * (alt - alt_init));  // meters
}
