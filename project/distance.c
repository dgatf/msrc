#include "distance.h"

static float degrees_to_radians(float degrees);
static float get_distance_to_home(float lat, float lon, float alt, float lat_init, float lon_init, float alt_init);

void distance_task(void *parameters)
{
    distance_parameters_t *parameter = (distance_parameters_t *)parameters;
    *parameter->distance = 0;
    vTaskDelay(DISTANCE_INIT_DELAY_MS / portTICK_PERIOD_MS);
    float latitude_init = *parameter->latitude;
    float longitude_init = *parameter->longitude;
    float altitude_init = *parameter->altitude;
    while (1)
    {
        if (*parameter->sat)
            *parameter->distance = get_distance_to_home(*parameter->latitude, *parameter->longitude, *parameter->altitude, latitude_init, longitude_init, altitude_init);
#ifdef SIM_SENSORS
        *parameter->distance = 1234.56;
#endif
        if (debug)
            printf("\nDistance (%u): %.2f", uxTaskGetStackHighWaterMark(NULL), *parameter->distance);
        vTaskDelay(DISTANCE_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static float degrees_to_radians(float degrees)
{
    return degrees * 3.1416 / 180; // TODO: fix PI
}

static float get_distance_to_home(float lat, float lon, float alt, float lat_init, float lon_init, float alt_init)
{
    uint16_t earth_radius_km = 6371;
    float rad_lat_init = degrees_to_radians(lat_init);
    float rad_lat_delta = degrees_to_radians(lat_init - lat);
    float rad_lon_delta = degrees_to_radians(lon_init - lon);
    float rad_lat = degrees_to_radians(lat);
    float a = sin(rad_lat_delta / 2) * sin(rad_lat_delta / 2) + sin(rad_lon_delta / 2) * sin(rad_lon_delta / 2) * cos(rad_lat) * cos(rad_lat_init);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance_on_ground = earth_radius_km * c * 1000;
    return sqrt(distance_on_ground * distance_on_ground + (alt - alt_init) * (alt - alt_init)); // meters
}
