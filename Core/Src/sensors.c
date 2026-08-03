#include "sensors.h"

static bool imu_healthy = false;
static bool barometer_healthy = false;
static bool magnetometer_healthy = false;

bool imu_is_healthy(void)
{
    return imu_healthy;
}

bool barometer_is_healthy(void)
{
    return barometer_healthy;
}

bool magnetometer_is_healthy(void)
{
    return magnetometer_healthy;
}
