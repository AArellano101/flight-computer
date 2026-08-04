#include "sensors.h"
#include "stm32h7xx_hal.h"

#include <limits.h>

#define SENSOR_MAX_CONSECUTIVE_FAILURES  3U
#define SENSOR_FRESHNESS_TIMEOUT_MS       2000U

static SensorHealth_t sensor_health[SENSOR_ID_COUNT];

static bool sensor_id_is_valid(SensorId_t sensor)
{
    return ((uint32_t)sensor < (uint32_t)SENSOR_ID_COUNT);
}

void sensor_health_set_initialized(SensorId_t sensor, bool initialized)
{
    if (!sensor_id_is_valid(sensor))
    {
        return;
    }

    sensor_health[sensor].initialized = initialized;
    sensor_health[sensor].has_sample = false;
    sensor_health[sensor].last_success_ms = 0U;
    sensor_health[sensor].consecutive_failures = 0U;
}

void sensor_health_record_success(SensorId_t sensor, uint32_t timestamp_ms)
{
    if (!sensor_id_is_valid(sensor))
    {
        return;
    }

    sensor_health[sensor].has_sample = true;
    sensor_health[sensor].last_success_ms = timestamp_ms;
    sensor_health[sensor].consecutive_failures = 0U;
}

void sensor_health_record_failure(SensorId_t sensor)
{
    if (!sensor_id_is_valid(sensor))
    {
        return;
    }

    if (sensor_health[sensor].consecutive_failures < UINT32_MAX)
    {
        sensor_health[sensor].consecutive_failures++;
    }
}

SensorHealth_t sensor_health_get(SensorId_t sensor)
{
    SensorHealth_t empty = {0};

    if (!sensor_id_is_valid(sensor))
    {
        return empty;
    }

    return sensor_health[sensor];
}

bool sensor_is_healthy(SensorId_t sensor)
{
    SensorHealth_t health;

    if (!sensor_id_is_valid(sensor))
    {
        return false;
    }

    health = sensor_health[sensor];

    if (!health.initialized ||
        !health.has_sample ||
        (health.consecutive_failures >= SENSOR_MAX_CONSECUTIVE_FAILURES))
    {
        return false;
    }

    return ((HAL_GetTick() - health.last_success_ms) <=
            SENSOR_FRESHNESS_TIMEOUT_MS);
}

bool imu_is_healthy(void)
{
    return sensor_is_healthy(SENSOR_ID_IMU);
}

bool barometer_is_healthy(void)
{
    return sensor_is_healthy(SENSOR_ID_BAROMETER);
}

bool magnetometer_is_healthy(void)
{
    return sensor_is_healthy(SENSOR_ID_MAGNETOMETER);
}
