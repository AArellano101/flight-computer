/*
 * sensors.h
 *
 *  Created on: Jul 14, 2026
 *      Author: aaron
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SENSOR_ID_IMU = 0,
    SENSOR_ID_BAROMETER,
    SENSOR_ID_MAGNETOMETER,
    SENSOR_ID_COUNT
} SensorId_t;

typedef struct
{
    bool initialized;
    bool has_sample;
    uint32_t last_success_ms;
    uint32_t consecutive_failures;
} SensorHealth_t;

void sensor_health_set_initialized(SensorId_t sensor, bool initialized);
void sensor_health_record_success(SensorId_t sensor, uint32_t timestamp_ms);
void sensor_health_record_failure(SensorId_t sensor);
SensorHealth_t sensor_health_get(SensorId_t sensor);
bool sensor_is_healthy(SensorId_t sensor);

bool imu_is_healthy(void);
bool barometer_is_healthy(void);
bool magnetometer_is_healthy(void);

#endif /* INC_SENSORS_H_ */
