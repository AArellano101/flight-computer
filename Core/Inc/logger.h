#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define LOGGER_FORMAT_VERSION              (2U)

#define FLIGHT_LOG_VALID_IMU               (1UL << 0)
#define FLIGHT_LOG_VALID_BAROMETER         (1UL << 1)
#define FLIGHT_LOG_VALID_ALTITUDE_AGL      (1UL << 2)
#define FLIGHT_LOG_GYRO_BIAS_CORRECTED     (1UL << 3)

typedef struct
{
    uint32_t timestamp_ms;             /* paired IMU sample time */
    uint32_t barometer_timestamp_ms;   /* source time for baro/AGL fields */
    uint32_t validity_flags;

    float acceleration_x;  /* mg, vehicle body frame */
    float acceleration_y;  /* mg, vehicle body frame */
    float acceleration_z;  /* mg, vehicle body frame */

    float angular_rate_x;  /* calibrated dps */
    float angular_rate_y;  /* calibrated dps */
    float angular_rate_z;  /* calibrated dps */

    float pressure_pa;     /* absolute compensated pressure */
    float temperature_c;
    float altitude_m;      /* AGL; NAN until ground calibration is valid */
} FlightLogRecord_t;

HAL_StatusTypeDef logger_init(void);

HAL_StatusTypeDef logger_append(
    const FlightLogRecord_t *record
);

uint32_t logger_get_write_address(void);

uint32_t logger_get_record_count(void);

HAL_StatusTypeDef logger_read_record(
    uint32_t record_index,
    FlightLogRecord_t *record);

bool logger_is_full(void);

HAL_StatusTypeDef logger_erase(void);

uint32_t logger_get_capacity(void);

uint16_t logger_get_format_version(void);

void flash_logger_init(void);

HAL_StatusTypeDef logger_start(void);
HAL_StatusTypeDef logger_stop(void);
bool logger_is_active(void);


#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
