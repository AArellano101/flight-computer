#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t timestamp_ms;

    float acceleration_x;
    float acceleration_y;
    float acceleration_z;

    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;

    float pressure_pa;
    float temperature_c;
    float altitude_m;
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

void flash_logger_init(void);

HAL_StatusTypeDef logger_start(void);
HAL_StatusTypeDef logger_stop(void);
bool logger_is_active(void);


#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
