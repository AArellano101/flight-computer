#ifndef TEST_STM32H7XX_HAL_H
#define TEST_STM32H7XX_HAL_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct
{
    void *Instance;
} I2C_HandleTypeDef;

#endif /* TEST_STM32H7XX_HAL_H */
