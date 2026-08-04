/*
 * ms5611.h
 * Driver for MS5611 pressure and temperature sensor (I2C, STM32 HAL)
 */

#ifndef PERIPHERALS_MS5611_H
#define PERIPHERALS_MS5611_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/** @brief 7-bit I2C address (0x76 when CSB is high; use 0x77 when low). */
#define MS5611_I2C_ADDR       0x76U
/** @brief Address format expected by the STM32 HAL I2C functions. */
#define MS5611_I2C_ADDR_HAL   (MS5611_I2C_ADDR << 1U)

/* MS5611 commands. */
#define MS5611_CMD_RESET        0x1EU
#define MS5611_CMD_CONV_D1      0x40U
#define MS5611_CMD_CONV_D2      0x50U
#define MS5611_CMD_ADC_READ     0x00U
#define MS5611_CMD_READ_PROM    0xA0U

/* Oversampling-ratio command suffixes. */
#define MS5611_OSR_256          0x00U
#define MS5611_OSR_512          0x02U
#define MS5611_OSR_1024         0x04U
#define MS5611_OSR_2048         0x06U
#define MS5611_OSR_4096         0x08U

/** A complete, compensated pressure/temperature sample. */
typedef struct {
    uint32_t timestamp_ms;
    uint32_t sequence;
    float temperature_c;
    float pressure_pa;
} MS5611_Sample_t;

/** Result of advancing the non-blocking conversion state machine. */
typedef enum {
    MS5611_POLL_NO_DATA = 0,
    MS5611_POLL_NEW_DATA,
    MS5611_POLL_ERROR
} MS5611_PollStatus_t;

/**
 * @brief Reset the sensor, load all PROM words, and validate PROM CRC4.
 *
 * The first D1 conversion is started by MS5611_Poll().
 *
 * @param hi2c Pointer to the I2C peripheral used by this sensor.
 * @return HAL_OK when the sensor is ready, the HAL error from a failed bus
 *         transaction, or HAL_ERROR for invalid coefficients/CRC.
 */
HAL_StatusTypeDef MS5611_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Advance the D1/D2 conversion state machine without delaying.
 *
 * Call this regularly. The output object is modified only when
 * MS5611_POLL_NEW_DATA is returned. A bus/ADC/compensation error discards the
 * incomplete pair, leaves @p sample unchanged, and restarts at a fresh D1.
 *
 * @param hi2c I2C handle previously passed to MS5611_Init().
 * @param sample Destination for a newly completed sample.
 */
MS5611_PollStatus_t MS5611_Poll(I2C_HandleTypeDef *hi2c,
                                MS5611_Sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_MS5611_H */
