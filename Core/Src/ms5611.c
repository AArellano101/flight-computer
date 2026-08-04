/*
 * ms5611.c
 * Driver for MS5611 pressure and temperature sensor (I2C, STM32 HAL)
 */

#include "ms5611.h"

#include <stdbool.h>
#include <stddef.h>

/* The PROM and ADC transfers are only a few bytes; never wait indefinitely. */
#define MS5611_I2C_TIMEOUT_MS       20U
/* Datasheet maximum at OSR 4096 is 9.04 ms.  This also covers tick quantizing. */
#define MS5611_CONVERSION_DELAY_MS  11U
#define MS5611_PROM_WORD_COUNT       8U
#define MS5611_ADC_INVALID_HIGH      0x00FFFFFFUL

typedef enum {
    MS5611_STATE_UNINITIALIZED = 0,
    MS5611_STATE_START_D1,
    MS5611_STATE_WAIT_D1,
    MS5611_STATE_WAIT_D2
} MS5611_State_t;

static uint16_t prom[MS5611_PROM_WORD_COUNT];
static uint32_t raw_pressure;
static uint32_t conversion_started_ms;
static uint32_t sample_sequence;
static I2C_HandleTypeDef *sensor_i2c;
static MS5611_State_t state = MS5611_STATE_UNINITIALIZED;

static HAL_StatusTypeDef MS5611_SendCommand(I2C_HandleTypeDef *hi2c,
                                             uint8_t command)
{
    return HAL_I2C_Master_Transmit(hi2c, MS5611_I2C_ADDR_HAL, &command, 1U,
                                   MS5611_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MS5611_ReadADC(I2C_HandleTypeDef *hi2c,
                                        uint32_t *adc)
{
    uint8_t bytes[3];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hi2c, MS5611_I2C_ADDR_HAL,
                              MS5611_CMD_ADC_READ, I2C_MEMADD_SIZE_8BIT,
                              bytes, sizeof(bytes), MS5611_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        *adc = ((uint32_t)bytes[0] << 16U)
             | ((uint32_t)bytes[1] << 8U)
             | (uint32_t)bytes[2];
    }

    return status;
}

static HAL_StatusTypeDef MS5611_ReadPROM(I2C_HandleTypeDef *hi2c,
                                         uint8_t index,
                                         uint16_t *value)
{
    uint8_t bytes[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        hi2c, MS5611_I2C_ADDR_HAL,
        (uint16_t)(MS5611_CMD_READ_PROM + ((uint16_t)index * 2U)),
        I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes), MS5611_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        *value = ((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1];
    }

    return status;
}

/* CRC routine from the MS5611 datasheet, without modifying the PROM image. */
static uint8_t MS5611_CalculateCRC4(const uint16_t words[8])
{
    uint16_t remainder = 0U;

    for (uint8_t byte_index = 0U; byte_index < 16U; ++byte_index) {
        uint16_t word = words[byte_index >> 1U];

        /* The stored CRC occupies the low nibble of PROM word 7.  The
         * datasheet algorithm clears the complete low byte for calculation. */
        if ((byte_index >> 1U) == 7U) {
            word &= 0xFF00U;
        }

        if ((byte_index & 1U) == 0U) {
            remainder ^= (uint16_t)(word >> 8U);
        } else {
            remainder ^= (uint16_t)(word & 0x00FFU);
        }

        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = (uint16_t)((remainder << 1U) ^ 0x3000U);
            } else {
                remainder = (uint16_t)(remainder << 1U);
            }
        }
    }

    return (uint8_t)((remainder >> 12U) & 0x0FU);
}

static bool MS5611_PROMIsValid(const uint16_t words[8])
{
    /* C1 through C6 are all required by compensation.  Zero and erased PROM
     * values cannot represent a usable factory calibration. */
    for (uint8_t index = 1U; index <= 6U; ++index) {
        if ((words[index] == 0U) || (words[index] == 0xFFFFU)) {
            return false;
        }
    }

    return MS5611_CalculateCRC4(words) == (uint8_t)(words[7] & 0x0FU);
}

static bool MS5611_ADCIsValid(uint32_t adc)
{
    return (adc != 0U) && (adc != MS5611_ADC_INVALID_HIGH);
}

/* Apply the integer first- and second-order compensation from the datasheet.
 * Pressure is produced in 0.01 mbar, which is numerically identical to Pa. */
static bool MS5611_Compensate(uint32_t d1, uint32_t d2,
                              float *temperature_c, float *pressure_pa)
{
    int64_t delta_temperature;
    int64_t temperature;
    int64_t offset;
    int64_t sensitivity;
    int64_t temperature_2 = 0;
    int64_t offset_2 = 0;
    int64_t sensitivity_2 = 0;
    int64_t pressure;

    delta_temperature = (int64_t)d2 - ((int64_t)prom[5] << 8U);
    temperature = 2000LL
                + ((delta_temperature * (int64_t)prom[6]) / (1LL << 23U));
    offset = ((int64_t)prom[2] << 16U)
           + (((int64_t)prom[4] * delta_temperature) / (1LL << 7U));
    sensitivity = ((int64_t)prom[1] << 15U)
                + (((int64_t)prom[3] * delta_temperature) / (1LL << 8U));

    if (temperature < 2000LL) {
        int64_t difference = temperature - 2000LL;

        temperature_2 = (delta_temperature * delta_temperature) / (1LL << 31U);
        offset_2 = (5LL * difference * difference) / 2LL;
        sensitivity_2 = (5LL * difference * difference) / 4LL;

        if (temperature < -1500LL) {
            difference = temperature + 1500LL;
            offset_2 += 7LL * difference * difference;
            sensitivity_2 += (11LL * difference * difference) / 2LL;
        }
    }

    temperature -= temperature_2;
    offset -= offset_2;
    sensitivity -= sensitivity_2;
    pressure = ((((int64_t)d1 * sensitivity) / (1LL << 21U)) - offset)
             / (1LL << 15U);

    /* These deliberately broad limits catch impossible arithmetic/results
     * while retaining useful readings slightly outside the rated envelope. */
    if ((temperature < -10000LL) || (temperature > 15000LL)
        || (pressure <= 0LL) || (pressure > 200000LL)) {
        return false;
    }

    *temperature_c = (float)temperature / 100.0f;
    *pressure_pa = (float)pressure;
    return true;
}

static bool MS5611_ConversionReady(void)
{
    return (uint32_t)(HAL_GetTick() - conversion_started_ms)
           >= MS5611_CONVERSION_DELAY_MS;
}

static void MS5611_DiscardPair(void)
{
    raw_pressure = 0U;
    state = MS5611_STATE_START_D1;
}

HAL_StatusTypeDef MS5611_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    state = MS5611_STATE_UNINITIALIZED;
    sensor_i2c = NULL;
    raw_pressure = 0U;
    sample_sequence = 0U;

    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    status = MS5611_SendCommand(hi2c, MS5611_CMD_RESET);
    if (status != HAL_OK) {
        return status;
    }

    /* The datasheet specifies 2.8 ms for PROM reload after reset. */
    HAL_Delay(3U);

    for (uint8_t index = 0U; index < MS5611_PROM_WORD_COUNT; ++index) {
        status = MS5611_ReadPROM(hi2c, index, &prom[index]);
        if (status != HAL_OK) {
            return status;
        }
    }

    if (!MS5611_PROMIsValid(prom)) {
        return HAL_ERROR;
    }

    sensor_i2c = hi2c;
    state = MS5611_STATE_START_D1;
    return HAL_OK;
}

MS5611_PollStatus_t MS5611_Poll(I2C_HandleTypeDef *hi2c,
                                MS5611_Sample_t *sample)
{
    HAL_StatusTypeDef status;
    uint32_t adc;

    if ((hi2c == NULL) || (sample == NULL) || (hi2c != sensor_i2c)
        || (state == MS5611_STATE_UNINITIALIZED)) {
        return MS5611_POLL_ERROR;
    }

    switch (state) {
    case MS5611_STATE_START_D1:
        status = MS5611_SendCommand(hi2c,
                                    MS5611_CMD_CONV_D1 | MS5611_OSR_4096);
        if (status != HAL_OK) {
            return MS5611_POLL_ERROR;
        }
        conversion_started_ms = HAL_GetTick();
        state = MS5611_STATE_WAIT_D1;
        return MS5611_POLL_NO_DATA;

    case MS5611_STATE_WAIT_D1:
        if (!MS5611_ConversionReady()) {
            return MS5611_POLL_NO_DATA;
        }

        status = MS5611_ReadADC(hi2c, &adc);
        if ((status != HAL_OK) || !MS5611_ADCIsValid(adc)) {
            MS5611_DiscardPair();
            return MS5611_POLL_ERROR;
        }
        raw_pressure = adc;

        status = MS5611_SendCommand(hi2c,
                                    MS5611_CMD_CONV_D2 | MS5611_OSR_4096);
        if (status != HAL_OK) {
            MS5611_DiscardPair();
            return MS5611_POLL_ERROR;
        }
        conversion_started_ms = HAL_GetTick();
        state = MS5611_STATE_WAIT_D2;
        return MS5611_POLL_NO_DATA;

    case MS5611_STATE_WAIT_D2:
        if (!MS5611_ConversionReady()) {
            return MS5611_POLL_NO_DATA;
        }

        status = MS5611_ReadADC(hi2c, &adc);
        if ((status != HAL_OK) || !MS5611_ADCIsValid(adc)) {
            MS5611_DiscardPair();
            return MS5611_POLL_ERROR;
        }

        {
            MS5611_Sample_t completed_sample;

            if (!MS5611_Compensate(raw_pressure, adc,
                                    &completed_sample.temperature_c,
                                    &completed_sample.pressure_pa)) {
                MS5611_DiscardPair();
                return MS5611_POLL_ERROR;
            }

            completed_sample.timestamp_ms = HAL_GetTick();
            completed_sample.sequence = sample_sequence + 1U;
            sample_sequence = completed_sample.sequence;

            MS5611_DiscardPair();
            *sample = completed_sample;
            return MS5611_POLL_NEW_DATA;
        }

    case MS5611_STATE_UNINITIALIZED:
    default:
        return MS5611_POLL_ERROR;
    }
}
