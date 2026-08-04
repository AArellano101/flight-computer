#include "ground_calibration.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void feed_stationary_window(uint32_t first_timestamp_ms,
                                   uint32_t first_imu_sequence,
                                   uint32_t first_baro_sequence)
{
    uint32_t baro_sequence = first_baro_sequence;

    for (uint32_t index = 0U; index < 300U; index++)
    {
        float sign = ((index & 1U) == 0U) ? 1.0f : -1.0f;
        ImuSample_t imu = {
            .timestamp_ms = first_timestamp_ms + (index * 20U),
            .sequence = first_imu_sequence + index,
            .acceleration_mg = {sign * 2.0f, -sign, 1000.0f + sign * 3.0f},
            .angular_rate_dps = {
                0.70f + sign * 0.02f,
                -0.40f - sign * 0.01f,
                0.20f + sign * 0.01f
            },
            .fresh_mask = IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH
        };

        ground_calibration_process_imu(&imu);

        if ((index & 1U) == 0U)
        {
            float baro_sign = ((baro_sequence & 1U) == 0U) ? 1.0f : -1.0f;
            MS5611_Sample_t baro = {
                .timestamp_ms = imu.timestamp_ms,
                .sequence = baro_sequence++,
                .temperature_c = 21.5f + baro_sign * 0.01f,
                .pressure_pa = 90000.0f + baro_sign * 2.0f
            };

            ground_calibration_process_baro(&baro);
        }
    }
}

int main(void)
{
    const GroundCalibrationResult_t *result;
    float corrected_gyro[3];
    float altitude_m;

    ground_calibration_init(0U);
    assert(ground_calibration_get_state() == GROUND_CALIBRATION_WARMUP);
    ground_calibration_update(1000U);
    assert(ground_calibration_get_state() == GROUND_CALIBRATION_CALIBRATING);

    feed_stationary_window(1000U, 1U, 1U);

    assert(ground_calibration_is_ready());
    result = ground_calibration_get_result();
    assert(result != NULL);
    assert(result->valid);
    assert(result->imu_sample_count >= GROUND_CALIBRATION_MIN_IMU_SAMPLES);
    assert(result->baro_sample_count >= GROUND_CALIBRATION_MIN_BARO_SAMPLES);
    assert(fabsf(result->gyro_bias_dps[0] - 0.70f) < 0.01f);
    assert(fabsf(result->gyro_bias_dps[1] + 0.40f) < 0.01f);
    assert(fabsf(result->gyro_bias_dps[2] - 0.20f) < 0.01f);
    assert(fabsf(result->ground_pressure_pa - 90000.0f) < 1.0f);

    ground_calibration_correct_gyro(
        (float[3]){0.70f, -0.40f, 0.20f},
        corrected_gyro);
    assert(fabsf(corrected_gyro[0]) < 0.01f);
    assert(fabsf(corrected_gyro[1]) < 0.01f);
    assert(fabsf(corrected_gyro[2]) < 0.01f);

    assert(ground_calibration_altitude_agl_m(90000.0f, &altitude_m));
    assert(fabsf(altitude_m) < 0.1f);
    assert(ground_calibration_altitude_agl_m(89000.0f, &altitude_m));
    assert(altitude_m > 0.0f);

    ground_calibration_restart(10000U);
    ground_calibration_update(11000U);
    {
        ImuSample_t motion = {
            .timestamp_ms = 11020U,
            .sequence = 500U,
            .acceleration_mg = {0.0f, 0.0f, 1500.0f},
            .angular_rate_dps = {0.0f, 0.0f, 0.0f},
            .fresh_mask = IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH
        };

        ground_calibration_process_imu(&motion);
    }
    assert(ground_calibration_get_failure() ==
           GROUND_CALIBRATION_FAILURE_MOTION);
    assert(ground_calibration_get_progress() == 0.0f);

    /* A stopped barometer must not be accepted using stale accumulated data. */
    ground_calibration_restart(12000U);
    ground_calibration_update(13000U);
    for (uint32_t index = 0U; index < 300U; index++)
    {
        ImuSample_t imu = {
            .timestamp_ms = 13000U + (index * 20U),
            .sequence = 1000U + index,
            .acceleration_mg = {0.0f, 0.0f, 1000.0f},
            .angular_rate_dps = {0.1f, -0.1f, 0.05f},
            .fresh_mask = IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH
        };

        if (index < GROUND_CALIBRATION_MIN_BARO_SAMPLES)
        {
            MS5611_Sample_t baro = {
                .timestamp_ms = imu.timestamp_ms,
                .sequence = 1000U + index,
                .temperature_c = 20.0f,
                .pressure_pa = 90000.0f
            };

            ground_calibration_process_baro(&baro);
        }

        ground_calibration_process_imu(&imu);
    }
    assert(!ground_calibration_is_ready());
    ground_calibration_update(19000U);
    assert(ground_calibration_get_failure() ==
           GROUND_CALIBRATION_FAILURE_IMU_ERROR ||
           ground_calibration_get_failure() ==
           GROUND_CALIBRATION_FAILURE_BARO_ERROR);
    assert(ground_calibration_get_progress() == 0.0f);

    ground_calibration_restart(20000U);
    ground_calibration_update(50000U);
    assert(ground_calibration_get_state() == GROUND_CALIBRATION_FAULT);
    assert(ground_calibration_get_failure() ==
           GROUND_CALIBRATION_FAILURE_TIMEOUT);

    /* All timing uses unsigned subtraction and must survive HAL tick rollover. */
    ground_calibration_restart(UINT32_MAX - 500U);
    ground_calibration_update(499U);
    assert(ground_calibration_get_state() == GROUND_CALIBRATION_CALIBRATING);
    ground_calibration_update(29499U);
    assert(ground_calibration_get_state() == GROUND_CALIBRATION_FAULT);
    assert(ground_calibration_get_failure() ==
           GROUND_CALIBRATION_FAILURE_TIMEOUT);

    puts("ground_calibration_test: PASS");
    return 0;
}
