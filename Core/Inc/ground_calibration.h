#ifndef INC_GROUND_CALIBRATION_H_
#define INC_GROUND_CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"
#include "ms5611.h"

/*
 * Ground-calibration defaults.
 *
 * The motion/noise limits are deliberately conservative starting points, not
 * universal sensor specifications. Tune them from logged samples with the IMU
 * and barometer installed in the finished vehicle.
 */
#ifndef GROUND_CALIBRATION_WARMUP_MS
#define GROUND_CALIBRATION_WARMUP_MS               (1000u)
#endif

#ifndef GROUND_CALIBRATION_MIN_COLLECTION_MS
#define GROUND_CALIBRATION_MIN_COLLECTION_MS        (4000u)
#endif

#ifndef GROUND_CALIBRATION_TIMEOUT_MS
#define GROUND_CALIBRATION_TIMEOUT_MS              (30000u)
#endif

#ifndef GROUND_CALIBRATION_MIN_IMU_SAMPLES
#define GROUND_CALIBRATION_MIN_IMU_SAMPLES           (256u)
#endif

#ifndef GROUND_CALIBRATION_MIN_BARO_SAMPLES
#define GROUND_CALIBRATION_MIN_BARO_SAMPLES          (100u)
#endif

#ifndef GROUND_CALIBRATION_ACCEL_NORM_MIN_MG
#define GROUND_CALIBRATION_ACCEL_NORM_MIN_MG        (800.0f)
#endif

#ifndef GROUND_CALIBRATION_ACCEL_NORM_MAX_MG
#define GROUND_CALIBRATION_ACCEL_NORM_MAX_MG       (1200.0f)
#endif

#ifndef GROUND_CALIBRATION_GYRO_NORM_MAX_DPS
#define GROUND_CALIBRATION_GYRO_NORM_MAX_DPS          (5.0f)
#endif

#ifndef GROUND_CALIBRATION_GYRO_STDDEV_MAX_DPS
#define GROUND_CALIBRATION_GYRO_STDDEV_MAX_DPS        (1.0f)
#endif

#ifndef GROUND_CALIBRATION_ACCEL_STDDEV_MAX_MG
#define GROUND_CALIBRATION_ACCEL_STDDEV_MAX_MG       (30.0f)
#endif

#ifndef GROUND_CALIBRATION_PRESSURE_STDDEV_MAX_PA
#define GROUND_CALIBRATION_PRESSURE_STDDEV_MAX_PA    (20.0f)
#endif

#ifndef GROUND_CALIBRATION_MAX_IMU_GAP_MS
#define GROUND_CALIBRATION_MAX_IMU_GAP_MS              (30u)
#endif

#ifndef GROUND_CALIBRATION_MAX_BARO_GAP_MS
#define GROUND_CALIBRATION_MAX_BARO_GAP_MS             (75u)
#endif

typedef enum
{
    GROUND_CALIBRATION_UNINITIALIZED = 0,
    GROUND_CALIBRATION_WARMUP,
    GROUND_CALIBRATION_CALIBRATING,
    GROUND_CALIBRATION_READY,
    GROUND_CALIBRATION_ABORTED,
    GROUND_CALIBRATION_FAULT
} GroundCalibrationState_t;

typedef enum
{
    GROUND_CALIBRATION_FAILURE_NONE = 0,
    GROUND_CALIBRATION_FAILURE_MOTION,
    GROUND_CALIBRATION_FAILURE_IMU_ERROR,
    GROUND_CALIBRATION_FAILURE_BARO_ERROR,
    GROUND_CALIBRATION_FAILURE_NOISY,
    GROUND_CALIBRATION_FAILURE_TIMEOUT,
    GROUND_CALIBRATION_FAILURE_ABORTED
} GroundCalibrationFailure_t;

typedef struct
{
    bool valid;

    float gyro_bias_dps[3];
    float gravity_reference_mg[3];

    float gyro_stddev_dps[3];
    float accel_stddev_mg[3];

    float ground_pressure_pa;
    float ground_temperature_c;
    float pressure_stddev_pa;

    uint32_t imu_sample_count;
    uint32_t baro_sample_count;
    uint32_t start_time_ms;
    uint32_t completion_time_ms;
} GroundCalibrationResult_t;

/** Begin automatic warmup followed by ground calibration. */
void ground_calibration_init(uint32_t now_ms);

/** Discard any result/progress and begin a new warmup interval. */
void ground_calibration_restart(uint32_t now_ms);

/** Abort an active calibration and invalidate its result. */
void ground_calibration_abort(void);

/** Put calibration into FAULT and invalidate its result. */
void ground_calibration_fault(GroundCalibrationFailure_t reason);

/** Advance time-dependent warmup and timeout handling; never blocks. */
void ground_calibration_update(uint32_t now_ms);

/**
 * Consume one newly published IMU sample. Acceleration and angular rate must
 * already use the vehicle body-axis convention.
 */
void ground_calibration_process_imu(const ImuSample_t *sample);

/** Consume one newly published, complete MS5611 sample. */
void ground_calibration_process_baro(const MS5611_Sample_t *sample);

GroundCalibrationState_t ground_calibration_get_state(void);
GroundCalibrationFailure_t ground_calibration_get_failure(void);
const GroundCalibrationResult_t *ground_calibration_get_result(void);

/**
 * Return collection progress in [0, 1]. Progress is the least-complete of the
 * required IMU count, barometer count, and collection duration.
 */
float ground_calibration_get_progress(void);

bool ground_calibration_is_ready(void);

/** Subtract gyro bias when ready; otherwise copy raw values unchanged. */
void ground_calibration_correct_gyro(const float raw[3], float out[3]);

/**
 * Convert absolute pressure to altitude above the calibrated ground datum.
 * Returns false and leaves altitude_m unchanged when calibration/input is
 * invalid.
 */
bool ground_calibration_altitude_agl_m(float pressure_pa, float *altitude_m);

#ifdef __cplusplus
}
#endif

#endif /* INC_GROUND_CALIBRATION_H_ */
