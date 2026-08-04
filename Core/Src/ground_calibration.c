#include "ground_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* MS5611 operating limits; these reject corrupt values before accumulation. */
#define CAL_PRESSURE_MIN_PA       (1000.0f)
#define CAL_PRESSURE_MAX_PA     (120000.0f)
#define CAL_TEMPERATURE_MIN_C      (-40.0f)
#define CAL_TEMPERATURE_MAX_C       (85.0f)

typedef struct
{
    uint32_t count;
    double mean;
    double m2;
} ScalarStatistics_t;

typedef struct
{
    uint32_t count;
    double mean[3];
    double m2[3];
} VectorStatistics_t;

typedef struct
{
    GroundCalibrationState_t state;
    GroundCalibrationFailure_t failure;
    GroundCalibrationResult_t result;

    uint32_t session_start_ms;
    uint32_t now_ms;
    uint32_t window_start_ms;
    uint32_t latest_sample_ms;
    bool window_started;
    bool have_latest_sample;

    bool have_imu_sequence;
    bool have_baro_sequence;
    bool have_imu_timestamp;
    bool have_baro_timestamp;
    uint32_t last_imu_sequence;
    uint32_t last_baro_sequence;
    uint32_t last_imu_timestamp_ms;
    uint32_t last_baro_timestamp_ms;

    VectorStatistics_t accel;
    VectorStatistics_t gyro;
    ScalarStatistics_t pressure;
    ScalarStatistics_t temperature;
} GroundCalibrationContext_t;

static GroundCalibrationContext_t calibration;

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    /* Unsigned subtraction deliberately supports HAL_GetTick() rollover. */
    return now_ms - then_ms;
}

static bool time_is_after(uint32_t candidate_ms, uint32_t reference_ms)
{
    return ((int32_t)(candidate_ms - reference_ms) > 0);
}

static float clamp_unit(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

static bool finite_vector3(const float value[3])
{
    return (value != NULL) &&
           isfinite(value[0]) &&
           isfinite(value[1]) &&
           isfinite(value[2]);
}

static void scalar_statistics_add(ScalarStatistics_t *statistics, float value)
{
    double delta;
    double delta2;

    statistics->count++;
    delta = (double)value - statistics->mean;
    statistics->mean += delta / (double)statistics->count;
    delta2 = (double)value - statistics->mean;
    statistics->m2 += delta * delta2;
}

static void vector_statistics_add(VectorStatistics_t *statistics,
                                  const float value[3])
{
    uint32_t axis;
    uint32_t new_count = statistics->count + 1u;

    for (axis = 0u; axis < 3u; axis++)
    {
        double delta = (double)value[axis] - statistics->mean[axis];
        double delta2;

        statistics->mean[axis] += delta / (double)new_count;
        delta2 = (double)value[axis] - statistics->mean[axis];
        statistics->m2[axis] += delta * delta2;
    }
    statistics->count = new_count;
}

static double scalar_statistics_stddev(const ScalarStatistics_t *statistics)
{
    if (statistics->count < 2u)
    {
        return 0.0;
    }
    return sqrt(statistics->m2 / (double)(statistics->count - 1u));
}

static double vector_statistics_stddev(const VectorStatistics_t *statistics,
                                       uint32_t axis)
{
    if ((statistics->count < 2u) || (axis >= 3u))
    {
        return 0.0;
    }
    return sqrt(statistics->m2[axis] /
                (double)(statistics->count - 1u));
}

static void clear_collection_statistics(void)
{
    memset(&calibration.accel, 0, sizeof(calibration.accel));
    memset(&calibration.gyro, 0, sizeof(calibration.gyro));
    memset(&calibration.pressure, 0, sizeof(calibration.pressure));
    memset(&calibration.temperature, 0, sizeof(calibration.temperature));

    calibration.window_started = false;
    calibration.have_latest_sample = false;
    calibration.window_start_ms = 0u;
    calibration.latest_sample_ms = 0u;
    calibration.have_imu_timestamp = false;
    calibration.have_baro_timestamp = false;
    calibration.last_imu_timestamp_ms = 0u;
    calibration.last_baro_timestamp_ms = 0u;
}

static void reset_collection(GroundCalibrationFailure_t reason)
{
    clear_collection_statistics();
    if (reason != GROUND_CALIBRATION_FAILURE_NONE)
    {
        /* Retain the latest reset cause so status output can explain progress. */
        calibration.failure = reason;
    }
}

static void begin_window_if_needed(uint32_t sample_time_ms)
{
    if (!calibration.window_started)
    {
        calibration.window_started = true;
        calibration.window_start_ms = sample_time_ms;
    }
}

static void note_sample_time(uint32_t sample_time_ms)
{
    begin_window_if_needed(sample_time_ms);

    if (!calibration.have_latest_sample ||
        time_is_after(sample_time_ms, calibration.latest_sample_ms))
    {
        calibration.latest_sample_ms = sample_time_ms;
        calibration.have_latest_sample = true;
    }
}

static bool required_samples_are_current(uint32_t reference_time_ms)
{
    if (!calibration.have_imu_timestamp ||
        !calibration.have_baro_timestamp)
    {
        return false;
    }

    return (elapsed_ms(reference_time_ms,
                       calibration.last_imu_timestamp_ms) <=
            GROUND_CALIBRATION_MAX_IMU_GAP_MS) &&
           (elapsed_ms(reference_time_ms,
                       calibration.last_baro_timestamp_ms) <=
            GROUND_CALIBRATION_MAX_BARO_GAP_MS);
}

static bool sample_requirements_met(void)
{
    if (!calibration.window_started || !calibration.have_latest_sample)
    {
        return false;
    }

    return (calibration.accel.count >= GROUND_CALIBRATION_MIN_IMU_SAMPLES) &&
           (calibration.gyro.count >= GROUND_CALIBRATION_MIN_IMU_SAMPLES) &&
           (calibration.pressure.count >= GROUND_CALIBRATION_MIN_BARO_SAMPLES) &&
           required_samples_are_current(calibration.latest_sample_ms) &&
           (elapsed_ms(calibration.latest_sample_ms,
                       calibration.window_start_ms) >=
            GROUND_CALIBRATION_MIN_COLLECTION_MS);
}

static void try_to_commit_result(void)
{
    GroundCalibrationResult_t candidate;
    float gravity_norm;
    uint32_t axis;
    bool noisy = false;

    if ((calibration.state != GROUND_CALIBRATION_CALIBRATING) ||
        !sample_requirements_met())
    {
        return;
    }

    memset(&candidate, 0, sizeof(candidate));

    for (axis = 0u; axis < 3u; axis++)
    {
        double gyro_stddev = vector_statistics_stddev(&calibration.gyro, axis);
        double accel_stddev = vector_statistics_stddev(&calibration.accel, axis);

        if (!isfinite(gyro_stddev) || !isfinite(accel_stddev) ||
            (gyro_stddev > (double)GROUND_CALIBRATION_GYRO_STDDEV_MAX_DPS) ||
            (accel_stddev > (double)GROUND_CALIBRATION_ACCEL_STDDEV_MAX_MG))
        {
            noisy = true;
        }

        candidate.gyro_bias_dps[axis] =
            (float)calibration.gyro.mean[axis];
        candidate.gravity_reference_mg[axis] =
            (float)calibration.accel.mean[axis];
        candidate.gyro_stddev_dps[axis] = (float)gyro_stddev;
        candidate.accel_stddev_mg[axis] = (float)accel_stddev;

        if (!isfinite(candidate.gyro_bias_dps[axis]) ||
            !isfinite(candidate.gravity_reference_mg[axis]))
        {
            noisy = true;
        }
    }

    gravity_norm = sqrtf((candidate.gravity_reference_mg[0] *
                          candidate.gravity_reference_mg[0]) +
                         (candidate.gravity_reference_mg[1] *
                          candidate.gravity_reference_mg[1]) +
                         (candidate.gravity_reference_mg[2] *
                          candidate.gravity_reference_mg[2]));
    if (!isfinite(gravity_norm) ||
        (gravity_norm < GROUND_CALIBRATION_ACCEL_NORM_MIN_MG) ||
        (gravity_norm > GROUND_CALIBRATION_ACCEL_NORM_MAX_MG))
    {
        noisy = true;
    }

    candidate.ground_pressure_pa = (float)calibration.pressure.mean;
    candidate.ground_temperature_c = (float)calibration.temperature.mean;
    candidate.pressure_stddev_pa =
        (float)scalar_statistics_stddev(&calibration.pressure);
    candidate.imu_sample_count = calibration.gyro.count;
    candidate.baro_sample_count = calibration.pressure.count;
    candidate.start_time_ms = calibration.window_start_ms;
    candidate.completion_time_ms = calibration.latest_sample_ms;

    if (!isfinite(candidate.ground_pressure_pa) ||
        !isfinite(candidate.ground_temperature_c) ||
        !isfinite(candidate.pressure_stddev_pa) ||
        (candidate.ground_pressure_pa < CAL_PRESSURE_MIN_PA) ||
        (candidate.ground_pressure_pa > CAL_PRESSURE_MAX_PA) ||
        (candidate.ground_temperature_c < CAL_TEMPERATURE_MIN_C) ||
        (candidate.ground_temperature_c > CAL_TEMPERATURE_MAX_C) ||
        (candidate.pressure_stddev_pa >
         GROUND_CALIBRATION_PRESSURE_STDDEV_MAX_PA))
    {
        noisy = true;
    }

    if (noisy)
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_NOISY);
        return;
    }

    /*
     * Publish only a complete, validated result. valid and READY are written
     * last, so interrupt-level readers cannot mistake an in-progress structure
     * copy for a usable calibration.
     */
    candidate.valid = false;
    calibration.result.valid = false;
    calibration.result = candidate;
    calibration.result.valid = true;
    calibration.failure = GROUND_CALIBRATION_FAILURE_NONE;
    calibration.state = GROUND_CALIBRATION_READY;
}

void ground_calibration_init(uint32_t now_ms)
{
    ground_calibration_restart(now_ms);
}

void ground_calibration_restart(uint32_t now_ms)
{
    memset(&calibration, 0, sizeof(calibration));
    calibration.state = GROUND_CALIBRATION_WARMUP;
    calibration.failure = GROUND_CALIBRATION_FAILURE_NONE;
    calibration.session_start_ms = now_ms;
    calibration.now_ms = now_ms;
}

void ground_calibration_abort(void)
{
    if ((calibration.state == GROUND_CALIBRATION_WARMUP) ||
        (calibration.state == GROUND_CALIBRATION_CALIBRATING))
    {
        calibration.result.valid = false;
        clear_collection_statistics();
        calibration.failure = GROUND_CALIBRATION_FAILURE_ABORTED;
        calibration.state = GROUND_CALIBRATION_ABORTED;
    }
}

void ground_calibration_fault(GroundCalibrationFailure_t reason)
{
    calibration.result.valid = false;
    clear_collection_statistics();
    calibration.failure = reason;
    calibration.state = GROUND_CALIBRATION_FAULT;
}

void ground_calibration_update(uint32_t now_ms)
{
    calibration.now_ms = now_ms;

    if ((calibration.state == GROUND_CALIBRATION_WARMUP) &&
        (elapsed_ms(now_ms, calibration.session_start_ms) >=
         GROUND_CALIBRATION_WARMUP_MS))
    {
        clear_collection_statistics();
        calibration.have_imu_sequence = false;
        calibration.have_baro_sequence = false;
        calibration.failure = GROUND_CALIBRATION_FAILURE_NONE;
        calibration.state = GROUND_CALIBRATION_CALIBRATING;
    }

    /*
     * A calibration window must contain a continuous stream from both
     * required sensors. Checking here prevents a stopped sensor from leaving
     * apparently usable statistics indefinitely while the other sensor keeps
     * sampling. The signed comparison avoids treating a sample timestamp a
     * few milliseconds newer than this loop's time snapshot as stale.
     */
    if ((calibration.state == GROUND_CALIBRATION_CALIBRATING) &&
        calibration.have_imu_timestamp &&
        ((int32_t)(now_ms - calibration.last_imu_timestamp_ms) >= 0) &&
        (elapsed_ms(now_ms, calibration.last_imu_timestamp_ms) >
         GROUND_CALIBRATION_MAX_IMU_GAP_MS))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
    }
    else if ((calibration.state == GROUND_CALIBRATION_CALIBRATING) &&
             calibration.have_baro_timestamp &&
             ((int32_t)(now_ms - calibration.last_baro_timestamp_ms) >= 0) &&
             (elapsed_ms(now_ms, calibration.last_baro_timestamp_ms) >
              GROUND_CALIBRATION_MAX_BARO_GAP_MS))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
    }

    if (((calibration.state == GROUND_CALIBRATION_WARMUP) ||
         (calibration.state == GROUND_CALIBRATION_CALIBRATING)) &&
        (elapsed_ms(now_ms, calibration.session_start_ms) >=
         GROUND_CALIBRATION_TIMEOUT_MS))
    {
        ground_calibration_fault(GROUND_CALIBRATION_FAILURE_TIMEOUT);
    }
}

void ground_calibration_process_imu(const ImuSample_t *sample)
{
    uint8_t required_fresh =
        (uint8_t)(IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH);
    float accel_norm;
    float gyro_norm;

    if (calibration.state != GROUND_CALIBRATION_CALIBRATING)
    {
        return;
    }

    if (sample == NULL)
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
        return;
    }

    if (calibration.have_imu_sequence &&
        (sample->sequence == calibration.last_imu_sequence))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
        return;
    }
    calibration.last_imu_sequence = sample->sequence;
    calibration.have_imu_sequence = true;

    if (((sample->fresh_mask & required_fresh) != required_fresh) ||
        !finite_vector3(sample->acceleration_mg) ||
        !finite_vector3(sample->angular_rate_dps))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
        return;
    }

    accel_norm = sqrtf((sample->acceleration_mg[0] *
                        sample->acceleration_mg[0]) +
                       (sample->acceleration_mg[1] *
                        sample->acceleration_mg[1]) +
                       (sample->acceleration_mg[2] *
                        sample->acceleration_mg[2]));
    gyro_norm = sqrtf((sample->angular_rate_dps[0] *
                       sample->angular_rate_dps[0]) +
                      (sample->angular_rate_dps[1] *
                       sample->angular_rate_dps[1]) +
                      (sample->angular_rate_dps[2] *
                       sample->angular_rate_dps[2]));

    if (!isfinite(accel_norm) || !isfinite(gyro_norm))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
        return;
    }

    if ((accel_norm < GROUND_CALIBRATION_ACCEL_NORM_MIN_MG) ||
        (accel_norm > GROUND_CALIBRATION_ACCEL_NORM_MAX_MG) ||
        (gyro_norm > GROUND_CALIBRATION_GYRO_NORM_MAX_DPS))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_MOTION);
        return;
    }

    if (calibration.have_imu_timestamp &&
        (elapsed_ms(sample->timestamp_ms,
                    calibration.last_imu_timestamp_ms) >
         GROUND_CALIBRATION_MAX_IMU_GAP_MS))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
    }

    calibration.last_imu_timestamp_ms = sample->timestamp_ms;
    calibration.have_imu_timestamp = true;

    note_sample_time(sample->timestamp_ms);
    vector_statistics_add(&calibration.accel, sample->acceleration_mg);
    vector_statistics_add(&calibration.gyro, sample->angular_rate_dps);
    try_to_commit_result();
}

void ground_calibration_process_baro(const MS5611_Sample_t *sample)
{
    if (calibration.state != GROUND_CALIBRATION_CALIBRATING)
    {
        return;
    }

    if (sample == NULL)
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
        return;
    }

    if (calibration.have_baro_sequence &&
        (sample->sequence == calibration.last_baro_sequence))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
        return;
    }
    calibration.last_baro_sequence = sample->sequence;
    calibration.have_baro_sequence = true;

    if (!isfinite(sample->pressure_pa) ||
        !isfinite(sample->temperature_c) ||
        (sample->pressure_pa < CAL_PRESSURE_MIN_PA) ||
        (sample->pressure_pa > CAL_PRESSURE_MAX_PA) ||
        (sample->temperature_c < CAL_TEMPERATURE_MIN_C) ||
        (sample->temperature_c > CAL_TEMPERATURE_MAX_C))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
        return;
    }

    if (calibration.have_baro_timestamp &&
        (elapsed_ms(sample->timestamp_ms,
                    calibration.last_baro_timestamp_ms) >
         GROUND_CALIBRATION_MAX_BARO_GAP_MS))
    {
        reset_collection(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
    }

    calibration.last_baro_timestamp_ms = sample->timestamp_ms;
    calibration.have_baro_timestamp = true;

    note_sample_time(sample->timestamp_ms);
    scalar_statistics_add(&calibration.pressure, sample->pressure_pa);
    scalar_statistics_add(&calibration.temperature, sample->temperature_c);
    try_to_commit_result();
}

GroundCalibrationState_t ground_calibration_get_state(void)
{
    return calibration.state;
}

GroundCalibrationFailure_t ground_calibration_get_failure(void)
{
    return calibration.failure;
}

const GroundCalibrationResult_t *ground_calibration_get_result(void)
{
    return &calibration.result;
}

float ground_calibration_get_progress(void)
{
    float imu_progress;
    float baro_progress;
    float time_progress;
    float progress;

    if (ground_calibration_is_ready())
    {
        return 1.0f;
    }

    if ((calibration.state != GROUND_CALIBRATION_CALIBRATING) ||
        !calibration.window_started)
    {
        return 0.0f;
    }

    imu_progress = clamp_unit((float)calibration.gyro.count /
                              (float)GROUND_CALIBRATION_MIN_IMU_SAMPLES);
    baro_progress = clamp_unit((float)calibration.pressure.count /
                               (float)GROUND_CALIBRATION_MIN_BARO_SAMPLES);

    if (calibration.have_latest_sample)
    {
        time_progress = clamp_unit(
            (float)elapsed_ms(calibration.latest_sample_ms,
                              calibration.window_start_ms) /
            (float)GROUND_CALIBRATION_MIN_COLLECTION_MS);
    }
    else
    {
        time_progress = 0.0f;
    }

    progress = imu_progress;
    if (baro_progress < progress)
    {
        progress = baro_progress;
    }
    if (time_progress < progress)
    {
        progress = time_progress;
    }
    return progress;
}

bool ground_calibration_is_ready(void)
{
    return (calibration.state == GROUND_CALIBRATION_READY) &&
           calibration.result.valid;
}

void ground_calibration_correct_gyro(const float raw[3], float out[3])
{
    uint32_t axis;
    bool ready;

    if ((raw == NULL) || (out == NULL))
    {
        return;
    }

    ready = ground_calibration_is_ready();
    for (axis = 0u; axis < 3u; axis++)
    {
        out[axis] = ready
                        ? (raw[axis] - calibration.result.gyro_bias_dps[axis])
                        : raw[axis];
    }
}

bool ground_calibration_altitude_agl_m(float pressure_pa, float *altitude_m)
{
    float pressure_ratio;
    float altitude;

    if ((altitude_m == NULL) || !ground_calibration_is_ready() ||
        !isfinite(pressure_pa) || (pressure_pa <= 0.0f) ||
        !isfinite(calibration.result.ground_pressure_pa) ||
        (calibration.result.ground_pressure_pa <= 0.0f))
    {
        return false;
    }

    pressure_ratio = pressure_pa / calibration.result.ground_pressure_pa;
    if (!isfinite(pressure_ratio) || (pressure_ratio <= 0.0f))
    {
        return false;
    }

    altitude = 44330.0f * (1.0f - powf(pressure_ratio, 0.190295f));
    if (!isfinite(altitude))
    {
        return false;
    }

    *altitude_m = altitude;
    return true;
}
