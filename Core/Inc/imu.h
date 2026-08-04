#ifndef IMU_H_
#define IMU_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "lsm6dso32x_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/
/* 7-bit address shifted for the STM32 HAL. SDO/SA0 is tied low. */
#define LSM6DSO32X_I2C_ADD          (0x6AU << 1)

#define IMU_SAMPLE_ACCEL_FRESH      (1u)
#define IMU_SAMPLE_GYRO_FRESH       (2u)

/*
 * Sensor-to-vehicle body-frame mapping. The defaults are an explicit identity
 * transform. Change these six macros to match the installed PCB orientation;
 * each source axis must be used exactly once and each sign must be +1 or -1.
 */
#define IMU_SENSOR_AXIS_X           (0)
#define IMU_SENSOR_AXIS_Y           (1)
#define IMU_SENSOR_AXIS_Z           (2)

#ifndef IMU_BODY_X_SOURCE_AXIS
#define IMU_BODY_X_SOURCE_AXIS      IMU_SENSOR_AXIS_X
#endif
#ifndef IMU_BODY_Y_SOURCE_AXIS
#define IMU_BODY_Y_SOURCE_AXIS      IMU_SENSOR_AXIS_Y
#endif
#ifndef IMU_BODY_Z_SOURCE_AXIS
#define IMU_BODY_Z_SOURCE_AXIS      IMU_SENSOR_AXIS_Z
#endif

#ifndef IMU_BODY_X_SOURCE_SIGN
#define IMU_BODY_X_SOURCE_SIGN      (1)
#endif
#ifndef IMU_BODY_Y_SOURCE_SIGN
#define IMU_BODY_Y_SOURCE_SIGN      (1)
#endif
#ifndef IMU_BODY_Z_SOURCE_SIGN
#define IMU_BODY_Z_SOURCE_SIGN      (1)
#endif

#if ((IMU_BODY_X_SOURCE_AXIS < IMU_SENSOR_AXIS_X) || \
     (IMU_BODY_X_SOURCE_AXIS > IMU_SENSOR_AXIS_Z) || \
     (IMU_BODY_Y_SOURCE_AXIS < IMU_SENSOR_AXIS_X) || \
     (IMU_BODY_Y_SOURCE_AXIS > IMU_SENSOR_AXIS_Z) || \
     (IMU_BODY_Z_SOURCE_AXIS < IMU_SENSOR_AXIS_X) || \
     (IMU_BODY_Z_SOURCE_AXIS > IMU_SENSOR_AXIS_Z) || \
     (IMU_BODY_X_SOURCE_AXIS == IMU_BODY_Y_SOURCE_AXIS) || \
     (IMU_BODY_X_SOURCE_AXIS == IMU_BODY_Z_SOURCE_AXIS) || \
     (IMU_BODY_Y_SOURCE_AXIS == IMU_BODY_Z_SOURCE_AXIS))
#error "IMU body-frame mapping must be a permutation of sensor X/Y/Z"
#endif

#if (((IMU_BODY_X_SOURCE_SIGN != 1) && (IMU_BODY_X_SOURCE_SIGN != -1)) || \
     ((IMU_BODY_Y_SOURCE_SIGN != 1) && (IMU_BODY_Y_SOURCE_SIGN != -1)) || \
     ((IMU_BODY_Z_SOURCE_SIGN != 1) && (IMU_BODY_Z_SOURCE_SIGN != -1)))
#error "IMU body-frame mapping signs must be +1 or -1"
#endif

/*
 * Acceleration is a polar vector while angular rate is an axial vector. The
 * same mapping is valid for both only for a proper, right-handed rotation
 * (determinant +1), so reject reflected/left-handed configurations.
 */
#define IMU_BODY_SIGN_PRODUCT \
    (IMU_BODY_X_SOURCE_SIGN * IMU_BODY_Y_SOURCE_SIGN * \
     IMU_BODY_Z_SOURCE_SIGN)

#if (((((IMU_BODY_X_SOURCE_AXIS == 0) && \
         (IMU_BODY_Y_SOURCE_AXIS == 1) && \
         (IMU_BODY_Z_SOURCE_AXIS == 2)) || \
        ((IMU_BODY_X_SOURCE_AXIS == 1) && \
         (IMU_BODY_Y_SOURCE_AXIS == 2) && \
         (IMU_BODY_Z_SOURCE_AXIS == 0)) || \
        ((IMU_BODY_X_SOURCE_AXIS == 2) && \
         (IMU_BODY_Y_SOURCE_AXIS == 0) && \
         (IMU_BODY_Z_SOURCE_AXIS == 1))) && \
       (IMU_BODY_SIGN_PRODUCT != 1)) || \
     ((((IMU_BODY_X_SOURCE_AXIS == 0) && \
         (IMU_BODY_Y_SOURCE_AXIS == 2) && \
         (IMU_BODY_Z_SOURCE_AXIS == 1)) || \
        ((IMU_BODY_X_SOURCE_AXIS == 2) && \
         (IMU_BODY_Y_SOURCE_AXIS == 1) && \
         (IMU_BODY_Z_SOURCE_AXIS == 0)) || \
        ((IMU_BODY_X_SOURCE_AXIS == 1) && \
         (IMU_BODY_Y_SOURCE_AXIS == 0) && \
         (IMU_BODY_Z_SOURCE_AXIS == 2))) && \
       (IMU_BODY_SIGN_PRODUCT != -1)))
#error "IMU body-frame mapping must be right-handed (determinant +1)"
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t timestamp_ms;
    uint32_t sequence;
    float acceleration_mg[3];
    float angular_rate_dps[3];
    uint8_t fresh_mask;
} ImuSample_t;

typedef enum
{
    IMU_POLL_NO_DATA = 0,
    IMU_POLL_NEW_DATA,
    IMU_POLL_ERROR
} ImuPollStatus_t;

/* Exported variables --------------------------------------------------------*/
extern stmdev_ctx_t dev_ctx;

/* Legacy mirrors retained while callers migrate to ImuSample_t. */
extern float acceleration_mg[3];
extern float angular_rate_dps[3];

/* Exported function prototypes ---------------------------------------------*/
/**
 * @brief Configure the LSM6DSO32X for 52 Hz acceleration and gyro sampling.
 * @return HAL_OK on success; HAL_ERROR on communication, identity, or reset
 *         failure.
 *
 * Sensor axes are reported without remapping. Apply the sensor-to-body axis
 * transform at the application boundary before calibration or flight use.
 */
HAL_StatusTypeDef lsm6dso32x_init(void);

/**
 * @brief Poll data-ready flags and return the most recent complete value set.
 *
 * Values whose fresh bits are clear are preserved from the preceding sample.
 * The sequence advances only when at least one fresh sensor value is read.
 */
ImuPollStatus_t lsm6dso32x_poll(ImuSample_t *sample);

/**
 * @brief Apply the configured sensor-axis to vehicle-body-axis transform.
 *
 * This application-boundary transform is intentionally separate from the
 * raw sensor driver so raw register interpretation stays unambiguous.
 */
void imu_transform_sensor_to_body(const ImuSample_t *sensor_sample,
                                  ImuSample_t *body_sample);

/** @brief Legacy polling wrapper. Prefer lsm6dso32x_poll(). */
void lsm6dso32x_read_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
