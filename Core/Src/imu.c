#include "imu.h"

#include <string.h>

#include "main.h"

#define IMU_I2C_TIMEOUT_MS       (100u)
#define IMU_RESET_MAX_POLLS      (100u)
#define IMU_RESET_POLL_DELAY_MS  (1u)
#define MDPS_PER_DPS             (1000.0f)

extern I2C_HandleTypeDef hi2c1;

stmdev_ctx_t dev_ctx;

/* Legacy mirrors. New code should consume ImuSample_t from lsm6dso32x_poll. */
float acceleration_mg[3] = {0.0f, 0.0f, 0.0f};
float angular_rate_dps[3] = {0.0f, 0.0f, 0.0f};

static ImuSample_t latest_sample;
static uint8_t imu_initialized;

static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)handle;

    if ((i2c == NULL) || ((bufp == NULL) && (len != 0u)))
    {
        return -1;
    }

    return (HAL_I2C_Mem_Write(i2c, LSM6DSO32X_I2C_ADD, reg,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len,
                              IMU_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    I2C_HandleTypeDef *i2c = (I2C_HandleTypeDef *)handle;

    if ((i2c == NULL) || ((bufp == NULL) && (len != 0u)))
    {
        return -1;
    }

    return (HAL_I2C_Mem_Read(i2c, LSM6DSO32X_I2C_ADD, reg,
                             I2C_MEMADD_SIZE_8BIT, bufp, len,
                             IMU_I2C_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

HAL_StatusTypeDef lsm6dso32x_init(void)
{
    uint8_t whoami = 0u;
    uint8_t reset_pending = PROPERTY_ENABLE;
    uint32_t poll_count;

    imu_initialized = 0u;
    memset(&latest_sample, 0, sizeof(latest_sample));
    memset(acceleration_mg, 0, sizeof(acceleration_mg));
    memset(angular_rate_dps, 0, sizeof(angular_rate_dps));
    memset(&dev_ctx, 0, sizeof(dev_ctx));

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = HAL_Delay;
    dev_ctx.handle = &hi2c1;

    if (lsm6dso32x_device_id_get(&dev_ctx, &whoami) != 0)
    {
        return HAL_ERROR;
    }
    if (whoami != LSM6DSO32X_ID)
    {
        return HAL_ERROR;
    }

    if (lsm6dso32x_reset_set(&dev_ctx, PROPERTY_ENABLE) != 0)
    {
        return HAL_ERROR;
    }

    for (poll_count = 0u; poll_count < IMU_RESET_MAX_POLLS; ++poll_count)
    {
        if (lsm6dso32x_reset_get(&dev_ctx, &reset_pending) != 0)
        {
            return HAL_ERROR;
        }
        if (reset_pending == PROPERTY_DISABLE)
        {
            break;
        }
        HAL_Delay(IMU_RESET_POLL_DELAY_MS);
    }
    if (reset_pending != PROPERTY_DISABLE)
    {
        return HAL_ERROR;
    }

    if (lsm6dso32x_i3c_disable_set(&dev_ctx, LSM6DSO32X_I3C_DISABLE) != 0)
    {
        return HAL_ERROR;
    }
    if (lsm6dso32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE) != 0)
    {
        return HAL_ERROR;
    }
    if (lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_52Hz) != 0)
    {
        return HAL_ERROR;
    }
    if (lsm6dso32x_xl_full_scale_set(&dev_ctx, LSM6DSO32X_32g) != 0)
    {
        return HAL_ERROR;
    }
    if (lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_52Hz) != 0)
    {
        return HAL_ERROR;
    }
    if (lsm6dso32x_gy_full_scale_set(&dev_ctx, LSM6DSO32X_2000dps) != 0)
    {
        return HAL_ERROR;
    }

    imu_initialized = 1u;
    return HAL_OK;
}

ImuPollStatus_t lsm6dso32x_poll(ImuSample_t *sample)
{
    lsm6dso32x_status_reg_t status = {0};
    int16_t raw_acceleration[3] = {0, 0, 0};
    int16_t raw_angular_rate[3] = {0, 0, 0};
    ImuSample_t candidate;

    if (sample == NULL)
    {
        return IMU_POLL_ERROR;
    }

    *sample = latest_sample;
    sample->fresh_mask = 0u;

    if (imu_initialized == 0u)
    {
        return IMU_POLL_ERROR;
    }

    if (lsm6dso32x_status_reg_get(&dev_ctx, &status) != 0)
    {
        return IMU_POLL_ERROR;
    }

    if ((status.xlda == 0u) && (status.gda == 0u))
    {
        return IMU_POLL_NO_DATA;
    }

    candidate = latest_sample;
    candidate.fresh_mask = 0u;

    if (status.xlda != 0u)
    {
        if (lsm6dso32x_acceleration_raw_get(&dev_ctx, raw_acceleration) != 0)
        {
            return IMU_POLL_ERROR;
        }

        candidate.acceleration_mg[0] =
            lsm6dso32x_from_fs32_to_mg(raw_acceleration[0]);
        candidate.acceleration_mg[1] =
            lsm6dso32x_from_fs32_to_mg(raw_acceleration[1]);
        candidate.acceleration_mg[2] =
            lsm6dso32x_from_fs32_to_mg(raw_acceleration[2]);
        candidate.fresh_mask |= IMU_SAMPLE_ACCEL_FRESH;
    }

    if (status.gda != 0u)
    {
        if (lsm6dso32x_angular_rate_raw_get(&dev_ctx, raw_angular_rate) != 0)
        {
            return IMU_POLL_ERROR;
        }

        candidate.angular_rate_dps[0] =
            lsm6dso32x_from_fs2000_to_mdps(raw_angular_rate[0]) / MDPS_PER_DPS;
        candidate.angular_rate_dps[1] =
            lsm6dso32x_from_fs2000_to_mdps(raw_angular_rate[1]) / MDPS_PER_DPS;
        candidate.angular_rate_dps[2] =
            lsm6dso32x_from_fs2000_to_mdps(raw_angular_rate[2]) / MDPS_PER_DPS;
        candidate.fresh_mask |= IMU_SAMPLE_GYRO_FRESH;
    }

    if (candidate.fresh_mask == 0u)
    {
        return IMU_POLL_NO_DATA;
    }

    candidate.timestamp_ms = HAL_GetTick();
    candidate.sequence = latest_sample.sequence + 1u;
    latest_sample = candidate;

    memcpy(acceleration_mg, latest_sample.acceleration_mg,
           sizeof(acceleration_mg));
    memcpy(angular_rate_dps, latest_sample.angular_rate_dps,
           sizeof(angular_rate_dps));
    *sample = latest_sample;

    return IMU_POLL_NEW_DATA;
}

void imu_transform_sensor_to_body(const ImuSample_t *sensor_sample,
                                  ImuSample_t *body_sample)
{
    static const uint8_t source_axis[3] = {
        IMU_BODY_X_SOURCE_AXIS,
        IMU_BODY_Y_SOURCE_AXIS,
        IMU_BODY_Z_SOURCE_AXIS
    };
    static const float source_sign[3] = {
        (float)IMU_BODY_X_SOURCE_SIGN,
        (float)IMU_BODY_Y_SOURCE_SIGN,
        (float)IMU_BODY_Z_SOURCE_SIGN
    };
    ImuSample_t transformed;
    uint32_t body_axis;

    if ((sensor_sample == NULL) || (body_sample == NULL))
    {
        return;
    }

    transformed = *sensor_sample;

    for (body_axis = 0u; body_axis < 3u; body_axis++)
    {
        transformed.acceleration_mg[body_axis] =
            source_sign[body_axis] *
            sensor_sample->acceleration_mg[source_axis[body_axis]];
        transformed.angular_rate_dps[body_axis] =
            source_sign[body_axis] *
            sensor_sample->angular_rate_dps[source_axis[body_axis]];
    }

    *body_sample = transformed;
}

void lsm6dso32x_read_data(void)
{
    ImuSample_t sample;

    (void)lsm6dso32x_poll(&sample);
}
