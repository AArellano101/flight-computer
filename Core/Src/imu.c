#include "lsm6dso32x_reg.h"
#include <stdio.h>
#include <string.h>
#include "main.h"


extern I2C_HandleTypeDef hi2c1;

stmdev_ctx_t dev_ctx; // Defined in line 137 in library. It is a generic sensor driver class
float acceleration_mg[3] = {0.0f, 0.0f, 0.0f};
float angular_rate_dps[3] = {0.0f, 0.0f, 0.0f};


// SA0/SDO are tied to GND; Address is 0x6A
// In pg. 35 of the datasheet
#define LSM6DSO32X_I2C_ADD  (0x6A << 1)

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    HAL_StatusTypeDef status;

    // Function def. here: https://sourcevu.sysprogs.com/stm32/HAL/symbols/HAL_I2C_Mem_Write
    status = HAL_I2C_Mem_Write(&hi2c1, LSM6DSO32X_I2C_ADD, reg,
                               I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? 0 : -1; // Returning 0 indicates success to the ST driver
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_StatusTypeDef status;

    // Function def. here: https://sourcevu.sysprogs.com/stm32/HAL/symbols/HAL_I2C_Mem_Read
    status = HAL_I2C_Mem_Read(&hi2c1, LSM6DSO32X_I2C_ADD, reg,
                              I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? 0 : -1;
}

void lsm6dso32x_init(void) {
    /* Link platform-specific I2C read/write functions to the ST driver */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = HAL_Delay;

    /* Validate sensor identity */
    uint8_t whoami = 0;
    lsm6dso32x_device_id_get(&dev_ctx, &whoami);

    if (whoami != 0x6C) {
        Error_Handler(); // Halts if the sensor is not found or wired incorrectly
    }

    /* Soft reset the sensor registers to default settings */
    lsm6dso32x_reset_set(&dev_ctx, PROPERTY_ENABLE);

    // Wait until reset bit clears
    uint8_t rst;
    do {
		lsm6dso32x_reset_get(&dev_ctx, &rst);
	} while (rst);


    /* Disable I3C mode to ensure robust, uninterrupted I2C communication */
    lsm6dso32x_i3c_disable_set(&dev_ctx, LSM6DSO32X_I3C_DISABLE);

    // Enable Block Data Update to avoid split multi-byte reading frames
    lsm6dso32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Accelerometer Output Data Rate & Scale (52Hz, 32g full scale) */
    lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_52Hz);
    lsm6dso32x_xl_full_scale_set(&dev_ctx, LSM6DSO32X_32g);

    /* Set Gyroscope Output Data Rate & Scale (52Hz, 2000 degrees per second) */
    lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_52Hz);
    lsm6dso32x_gy_full_scale_set(&dev_ctx, LSM6DSO32X_2000dps);
}

void lsm6dso32x_read_data(void) {
	lsm6dso32x_reg_t status; // "Group[s] all the registers that has a bitfield description"
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];

    /* Query status register flags */
    lsm6dso32x_read_reg(&dev_ctx, LSM6DSO32X_STATUS_REG, &status.byte, 1);

    /* Process Accelerometer data if new sample frame is available */
    if (status.status_reg.xlda) {
        lsm6dso32x_acceleration_raw_get(&dev_ctx, &data_raw_acceleration[0]);

        /* Convert integers to physical milli-G values using the 32g scale math */
        acceleration_mg[0] = lsm6dso32x_from_fs32_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] = lsm6dso32x_from_fs32_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lsm6dso32x_from_fs32_to_mg(data_raw_acceleration[2]);

//        printf("\n\nAccel [mg]: X=%8.2f  Y=%8.2f  Z=%8.2f\r",
//               acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }

    /* Process Gyroscope data if new sample frame is available */
    if (status.status_reg.gda) {
        lsm6dso32x_angular_rate_raw_get(&dev_ctx, &data_raw_angular_rate[0]);

        /* Convert integers to real Degrees Per Second (dps) using 2000dps scale math */
        angular_rate_dps[0] = lsm6dso32x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
        angular_rate_dps[1] = lsm6dso32x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
        angular_rate_dps[2] = lsm6dso32x_from_fs2000_to_mdps(data_raw_angular_rate[2]);

//        printf("\n\nGyro [dps]: X=%8.2f  Y=%8.2f  Z=%8.2f\r\n\n",
//               angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);
    }
}
