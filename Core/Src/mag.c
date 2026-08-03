#include "iis2mdc_reg.h"
#include "mag.h"
#include "main.h"
#include <stdio.h>


extern I2C_HandleTypeDef hi2c3;

stmdev_ctx_t dev_ctx_mag;

/* I2C Device Address (IIS2MDC fixed 7-bit address is 0x1E -> shifted left by 1 for STM32 HAL = 0x3C) */

// https://www.st.com/content/ccc/resource/technical/document/datasheet/group3/06/f2/a3/a7/74/fe/4b/16/DM00431721/files/DM00431721.pdf/jcr:content/translations/en.DM00431721.pdf
// Table 21 in the link above
#define IIS2MDC_I2C_ADD_HAL  (IIS2MDC_I2C_ADD & 0xFEU)

int32_t platform_write_iis2mdc(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_StatusTypeDef status;

    // Cast the generic handle back to the STM32 I2C handle
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;

    // Use the casted handle (hi2c) and the magnetometer's address
    status = HAL_I2C_Mem_Write(hi2c, IIS2MDC_I2C_ADD_HAL, reg,
                               I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? 0 : -1;
}

int32_t platform_read_iis2mdc(
    void *handle,
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{
    HAL_StatusTypeDef status;

    I2C_HandleTypeDef *hi2c =
        (I2C_HandleTypeDef *)handle;

    printf("[MAG READ] Entered platform read\r\n");
    printf("[MAG READ] Address: 0x%02X\r\n",
           IIS2MDC_I2C_ADD_HAL);
    printf("[MAG READ] Register: 0x%02X\r\n", reg);
    printf("[MAG READ] Length: %u\r\n", len);

    printf("[MAG READ] PRIMASK: %lu\r\n",
           (unsigned long)__get_PRIMASK());

    printf("[MAG READ] HAL I2C state: 0x%02X\r\n",
           (unsigned int)HAL_I2C_GetState(hi2c));

    printf("[MAG READ] HAL I2C error: 0x%08lX\r\n",
           (unsigned long)HAL_I2C_GetError(hi2c));

    printf("[MAG READ] CR1: 0x%08lX\r\n",
           (unsigned long)hi2c->Instance->CR1);

    printf("[MAG READ] CR2: 0x%08lX\r\n",
           (unsigned long)hi2c->Instance->CR2);

    printf("[MAG READ] ISR: 0x%08lX\r\n",
           (unsigned long)hi2c->Instance->ISR);

    printf("[MAG READ] I2C BUSY: %lu\r\n",
           (unsigned long)
           ((hi2c->Instance->ISR & I2C_ISR_BUSY) != 0U));

    printf("[MAG READ] Calling HAL_I2C_Mem_Read\r\n");

    status = HAL_I2C_Mem_Read(
        hi2c,
        IIS2MDC_I2C_ADD_HAL,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        bufp,
        len,
        HAL_MAX_DELAY
    );

    printf("[MAG READ] HAL status: %d\r\n", (int)status);

    printf("[MAG READ] HAL error: 0x%08lX\r\n",
           (unsigned long)HAL_I2C_GetError(hi2c));

    printf("[MAG READ] ISR after read: 0x%08lX\r\n",
           (unsigned long)hi2c->Instance->ISR);

    printf("[MAG READ] HAL state after read: 0x%02X\r\n",
           (unsigned int)HAL_I2C_GetState(hi2c));

    printf("[MAG READ] NACKF: %lu\r\n",
           (unsigned long)
           ((hi2c->Instance->ISR & I2C_ISR_NACKF) != 0U));

    printf("[MAG READ] BERR: %lu\r\n",
           (unsigned long)
           ((hi2c->Instance->ISR & I2C_ISR_BERR) != 0U));

    printf("[MAG READ] ARLO: %lu\r\n",
           (unsigned long)
           ((hi2c->Instance->ISR & I2C_ISR_ARLO) != 0U));

    printf("[MAG READ] STOPF: %lu\r\n",
           (unsigned long)
           ((hi2c->Instance->ISR & I2C_ISR_STOPF) != 0U));

    return (status == HAL_OK) ? 0 : -1;
}

void iis2mdctr_init(void) {
	uint8_t whoamI; // Added missing declaration
	uint8_t rst;
	int32_t ret;

	printf("[MAG] Entered init function\r\n");

	printf("[MAG] Assigning write callback\r\n");

	/* Point to your new magnetometer-specific callbacks */
	dev_ctx_mag.write_reg = platform_write_iis2mdc;

	printf("[MAG] Assigning read callback\r\n");
	dev_ctx_mag.read_reg  = platform_read_iis2mdc;

	/* Pass the I2C3 handle */
	printf("[MAG] Assigning I2C3 handle\r\n");
	dev_ctx_mag.handle = &hi2c3;

	printf("[MAG] Reading WHO_AM_I\r\n");
	ret = iis2mdc_device_id_get(&dev_ctx_mag, &whoamI);
	printf("[MAG] WHO_AM_I read finished: 0x%02X\r\n", whoamI);

	if (ret != 0) {
		printf("[MAG] Entering Error_Handler\r\n");
	    printf("IIS2MDC WHO_AM_I read failed\r\n");
	    return;
	}
	if (whoamI != IIS2MDC_ID) {
		Error_Handler(); // Handle communication failure / incorrect device ID
	}

	printf("[MAG] Device ID correct\r\n");

	/* Software reset */

	printf("[MAG] Starting software reset\r\n");
	iis2mdc_reset_set(&dev_ctx_mag, PROPERTY_ENABLE);
	printf("[MAG] Reset command sent\r\n");

	do {
		printf("[MAG] Reading reset status\r\n");
		iis2mdc_reset_get(&dev_ctx_mag, &rst);
		printf("[MAG] Reset status: %u\r\n", rst);
	} while (rst);
	printf("[MAG] Software reset complete\r\n");

	printf("[MAG] Enabling block data update\r\n");
	/* Enable Block Data Update (prevents reading intermediate data) */
	iis2mdc_block_data_update_set(&dev_ctx_mag, PROPERTY_ENABLE);
	printf("[MAG] Block data update configured\r\n");

	printf("[MAG] Setting continuous mode\r\n");
	/* Set Output Data Rate (ODR) to 10 Hz & High-Resolution mode */
	iis2mdc_operating_mode_set(&dev_ctx_mag, IIS2MDC_CONTINUOUS_MODE);

	 printf("[MAG] Continuous mode configured\r\n");

	    printf("[MAG] Setting output data rate\r\n");
	iis2mdc_data_rate_set(&dev_ctx_mag, IIS2MDC_ODR_10Hz);
	printf("[MAG] Output data rate configured\r\n");

	    printf("[MAG] Initialization function finished\r\n");
}

uint8_t iis2mdctr_read(void) {
    int16_t data_raw_magnetic[3];
    uint8_t reg;
    float magnetic_mG[3];

    /* Check if new magnetic data is ready */
    iis2mdc_mag_data_ready_get(&dev_ctx_mag, &reg);

    if (reg) {
        /* Read raw magnetic data */
        iis2mdc_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic);

        /* Convert raw data to milliGauss (sensitivity is 1.5 mG/LSB) */
        magnetic_mG[0] = iis2mdc_from_lsb_to_mgauss(data_raw_magnetic[0]);
        magnetic_mG[1] = iis2mdc_from_lsb_to_mgauss(data_raw_magnetic[1]);
        magnetic_mG[2] = iis2mdc_from_lsb_to_mgauss(data_raw_magnetic[2]);

        printf("\n\nMag [mG]: X=%8.2f  Y=%8.2f  Z=%8.2f\r",
               magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);

        return 1; // Success: new data read
    }

    return 0; // No new data available
}
