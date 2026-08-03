/**
 ******************************************************************************
 * @file    mag.h
 * @brief   Header for IIS2MDC Magnetometer integration
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Include the STM32H7 HAL header so I2C_HandleTypeDef is recognized
   if this header is included before others. */
#include "stm32h7xx_hal.h"
#include "iis2mdc_reg.h"

/* Exported variables --------------------------------------------------------*/
/* Exposing the context allows you to use ST's read functions (like
   iis2mdc_magnetic_raw_get) in other files like main.c */
extern stmdev_ctx_t dev_ctx_mag;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initializes the IIS2MDC magnetometer, validates the WHO_AM_I,
 *         and configures the data rate and block data update.
 */
void iis2mdctr_init(void);

uint8_t iis2mdctr_read(void);


#ifdef __cplusplus
}
#endif

#endif /* MAG_H */
