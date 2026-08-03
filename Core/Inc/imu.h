#ifndef IMU_H_
#define IMU_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"   // Grants access to standard HAL types like I2C_HandleTypeDef
#include "lsm6dso32x_reg.h"  // Grants access to stmdev_ctx_t and sensor types

/* Exported Macros -----------------------------------------------------------*/
/*
 * 7-bit standard target address shifted left by 1 bit.
 * Change to (0x6B << 1) if SDO/SA0 pin is physically tied high to 3.3V.
 */
#define LSM6DSO32X_I2C_ADD  (0x6A << 1)

/* Exported Variables --------------------------------------------------------*/
/*
 * Extern declarations make these visible to main.c without duplication errors.
 * They must be instantiated globally inside your imu.c file.
 */
extern stmdev_ctx_t dev_ctx;
extern float acceleration_mg[3]; // [0]=X, [1]=Y, [2]=Z
extern float angular_rate_dps[3]; // [0]=X, [1]=Y, [2]=Z

/* Exported Functions Prototypes ---------------------------------------------*/
/**
  * @brief  Initializes communication parameters and applies the baseline
  *         accelerometer and gyroscope hardware profiling configurations.
  * @retval None
  */
void lsm6dso32x_init(void);

/**
  * @brief  Checks for available physical data-ready signals and extracts
  *         scaled raw variables into readable engineering float formats.
  * @retval None
  */
void lsm6dso32x_read_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
