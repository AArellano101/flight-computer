/*
 * sensors.h
 *
 *  Created on: Jul 14, 2026
 *      Author: aaron
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdbool.h>

bool imu_is_healthy(void);
bool barometer_is_healthy(void);
bool magnetometer_is_healthy(void);

#endif /* INC_SENSORS_H_ */
