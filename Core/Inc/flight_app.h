#ifndef FLIGHT_APP_H
#define FLIGHT_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu.h"
#include "ms5611.h"

#include <stdbool.h>

/*
 * Console-facing application boundary.  Commands request state changes and
 * inspect cached samples; they never perform sensor bus transactions directly.
 */
bool flight_request_ground_calibration_restart(void);
void flight_request_ground_calibration_abort(void);

/** Confirm the vehicle is safely on the ground and request calibration. */
void flight_confirm_ground_mode(void);

/** Persistently inhibit recalibration; call this before arming/flight. */
void flight_lock_ground_calibration(void);

bool flight_is_ground_mode_confirmed(void);

bool flight_get_latest_imu_sample(ImuSample_t *sample);
bool flight_get_latest_baro_sample(MS5611_Sample_t *sample);
bool flight_get_latest_altitude_agl_m(float *altitude_m);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_APP_H */
