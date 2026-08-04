/*
 * commands.h
 *
 *  Created on: Jul 13, 2026
 *      Author: aaron
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#include <stdbool.h>

void command_process(const char *command);

void command_help(void);
void command_status(void);
void command_sensor_status(void);
void command_reset(void);
void print_rtc_datetime(void);
void reset_cause_capture(void);
void command_reset_cause(void);
void command_imu_readout(void);
void command_baro_readout(void);
void command_calibrate_ground(void);
void command_ground_confirm(void);
void command_flight_lock(void);
void command_calibration_status(void);
void command_calibration_abort(void);
void command_flash_read(const char *arguments);
void command_log_dump(const char *arguments);
void command_log_start(void);
void command_log_stop(void);
void command_log_clear(void);

extern bool flash_available;
extern bool logger_available;


#endif /* INC_COMMANDS_H_ */
