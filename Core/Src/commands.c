#include "commands.h"
#include "main.h"
#include "sensors.h"
#include "imu.h"
#include "ms5611.h"
#include "flash.h"
#include "logger.h"
#include "ground_calibration.h"
#include "flight_app.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FIRMWARE_VERSION "0.1.0"

#define RESET_CAUSE_POWER_ON       (1UL << 0)
#define RESET_CAUSE_BROWNOUT       (1UL << 1)
#define RESET_CAUSE_PIN            (1UL << 2)
#define RESET_CAUSE_SOFTWARE       (1UL << 3)
#define RESET_CAUSE_IWDG           (1UL << 4)
#define RESET_CAUSE_WWDG           (1UL << 5)
#define RESET_CAUSE_LOW_POWER_D1   (1UL << 6)
#define RESET_CAUSE_LOW_POWER_D2   (1UL << 7)
#define RESET_CAUSE_CPU            (1UL << 8)
#define RESET_CAUSE_D1_DOMAIN      (1UL << 9)
#define RESET_CAUSE_D2_DOMAIN      (1UL << 10)

static uint32_t saved_reset_causes = 0; // Stores interpreted causes
static uint32_t saved_rcc_rsr = 0; // Stores raw register value

extern RTC_HandleTypeDef hrtc; // Contains info like RTC peripherals, prescalar settings, hour format, etc.
static const char *command_calibration_state_name(
    GroundCalibrationState_t state);
static const char *command_calibration_failure_name(
    GroundCalibrationFailure_t failure);
static bool command_calibration_is_busy(void);


void command_process(const char *command)
{
    if (strcmp(command, "help") == 0)
    {
        command_help();
    }
    else if (strcmp(command, "status") == 0)
    {
        command_status();
    }
    else if (strcmp(command, "reset") == 0)
    {
        command_reset();
    }
    else if (strcmp(command, "sensor_status") == 0)
    {
        command_sensor_status();
    }
    else if (strcmp(command, "reset_cause") == 0)
    {
        command_reset_cause();
    }
    else if (strcmp(command, "imu") == 0) {
		command_imu_readout();
    }
    else if (strcmp(command, "baro") == 0) {
		command_baro_readout();
    }
    else if (strcmp(command, "calibrate_ground") == 0)
    {
        command_calibrate_ground();
    }
    else if (strcmp(command, "ground_confirm") == 0)
    {
        command_ground_confirm();
    }
    else if (strcmp(command, "flight_lock") == 0)
    {
        command_flight_lock();
    }
    else if (strcmp(command, "cal_status") == 0)
    {
        command_calibration_status();
    }
    else if (strcmp(command, "cal_abort") == 0)
    {
        command_calibration_abort();
    }
    else if ((strncmp(command, "flash_read", 10U) == 0) &&
        ((command[10] == '\0') || (command[10] == ' ')))
    {
        command_flash_read(&command[10]);
    }
    else if (strcmp(command, "log_start") == 0)
    {
        command_log_start();
    }
    else if (strcmp(command, "log_stop") == 0)
    {
        command_log_stop();
    }
    else if ((strncmp(command, "log_dump", 8U) == 0) &&
             ((command[8] == '\0') || (command[8] == ' ')))
    {
        command_log_dump(&command[8]);
    }
    else if (strcmp(command, "log_clear") == 0)
    {
        command_log_clear();
    }
    else if (command[0] != '\0')
    {
        printf("Unknown command: %s\r\n", command);
        printf("Type 'help' for available commands.\r\n");
    }
}

// Print out commands
void command_help(void)
{
    printf("\r\n");
    printf("Mini Flight Computer Commands\r\n");
    printf("-----------------------------\r\n");

    printf("help                         Show this command list\r\n");
    printf("status                       Show overall system status\r\n");
    printf("sensor_status                Show sensor connection and health\r\n");
    printf("reset_cause                  Show reasons for previous reset\r\n");
    printf("uptime                       Show system uptime\r\n");

    printf("\r\n");
    printf("Sensor Commands\r\n");
    printf("-----------------------------\r\n");
    printf("imu                          Show data from IMU\r\n");
    printf("baro                         Show data from barometer\r\n");
    printf("ground_confirm               Confirm safe ground mode and calibrate\r\n");
    printf("flight_lock                  Persistently inhibit recalibration\r\n");
    printf("calibrate_ground             Restart ground calibration\r\n");
    printf("cal_status                   Show ground calibration status\r\n");
    printf("cal_abort                    Abort ground calibration\r\n");

    printf("\r\n");
    printf("Flash and Logger Commands\r\n");
    printf("-----------------------------\r\n");
    printf("flash_read <address> <length> Read bytes from external flash\r\n");
    printf("log_start                    Start data logging\r\n");
    printf("log_stop                     Stop data logging\r\n");
    printf("log_dump [count]             Print stored log records\r\n");
    printf("log_clear                    Erase all stored log records\r\n");

    printf("\r\n");
    printf("System Commands\r\n");
    printf("-----------------------------\r\n");
    printf("reset                        Restart the flight computer\r\n");

    printf("\r\n");
}

// Print out general summary of flight computer
void command_status(void)
{
    uint32_t uptime_ms = HAL_GetTick();
    uint32_t uptime_seconds = uptime_ms / 1000U;

    uint32_t hours = uptime_seconds / 3600u;
    uint32_t minutes = (uptime_seconds % 3600U) / 60U;
    uint32_t seconds = uptime_seconds % 60U;

    uint32_t system_clock_hz = HAL_RCC_GetSysClockFreq();
    uint32_t system_clock_mhz = system_clock_hz / 1000000U;
    GroundCalibrationState_t calibration_state =
        ground_calibration_get_state();

    printf("\r\n");
    printf("Mini Flight Computer Status\r\n");
    printf("---------------------------\r\n");
    printf("System:           %s\r\n",
           command_calibration_state_name(calibration_state));
    printf("Calibration:      %s (%lu%%)\r\n",
           ground_calibration_is_ready() ? "VALID" : "NOT READY",
           (unsigned long)(ground_calibration_get_progress() * 100.0f));
    printf("Ground mode:      %s\r\n",
           flight_is_ground_mode_confirmed() ?
               "CONFIRMED" : "LOCKED/UNKNOWN");
    printf("MCU:              STM32H743\r\n");
    printf("Firmware:         v%s\r\n", FIRMWARE_VERSION);
    printf("System clock:     %lu MHz\r\n",
           (unsigned long)system_clock_mhz);
    printf("Uptime:           %02lu:%02lu:%02lu\r\n",
           (unsigned long)hours,
           (unsigned long)minutes,
           (unsigned long)seconds);

    printf("HCLK:   %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());

    printf("PCLK1:  %lu Hz\r\n", (unsigned long) HAL_RCC_GetPCLK1Freq());

    printf("PCLK2:  %lu Hz\r\n",(unsigned long)HAL_RCC_GetPCLK2Freq());

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) // Checks if LSE ready flag is set
	{
		printf("HSE:    READY\r\n");
	}
	else
	{
		printf("HSE:    NOT READY\r\n");
	}

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET) // Checks if HSE ready flag is set
	{
		printf("LSE:    READY (%lu Hz nominal)\r\n",
			   (unsigned long)LSE_VALUE);
	}
	else
	{
		printf("LSE:    NOT READY\r\n");
	}

	print_rtc_datetime();

	printf("---------------------\r\n");
    printf("\r\n");


}

void command_sensor_status(void)
{
    bool imu_ok = imu_is_healthy();
    bool barometer_ok = barometer_is_healthy();
    bool magnetometer_ok = magnetometer_is_healthy();

    uint8_t healthy_count = 0U;
    uint8_t required_healthy_count = 0U;

    healthy_count += imu_ok ? 1U : 0U;
    healthy_count += barometer_ok ? 1U : 0U;
    healthy_count += magnetometer_ok ? 1U : 0U;
    required_healthy_count += imu_ok ? 1U : 0U;
    required_healthy_count += barometer_ok ? 1U : 0U;

    printf("\r\n");
    printf("Sensor Status\r\n");
    printf("---------------------------\r\n");

    printf("IMU:              %s\r\n",
           imu_ok ? "OK" : "ERROR/STALE");

    printf("Barometer:        %s\r\n",
           barometer_ok ? "OK" : "ERROR/STALE");

    printf("Magnetometer:     %s\r\n",
           magnetometer_ok ? "OK" : "DISABLED/ERROR");

    printf("---------------------------\r\n");
    printf("Healthy sensors:  %u / 3\r\n",
           (unsigned int)healthy_count);
    printf("Required sensors: %u / 2\r\n",
           (unsigned int)required_healthy_count);

    if (required_healthy_count == 2U)
    {
        printf("Overall status:   OK\r\n");
    }
    else if (required_healthy_count > 0U)
    {
        printf("Overall status:   DEGRADED\r\n");
    }
    else
    {
        printf("Overall status:   FAILURE\r\n");
    }

    printf("\r\n");
}


void command_reset(void)
{
    printf("\r\nResetting flight computer...\r\n");

    /*
     * Gives the blocking UART output time to finish before reset.
     * This is acceptable for a manual console command.
     */
    HAL_Delay(50U);

    NVIC_SystemReset();

    /*
     * Execution should never reach this point.
     */
    while (1)
    {
    }
}

void print_rtc_datetime(void)
{
    RTC_TimeTypeDef time = {0}; // this class can hold hours, mins, secs
    RTC_DateTypeDef date = {0}; // this class can hold day, month, date, year

    /*
     * Read time first, then date.
     * Reading the date unlocks the RTC shadow registers.
     */
    if (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("RTC time read failed\r\n");
        return;
    }

    if (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("RTC date read failed\r\n");
        return;
    }

    printf("Date/Time: 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
           date.Year,
           date.Month,
           date.Date,
           time.Hours,
           time.Minutes,
           time.Seconds);
}

void reset_cause_capture(void)
{
	// https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
	// p332
    saved_reset_causes = 0;
    saved_rcc_rsr = RCC->RSR;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) // power-on/power-down reset
    {
        saved_reset_causes |= RESET_CAUSE_POWER_ON;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) // Brown-out
    {
        saved_reset_causes |= RESET_CAUSE_BROWNOUT;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) //External reset pin triggered
    {
        saved_reset_causes |= RESET_CAUSE_PIN;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) // Software reset
    {
        saved_reset_causes |= RESET_CAUSE_SOFTWARE;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) // Independent Watchdog triggered
    {
        saved_reset_causes |= RESET_CAUSE_IWDG;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) // Window Watchdog triggered
    {
        saved_reset_causes |= RESET_CAUSE_WWDG;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST)) // Illegal power-state entry affecting Domain 1 (D1) or CPU1
    {
        saved_reset_causes |= RESET_CAUSE_LOW_POWER_D1;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR2RST)) //  An illegal entry into D2 DSTANDBY is attempted.
    // A sub-domain power sequence goes out of bounds, forcing a protective domain reset instead of a clean sleep.
    {
        saved_reset_causes |= RESET_CAUSE_LOW_POWER_D2;
    }

    /*
     * These flags describe CPU/domain resets. They may appear alongside
     * one of the more specific causes above.
     */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_CPURST))
    {
        saved_reset_causes |= RESET_CAUSE_CPU;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_D1RST))
    {
        saved_reset_causes |= RESET_CAUSE_D1_DOMAIN;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_D2RST))
    {
        saved_reset_causes |= RESET_CAUSE_D2_DOMAIN;
    }

    /*
     * Clear RCC reset flags so the next reset starts with fresh flags.
     */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void command_reset_cause(void)
{
    bool cause_found = false;

    printf("Last reset cause(s):\r\n");

    if (saved_reset_causes & RESET_CAUSE_POWER_ON)
    {
        printf("  - Power-on / power-down reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_BROWNOUT)
    {
        printf("  - Brownout reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_PIN)
    {
        printf("  - External NRST pin reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_SOFTWARE)
    {
        printf("  - Software reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_IWDG)
    {
        printf("  - Independent watchdog reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_WWDG)
    {
        printf("  - Window watchdog reset\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_LOW_POWER_D1)
    {
        printf("  - Illegal D1 low-power transition\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_LOW_POWER_D2)
    {
        printf("  - Illegal D2 low-power transition\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_CPU)
    {
        printf("  - CPU reset flag set\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_D1_DOMAIN)
    {
        printf("  - D1 domain reset flag set\r\n");
        cause_found = true;
    }

    if (saved_reset_causes & RESET_CAUSE_D2_DOMAIN)
    {
        printf("  - D2 domain reset flag set\r\n");
        cause_found = true;
    }

    if (!cause_found)
    {
        printf("  - Unknown, or reset flags were cleared before capture\r\n");
    }

    printf("Raw RCC RSR: 0x%08lX\r\n", // in order: pads output with leadings zeroes, sets min. width to 8 chars,
    									// specifies input as 'long', converts number to hexadecimal
           (unsigned long)saved_rcc_rsr);
}

static const char *command_calibration_state_name(
    GroundCalibrationState_t state)
{
    switch (state)
    {
        case GROUND_CALIBRATION_UNINITIALIZED:
            return "UNINITIALIZED";
        case GROUND_CALIBRATION_WARMUP:
            return "GROUND WARMUP";
        case GROUND_CALIBRATION_CALIBRATING:
            return "GROUND CALIBRATING";
        case GROUND_CALIBRATION_READY:
            return "GROUND READY";
        case GROUND_CALIBRATION_ABORTED:
            return "CALIBRATION ABORTED";
        case GROUND_CALIBRATION_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

static const char *command_calibration_failure_name(
    GroundCalibrationFailure_t failure)
{
    switch (failure)
    {
        case GROUND_CALIBRATION_FAILURE_NONE:
            return "none";
        case GROUND_CALIBRATION_FAILURE_MOTION:
            return "motion detected";
        case GROUND_CALIBRATION_FAILURE_IMU_ERROR:
            return "IMU error";
        case GROUND_CALIBRATION_FAILURE_BARO_ERROR:
            return "barometer error";
        case GROUND_CALIBRATION_FAILURE_NOISY:
            return "sample window too noisy";
        case GROUND_CALIBRATION_FAILURE_TIMEOUT:
            return "timeout";
        case GROUND_CALIBRATION_FAILURE_ABORTED:
            return "aborted";
        default:
            return "unknown";
    }
}

static bool command_calibration_is_busy(void)
{
    GroundCalibrationState_t state = ground_calibration_get_state();

    return (state == GROUND_CALIBRATION_WARMUP) ||
           (state == GROUND_CALIBRATION_CALIBRATING);
}

void command_imu_readout(void)
{
    ImuSample_t sample;
    float corrected_gyro_dps[3];

    if (!flight_get_latest_imu_sample(&sample))
    {
        printf("No valid IMU sample is available.\r\n");
        return;
    }

    ground_calibration_correct_gyro(
        sample.angular_rate_dps,
        corrected_gyro_dps);

    printf("IMU sample %lu | age %lu ms | fresh mask 0x%02X\r\n",
           (unsigned long)sample.sequence,
           (unsigned long)(HAL_GetTick() - sample.timestamp_ms),
           sample.fresh_mask);
    printf("  Accel [mg]: X=%8.2f Y=%8.2f Z=%8.2f\r\n",
           (double)sample.acceleration_mg[0],
           (double)sample.acceleration_mg[1],
           (double)sample.acceleration_mg[2]);
    printf("  Gyro raw [dps]: X=%8.3f Y=%8.3f Z=%8.3f\r\n",
           (double)sample.angular_rate_dps[0],
           (double)sample.angular_rate_dps[1],
           (double)sample.angular_rate_dps[2]);
    printf("  Gyro calibrated [dps]: X=%8.3f Y=%8.3f Z=%8.3f\r\n",
           (double)corrected_gyro_dps[0],
           (double)corrected_gyro_dps[1],
           (double)corrected_gyro_dps[2]);
}

void command_baro_readout(void)
{
    MS5611_Sample_t sample;
    float altitude_agl_m;

    if (!flight_get_latest_baro_sample(&sample))
    {
        printf("No valid barometer sample is available.\r\n");
        return;
    }

    printf("Barometer sample %lu | age %lu ms\r\n",
           (unsigned long)sample.sequence,
           (unsigned long)(HAL_GetTick() - sample.timestamp_ms));
    printf("  Pressure: %.2f Pa (%.2f hPa)\r\n",
           (double)sample.pressure_pa,
           (double)(sample.pressure_pa / 100.0f));
    printf("  Temperature: %.2f C\r\n", (double)sample.temperature_c);

    if (flight_get_latest_altitude_agl_m(&altitude_agl_m))
    {
        printf("  Altitude AGL: %.2f m\r\n", (double)altitude_agl_m);
    }
    else
    {
        printf("  Altitude AGL: invalid until ground calibration completes\r\n");
    }
}

void command_calibrate_ground(void)
{
    if (logger_is_active())
    {
        printf("Stop logging before restarting ground calibration.\r\n");
        return;
    }

    if (!flight_is_ground_mode_confirmed())
    {
        printf("Calibration is locked for flight/unknown mode.\r\n");
        printf("When safely stationary on the ground, use 'ground_confirm'.\r\n");
        return;
    }

    if (!flight_request_ground_calibration_restart())
    {
        printf("Ground calibration restart was rejected.\r\n");
        return;
    }

    printf("Ground calibration restart requested. Keep vehicle still.\r\n");
}

void command_ground_confirm(void)
{
    if (logger_is_active())
    {
        printf("Stop logging before confirming ground mode.\r\n");
        return;
    }

    flight_confirm_ground_mode();
    printf("Ground mode confirmed and calibration requested.\r\n");
    printf("Keep the vehicle stationary until calibration is READY.\r\n");
}

void command_flight_lock(void)
{
    flight_lock_ground_calibration();
    printf("Flight lock set; ground recalibration is inhibited across resets.\r\n");
}

void command_calibration_abort(void)
{
    if (!command_calibration_is_busy())
    {
        printf("Ground calibration is not active.\r\n");
        return;
    }

    flight_request_ground_calibration_abort();
    printf("Ground calibration abort requested.\r\n");
}

void command_calibration_status(void)
{
    GroundCalibrationState_t state = ground_calibration_get_state();
    GroundCalibrationFailure_t failure = ground_calibration_get_failure();
    const GroundCalibrationResult_t *result = ground_calibration_get_result();

    printf("\r\nGround Calibration\r\n");
    printf("---------------------------\r\n");
    printf("State:       %s\r\n", command_calibration_state_name(state));
    printf("Ground mode: %s\r\n",
           flight_is_ground_mode_confirmed() ?
               "CONFIRMED" : "LOCKED/UNKNOWN");
    printf("Progress:    %lu%%\r\n",
           (unsigned long)(ground_calibration_get_progress() * 100.0f));
    printf("Last reason: %s\r\n",
           command_calibration_failure_name(failure));

    if ((result != NULL) && result->valid)
    {
        printf("Gyro bias [dps]: X=%.4f Y=%.4f Z=%.4f\r\n",
               (double)result->gyro_bias_dps[0],
               (double)result->gyro_bias_dps[1],
               (double)result->gyro_bias_dps[2]);
        printf("Gravity reference [mg]: X=%.2f Y=%.2f Z=%.2f\r\n",
               (double)result->gravity_reference_mg[0],
               (double)result->gravity_reference_mg[1],
               (double)result->gravity_reference_mg[2]);
        printf("Ground pressure: %.2f Pa\r\n",
               (double)result->ground_pressure_pa);
        printf("Pressure sigma: %.2f Pa\r\n",
               (double)result->pressure_stddev_pa);
        printf("Samples: IMU=%lu Baro=%lu\r\n",
               (unsigned long)result->imu_sample_count,
               (unsigned long)result->baro_sample_count);
    }
}

static bool command_parse_u32(
    const char **text,
    uint32_t *value)
{
    char *end_pointer;
    unsigned long parsed_value;

    while ((**text == ' ') || (**text == '\t'))
    {
        (*text)++;
    }

    if (**text == '\0')
    {
        return false;
    }

    parsed_value = strtoul(*text, &end_pointer, 0);

    if (end_pointer == *text)
    {
        return false;
    }

    *value = (uint32_t)parsed_value;
    *text = end_pointer;

    return true;
}

void command_flash_read(const char *arguments)
{
    uint32_t address;
    uint32_t length;
    uint32_t offset = 0U;
    uint32_t chunk_length;
    uint8_t buffer[16];

    if (command_calibration_is_busy())
    {
        printf("Flash reads are disabled during ground calibration.\r\n");
        return;
    }

    if (!flash_available || !flash_is_ready())
    {
        printf("Flash is not available.\r\n");
        return;
    }

    if (!command_parse_u32(&arguments, &address) ||
        !command_parse_u32(&arguments, &length))
    {
        printf("Usage: flash_read <address> <length>\r\n");
        printf("Example: flash_read 0x1000 64\r\n");
        return;
    }

    if ((length == 0U) || (length > 256U))
    {
        printf("Length must be between 1 and 256 bytes.\r\n");
        return;
    }

    if (address > (UINT32_MAX - length))
    {
        printf("Invalid address range.\r\n");
        return;
    }

    if (logger_is_active())
    {
        printf("Stop logging before reading flash.\r\n");
        return;
    }

    while (offset < length)
    {
        chunk_length = length - offset;

        if (chunk_length > sizeof(buffer))
        {
            chunk_length = sizeof(buffer);
        }

        if (flash_read(
                address + offset,
                buffer,
                chunk_length) != HAL_OK)
        {
            printf(
                "Flash read failed at 0x%08lX.\r\n",
                (unsigned long)(address + offset)
            );
            return;
        }

        printf(
            "%08lX: ",
            (unsigned long)(address + offset)
        );

        for (uint32_t i = 0U; i < chunk_length; i++)
        {
            printf("%02X ", buffer[i]);
        }

        printf("\r\n");

        offset += chunk_length;
    }
}

void command_log_start(void)
{
    if (!logger_available)
    {
        printf("Logger is not available.\r\n");
        return;
    }

    if (!ground_calibration_is_ready())
    {
        printf("Logging requires a valid ground calibration.\r\n");
        return;
    }

    if (!imu_is_healthy() || !barometer_is_healthy())
    {
        printf("Logging requires healthy, fresh IMU and barometer data.\r\n");
        return;
    }

    /* Persist the flight lock before the session can begin. */
    flight_lock_ground_calibration();

    if (logger_start() != HAL_OK)
    {
        printf("Logger start failed.\r\n");
        printf("Ground calibration remains locked; use 'ground_confirm' "
               "only when safely on the ground.\r\n");
    }
}

void command_log_stop(void)
{
    if (!logger_available)
    {
        printf("Logger is not available.\r\n");
        return;
    }

    if (logger_stop() != HAL_OK)
    {
        printf("Logger stop failed.\r\n");
    }
}

void command_log_clear(void)
{
    if (!flash_available || !flash_is_ready())
    {
        printf("External flash is not available.\r\n");
        return;
    }

    if (command_calibration_is_busy())
    {
        printf("Log erase is disabled during ground calibration.\r\n");
        return;
    }

    if (logger_is_active())
    {
        printf("Stop logging before clearing records.\r\n");
        return;
    }

    printf("Clearing logged records...\r\n");

    if (logger_erase() != HAL_OK)
    {
        printf("Failed to clear logged records.\r\n");
        return;
    }

    /* Also recovers a logger disabled by legacy/corrupt format metadata. */
    logger_available = true;

    printf("Logged records cleared.\r\n");
}

void command_log_dump(const char *arguments)
{
    FlightLogRecord_t record;

    uint32_t requested_count = 10U;
    uint32_t stored_count;
    uint32_t dump_count;

    if (command_calibration_is_busy())
    {
        printf("Log dump is disabled during ground calibration.\r\n");
        return;
    }

    if (!logger_available)
    {
        printf("Logger is not available.\r\n");
        return;
    }

    if (logger_is_active())
    {
        printf("Stop logging before dumping records.\r\n");
        return;
    }

    while ((*arguments == ' ') || (*arguments == '\t'))
    {
        arguments++;
    }

    if (*arguments != '\0')
    {
        if (!command_parse_u32(
                &arguments,
                &requested_count))
        {
            printf("Usage: log_dump [count]\r\n");
            return;
        }
    }

    if (requested_count == 0U)
    {
        printf("Count must be greater than zero.\r\n");
        return;
    }

    if (requested_count > 100U)
    {
        requested_count = 100U;
        printf("Dump limited to 100 records.\r\n");
    }

    stored_count = logger_get_record_count();
    dump_count = requested_count;

    if (dump_count > stored_count)
    {
        dump_count = stored_count;
    }

    if (dump_count == 0U)
    {
        printf("No records are stored.\r\n");
        return;
    }

    printf("\r\n");
    printf("\r\n");
    printf("Log Dump\r\n");
    printf("Records stored: %lu\r\n",
           (unsigned long)stored_count);
    printf("Records shown:  %lu\r\n",
           (unsigned long)dump_count);
    printf("Log format:     v%u\r\n",
           (unsigned int)logger_get_format_version());
    printf("========================================\r\n");

    for (uint32_t index = 0U;
         index < dump_count;
         index++)
    {
        if (logger_read_record(index, &record) != HAL_OK)
        {
            printf(
                "Dump stopped: failed to read record %lu.\r\n",
                (unsigned long)index
            );

            return;
        }

        printf(
            "Record %lu | IMU time: %.3f s | Baro time: %.3f s | "
            "valid=0x%08lX\r\n",
            (unsigned long)index,
            (double)record.timestamp_ms / 1000.0,
            (double)record.barometer_timestamp_ms / 1000.0,
            (unsigned long)record.validity_flags
        );

        printf(
            "  Accel [mg]: X=%8.2f  Y=%8.2f  Z=%8.2f\r\n",
            (double)record.acceleration_x,
            (double)record.acceleration_y,
            (double)record.acceleration_z
        );

        printf(
            "  Gyro [dps]: X=%8.2f  Y=%8.2f  Z=%8.2f\r\n",
            (double)record.angular_rate_x,
            (double)record.angular_rate_y,
            (double)record.angular_rate_z
        );

        printf(
            "  Baro:   Pressure=%10.2f Pa\r\n",
            (double)record.pressure_pa
        );

        printf(
            "          Temperature=%6.2f C  Altitude AGL=%7.2f m\r\n",
            (double)record.temperature_c,
            (double)record.altitude_m
        );

        printf("----------------------------------------\r\n");
    }

    printf("End of log dump.\r\n\r\n");
}
