/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "commands.h"
#include <stdio.h>
#include "imu.h"
#include "ms5611.h"
#include "mag.h"
#include "flash.h"
#include "logger.h"
#include "sensors.h"
#include "ground_calibration.h"
#include "flight_app.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define COMMAND_BUFFER_SIZE 64U
#define RTC_INIT_MARKER  0xA5A55A5AU // Arbitrary value for a flag
#define RTC_FLIGHT_LOCK_MARKER 0x464C544CUL /* "FLTL" */

static uint8_t uart_rx_byte;
static char command_buffer[COMMAND_BUFFER_SIZE];
static volatile uint32_t command_index = 0U;
static volatile uint8_t command_ready = 0U;

#define IMU_POLL_INTERVAL_MS 10U
#define BAROMETER_POLL_INTERVAL_MS 2U
#define IMU_PAIR_MAX_SKEW_MS 25U
#define BAROMETER_LOG_MAX_AGE_MS 100U
#define SENSOR_REINIT_INTERVAL_MS 1000U
#define IMU_NO_DATA_TIMEOUT_MS 500U
#define BAROMETER_NO_DATA_TIMEOUT_MS 500U

static ImuSample_t latest_imu_sample;
static ImuSample_t pending_imu_pair;
static MS5611_Sample_t latest_baro_sample;
static float latest_altitude_agl_m = NAN;
static bool latest_imu_sample_valid = false;
static bool latest_baro_sample_valid = false;
static bool latest_altitude_valid = false;
static bool imu_available = false;
static bool barometer_available = false;
static uint8_t pending_imu_fresh_mask = 0U;
static uint32_t pending_accel_timestamp_ms = 0U;
static uint32_t pending_gyro_timestamp_ms = 0U;
static uint32_t last_imu_pair_timestamp_ms = 0U;
static bool have_last_imu_pair = false;
static uint32_t last_baro_sample_timestamp_ms = 0U;
static bool have_last_baro_sample = false;

static volatile uint8_t calibration_restart_requested = 0U;
static volatile uint8_t calibration_abort_requested = 0U;
static bool ground_mode_confirmed = false;

#define FLASH_ENABLE_SELF_TEST 0U

bool flash_available = false;
bool logger_available = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 				DEBUGGING FUNCTIONS
//---------------------------------------------------------

// Redirect printf to UART1
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  reset_cause_capture(); // Check RCC flags for reasons for reset

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /*
   * Disable stdout buffering so printf output is sent to UART1 immediately.
   */
  setvbuf(stdout, NULL, _IONBF, 0);

  // Accelerometer + Gyroscope
  imu_available = (lsm6dso32x_init() == HAL_OK);
  sensor_health_set_initialized(SENSOR_ID_IMU, imu_available);

  if (!imu_available)
  {
    printf("[MAIN] IMU initialization failed.\r\n");
  }

  // Magnetometer
//  printf("\r\nInitializing magnetometer\r\n");
//  iis2mdctr_init();
//  printf("Returned from magnetometer initialization\r\n");

  // Barometer
  barometer_available = (MS5611_Init(&hi2c2) == HAL_OK);
  sensor_health_set_initialized(SENSOR_ID_BAROMETER, barometer_available);

  if (!barometer_available)
  {
    printf("[MAIN] Barometer initialization or PROM CRC check failed.\r\n");
  }

  /* Magnetometer is intentionally optional until its application path is enabled. */
  sensor_health_set_initialized(SENSOR_ID_MAGNETOMETER, false);

  // Flash
  flash_logger_init();

  uint32_t last_imu_poll_tick = HAL_GetTick();
  uint32_t last_baro_poll_tick = HAL_GetTick();
  uint32_t last_imu_activity_tick = last_imu_poll_tick;
  uint32_t last_baro_activity_tick = last_baro_poll_tick;
  uint32_t last_imu_reinit_attempt_tick = last_imu_poll_tick;
  uint32_t last_baro_reinit_attempt_tick = last_baro_poll_tick;

  /*
   * Ground confirmation is deliberately boot-local and single-use. Never
   * infer that the vehicle is still on the ground from state saved before a
   * reset; an in-flight reset must come up calibration-locked.
   */
  ground_mode_confirmed = false;
  printf("[CAL] Recalibration inhibited after reset.\r\n");
  printf("[CAL] When safely stationary on the ground, use 'ground_confirm'.\r\n");

  printf("\r\nMini Flight Computer Console\r\n");
  printf("Type 'help' for available commands.\r\n");
  printf("> ");

  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now_ms = HAL_GetTick();
      bool fresh_paired_imu = false;
      bool required_sensor_recovered = false;
      ImuSample_t imu_sample_for_log = {0};

      if (command_ready != 0U)
      {
          command_process(command_buffer);
          command_ready = 0U;
          printf("> ");
      }

      if (calibration_abort_requested != 0U)
      {
          calibration_abort_requested = 0U;
          ground_calibration_abort();
      }

      if (calibration_restart_requested != 0U)
      {
          calibration_restart_requested = 0U;

          if (!ground_mode_confirmed)
          {
              printf("[CAL] Restart rejected: ground mode is not confirmed.\r\n");
          }
          else if (logger_is_active())
          {
              printf("[CAL] Stop logging before restarting calibration.\r\n");
          }
          else if (!imu_available)
          {
              ground_calibration_fault(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
          }
          else if (!barometer_available)
          {
              ground_calibration_fault(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
          }
          else
          {
              latest_altitude_valid = false;
              latest_altitude_agl_m = NAN;
              ground_calibration_restart(now_ms);
              printf("[CAL] Ground calibration restarted. Keep vehicle still.\r\n");
          }
      }

      if (!imu_available &&
          ((now_ms - last_imu_reinit_attempt_tick) >=
           SENSOR_REINIT_INTERVAL_MS))
      {
          last_imu_reinit_attempt_tick = now_ms;

          if (lsm6dso32x_init() == HAL_OK)
          {
              imu_available = true;
              latest_imu_sample_valid = false;
              pending_imu_fresh_mask = 0U;
              pending_accel_timestamp_ms = 0U;
              pending_gyro_timestamp_ms = 0U;
              have_last_imu_pair = false;
              sensor_health_set_initialized(SENSOR_ID_IMU, true);
              last_imu_activity_tick = HAL_GetTick();
              last_imu_poll_tick = last_imu_activity_tick;
              required_sensor_recovered = true;
              printf("[MAIN] IMU reinitialized.\r\n");
          }
      }

      if (!barometer_available &&
          ((now_ms - last_baro_reinit_attempt_tick) >=
           SENSOR_REINIT_INTERVAL_MS))
      {
          last_baro_reinit_attempt_tick = now_ms;

          if (MS5611_Init(&hi2c2) == HAL_OK)
          {
              barometer_available = true;
              latest_baro_sample_valid = false;
              latest_altitude_valid = false;
              latest_altitude_agl_m = NAN;
              sensor_health_set_initialized(SENSOR_ID_BAROMETER, true);
              have_last_baro_sample = false;
              last_baro_activity_tick = HAL_GetTick();
              last_baro_poll_tick = last_baro_activity_tick;
              required_sensor_recovered = true;
              printf("[MAIN] Barometer reinitialized.\r\n");
          }
      }

      now_ms = HAL_GetTick();

      if (required_sensor_recovered &&
          imu_available &&
          barometer_available &&
          ground_mode_confirmed &&
          !logger_is_active() &&
          (ground_calibration_get_state() == GROUND_CALIBRATION_FAULT) &&
          ((ground_calibration_get_failure() ==
            GROUND_CALIBRATION_FAILURE_IMU_ERROR) ||
           (ground_calibration_get_failure() ==
            GROUND_CALIBRATION_FAILURE_BARO_ERROR)))
      {
          ground_calibration_restart(now_ms);
          printf("[CAL] Required sensors recovered; calibration restarted.\r\n");
      }

      ground_calibration_update(now_ms);

      if (imu_available &&
          ((now_ms - last_imu_poll_tick) >= IMU_POLL_INTERVAL_MS))
      {
          ImuSample_t sensor_sample;
          ImuSample_t sample;
          ImuPollStatus_t poll_status;

          /* Skip missed periods instead of issuing catch-up bursts. */
          last_imu_poll_tick = now_ms;
          poll_status = lsm6dso32x_poll(&sensor_sample);

          if (poll_status == IMU_POLL_NEW_DATA)
          {
              /* Calibration, health, commands, and logs all use body axes. */
              imu_transform_sensor_to_body(&sensor_sample, &sample);
              latest_imu_sample = sample;
              latest_imu_sample_valid = true;

              /*
               * Accel and gyro DRDY bits are not required to assert in the
               * same poll. The driver preserves the latest counterpart, so
               * publish only after both have advanced within one nominal
               * 52-Hz sample period.
               */
              pending_imu_pair = sample;

              if ((sample.fresh_mask & IMU_SAMPLE_ACCEL_FRESH) != 0U)
              {
                  pending_accel_timestamp_ms = sample.timestamp_ms;
                  pending_imu_fresh_mask |= IMU_SAMPLE_ACCEL_FRESH;
              }

              if ((sample.fresh_mask & IMU_SAMPLE_GYRO_FRESH) != 0U)
              {
                  pending_gyro_timestamp_ms = sample.timestamp_ms;
                  pending_imu_fresh_mask |= IMU_SAMPLE_GYRO_FRESH;
              }

              if ((pending_imu_fresh_mask &
                   (IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH)) ==
                  (IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH))
              {
                  int32_t signed_skew =
                      (int32_t)(pending_accel_timestamp_ms -
                                pending_gyro_timestamp_ms);
                  uint32_t pair_skew_ms = (signed_skew >= 0)
                                              ? (uint32_t)signed_skew
                                              : (pending_gyro_timestamp_ms -
                                                 pending_accel_timestamp_ms);

                  if (pair_skew_ms > IMU_PAIR_MAX_SKEW_MS)
                  {
                      GroundCalibrationState_t calibration_state =
                          ground_calibration_get_state();

                      sensor_health_record_failure(SENSOR_ID_IMU);

                      if (calibration_state ==
                          GROUND_CALIBRATION_CALIBRATING)
                      {
                          ground_calibration_process_imu(NULL);
                      }

                      /* Retain only the newer half for the next pairing. */
                      if (signed_skew > 0)
                      {
                          pending_imu_fresh_mask = IMU_SAMPLE_ACCEL_FRESH;
                          pending_gyro_timestamp_ms = 0U;
                      }
                      else
                      {
                          pending_imu_fresh_mask = IMU_SAMPLE_GYRO_FRESH;
                          pending_accel_timestamp_ms = 0U;
                      }
                  }
                  else
                  {
                      bool cadence_valid =
                          !have_last_imu_pair ||
                          ((pending_imu_pair.timestamp_ms -
                            last_imu_pair_timestamp_ms) <=
                           GROUND_CALIBRATION_MAX_IMU_GAP_MS);

                      pending_imu_pair.fresh_mask =
                          IMU_SAMPLE_ACCEL_FRESH | IMU_SAMPLE_GYRO_FRESH;
                      pending_imu_pair.timestamp_ms =
                          (signed_skew >= 0)
                              ? pending_accel_timestamp_ms
                              : pending_gyro_timestamp_ms;

                      if (cadence_valid)
                      {
                          sensor_health_record_success(
                              SENSOR_ID_IMU,
                              pending_imu_pair.timestamp_ms);
                          last_imu_activity_tick =
                              pending_imu_pair.timestamp_ms;
                      }
                      else
                      {
                          sensor_health_record_failure(SENSOR_ID_IMU);

                          if (ground_calibration_get_state() ==
                              GROUND_CALIBRATION_CALIBRATING)
                          {
                              ground_calibration_process_imu(NULL);
                          }
                      }

                      last_imu_pair_timestamp_ms =
                          pending_imu_pair.timestamp_ms;
                      have_last_imu_pair = true;
                      ground_calibration_process_imu(&pending_imu_pair);
                      imu_sample_for_log = pending_imu_pair;
                      fresh_paired_imu = true;
                      pending_imu_fresh_mask = 0U;
                      pending_accel_timestamp_ms = 0U;
                      pending_gyro_timestamp_ms = 0U;
                  }
              }
          }
          else if (poll_status == IMU_POLL_ERROR)
          {
              SensorHealth_t health;
              GroundCalibrationState_t calibration_state =
                  ground_calibration_get_state();

              sensor_health_record_failure(SENSOR_ID_IMU);
              pending_imu_fresh_mask = 0U;
              pending_accel_timestamp_ms = 0U;
              pending_gyro_timestamp_ms = 0U;
              health = sensor_health_get(SENSOR_ID_IMU);

              if (calibration_state == GROUND_CALIBRATION_CALIBRATING)
              {
                  /* A missing frame invalidates the contiguous sample window. */
                  ground_calibration_process_imu(NULL);
              }

              if (health.consecutive_failures >= 3U)
              {
                  imu_available = false;
                  latest_imu_sample_valid = false;
                  have_last_imu_pair = false;
                  sensor_health_set_initialized(SENSOR_ID_IMU, false);
                  last_imu_reinit_attempt_tick = HAL_GetTick();
                  printf("[MAIN] IMU offline; periodic reinitialization enabled.\r\n");

                  if ((calibration_state == GROUND_CALIBRATION_WARMUP) ||
                      (calibration_state == GROUND_CALIBRATION_CALIBRATING) ||
                      (ground_mode_confirmed &&
                       (calibration_state == GROUND_CALIBRATION_READY)))
                  {
                      ground_calibration_fault(
                          GROUND_CALIBRATION_FAILURE_IMU_ERROR);
                  }
              }
          }
      }

      if (barometer_available &&
          ((now_ms - last_baro_poll_tick) >= BAROMETER_POLL_INTERVAL_MS))
      {
          MS5611_Sample_t sample;
          MS5611_PollStatus_t poll_status;

          last_baro_poll_tick = now_ms;
          poll_status = MS5611_Poll(&hi2c2, &sample);

          if (poll_status == MS5611_POLL_NEW_DATA)
          {
              bool cadence_valid =
                  !have_last_baro_sample ||
                  ((sample.timestamp_ms - last_baro_sample_timestamp_ms) <=
                   GROUND_CALIBRATION_MAX_BARO_GAP_MS);

              latest_baro_sample = sample;
              latest_baro_sample_valid = true;

              if (cadence_valid)
              {
                  sensor_health_record_success(
                      SENSOR_ID_BAROMETER,
                      sample.timestamp_ms);
                  last_baro_activity_tick = sample.timestamp_ms;
              }
              else
              {
                  sensor_health_record_failure(SENSOR_ID_BAROMETER);
              }

              last_baro_sample_timestamp_ms = sample.timestamp_ms;
              have_last_baro_sample = true;
              ground_calibration_process_baro(&sample);

              latest_altitude_valid = ground_calibration_altitude_agl_m(
                  sample.pressure_pa,
                  &latest_altitude_agl_m);

              if (!latest_altitude_valid)
              {
                  latest_altitude_agl_m = NAN;
              }
          }
          else if (poll_status == MS5611_POLL_ERROR)
          {
              SensorHealth_t health;
              GroundCalibrationState_t calibration_state =
                  ground_calibration_get_state();

              sensor_health_record_failure(SENSOR_ID_BAROMETER);
              latest_baro_sample_valid = false;
              latest_altitude_valid = false;
              latest_altitude_agl_m = NAN;
              health = sensor_health_get(SENSOR_ID_BAROMETER);

              if (calibration_state == GROUND_CALIBRATION_CALIBRATING)
              {
                  ground_calibration_process_baro(NULL);
              }

              if (health.consecutive_failures >= 3U)
              {
                  barometer_available = false;
                  have_last_baro_sample = false;
                  sensor_health_set_initialized(SENSOR_ID_BAROMETER, false);
                  last_baro_reinit_attempt_tick = HAL_GetTick();
                  printf("[MAIN] Barometer offline; periodic reinitialization enabled.\r\n");

                  if ((calibration_state == GROUND_CALIBRATION_WARMUP) ||
                      (calibration_state == GROUND_CALIBRATION_CALIBRATING) ||
                      (ground_mode_confirmed &&
                       (calibration_state == GROUND_CALIBRATION_READY)))
                  {
                      ground_calibration_fault(
                          GROUND_CALIBRATION_FAILURE_BARO_ERROR);
                  }
              }
          }
      }

      if (imu_available &&
          ((HAL_GetTick() - last_imu_activity_tick) >
           IMU_NO_DATA_TIMEOUT_MS))
      {
          GroundCalibrationState_t calibration_state =
              ground_calibration_get_state();

          imu_available = false;
          latest_imu_sample_valid = false;
          pending_imu_fresh_mask = 0U;
          pending_accel_timestamp_ms = 0U;
          pending_gyro_timestamp_ms = 0U;
          have_last_imu_pair = false;
          sensor_health_set_initialized(SENSOR_ID_IMU, false);
          last_imu_reinit_attempt_tick = HAL_GetTick();
          printf("[MAIN] IMU data timeout; periodic reinitialization enabled.\r\n");

          if ((calibration_state == GROUND_CALIBRATION_WARMUP) ||
              (calibration_state == GROUND_CALIBRATION_CALIBRATING) ||
              (ground_mode_confirmed &&
               (calibration_state == GROUND_CALIBRATION_READY)))
          {
              ground_calibration_fault(GROUND_CALIBRATION_FAILURE_IMU_ERROR);
          }
      }

      if (barometer_available &&
          ((HAL_GetTick() - last_baro_activity_tick) >
           BAROMETER_NO_DATA_TIMEOUT_MS))
      {
          GroundCalibrationState_t calibration_state =
              ground_calibration_get_state();

          barometer_available = false;
          have_last_baro_sample = false;
          latest_baro_sample_valid = false;
          latest_altitude_valid = false;
          latest_altitude_agl_m = NAN;
          sensor_health_set_initialized(SENSOR_ID_BAROMETER, false);
          last_baro_reinit_attempt_tick = HAL_GetTick();
          printf("[MAIN] Barometer data timeout; periodic reinitialization enabled.\r\n");

          if ((calibration_state == GROUND_CALIBRATION_WARMUP) ||
              (calibration_state == GROUND_CALIBRATION_CALIBRATING) ||
              (ground_mode_confirmed &&
               (calibration_state == GROUND_CALIBRATION_READY)))
          {
              ground_calibration_fault(GROUND_CALIBRATION_FAILURE_BARO_ERROR);
          }
      }

      if (ground_mode_confirmed && ground_calibration_is_ready())
      {
          /* A confirmation authorizes one completed calibration only. */
          flight_lock_ground_calibration();
          printf("[CAL] Calibration READY; ground confirmation consumed.\r\n");
      }

      /* Calibration can complete on an IMU frame between barometer frames. */
      if (!latest_altitude_valid &&
          latest_baro_sample_valid &&
          ground_calibration_is_ready())
      {
          latest_altitude_valid = ground_calibration_altitude_agl_m(
              latest_baro_sample.pressure_pa,
              &latest_altitude_agl_m);
      }

      /* Do not carry an old pressure/AGL value into a newer IMU log frame. */
      if (latest_baro_sample_valid &&
          ((HAL_GetTick() - latest_baro_sample.timestamp_ms) >
           BAROMETER_LOG_MAX_AGE_MS))
      {
          latest_baro_sample_valid = false;
          latest_altitude_valid = false;
          latest_altitude_agl_m = NAN;
      }

      /* Check required sources before appending a record from this frame. */
      if (logger_is_active() &&
          (!imu_is_healthy() ||
           !barometer_is_healthy() ||
           !latest_baro_sample_valid ||
           !latest_altitude_valid))
      {
          printf("[MAIN] Required sensor data became invalid; logging stopped.\r\n");
          (void)logger_stop();
      }

      if (fresh_paired_imu && logger_available && logger_is_active())
      {
          FlightLogRecord_t record = {0};
          float corrected_gyro_dps[3];

          ground_calibration_correct_gyro(
              imu_sample_for_log.angular_rate_dps,
              corrected_gyro_dps);

          record.timestamp_ms = imu_sample_for_log.timestamp_ms;
          record.barometer_timestamp_ms = latest_baro_sample.timestamp_ms;
          record.validity_flags = FLIGHT_LOG_VALID_IMU |
                                  FLIGHT_LOG_VALID_BAROMETER |
                                  FLIGHT_LOG_VALID_ALTITUDE_AGL |
                                  FLIGHT_LOG_GYRO_BIAS_CORRECTED;
          record.acceleration_x = imu_sample_for_log.acceleration_mg[0];
          record.acceleration_y = imu_sample_for_log.acceleration_mg[1];
          record.acceleration_z = imu_sample_for_log.acceleration_mg[2];
          record.angular_rate_x = corrected_gyro_dps[0];
          record.angular_rate_y = corrected_gyro_dps[1];
          record.angular_rate_z = corrected_gyro_dps[2];

          record.pressure_pa = latest_baro_sample.pressure_pa;
          record.temperature_c = latest_baro_sample.temperature_c;
          record.altitude_m = latest_altitude_agl_m;

          if (logger_append(&record) != HAL_OK)
          {
              printf(
                  "[MAIN] Failed to store log record. "
                  "Logging will be stopped.\r\n"
              );

              (void)logger_stop();
          }
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30910D22;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C0ECFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10C0ECFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 19;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  // When the backup register found in HAL_RTCEx is equal to the macro, it means the date has been set
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == RTC_INIT_MARKER)
  {
	  /*
	   * RTC was already initialized.
	   * Return before CubeMX resets the date and time.
	   */
	  return;
  }

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_INIT_MARKER); // Writes marker into backup register

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD13 PD0 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool flight_request_ground_calibration_restart(void)
{
    if (!ground_mode_confirmed)
    {
        return false;
    }

    calibration_restart_requested = 1U;
    return true;
}

void flight_request_ground_calibration_abort(void)
{
    calibration_abort_requested = 1U;
}

void flight_confirm_ground_mode(void)
{
    ground_mode_confirmed = true;
    calibration_restart_requested = 1U;
}

void flight_lock_ground_calibration(void)
{
    /*
     * The backup-domain latch survives MCU resets. Future arming paths must
     * call this boundary before enabling flight outputs, even if logging is
     * not used as the arming action.
    */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_FLIGHT_LOCK_MARKER);
    ground_mode_confirmed = false;

    if ((ground_calibration_get_state() == GROUND_CALIBRATION_WARMUP) ||
        (ground_calibration_get_state() == GROUND_CALIBRATION_CALIBRATING))
    {
        ground_calibration_abort();
    }
}

bool flight_is_ground_mode_confirmed(void)
{
    return ground_mode_confirmed;
}

bool flight_get_latest_imu_sample(ImuSample_t *sample)
{
    if ((sample == NULL) || !latest_imu_sample_valid)
    {
        return false;
    }

    *sample = latest_imu_sample;
    return true;
}

bool flight_get_latest_baro_sample(MS5611_Sample_t *sample)
{
    if ((sample == NULL) || !latest_baro_sample_valid)
    {
        return false;
    }

    *sample = latest_baro_sample;
    return true;
}

bool flight_get_latest_altitude_agl_m(float *altitude_m)
{
    if ((altitude_m == NULL) || !latest_altitude_valid)
    {
        return false;
    }

    *altitude_m = latest_altitude_agl_m;
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    /*
     * Enter may arrive as:
     * '\r'       carriage return
     * '\n'       line feed
     * "\r\n"     both
     */
    if ((uart_rx_byte == '\r') || (uart_rx_byte == '\n'))
    {
        if ((command_index > 0U) && (command_ready == 0U))
        {
            command_buffer[command_index] = '\0';
            command_ready = 1U;
            command_index = 0U;
        }
    }
    else if ((uart_rx_byte == '\b') || (uart_rx_byte == 0x7FU))
    {
        /* Handle Backspace and Delete. */
        if ((command_index > 0U) && (command_ready == 0U))
        {
            command_index--;
        }
    }
    else
    {
        if ((command_ready == 0U) &&
            (command_index < (COMMAND_BUFFER_SIZE - 1U)))
        {
            command_buffer[command_index] = (char)uart_rx_byte;
            command_index++;
        }
    }

    /*
     * Rearm reception so the next character can be received.
     */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
