#include "logger.h"
#include "flash.h"

#include <stdio.h>

#define LOGGER_REGION_START         EXT_FLASH_LOG_START_ADDRESS
#define LOGGER_REGION_END           EXT_FLASH_LOG_END_ADDRESS
#define LOGGER_SECTOR_SIZE          EXT_FLASH_SECTOR_SIZE_4K

#define LOGGER_RECORD_SIZE          ((uint32_t)sizeof(FlightLogRecord_t))

#define LOGGER_RECORDS_PER_SECTOR   \
    (LOGGER_SECTOR_SIZE / LOGGER_RECORD_SIZE)

#define LOGGER_SECTOR_COUNT         \
    ((LOGGER_REGION_END - LOGGER_REGION_START) / LOGGER_SECTOR_SIZE)

#define LOGGER_MAX_RECORDS          \
    (LOGGER_RECORDS_PER_SECTOR * LOGGER_SECTOR_COUNT)

extern bool flash_available;
extern bool logger_available;


static uint32_t logger_write_address = LOGGER_REGION_START;
static uint32_t logger_record_count = 0U;
static bool logger_initialized = false;
static bool logger_full = false;
static bool logger_active = false;

static bool logger_record_is_erased(
    const FlightLogRecord_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;

    for (uint32_t i = 0U;
         i < sizeof(FlightLogRecord_t);
         i++)
    {
        if (bytes[i] != 0xFFU)
        {
            return false;
        }
    }

    return true;
}

static uint32_t logger_get_record_address(uint32_t record_index)
{
    uint32_t sector_index;
    uint32_t slot_index;

    sector_index =
        record_index / LOGGER_RECORDS_PER_SECTOR;

    slot_index =
        record_index % LOGGER_RECORDS_PER_SECTOR;

    return LOGGER_REGION_START +
           (sector_index * LOGGER_SECTOR_SIZE) +
           (slot_index * LOGGER_RECORD_SIZE);
}


HAL_StatusTypeDef logger_init(void)
{
    FlightLogRecord_t record;
    uint32_t address;

    logger_initialized = false;
    logger_active = false;
    logger_full = false;
    logger_record_count = 0U;
    logger_write_address = LOGGER_REGION_START;

    if (!flash_is_ready())
    {
        printf(
            "[LOGGER] Initialization failed: flash is not ready.\r\n"
        );

        return HAL_ERROR;
    }

    printf("[LOGGER] Scanning existing records...\r\n");

    for (uint32_t index = 0U;
         index < LOGGER_MAX_RECORDS;
         index++)
    {
        address = logger_get_record_address(index);

        if (flash_read(
                address,
                (uint8_t *)&record,
                sizeof(FlightLogRecord_t)) != HAL_OK)
        {
            printf(
                "[LOGGER] Scan failed at record %lu, "
                "address 0x%08lX.\r\n",
                (unsigned long)index,
                (unsigned long)address
            );

            return HAL_ERROR;
        }

        if (logger_record_is_erased(&record))
        {
            logger_record_count = index;
            logger_write_address = address;
            logger_initialized = true;

            printf("[LOGGER] Initialization complete.\r\n");

            printf(
                "[LOGGER] Existing records: %lu\r\n",
                (unsigned long)logger_record_count
            );

            printf(
                "[LOGGER] Next write address: 0x%08lX\r\n",
                (unsigned long)logger_write_address
            );

            printf(
                "[LOGGER] Record size: %lu bytes\r\n",
                (unsigned long)LOGGER_RECORD_SIZE
            );

            printf(
                "[LOGGER] Records per sector: %lu\r\n",
                (unsigned long)LOGGER_RECORDS_PER_SECTOR
            );

            printf(
                "[LOGGER] Maximum records: %lu\r\n",
                (unsigned long)LOGGER_MAX_RECORDS
            );

            return HAL_OK;
        }
    }

    logger_record_count = LOGGER_MAX_RECORDS;
    logger_write_address = LOGGER_REGION_END;
    logger_full = true;
    logger_initialized = true;

    printf(
        "[LOGGER] Initialization complete: logging region is full.\r\n"
    );

    return HAL_OK;
}


HAL_StatusTypeDef logger_append(
    const FlightLogRecord_t *record)
{
    uint32_t record_address;
    uint32_t slot_index;

    if (!logger_initialized)
    {
        printf("[LOGGER] Append rejected: logger is not initialized.\r\n");
        return HAL_ERROR;
    }

    if (!logger_active)
    {
        return HAL_BUSY;
    }

    if (record == NULL)
    {
        printf("[LOGGER] Append rejected: record is NULL.\r\n");
        return HAL_ERROR;
    }

    if (logger_full ||
        (logger_record_count >= LOGGER_MAX_RECORDS))
    {
        logger_full = true;

        printf("[LOGGER] Append rejected: logging region is full.\r\n");
        return HAL_ERROR;
    }

    record_address =
        logger_get_record_address(logger_record_count);

    slot_index =
        logger_record_count % LOGGER_RECORDS_PER_SECTOR;

    /*
     * When advancing into a new sector, erase that sector before
     * writing its first record.
     *
     * Do not erase the first sector automatically because it may
     * contain records recovered during logger_init().
     */
    if ((slot_index == 0U) &&
        (logger_record_count != 0U))
    {
        printf(
            "[LOGGER] Preparing sector at 0x%08lX.\r\n",
            (unsigned long)record_address
        );

        if (flash_erase_4k(record_address) != HAL_OK)
        {
            printf(
                "[LOGGER] Failed to erase sector at 0x%08lX.\r\n",
                (unsigned long)record_address
            );

            return HAL_ERROR;
        }
    }

    if (flash_write(
            record_address,
            (const uint8_t *)record,
            sizeof(FlightLogRecord_t)) != HAL_OK)
    {
        printf(
            "[LOGGER] Record write failed at 0x%08lX.\r\n",
            (unsigned long)record_address
        );

        return HAL_ERROR;
    }

    logger_record_count++;

    if (logger_record_count >= LOGGER_MAX_RECORDS)
    {
        logger_write_address = LOGGER_REGION_END;
        logger_full = true;
    }
    else
    {
        logger_write_address =
            logger_get_record_address(logger_record_count);
    }

    return HAL_OK;
}

uint32_t logger_get_write_address(void)
{
    return logger_write_address;
}

uint32_t logger_get_record_count(void)
{
    return logger_record_count;
}

bool logger_is_full(void)
{
    return logger_full;
}

HAL_StatusTypeDef logger_read_record(
    uint32_t record_index,
    FlightLogRecord_t *record)
{
    uint32_t record_address;

    if (!logger_initialized)
    {
        printf("[LOGGER] Read rejected: logger is not initialized.\r\n");
        return HAL_ERROR;
    }

    if (record == NULL)
    {
        printf("[LOGGER] Read rejected: output record is NULL.\r\n");
        return HAL_ERROR;
    }

    if (record_index >= logger_record_count)
    {
        printf(
            "[LOGGER] Invalid record index: %lu. "
            "Record count: %lu.\r\n",
            (unsigned long)record_index,
            (unsigned long)logger_record_count
        );

        return HAL_ERROR;
    }

    record_address =
        logger_get_record_address(record_index);

    if (flash_read(
            record_address,
            (uint8_t *)record,
            sizeof(FlightLogRecord_t)) != HAL_OK)
    {
        printf(
            "[LOGGER] Read failed at 0x%08lX.\r\n",
            (unsigned long)record_address
        );

        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef logger_erase(void)
{
    if (!flash_is_ready())
    {
        printf("[LOGGER] Erase failed: flash is not ready.\r\n");
        return HAL_ERROR;
    }

    logger_active = false;

    if (flash_erase_4k(LOGGER_REGION_START) != HAL_OK)
    {
        printf("[LOGGER] Failed to erase logger region.\r\n");
        return HAL_ERROR;
    }

    logger_write_address = LOGGER_REGION_START;
    logger_record_count = 0U;
    logger_full = false;
    logger_initialized = true;

    printf("[LOGGER] Log cleared.\r\n");
	printf(
		"[LOGGER] Next address: 0x%08lX\r\n",
		(unsigned long)logger_write_address
	);

    return HAL_OK;
}

uint32_t logger_get_capacity(void)
{
    return LOGGER_MAX_RECORDS;
}

void flash_logger_init(void) {
	if (flash_init() != HAL_OK)
	{
	  printf("[MAIN] Flash initialization failed.\r\n");
	}
	else
	{
	  flash_available = true;
	  printf("[MAIN] External flash is ready.\r\n");

	  if (logger_init() != HAL_OK)
	  {
		  printf("[MAIN] Logger initialization failed.\r\n");
	  }
	  else
	  {
		  logger_available = true;

		  printf(
			  "[MAIN] Logger ready with %lu existing records.\r\n",
			  (unsigned long)logger_get_record_count()
		  );
	  }
	}
}

HAL_StatusTypeDef logger_start(void)
{
    if (!logger_initialized)
    {
        printf("[LOGGER] Cannot start: logger is not initialized.\r\n");
        return HAL_ERROR;
    }

    if (!flash_is_ready())
    {
        printf("[LOGGER] Cannot start: flash is not ready.\r\n");
        return HAL_ERROR;
    }

    if (logger_full)
    {
        printf("[LOGGER] Cannot start: logging region is full.\r\n");
        return HAL_ERROR;
    }

    if (logger_active)
    {
        printf("[LOGGER] Logging is already active.\r\n");
        return HAL_OK;
    }

    logger_active = true;

    printf("[LOGGER] Logging started.\r\n");
    printf(
        "[LOGGER] Record count: %lu\r\n",
        (unsigned long)logger_record_count
    );
    printf(
        "[LOGGER] Next address: 0x%08lX\r\n",
        (unsigned long)logger_write_address
    );

    return HAL_OK;
}


HAL_StatusTypeDef logger_stop(void)
{
    if (!logger_initialized)
    {
        printf("[LOGGER] Cannot stop: logger is not initialized.\r\n");
        return HAL_ERROR;
    }

    if (!logger_active)
    {
        printf("[LOGGER] Logging is already stopped.\r\n");
        return HAL_OK;
    }

    logger_active = false;

    printf("[LOGGER] Logging stopped.\r\n");
    printf(
        "[LOGGER] Records stored: %lu\r\n",
        (unsigned long)logger_record_count
    );

    return HAL_OK;
}


bool logger_is_active(void)
{
    return logger_active;
}
