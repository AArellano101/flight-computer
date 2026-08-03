#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* S25FL128L external flash geometry */
#define EXT_FLASH_SIZE_BYTES          0x01000000UL
#define EXT_FLASH_PAGE_SIZE           0x00000100UL
#define EXT_FLASH_SECTOR_SIZE_4K      0x00001000UL
#define EXT_FLASH_BLOCK_SIZE_64K      0x00010000UL

/* External flash memory map */
#define EXT_FLASH_METADATA_ADDRESS    0x00000000UL
#define EXT_FLASH_LOG_START_ADDRESS   0x00001000UL
#define EXT_FLASH_LOG_END_ADDRESS     0x00FFF000UL
#define EXT_FLASH_TEST_ADDRESS        0x00FFF000UL

#define FLASH_ENABLE_SELF_TEST       0U

HAL_StatusTypeDef flash_init(void);

HAL_StatusTypeDef flash_read(
    uint32_t address,
    uint8_t *data,
    uint32_t size);

HAL_StatusTypeDef flash_write(
    uint32_t address,
    const uint8_t *data,
    uint32_t size);

HAL_StatusTypeDef flash_erase_4k(uint32_t address);

HAL_StatusTypeDef flash_test_write_enable(void);
HAL_StatusTypeDef flash_test_erase_program_read(void);
HAL_StatusTypeDef flash_test_api(void);

bool flash_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
