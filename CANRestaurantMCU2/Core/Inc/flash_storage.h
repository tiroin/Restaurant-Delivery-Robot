#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include "stm32h5xx_hal.h"

/* STM32H503CBTx: 128KB flash, 8KB sectors (FLASH_SECTOR_NB==8)
 * Using Bank 2, Sector 7  ->  base = 0x0801E000, size = 8KB
 * Each checkpoint is exactly 16 bytes (one QUADWORD write). */
#define FLASH_STORAGE_BANK          FLASH_BANK_2
#define FLASH_STORAGE_SECTOR        7U
#define FLASH_STORAGE_BASE_ADDR     0x0801E000UL
#define FLASH_STORAGE_MAX_CP        16U     /* checkpoints 0..15 */

/* Checkpoint record — must be exactly 16 bytes (128-bit QUADWORD) */
typedef struct __attribute__((packed, aligned(16))) {
    int32_t  steps_l;      /* left  stepper cumulative steps  */
    int32_t  steps_r;      /* right stepper cumulative steps  */
    int32_t  heading_x10;  /* heading in 0.1-degree units     */
    uint8_t  valid;        /* 0xA5 = valid entry              */
    uint8_t  id;           /* checkpoint id 0..15             */
    uint8_t  _pad[2];      /* alignment padding               */
} checkpoint_t;            /* 4+4+4+1+1+2 = 16 bytes          */

/**
 * @brief  Initialise storage (validates sector magic).
 *         Call once after HAL_Init() before flash reads.
 */
void flash_storage_init(void);

/**
 * @brief  Save a checkpoint to flash.
 * @param  id          Checkpoint index 0..15
 * @param  steps_l     Left  motor cumulative steps
 * @param  steps_r     Right motor cumulative steps
 * @param  heading_x10 Heading ×10 (int, so 1800 = 180.0°)
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef flash_storage_save_cp(uint8_t id,
                                        int32_t steps_l,
                                        int32_t steps_r,
                                        int32_t heading_x10);

/**
 * @brief  Load a checkpoint from flash.
 * @param  id   Checkpoint index 0..15
 * @param  out  Output buffer (must not be NULL)
 * @retval HAL_OK if entry is valid, HAL_ERROR if invalid/blank
 */
HAL_StatusTypeDef flash_storage_load_cp(uint8_t id, checkpoint_t *out);

/**
 * @brief  Erase the entire storage sector (all 16 checkpoints).
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef flash_storage_erase_all(void);

#endif /* FLASH_STORAGE_H */
