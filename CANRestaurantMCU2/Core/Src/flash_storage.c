#include "flash_storage.h"
#include "stm32h5xx_hal.h"
#include <string.h>

/* ---------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------- */

/** Return the flash address for checkpoint slot [id]. */
static inline uint32_t cp_addr(uint8_t id) {
    return FLASH_STORAGE_BASE_ADDR + ((uint32_t)id * sizeof(checkpoint_t));
}

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */

void flash_storage_init(void)
{
    /* Nothing to do — reads are direct memory-mapped.
     * Call before first save/load to ensure FLASH clock is ready. */
}

HAL_StatusTypeDef flash_storage_erase_all(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks     = FLASH_STORAGE_BANK,
        .Sector    = FLASH_STORAGE_SECTOR,
        .NbSectors = 1U,
    };

    uint32_t sectorError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sectorError);

    HAL_FLASH_Lock();
    return status;
}

HAL_StatusTypeDef flash_storage_save_cp(uint8_t  id,
                                        int32_t  steps_l,
                                        int32_t  steps_r,
                                        int32_t  heading_x10)
{
    if (id >= FLASH_STORAGE_MAX_CP) {
        return HAL_ERROR;
    }

    /* Build a 16-byte aligned buffer for QUADWORD write. */
    checkpoint_t cp;
    memset(&cp, 0xFF, sizeof(cp));   /* erased flash value */
    cp.steps_l      = steps_l;
    cp.steps_r      = steps_r;
    cp.heading_x10  = heading_x10;
    cp.valid        = 0xA5U;
    cp.id           = id;
    cp._pad[0]      = 0x00U;
    cp._pad[1]      = 0x00U;

    /* Erase the full sector first (STM32H5 must erase before write). */
    HAL_StatusTypeDef status = flash_storage_erase_all();
    if (status != HAL_OK) {
        return status;
    }

    /* Re-read all existing checkpoints so we can re-write them. */
    checkpoint_t buf[FLASH_STORAGE_MAX_CP];
    for (uint8_t i = 0; i < FLASH_STORAGE_MAX_CP; i++) {
        memcpy(&buf[i],
               (const void *)cp_addr(i),
               sizeof(checkpoint_t));
        /* After erase all bytes are 0xFF, so valid != 0xA5 → blank. */
    }

    /* Overwrite the target slot with the new data. */
    buf[id] = cp;

    /* Write all 16 slots back. */
    HAL_FLASH_Unlock();
    status = HAL_OK;

    for (uint8_t i = 0; i < FLASH_STORAGE_MAX_CP && status == HAL_OK; i++) {
        /* Skip blank slots (0xFF fill) — no need to write erased data. */
        if (buf[i].valid != 0xA5U) {
            continue;
        }

        /* HAL_FLASH_Program QUADWORD: address must be 16-byte aligned.
         * Third argument is a pointer to the 128-bit data buffer. */
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                                   cp_addr(i),
                                   (uint32_t)(uintptr_t)&buf[i]);
    }

    HAL_FLASH_Lock();
    return status;
}

HAL_StatusTypeDef flash_storage_load_cp(uint8_t id, checkpoint_t *out)
{
    if (id >= FLASH_STORAGE_MAX_CP || out == NULL) {
        return HAL_ERROR;
    }

    memcpy(out, (const void *)cp_addr(id), sizeof(checkpoint_t));

    if (out->valid != 0xA5U || out->id != id) {
        return HAL_ERROR;   /* blank or corrupted */
    }

    return HAL_OK;
}
