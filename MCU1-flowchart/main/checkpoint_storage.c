/**
 * @file    checkpoint_storage.c
 * @brief   Lưu trữ checkpoint vào NVS Flash — Implementation
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "checkpoint_storage.h"

static const char *TAG = "CheckpointStorage";

/** NVS namespace dành riêng cho module này */
#define NVS_NS          "checkpoints"

/** Key format: "cp_XX" — XX là id dạng hex 2 chữ số */
#define KEY_FMT         "cp_%02X"
#define KEY_LEN         8

/* ─────────────────────────────────────────────────────────── */

esp_err_t checkpoint_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS full/outdated — erasing partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    /* Tạo checkpoint Home mặc định nếu chưa có */
    checkpoint_t tmp;
    if (checkpoint_load(CHECKPOINT_ID_HOME, &tmp) != ESP_OK) {
        checkpoint_t home = {
            .id    = CHECKPOINT_ID_HOME,
            .name  = "Home",
            .pos_x = 0.0f,
            .pos_y = 0.0f,
            .valid = true,
        };
        checkpoint_save(&home);
        ESP_LOGI(TAG, "Created default: Home (id=%d)", CHECKPOINT_ID_HOME);
    }

    /* Tạo checkpoint Kitchen mặc định nếu chưa có */
    if (checkpoint_load(CHECKPOINT_ID_KITCHEN, &tmp) != ESP_OK) {
        checkpoint_t kitchen = {
            .id    = CHECKPOINT_ID_KITCHEN,
            .name  = "Kitchen",
            .pos_x = 1000.0f,
            .pos_y = 0.0f,
            .valid = true,
        };
        checkpoint_save(&kitchen);
        ESP_LOGI(TAG, "Created default: Kitchen (id=%d)", CHECKPOINT_ID_KITCHEN);
    }

    ESP_LOGI(TAG, "Initialized (namespace='%s')", NVS_NS);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────── */

esp_err_t checkpoint_save(const checkpoint_t *cp)
{
    if (!cp || cp->id >= CHECKPOINT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char key[KEY_LEN];
    snprintf(key, sizeof(key), KEY_FMT, cp->id);

    err = nvs_set_blob(h, key, cp, sizeof(checkpoint_t));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved  id=%-2d  name='%-20s'  pos=(%.1f, %.1f)",
                 cp->id, cp->name, cp->pos_x, cp->pos_y);
    } else {
        ESP_LOGE(TAG, "Save failed id=%d: %s", cp->id, esp_err_to_name(err));
    }
    return err;
}

/* ─────────────────────────────────────────────────────────── */

esp_err_t checkpoint_load(uint8_t id, checkpoint_t *cp)
{
    if (!cp || id >= CHECKPOINT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    char key[KEY_LEN];
    snprintf(key, sizeof(key), KEY_FMT, id);

    size_t sz = sizeof(checkpoint_t);
    err = nvs_get_blob(h, key, cp, &sz);
    nvs_close(h);
    return err;
}

/* ─────────────────────────────────────────────────────────── */

esp_err_t checkpoint_load_all(checkpoint_t *out, uint8_t *count)
{
    if (!out || !count) return ESP_ERR_INVALID_ARG;
    *count = 0;

    for (uint8_t id = 0; id < CHECKPOINT_MAX; id++) {
        checkpoint_t cp;
        if (checkpoint_load(id, &cp) == ESP_OK && cp.valid) {
            out[(*count)++] = cp;
        }
    }
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────── */

esp_err_t checkpoint_delete(uint8_t id)
{
    if (id == CHECKPOINT_ID_HOME || id == CHECKPOINT_ID_KITCHEN) {
        ESP_LOGW(TAG, "Cannot delete protected checkpoint id=%d", id);
        return ESP_ERR_NOT_ALLOWED;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char key[KEY_LEN];
    snprintf(key, sizeof(key), KEY_FMT, id);

    err = nvs_erase_key(h, key);
    if (err == ESP_OK) nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Deleted checkpoint id=%d", id);
    }
    return err;
}

/* ─────────────────────────────────────────────────────────── */

void checkpoint_print_all(void)
{
    checkpoint_t all[CHECKPOINT_MAX];
    uint8_t count = 0;
    checkpoint_load_all(all, &count);

    ESP_LOGI(TAG, "╔══ Checkpoints (%d stored) ══════════════════════╗", count);
    for (int i = 0; i < count; i++) {
        ESP_LOGI(TAG, "║  [%2d] %-20s  (%.1f, %.1f)",
                 all[i].id, all[i].name, all[i].pos_x, all[i].pos_y);
    }
    ESP_LOGI(TAG, "╚═════════════════════════════════════════════════╝");
}
