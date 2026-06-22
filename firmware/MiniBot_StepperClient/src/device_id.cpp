#include "device_id.h"
#include <Arduino.h>
#include <nvs_flash.h>
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_DEVICE_ID
#include "config.h"
#include "esp_log.h"

static const char *TAG = "DEVICE_ID";

static uint8_t cached_device_id = 0xFF;
static bool nvs_initialized = false;

static void ensureNVSInitialized() {
  if (!nvs_initialized) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated, erase and retry
      nvs_flash_erase();
      err = nvs_flash_init();
    }
    nvs_initialized = true;
  }
}

static void openNVShandle(nvs_handle_t *my_handle) {
  ESP_LOGD(TAG, "Opening Non-Volatile Storage (NVS) handle...");
  esp_err_t err = nvs_open("storage", NVS_READWRITE, my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }
}

uint8_t getDeviceID() {
  if (cached_device_id != 0xFF) {
    return cached_device_id;
  }
  ensureNVSInitialized();
  nvs_handle_t nvshandle;
  openNVShandle(&nvshandle);

  int16_t dev_id = 0xFF;
  esp_err_t err = nvs_get_i16(nvshandle, "device_id", &dev_id);
  switch (err) {
  case ESP_OK:
    ESP_LOGI(TAG, "Device ID Read successful - %d", dev_id);
    break;
  case ESP_ERR_NVS_NOT_FOUND:
    ESP_LOGW(TAG, "The device ID is not initialized yet!");
    break;
  default:
    ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
  }
  nvs_close(nvshandle);
  cached_device_id = (uint8_t)dev_id;
  return (uint8_t)dev_id;
}

void setDeviceID(uint8_t id) {
  ensureNVSInitialized();
  nvs_handle_t nvshandle;
  openNVShandle(&nvshandle);

  int16_t dev_id = (int16_t)id;
  esp_err_t err = nvs_set_i16(nvshandle, "device_id", dev_id);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write device ID!");
  } else {
    ESP_LOGI(TAG, "Device ID set to: 0x%02X", id);
  }
  nvs_close(nvshandle);
  return;
}
