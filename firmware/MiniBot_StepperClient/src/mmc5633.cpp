#include "mmc5633.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_MMC5633
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MMC5633";

MMC5633NJL::MMC5633NJL(TwoWire &wire) : _wire(wire), _continuous_mode(false) {}

bool MMC5633NJL::begin(int sda_pin, int scl_pin, uint32_t i2c_freq) {
  if (sda_pin >= 0 && scl_pin >= 0)
    _wire.begin(sda_pin, scl_pin, i2c_freq);
  else
    _wire.begin();

  vTaskDelay(pdMS_TO_TICKS(5));

  // Reset chip and check product ID
  if (!writeRegister(REG_CTRL1, 0x80))
    return false;
  vTaskDelay(pdMS_TO_TICKS(20));
  uint8_t pid = 0;
  if (!readRegister(REG_PRODUCT_ID, &pid))
    return false;
  if (pid != 0x10)
    return false;

  if (!runSelfTest()){
    ESP_LOGE(TAG, "Self test failed");
    return false;
  }

  return true;
}

bool MMC5633NJL::setReset() {
  // manual set/reset, will take ~4ms to complete
  bool reenable_continuous = _continuous_mode;
  if (reenable_continuous){
    disableContinuousMode();
  }
  if (!writeRegister(REG_CTRL0, 0x08))
    return false;
  vTaskDelay(pdMS_TO_TICKS(1));
  if (!writeRegister(REG_CTRL0, 0x10))
    return false;
  vTaskDelay(pdMS_TO_TICKS(1));
  if (reenable_continuous){
    enableContinuousMode();
  }
  return true;
}

bool MMC5633NJL::triggerMeasurement() {
  uint8_t ctrl0 = (1 << 5) | (1 << 0);
  return writeRegister(REG_CTRL0, ctrl0);
}

bool MMC5633NJL::readMeasurementData() {
  uint8_t buf[9] = {0};
  if (!readRegisters(REG_XOUT0, buf, 9)) {
    return false;
  }
  unpackRawXYZFromBuffer(buf, 9);
  return true;
}

bool MMC5633NJL::readMeasurement(uint32_t timeout_ms) {
  if (_continuous_mode) {
    if (!readMeasurementData()) {
      ++read_err_count;
      return false;
    }

    if (rawX == _lastX && rawY == _lastY && rawZ == _lastZ) {
      ++read_dupe_count;
      // temporary print status registers
      // uint8_t stat = 0;
      // readRegister(REG_STATUS0, &stat);
      // ESP_LOGE(TAG, "STATUS0 register: 0x%02X", stat);
      // readRegister(REG_STATUS1, &stat);
      // ESP_LOGE(TAG, "STATUS1 register: 0x%02X", stat);
      return false;
    }

    _lastX = rawX;
    _lastY = rawY;
    _lastZ = rawZ;
    return true;
  } else {
    // On-demand mode: trigger measurement and wait
    if (!triggerMeasurement())
      return false;
    if (!waitForMeasurementDone(timeout_ms))
      return false;
    return readMeasurementData();
  }
}

bool MMC5633NJL::isMeasurementReady() {
  // This takes about 200us so only use if needed
  uint8_t status = 0;
  if (!readRegister(REG_STATUS1, &status))
    return false;
  return (status & STAT1_MEAS_M_DONE) != 0;
}

bool MMC5633NJL::enableContinuousMode() {
  // Enable continuous measurement mode at 1000hz sampling
  if (!writeRegister(REG_CTRL1, 0x03))
    return false;
  if (!writeRegister(REG_ODR, 0xFF))
    return false;
  if (!writeRegister(REG_CTRL2, 0x80))
    return false;
  // Cmm_freq_en (0x80) | Auto_SR_en (0x20)
  if (!writeRegister(REG_CTRL0, 0x80))
    return false;
  vTaskDelay(pdMS_TO_TICKS(2));
  if (!writeRegister(REG_CTRL2, 0x90))
    return false;

  _continuous_mode = true;
  return true;
}

bool MMC5633NJL::disableContinuousMode() {
  // Disable continuous measurement mode
  if (!writeRegister(REG_CTRL2, 0x00))
    return false;
  _continuous_mode = false;
  return true;
}

void MMC5633NJL::self_benchmark(uint32_t sample_count,
                               uint32_t nominal_period_us) {
  static int64_t ready_times_us[1000] = {0};
  static int64_t intervals_us[999] = {0};

  if (sample_count == 0) {
    ESP_LOGW(TAG, "MMC5633 self_benchmark requested zero samples");
    return;
  }

  if (sample_count > 1000) {
    sample_count = 1000;
  }

  if (!enableContinuousMode()) {
    ESP_LOGE(TAG, "Failed to enable MMC5633 continuous mode for self benchmark");
    return;
  }

  ESP_LOGI(TAG,
           "MMC5633 timing benchmark: polling readiness and reading %lu times "
           "in continuous mode at nominal %lu us",
           sample_count, nominal_period_us);

  for (uint32_t i = 0; i < sample_count; ++i) {
    uint32_t poll_attempts = 0;
    while (!isMeasurementReady()) {
      poll_attempts++;
      if (poll_attempts > 100000) {
        ESP_LOGE(TAG,
                 "Measurement %lu never became ready; last status poll at %lld us",
                 i, esp_timer_get_time());
        disableContinuousMode();
        return;
      }
    }

    int64_t ready_us = esp_timer_get_time();
    ready_times_us[i] = ready_us;

    if (!readMeasurementData()) {
      ESP_LOGE(TAG, "Failed to read measurement %lu at %lld us", i, ready_us);
      disableContinuousMode();
      return;
    }

    if (i > 0) {
      intervals_us[i - 1] = ready_us - ready_times_us[i - 1];
    }
  }

  float sum_period_us = 0.0f;
  float sum_of_squares = 0.0f;
  int64_t max_dev_us = 0;

  for (uint32_t i = 0; i < sample_count - 1; ++i) {
    float period_us = (float)intervals_us[i];
    sum_period_us += period_us;
    sum_of_squares += period_us * period_us;

    int64_t dev_us = intervals_us[i] - (int64_t)nominal_period_us;
    if (dev_us < 0)
      dev_us = -dev_us;
    if (dev_us > max_dev_us)
      max_dev_us = dev_us;
  }

  float mean_period_us = sum_period_us / (float)(sample_count - 1);
  float variance_us = (sum_of_squares / (float)(sample_count - 1)) -
                      (mean_period_us * mean_period_us);
  if (variance_us < 0.0f)
    variance_us = 0.0f;
  float stdev_us = sqrtf(variance_us);
  float sample_rate_hz = 1000000.0f / mean_period_us;

  ESP_LOGI(TAG,
           "MMC5633 benchmark complete: ready_times[0]=%lld us, ready_times[%lu]=%lld us, "
           "sample rate %.6f Hz, mean period %.3f us, "
           "stdev %.3f us, max deviation from nominal %lld us",
           ready_times_us[0], sample_count - 1, ready_times_us[sample_count - 1],
           sample_rate_hz, mean_period_us, stdev_us, max_dev_us);

  // Store benchmark metrics for later use
  _bench_mean_period_us = mean_period_us;
  _bench_stdev_us = stdev_us;
  _bench_ref_time_us = ready_times_us[0];

  disableContinuousMode();
}

float MMC5633NJL::getBenchmarkMeanPeriodUs() const { return _bench_mean_period_us; }
float MMC5633NJL::getBenchmarkStdevUs() const { return _bench_stdev_us; }
int64_t MMC5633NJL::getBenchmarkReferenceTimeUs() const { return _bench_ref_time_us; }

void MMC5633NJL::setNominalPeriodUs(int64_t period_us) { _nominal_period_us = period_us; }
int64_t MMC5633NJL::getNominalPeriodUs() const { return _nominal_period_us; }

bool MMC5633NJL::runSelfTest(uint32_t timeout_ms) {
  uint8_t stx = 0, sty = 0, stz = 0;
  if (!readRegister(REG_ST_X, &stx))
    return false;
  if (!readRegister(REG_ST_Y, &sty))
    return false;
  if (!readRegister(REG_ST_Z, &stz))
    return false;

  uint8_t thx = (uint8_t)max(0, (int)round(stx * 0.8f));
  uint8_t thy = (uint8_t)max(0, (int)round(sty * 0.8f));
  uint8_t thz = (uint8_t)max(0, (int)round(stz * 0.8f));

  if (!writeRegister(REG_ST_X_TH, thx))
    return false;
  if (!writeRegister(REG_ST_Y_TH, thy))
    return false;
  if (!writeRegister(REG_ST_Z_TH, thz))
    return false;

  if (!writeRegister(REG_CTRL0, 0x41))
    return false;

  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t status = 0;
    if (!readRegister(REG_STATUS1, &status))
      return false;

    if ((status & STAT1_SAT_SENSOR) == 0) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return false;
}

int32_t MMC5633NJL::signedX() const {
  return int32_t((int64_t)rawX - NULL_VALUE_20BIT);
}
int32_t MMC5633NJL::signedY() const {
  return int32_t((int64_t)rawY - NULL_VALUE_20BIT);
}
int32_t MMC5633NJL::signedZ() const {
  return int32_t((int64_t)rawZ - NULL_VALUE_20BIT);
}

float MMC5633NJL::getFieldGaussX() const {
  return ((float)signedX()) / COUNTS_PER_G_20BIT;
}
float MMC5633NJL::getFieldGaussY() const {
  return ((float)signedY()) / COUNTS_PER_G_20BIT;
}
float MMC5633NJL::getFieldGaussZ() const {
  return ((float)signedZ()) / COUNTS_PER_G_20BIT;
}

float MMC5633NJL::getAzimuthDegrees() const {
  float fx = (float)signedX();
  float fy = (float)signedY();
  float ang = atan2f(fy, fx);
  float deg = ang * 180.0f / M_PI;
  return deg;
}

float MMC5633NJL::getAzimuthRadians() const {
  float fx = (float)signedX();
  float fy = (float)signedY();
  return atan2f(fy, fx);
}

void MMC5633NJL::checkDeviceStatus() {
  uint8_t status1 = 0;
  bool ok_status1 = readRegister(REG_STATUS1, &status1);

  if (ok_status1) {
    ESP_LOGD(TAG, "STATUS 0x%02X", status1);
  } else {
    ESP_LOGE(TAG, "Failed to read status register");
  }
}

bool MMC5633NJL::recoverDevice() {
  ESP_LOGW(TAG, "Attempting device recovery...");

  // Full reinit sequence
  uint8_t pid = 0;
  if (!readRegister(REG_PRODUCT_ID, &pid) || pid != 0x10) {
    ESP_LOGE(TAG, "Cannot reach device during recovery");
    return false;
  }

  // Soft reset via CTRL1 bit 7
  if (!writeRegister(REG_CTRL1, 0x80)) {
    ESP_LOGE(TAG, "Soft reset write failed");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(25));

  if(!setReset()) {
    ESP_LOGE(TAG, "Set/reset failed during recovery");
    return false;
  }

  _lastX = UINT32_MAX;
  _lastY = UINT32_MAX;
  _lastZ = UINT32_MAX;
  return true;
}

bool MMC5633NJL::waitForMeasurementDone(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t status = 0;
    if (!readRegister(REG_STATUS1, &status))
      return false;
    if (status & STAT1_MEAS_M_DONE)
      return true;
  }
  return false;
}

void MMC5633NJL::unpackRawXYZFromBuffer(const uint8_t *buf, size_t len) {
  if (len < 9)
    return;

  uint32_t x_hi = ((uint32_t)buf[0] << 12);
  uint32_t x_mid = ((uint32_t)buf[1] << 4);
  uint32_t x_lo = ((uint32_t)buf[6] >> 4) & 0x0F;
  rawX = x_hi | x_mid | x_lo;

  uint32_t y_hi = ((uint32_t)buf[2] << 12);
  uint32_t y_mid = ((uint32_t)buf[3] << 4);
  uint32_t y_lo = ((uint32_t)buf[6] & 0x0F);
  rawY = y_hi | y_mid | y_lo;

  uint32_t z_hi = ((uint32_t)buf[4] << 12);
  uint32_t z_mid = ((uint32_t)buf[5] << 4);
  uint32_t z_lo = ((uint32_t)buf[7] >> 4) & 0x0F;
  rawZ = z_hi | z_mid | z_lo;
}

bool MMC5633NJL::writeRegister(uint8_t reg, uint8_t value) {
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(reg);
  _wire.write(value);
  return (_wire.endTransmission() == 0);
}

bool MMC5633NJL::readRegister(uint8_t reg, uint8_t *value) {
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(reg);
  if (_wire.endTransmission(false) != 0)
    return false;
  if (_wire.requestFrom(I2C_ADDR, (uint8_t)1) != 1)
    return false;
  *value = _wire.read();
  return true;
}

bool MMC5633NJL::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(reg);
  if (_wire.endTransmission(false) != 0)
    return false;
  if (_wire.requestFrom(I2C_ADDR, (uint8_t)len) != len)
    return false;
  for (size_t i = 0; i < len; ++i)
    buf[i] = _wire.read();
  return true;
}

void MMC5633NJL::getErrorCounters(uint32_t &read_err, uint32_t &read_dupe) const {
  read_err = read_err_count;
  read_dupe = read_dupe_count;
}

void MMC5633NJL::resetErrorCounters() {
  read_err_count = 0;
  read_dupe_count = 0;
}
