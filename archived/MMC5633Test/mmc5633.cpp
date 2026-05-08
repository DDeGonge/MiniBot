#include "mmc5633.h"
#include <math.h>

static const char* TAG = "MMC5633";

MMC5633NJL::MMC5633NJL(TwoWire &wire) : _wire(wire), _continuous_mode(false) {}

bool MMC5633NJL::begin(int sda_pin, int scl_pin, uint32_t i2c_freq) {
    if (sda_pin >= 0 && scl_pin >= 0) _wire.begin(sda_pin, scl_pin, i2c_freq);
    else _wire.begin();

    delay(5);

    // Reset chip and check product ID
    if (!writeRegister(REG_CTRL1, 0x80)) return false;
    delay(20);
    uint8_t pid = 0;
    if (!readRegister(REG_PRODUCT_ID, &pid)) return false;
    if (pid != 0x10) return false;

    // if (!writeRegister(REG_CTRL1, 0x03)) return false;
    // if (!disableContinuousMode()) return false;
    // if (!runSelfTest()) return false;

    return true;
}

bool MMC5633NJL::setReset() {
    // manual set/reset, will take ~4ms to complete
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!writeRegister(REG_CTRL0, 0x08)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!writeRegister(REG_CTRL0, 0x10)) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    return true;
}

bool MMC5633NJL::readMeasurement(uint32_t timeout_ms) {
    if (_continuous_mode) {
        uint8_t buf[9] = {0};
        if (!readRegisters(REG_XOUT0, buf, 9)) return false;
        unpackRawXYZFromBuffer(buf, 9);
        rawTemp = buf[8];
        if (rawX == _lastX && rawY == _lastY && rawZ == _lastZ) return false;
        _lastX = rawX; _lastY = rawY; _lastZ = rawZ;
        return true;
    } else {
        // On-demand mode: trigger measurement and wait
        uint8_t ctrl0 = (1 << 5) | (1 << 0);
        if (!writeRegister(REG_CTRL0, ctrl0)) return false;

        if (!waitForMeasurementDone(timeout_ms)) return false;

        uint8_t buf[9] = {0};
        if (!readRegisters(REG_XOUT0, buf, 9)) return false;

        unpackRawXYZFromBuffer(buf, 9);
        rawTemp = buf[8];
        return true;
    }
}

bool MMC5633NJL::isMeasurementReady() {
    uint8_t status = 0;
    if (!readRegister(REG_STATUS1, &status)) return false;
    return (status & STAT1_MEAS_M_DONE) != 0;
}

bool MMC5633NJL::enableContinuousMode() {
    // Enable continuous measurement mode at 1000hz sampling
    if (!writeRegister(REG_CTRL1, 0x03)) return false;
    if (!writeRegister(REG_ODR,   0xFF)) return false;
    if (!writeRegister(REG_CTRL0, 0x80)) return false;
    delay(2);
    if (!writeRegister(REG_CTRL2, 0x90)) return false;
    if (!writeRegister(REG_CTRL2, 0x90)) return false;
    delay(2);

    _continuous_mode = true;
    return true;
}

bool MMC5633NJL::disableContinuousMode() {
    // Disable continuous measurement mode
    // TODO untested
    if (!writeRegister(REG_CTRL2, 0x00)) return false;
    _continuous_mode = false;
    return true;
}

bool MMC5633NJL::runSelfTest(uint32_t timeout_ms) {
    uint8_t stx = 0, sty = 0, stz = 0;
    if (!readRegister(REG_ST_X, &stx)) return false;
    if (!readRegister(REG_ST_Y, &sty)) return false;
    if (!readRegister(REG_ST_Z, &stz)) return false;

    uint8_t thx = (uint8_t)max(0, (int)round(stx * 0.8f));
    uint8_t thy = (uint8_t)max(0, (int)round(sty * 0.8f));
    uint8_t thz = (uint8_t)max(0, (int)round(stz * 0.8f));

    if (!writeRegister(REG_ST_X_TH, thx)) return false;
    if (!writeRegister(REG_ST_Y_TH, thy)) return false;
    if (!writeRegister(REG_ST_Z_TH, thz)) return false;

    if (!writeRegister(REG_CTRL0, 0x41)) return false;

    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        uint8_t status = 0;
        if (!readRegister(REG_STATUS1, &status)) return false;

        if ((status & STAT1_SAT_SENSOR) == 0) {
            return true;
        }
        delay(1);
    }

    return false;
}

int32_t MMC5633NJL::signedX() const { return int32_t((int64_t)rawX - NULL_VALUE_20BIT); }
int32_t MMC5633NJL::signedY() const { return int32_t((int64_t)rawY - NULL_VALUE_20BIT); }
int32_t MMC5633NJL::signedZ() const { return int32_t((int64_t)rawZ - NULL_VALUE_20BIT); }

float MMC5633NJL::getFieldGaussX() const { return ((float)signedX()) / COUNTS_PER_G_20BIT; }
float MMC5633NJL::getFieldGaussY() const { return ((float)signedY()) / COUNTS_PER_G_20BIT; }
float MMC5633NJL::getFieldGaussZ() const { return ((float)signedZ()) / COUNTS_PER_G_20BIT; }

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
        Serial.printf("STATUS 0x%02X\n", status1);
    }
    else {
        Serial.println("Failed to read status register");
    }
}

bool MMC5633NJL::recoverDevice() {
    Serial.println("Attempting device recovery...");

    // Full reinit sequence
    uint8_t pid = 0;
    if (!readRegister(REG_PRODUCT_ID, &pid) || pid != 0x10) {
        Serial.println("Cannot reach device during recovery");
        return false;
    }

    // Soft reset via CTRL1 bit 7
    if (!writeRegister(REG_CTRL1, 0x80)) {
        Serial.println("Soft reset write failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    if (!writeRegister(REG_CTRL1, 0x03)) return false;
    if (!writeRegister(REG_ODR,   0xFF)) return false;
    if (!writeRegister(REG_CTRL0, 0x00)) return false;

    if (_continuous_mode) {
        if (!enableContinuousMode()) {
            Serial.println("Failed to re-enable continuous mode after recovery");
            return false;
        }
        Serial.println("Recovery successful -- continuous mode re-enabled");
    } else {
        Serial.println("Recovery successful -- on-demand mode restored");
    }

    _lastX = UINT32_MAX; _lastY = UINT32_MAX; _lastZ = UINT32_MAX;
    return true;
}

bool MMC5633NJL::waitForMeasurementDone(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        uint8_t status = 0;
        if (!readRegister(REG_STATUS1, &status)) return false;
        if (status & STAT1_MEAS_M_DONE) return true;
    }
    return false;
}

void MMC5633NJL::unpackRawXYZFromBuffer(const uint8_t *buf, size_t len) {
    if (len < 9) return;

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
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(I2C_ADDR, (uint8_t)1) != 1) return false;
    *value = _wire.read();
    return true;
}

bool MMC5633NJL::readRegisters(uint8_t reg, uint8_t *buf, size_t len) {
    _wire.beginTransmission(I2C_ADDR);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(I2C_ADDR, (uint8_t)len) != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = _wire.read();
    return true;
}
