#ifndef __MMC5633_H__
#define __MMC5633_H__

#include <Arduino.h>
#include <Wire.h>

class MMC5633NJL {
public:
    static constexpr uint8_t I2C_ADDR = 0x30;

    static constexpr uint8_t REG_XOUT0 = 0x00;
    static constexpr uint8_t REG_PRODUCT_ID = 0x39;
    static constexpr uint8_t REG_STATUS1 = 0x18;
    static constexpr uint8_t REG_ODR = 0x1A;
    static constexpr uint8_t REG_CTRL0 = 0x1B;
    static constexpr uint8_t REG_CTRL1 = 0x1C;
    static constexpr uint8_t REG_CTRL2 = 0x1D;
    static constexpr uint8_t REG_ST_X_TH = 0x1E;
    static constexpr uint8_t REG_ST_Y_TH = 0x1F;
    static constexpr uint8_t REG_ST_Z_TH = 0x20;
    static constexpr uint8_t REG_ST_X = 0x27;
    static constexpr uint8_t REG_ST_Y = 0x28;
    static constexpr uint8_t REG_ST_Z = 0x29;

    static constexpr uint8_t STAT1_MEAS_M_DONE = (1 << 6);
    static constexpr uint8_t STAT1_SAT_SENSOR  = (1 << 5);

    static constexpr int32_t NULL_VALUE_20BIT = 524288;
    static constexpr float COUNTS_PER_G_20BIT = 16384.0f;

    MMC5633NJL(TwoWire &wire = Wire);

    bool begin(int sda_pin = -1, int scl_pin = -1, uint32_t i2c_freq = 400000);
    bool readMeasurement(uint32_t timeout_ms = 20);
    bool isMeasurementReady();
    bool enableContinuousMode();
    bool disableContinuousMode();
    bool runSelfTest(uint32_t timeout_ms = 100);
    bool setReset();
    void checkDeviceStatus();
    bool recoverDevice();

    int32_t signedX() const;
    int32_t signedY() const;
    int32_t signedZ() const;

    float getFieldGaussX() const;
    float getFieldGaussY() const;
    float getFieldGaussZ() const;

    float getAzimuthDegrees() const;
    float getAzimuthRadians() const;

private:
    TwoWire &_wire;
    bool _continuous_mode;
    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    uint32_t _lastX = UINT32_MAX, _lastY = UINT32_MAX, _lastZ = UINT32_MAX;
    int8_t rawTemp = 0;

    bool waitForMeasurementDone(uint32_t timeout_ms);
    void unpackRawXYZFromBuffer(const uint8_t *buf, size_t len);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t *value);
    bool readRegisters(uint8_t reg, uint8_t *buf, size_t len);
};

#endif // __MMC5633_H__
