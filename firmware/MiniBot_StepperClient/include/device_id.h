#ifndef __DEVICE_ID_H__
#define __DEVICE_ID_H__

#include <stdint.h>

/**
 * Device ID Management
 *
 * The device ID is stored in NVS (Non-Volatile Storage) and persists
 * across firmware flashes. This allows the same firmware to be deployed
 * to multiple robots while each maintains its unique ID.
 *
 * Usage:
 * 1. Flash firmware to robot
 * 2. Temporarily call setDeviceID(id) in setup() with unique ID (0x01-0xFE)
 * 3. Upload and run once to save ID
 * 4. Remove setDeviceID() call and reflash
 * 5. ID persists forever
 */

/**
 * Get the device ID from NVS
 * @return Device ID (0x01-0xFE), or 0xFF if not configured
 */
uint8_t getDeviceID();

/**
 * Set the device ID and save to NVS (persists across firmware flashes)
 * @param id Device ID to set (0x01-0xFE recommended, avoid 0x00 and 0xFF)
 */
void setDeviceID(uint8_t id);

#endif // __DEVICE_ID_H__
