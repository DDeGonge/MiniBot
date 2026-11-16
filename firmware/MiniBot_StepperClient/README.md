# MiniBot StepperClient Firmware

ESP32-C3 firmware for each individual MiniBot. Handles motion control, wireless
communication, position estimation, battery management, and over-the-air firmware updates via six
concurrent FreeRTOS tasks.

Built with PlatformIO. All tasks are created in `src/main.cpp`; configuration
constants are in `include/config.h`. Device ID is read from NVS at boot.

Note: Status LED task is disabled on v3 boards and left commented out for future use.

---

## FreeRTOS Tasks

| Task | Priority | Main Loop |
|------|----------|-----------|
| `KinematicsController` | 5 | Dequeue `MotionCommand`, compute inverse kinematics (Cartesian to wheel velocities), drive steppers via ESP32 RMT peripheral; also handles `MotorTestQueue` for direct velocity commands |
| `PositionEstimator_Sensor` | 4 | Read MMC5633 magnetometer at high rate, detect sync frame start, timestamp samples per electromagnet slot, post complete `EmagFrameData` to internal queue |
| `EspNowCommunicator` | 3 | Receive ESP-NOW messages, dispatch to `MotionQueue` or `MotorTestQueue`, send `AckMessage` or `NackMessage` back to server; manages radio duty cycling for power savings |
| `PositionEstimator_Calc` | 2 | Dequeue `EmagFrameData`, trilaterate position and orientation from electromagnet readings, apply confidence-weighted low-pass filter, call `robot.setTruePose()` |
| `OtaUpdater` | 2 | Wait on the `OtaCommand` semaphore, skip the flash when the NVS version stamp matches, otherwise join the Server's WiFi AP, `GET /firmware.bin`, verify size + CRC-32, write the image into the alternate app partition with `Update.begin/write/end`, persist the new version to NVS, reply `OtaResult`, and reboot; disables motors and suspends the four tasks above it for the duration |
| `BatteryMonitor` | 1 | Periodically read battery voltage, update robot state; suspend all other tasks if voltage drops to a critical level |

The `KinematicsController` uses the ESP32 RMT peripheral to generate stepper pulse
trains without blocking the CPU. Inter-task communication uses FreeRTOS queues
wrapped in `MotionQueue` and `MotorTestQueue` helper modules.

---

## Key Objects

| Object | Source | Purpose |
|--------|--------|---------|
| `Robot` | `robot.h` | Central state: position, orientation, true pose estimate, battery voltage; shared across all tasks |
| `StepperDriver` | `stepper.h` | One wheel: step/dir GPIO, direction, RMT channel, microstepping mode |
| `MotionCommand` | `messages_ipc.h` | IPC struct passed through `MotionQueue`: target x/y/theta + duration |
| `MotorTestRequest` | `messages_ipc.h` | IPC struct passed through `MotorTestQueue`: left/right wheel velocities |
| `EmagFrameData` | `position_estimator.h` | One complete localization frame: magnetometer samples per electromagnet slot + background |
| `CalculatedPosition` | `position_estimator.h` | Trilateration result: x, y, orientation, confidence score |

---

## Build

```bash
pio run --target upload
```

Before flashing, write the device ID to NVS using the provisioning utility or
set it directly via the serial console.

OTA updates need the two-app layout declared in `partitions.csv` (`factory` + `recovery`,
`0xF0000` = 960 KiB each, table at `0x8000`). Without that file the core falls back to a single
`factory` app slot and `Update.begin()` in `ota_updater.cpp` always reports "partition not
found"; if the core does not pick the csv up on its own, reflash the layout once:

```bash
esptool.py --partition-table=partitions.csv --partition-table-offset=0x8000 partition-table
```

Two slots are what make the update safe: the ESP OTA API writes the slot that is not currently
running, and `main.cpp` calls `esp_ota_mark_app_valid_cancel_rollback()` at boot to accept the
new image, so an image that fails to boot rolls back to the previous slot on the next reset. The
staged image a bot receives has to fit one 960 KiB slot. The device ID and the OTA version stamp
both live in NVS, which this table does not declare - confirm it is present on the board.
