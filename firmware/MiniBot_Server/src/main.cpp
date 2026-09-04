#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#if ENABLE_JOYSTICK_MODE
#include "JoystickTask.h"
#else
#include "GUITask.h"
#endif
#include "ESPNowTask.h"
#include "ElectromagnetTask.h"
#include "LEDStatusTask.h"
#include "QueueStructs.h"
#include "SerialTask.h"

// WiFi Configuration
const char *ssid = "ChessBot-Server";
const char *password = "qwertyuiop";
const int wifiChannel = 6;
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  // Initialize serial task first, to allow for debug prints if enabled
  initSerial();

  DEBUG_PRINTLN("\n\nMiniBot Server Starting...");
  initQueues();

#if ENABLE_WEB_GUI
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password, wifiChannel);
#else
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
#endif

  Serial.print("AP IP address: ");
  DEBUG_PRINTLN(WiFi.softAPIP());

  // Initialize ESP-NOW
  initESPNow();

#if ENABLE_JOYSTICK_MODE
  initJoystick();
  DEBUG_PRINTLN("Running in JOYSTICK MODE");
#endif
#if ENABLE_WEB_GUI
  initGUI();
  DEBUG_PRINTLN("Running in GUI MODE");
#endif

  // Initialize Electromagnets
  initElectromagnets();

  // Initialize LED status indicator
  initLEDStatus();

// Create FreeRTOS Tasks
#if ENABLE_JOYSTICK_MODE
  xTaskCreatePinnedToCore(joystickTask,        // Task function
                          "Joystick Task",     // Task name
                          4096,                // Stack size (bytes)
                          NULL,                // Parameter
                          2,                   // Priority
                          &joystickTaskHandle, // Task handle
                          1                    // Core (0 or 1)
  );
#endif
#if ENABLE_WEB_GUI
  xTaskCreatePinnedToCore(guiTask,        // Task function
                          "GUI Task",     // Task name
                          8192,           // Stack size (bytes)
                          NULL,           // Parameter
                          2,              // Priority
                          &guiTaskHandle, // Task handle
                          0               // Core (0 or 1)
  );
#endif

  xTaskCreatePinnedToCore(espNowTask,        // Task function
                          "ESP-NOW Task",    // Task name
                          4096,              // Stack size (bytes)
                          NULL,              // Parameter
                          3,                 // Priority
                          &espNowTaskHandle, // Task handle
                          0                  // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(electromagnetTask, // Task function
                          "Emag Task",       // Task name
                          2048,              // Stack size (bytes)
                          NULL,              // Parameter
                          4,                 // Priority (lower than comm/gui)
                          &emagTaskHandle,   // Task handle
                          1                  // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(ledStatusTask,        // Task function
                          "LED Task",           // Task name
                          2048,                 // Stack size (bytes)
                          NULL,                 // Parameter
                          1,                    // Priority
                          &ledStatusTaskHandle, // Task handle
                          0                     // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(serialTask,        // Task function
                          "Serial Task",     // Task name
                          4096,              // Stack size (bytes)
                          NULL,              // Parameter
                          3,                 // Priority
                          &serialTaskHandle, // Task handle
                          0                  // Core (0 or 1)
  );

  Serial.println("Setup complete!");
}

// ============================================
// Loop (unused - FreeRTOS tasks handle everything)
// ============================================
void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}