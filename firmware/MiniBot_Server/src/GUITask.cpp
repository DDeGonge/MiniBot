#include "GUITask.h"
#include "ElectromagnetTask.h"
#include "QueueStructs.h"

// Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Status storage for 36 robots (12x3 grid)
GUIStatus robotStatus[36];

// Task handle
TaskHandle_t guiTaskHandle = NULL;

// HTML Webpage
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>MiniBot Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; margin: 10px; background-color: #1a1a1a; color: #fff; }
    h1 { text-align: center; color: #3e8bcaff; }
    .grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 10px; margin: 20px auto; max-width: 2400px; }
    .cell { background: #2a2a2a; border: 2px solid #444; border-radius: 8px; padding: 10px; }
    .cell-header { background: #3a3a3a; padding: 5px; margin: -10px -10px 10px -10px; border-radius: 6px 6px 0 0; text-align: center; font-weight: bold; }
    .input-group { margin: 5px 0; }
    label { display: block; font-size: 11px; color: #aaa; margin-bottom: 2px; }
    input[type="text"], input[type="number"] { width: 100%; padding: 4px; box-sizing: border-box; background: #1a1a1a; color: #fff; border: 1px solid #555; border-radius: 4px; font-size: 12px; }
    button { width: 100%; padding: 8px; margin-top: 5px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; }
    button:hover { background: #45a049; }
    button:active { background: #3d8b40; }
    .status { margin-top: 10px; padding-top: 10px; border-top: 1px solid #444; font-size: 11px; }
    .status div { margin: 3px 0; }
    .status-label { color: #888; }
    .status-value { color: #4CAF50; font-weight: bold; }
    .status-error { color: #c54a4aff; font-weight: bold; }
    .btn-request { background: #2196F3; margin-top: 5px; }
    .btn-request:hover { background: #1976D2; }
    .btn-request:active { background: #1565C0; }
    input[type="text"]:focus, input[type="number"]:focus { outline: none; border-color: #4CAF50; }
    .control-panel { background: #2a2a2a; border: 2px solid #444; border-radius: 8px; padding: 15px; margin: 20px auto; max-width: 2400px; }
    .switch { position: relative; display: inline-block; width: 60px; height: 34px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: #4CAF50; }
    input:checked + .slider:before { transform: translateX(26px); }
    .control-row { display: flex; align-items: center; margin: 10px 0; }
    .control-label { margin-right: 15px; font-size: 16px; min-width: 200px; }
  </style>
</head>
<body>
  <h1>ChessBot Debug Controller</h1>
  
  <div class="control-panel">
    <div class="control-row">
      <span class="control-label">Electromagnet Control:</span>
      <label class="switch">
        <input type="checkbox" id="emagToggle" onchange="toggleElectromagnet()">
        <span class="slider"></span>
      </label>
      <span id="emagStatus" style="margin-left: 15px; color: #888;">OFF</span>
    </div>
    <div class="control-row">
      <span class="control-label">Position Sync:</span>
      <button onclick="sendSyncCommand()" style="width: auto; padding: 8px 20px; background: #FF9800;">SYNC POSITIONS</button>
    </div>
  </div>
  
  <div class="grid" id="robotGrid"></div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    var robotData = {};

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = onOpen;
      websocket.onclose = onClose;
      websocket.onmessage = onMessage;
    }

    function onOpen(event) {
      console.log('WebSocket Connected');
    }

    function onClose(event) {
      console.log('WebSocket Disconnected');
      setTimeout(initWebSocket, 2000);
    }

    function onMessage(event) {
      var data = JSON.parse(event.data);
      robotData[data.id] = data;
      updateStatus(data.id);
    }

    function sendCommand(id) {
      var targetId = document.getElementById('tid_' + id).value;
      var x = document.getElementById('x_' + id).value;
      var y = document.getElementById('y_' + id).value;
      var angle = document.getElementById('a_' + id).value;
      var duration = document.getElementById('d_' + id).value;
      
      var message = 'cmd,' + targetId + ',' + x + ',' + y + ',' + angle + ',' + duration;
      websocket.send(message);
      console.log('Sent: ' + message);
    }

    function requestPosition(id) {
      var targetId = document.getElementById('tid_' + id).value;
      var message = 'req,' + targetId;
      websocket.send(message);
      console.log('Position request: ' + message);
    }

    function requestMagnetField(id) {
      var targetId = document.getElementById('tid_' + id).value;
      var message = 'mag,' + targetId;
      websocket.send(message);
      console.log('Magnet field request: ' + message);
    }

    function toggleElectromagnet() {
      var enabled = document.getElementById('emagToggle').checked;
      var message = 'emag,' + (enabled ? '1' : '0');
      websocket.send(message);
      document.getElementById('emagStatus').innerHTML = enabled ? 
        '<span style="color: #4CAF50;">ON</span>' : 
        '<span style="color: #888;">OFF</span>';
      console.log('Electromagnet: ' + message);
    }

    function sendSyncCommand() {
      websocket.send('sync');
      console.log('Sync command sent');
    }

    function updateStatus(id) {
      var data = robotData[id];
      if (data) {
        if (data.magValid) {
          document.getElementById('mag_' + id).innerHTML = 
            'Magnet: <span class="status-value">[' + data.magX.toFixed(2) + ', ' + data.magY.toFixed(2) + ', ' + data.magZ.toFixed(2) + ']</span>';
        } else {
          document.getElementById('mag_' + id).innerHTML = 
            'Magnet: <span class="status-label">--</span>';
        }
        
        if (data.ack === false) {
          document.getElementById('pos_' + id).innerHTML = 
            'Position: <span class="status-error">NO ACK</span>';
          document.getElementById('ang_' + id).innerHTML = 
            'Angle: <span class="status-error">TIMEOUT</span>';
          document.getElementById('ts_' + id).innerHTML = 
            'Last Update: <span class="status-error">--</span>';
          document.getElementById('batt_' + id).innerHTML = 
            'Battery: <span class="status-error">--</span>';
        } else {
          document.getElementById('pos_' + id).innerHTML = 
            'Position: <span class="status-value">(' + data.x.toFixed(2) + ', ' + data.y.toFixed(2) + ')</span>';
          document.getElementById('ang_' + id).innerHTML = 
            'Angle: <span class="status-value">' + data.angle.toFixed(3) + ' rad</span>';
          document.getElementById('ts_' + id).innerHTML = 
            'Last Update: <span class="status-value">' + data.ts + ' ms</span>';
          document.getElementById('batt_' + id).innerHTML = 
            'Battery: <span class="status-value">' + data.batt.toFixed(2) + 'V</span>';
        }
        if (data.syncStatus === 1) {
          document.getElementById('sync_' + id).innerHTML =
            'Sync: <span class="status-value">ACK &#10003;</span>';
        } else if (data.syncStatus === 2) {
          document.getElementById('sync_' + id).innerHTML =
            'Sync: <span class="status-error">NACK &#10007;</span>';
        }
      }
    }

    function createGrid() {
      var grid = document.getElementById('robotGrid');
      for (var i = 1; i < 37; i++) {
        var cell = document.createElement('div');
        cell.className = 'cell';
        
        var hexId = '0x' + (i < 16 ? '0' : '') + i.toString(16).toUpperCase();
        
        cell.innerHTML = `
          <div class="cell-header">Robot ${i}</div>
          <div class="input-group">
            <label>Target ID:</label>
            <input type="text" id="tid_${i}" value="${hexId}">
          </div>
          <div class="input-group">
            <label>X Target (mm):</label>
            <input type="number" id="x_${i}" value="0.0" step="1">
          </div>
          <div class="input-group">
            <label>Y Target (mm):</label>
            <input type="number" id="y_${i}" value="0.0" step="1">
          </div>
          <div class="input-group">
            <label>Angle (rad):</label>
            <input type="number" id="a_${i}" value="0.0" step="0.1">
          </div>
          <div class="input-group">
            <label>Duration (s):</label>
            <input type="number" id="d_${i}" value="1.0" step="0.1">
          </div>
          <button onclick="sendCommand(${i})">SEND</button>
          <button class="btn-request" onclick="requestPosition(${i})">REQUEST POSE</button>
          <button class="btn-request" onclick="requestMagnetField(${i})">REQUEST MAGNET</button>
          <div class="status">
            <div id="mag_${i}" class="status-label">Magnet: --</div>
            <div id="pos_${i}" class="status-label">Position: --</div>
            <div id="ang_${i}" class="status-label">Angle: --</div>
            <div id="ts_${i}" class="status-label">Last Update: --</div>
            <div id="batt_${i}" class="status-label">Battery: --</div>
            <div id="sync_${i}" class="status-label">Sync: --</div>
          </div>
        `;
        
        grid.appendChild(cell);
        robotData[i] = {id: i, x: 0, y: 0, angle: 0, ts: 0, batt: 0, ack: true, magX: 0, magY: 0, magZ: 0, magValid: false, syncStatus: 0};
      }
    }

    window.addEventListener('load', function() {
      createGrid();
      initWebSocket();
    });
  </script>
</body>
</html>
)rawliteral";

// WebSocket event handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    DEBUG_PRINTF("WebSocket client #%u connected\n", client->id());
    // Don't send initial status - client will get updates as they occur
    // This prevents overwhelming the WebSocket queue
  } else if (type == WS_EVT_DISCONNECT) {
    DEBUG_PRINTF("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char *)data;

      // Check message type: "cmd,..." or "req,..."
      if (message.startsWith("cmd,")) {
        // Parse command: format "cmd,id,x,y,angle,duration"
        CommandMessage msg = {};
        msg.commandType = CMD_TYPE_GUI;
        GUICommand &cmd = msg.data.guiCmd;
        cmd.requestType = 0; // Position command

        int idx1 = message.indexOf(',', 4); // After "cmd,"
        int idx2 = message.indexOf(',', idx1 + 1);
        int idx3 = message.indexOf(',', idx2 + 1);
        int idx4 = message.indexOf(',', idx3 + 1);

        if (idx1 > 0 && idx2 > 0 && idx3 > 0 && idx4 > 0) {
          String targetIdStr = message.substring(4, idx1);
          // Parse hex string (e.g., "0x05" or "5")
          if (targetIdStr.startsWith("0x") || targetIdStr.startsWith("0X")) {
            cmd.targetID = (uint8_t)strtol(targetIdStr.c_str(), NULL, 16);
          } else {
            cmd.targetID = targetIdStr.toInt();
          }

          cmd.x = message.substring(idx1 + 1, idx2).toFloat();
          cmd.y = message.substring(idx2 + 1, idx3).toFloat();
          cmd.angle = message.substring(idx3 + 1, idx4).toFloat();
          cmd.duration = message.substring(idx4 + 1).toFloat();

          // Send to command queue
          if (xQueueSend(commandQueue, &msg, 0) == pdPASS) {
            DEBUG_PRINTF("Command queued for robot 0x%02X: x=%.2f y=%.2f "
                         "a=%.2f d=%.2f\n",
                         cmd.targetID, cmd.x, cmd.y, cmd.angle, cmd.duration);
          } else {
            DEBUG_PRINTLN("Command queue full!");
          }
        } else {
          DEBUG_PRINTLN("Failed to parse command message");
        }
      } else if (message.startsWith("req,")) {
        // Parse request: format "req,id"
        CommandMessage msg = {};
        msg.commandType = CMD_TYPE_GUI;
        GUICommand &cmd = msg.data.guiCmd;
        cmd.requestType = 1; // Position request

        String targetIdStr = message.substring(4);
        // Parse hex string (e.g., "0x05" or "5")
        if (targetIdStr.startsWith("0x") || targetIdStr.startsWith("0X")) {
          cmd.targetID = (uint8_t)strtol(targetIdStr.c_str(), NULL, 16);
        } else {
          cmd.targetID = targetIdStr.toInt();
        }

        cmd.x = 0;
        cmd.y = 0;
        cmd.angle = 0;
        cmd.duration = 0;

        // Send to command queue
        if (xQueueSend(commandQueue, &msg, 0) == pdPASS) {
          DEBUG_PRINTF("Position request queued for robot 0x%02X\n",
                       cmd.targetID);
        } else {
          DEBUG_PRINTLN("Command queue full!");
        }
      } else if (message.startsWith("mag,")) {
        // Parse magnet field request: format "mag,id"
        CommandMessage msg = {};
        msg.commandType = CMD_TYPE_GUI;
        GUICommand &cmd = msg.data.guiCmd;
        cmd.requestType = 2; // Magnet field request

        String targetIdStr = message.substring(4);
        // Parse hex string (e.g., "0x05" or "5")
        if (targetIdStr.startsWith("0x") || targetIdStr.startsWith("0X")) {
          cmd.targetID = (uint8_t)strtol(targetIdStr.c_str(), NULL, 16);
        } else {
          cmd.targetID = targetIdStr.toInt();
        }

        cmd.x = 0;
        cmd.y = 0;
        cmd.angle = 0;
        cmd.duration = 0;

        // Send to command queue
        if (xQueueSend(commandQueue, &msg, 0) == pdPASS) {
          DEBUG_PRINTF("Magnet field request queued for robot 0x%02X\n",
                       cmd.targetID);
        } else {
          DEBUG_PRINTLN("Command queue full!");
        }
      } else if (message.startsWith("emag,")) {
        // Parse electromagnet control: format "emag,0" or "emag,1"
        int enabled = message.substring(5).toInt();
        setElectromagnetEnabled(enabled == 1);
        DEBUG_PRINTF("Electromagnet control set to: %s\n",
                     enabled ? "ENABLED" : "DISABLED");
      } else if (message == "sync") {
        // Position sync: broadcast PosSyncCommand to all units
        CommandMessage syncMsg = {};
        syncMsg.commandType = CMD_TYPE_POS_SYNC;
        if (xQueueSend(commandQueue, &syncMsg, 0) == pdPASS) {
          DEBUG_PRINTLN("Pos sync command queued");
        } else {
          DEBUG_PRINTLN("Command queue full!");
        }
      }
    }
  }
}

// Initialize GUI components
void initGUI() {
  // Initialize robot status array
  for (int i = 0; i < 36; i++) {
    robotStatus[i].targetID = i;
    robotStatus[i].ackReceived = true;
    robotStatus[i].currentX = 0.0;
    robotStatus[i].currentY = 0.0;
    robotStatus[i].currentAngle = 0.0;
    robotStatus[i].timestamp = 0;
    robotStatus[i].batteryVoltage = 0.0;
    robotStatus[i].magnetX_gauss = 0.0;
    robotStatus[i].magnetY_gauss = 0.0;
    robotStatus[i].magnetZ_gauss = 0.0;
    robotStatus[i].magnetFieldValid = false;
  }

  // Setup WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Setup web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
  DEBUG_PRINTLN("Web server started");
}

// FreeRTOS GUI Task
void guiTask(void *parameter) {
  DEBUG_PRINTLN("GUI Task started");

  GUIStatus status;

  while (1) {
    // Check for status updates from communicator
    if (xQueueReceive(guiStatusQueue, &status, pdMS_TO_TICKS(10)) == pdPASS) {
      // Update robot status array
      if (status.targetID < 36) {
        // Preserve battery voltage if sentinel value (-1.0f) is used
        if (status.batteryVoltage >= 0.0f) {
          robotStatus[status.targetID].batteryVoltage = status.batteryVoltage;
        }
        // Update other fields
        robotStatus[status.targetID].targetID = status.targetID;
        robotStatus[status.targetID].ackReceived = status.ackReceived;
        robotStatus[status.targetID].currentX = status.currentX;
        robotStatus[status.targetID].currentY = status.currentY;
        robotStatus[status.targetID].currentAngle = status.currentAngle;
        robotStatus[status.targetID].timestamp = status.timestamp;
        robotStatus[status.targetID].magnetX_gauss = status.magnetX_gauss;
        robotStatus[status.targetID].magnetY_gauss = status.magnetY_gauss;
        robotStatus[status.targetID].magnetZ_gauss = status.magnetZ_gauss;
        robotStatus[status.targetID].magnetFieldValid = status.magnetFieldValid;
        if (status.syncStatus != 0) {
          robotStatus[status.targetID].syncStatus = status.syncStatus;
        }

        // Only send if we have connected clients and queue isn't full
        if (ws.count() > 0) {
          // Broadcast to all WebSocket clients
          String json =
              "{\"id\":" + String(status.targetID) +
              ",\"x\":" + String(robotStatus[status.targetID].currentX, 2) +
              ",\"y\":" + String(robotStatus[status.targetID].currentY, 2) +
              ",\"angle\":" +
              String(robotStatus[status.targetID].currentAngle, 3) +
              ",\"ts\":" + String(robotStatus[status.targetID].timestamp) +
              ",\"batt\":" +
              String(robotStatus[status.targetID].batteryVoltage, 2) +
              ",\"ack\":" +
              String(robotStatus[status.targetID].ackReceived ? "true"
                                                              : "false") +
              ",\"magX\":" +
              String(robotStatus[status.targetID].magnetX_gauss, 2) +
              ",\"magY\":" +
              String(robotStatus[status.targetID].magnetY_gauss, 2) +
              ",\"magZ\":" +
              String(robotStatus[status.targetID].magnetZ_gauss, 2) +
              ",\"magValid\":" +
              String(robotStatus[status.targetID].magnetFieldValid ? "true"
                                                                   : "false") +
              ",\"syncStatus\":" +
              String(robotStatus[status.targetID].syncStatus) + "}";

          // Try to send, but don't block if queue is full
          ws.textAll(json);

          // Small delay to prevent overwhelming the WebSocket
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (status.ackReceived) {
          DEBUG_PRINTF(
              "Status update for robot 0x%02X: (%.2f, %.2f) %.3frad %.2fV\n",
              status.targetID, status.currentX, status.currentY,
              status.currentAngle, status.batteryVoltage);
        } else if (status.magnetFieldValid) {
          DEBUG_PRINTF("Status update for robot 0x%02X: Magnet [%.2f, %.2f, "
                       "%.2f] gauss\n",
                       status.targetID, status.magnetX_gauss,
                       status.magnetY_gauss, status.magnetZ_gauss);
        } else {
          DEBUG_PRINTF("ACK timeout for robot 0x%02X\n", status.targetID);
        }
      }
    }

    // Clean up WebSocket clients
    ws.cleanupClients();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
