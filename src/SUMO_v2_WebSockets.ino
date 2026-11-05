#include <Arduino.h>

/*
 * TC1508 + ESP32-C3 Super Mini Web UI Controller (WebSocket edition)
 * -----------------------------------------------------------------
 * Creates a WiFi access point with a mobile-friendly web UI:
 *   - Touch joystick for differential drive of the 2 DC motors
 *   - 2 buttons that cycle each servo through 0° / 90° / 180°
 *   - All control goes over a single persistent WebSocket (sub-50ms lag)
 *
 * Wiring:
 *   Motors:
 *     GPIO 21 -> INT1  (Motor A dir A)
 *     GPIO 20 -> INT2  (Motor A dir B)
 *     GPIO 10 -> INT3  (Motor B dir A)
 *     GPIO  9 -> INT4  (Motor B dir B)
 *   Servos:
 *     GPIO  5 -> Servo 1 signal
 *     GPIO  6 -> Servo 2 signal
 *
 * How to use:
 *   1) Install these libraries via Library Manager:
 *        - "ESP32Servo" by Kevin Harrington / John K. Bennett
 *        - "WebSockets" by Markus Sattler  (a.k.a. arduinoWebSockets)
 *   2) Upload this sketch.
 *   3) On your phone/computer, join WiFi network "ESP32-Robot".
 *   4) Open http://192.168.4.1 (or http://robot.local).
 *
 *   To use station mode (join your home WiFi), set USE_AP_MODE = false
 *   and fill in WIFI_SSID / WIFI_PASS below.
 *
 * Requires Arduino-ESP32 core 2.x (legacy ledcSetup / ledcAttachPin API).
 *
 * LEDC channel allocation on ESP32-C3 (6 channels, 3 timers in use):
 *   Ch 0,1: servos (50 Hz, 14-bit)   <- auto-allocated by ESP32Servo library
 *   Ch 2,3: Motor A dir A/B (5 kHz, 8-bit)  -> timer 1
 *   Ch 4,5: Motor B dir A/B (5 kHz, 8-bit)  -> timer 2
 *
 * IMPORTANT: GPIO 20/21 are UART0. Do NOT call Serial.begin() — it would
 *            reclaim those pins and break Motor A. This sketch does not
 *            use Serial at all.
 *
 * WebSocket protocol:
 *   Client -> Server text frames:
 *     {"cmd":"motor","l":<int>,"r":<int>}     l,r in -255..255
 *     {"cmd":"servo","id":0|1,"cycle":1}
 *     {"cmd":"stop"}
 *   Server -> Client text frames:
 *     {"type":"state","s1":<0|90|180>,"s2":<0|90|180>}
 *     {"type":"pong"}
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ---- Network config ------------------------------------------------------
#define USE_AP_MODE   true              // true = ESP32 creates its own WiFi
const char* AP_SSID   = "sumo";
const char* AP_PASS   = "12345678";             // empty = open network
const char* WIFI_SSID = "YOUR_SSID";    // used only if USE_AP_MODE = false
const char* WIFI_PASS = "YOUR_PASS";

// ---- Pin map -------------------------------------------------------------
static const uint8_t PIN_INT1   = 21;  // Motor A dir A
static const uint8_t PIN_INT2   = 20;  // Motor A dir B
static const uint8_t PIN_INT3   = 10;  // Motor B dir A
static const uint8_t PIN_INT4   = 9;   // Motor B dir B
static const uint8_t PIN_SERVO1 = 5;
static const uint8_t PIN_SERVO2 = 6;

// ---- PWM: motors (5 kHz, 8-bit) ------------------------------------------
static const uint8_t  MOTOR_PWM_RES  = 8;
static const uint32_t MOTOR_PWM_FREQ = 5000;
static const uint8_t CH_INT1 = 2, CH_INT2 = 3, CH_INT3 = 4, CH_INT4 = 5;

// ---- Servos (via ESP32Servo library) -------------------------------------
Servo servo1;
Servo servo2;

const int SERVO_PRESETS[] = {0, 90, 180};
const int NUM_PRESETS = 3;
int servo1Idx = 1, servo2Idx = 1;  // both start at 90°

int getServoAngle(int id) {
  return (id == 0) ? SERVO_PRESETS[servo1Idx] : SERVO_PRESETS[servo2Idx];
}
void applyServo(int id) {
  int angle = getServoAngle(id);
  if (id == 0) servo1.write(angle);
  else         servo2.write(angle);
}
void cycleServo(int id) {
  if (id == 0) servo1Idx = (servo1Idx + 1) % NUM_PRESETS;
  else         servo2Idx = (servo2Idx + 1) % NUM_PRESETS;
  applyServo(id);
}

// ---- Motor abstraction ---------------------------------------------------
struct Motor { uint8_t inA_ch; uint8_t inB_ch; };
Motor motorA = { CH_INT1, CH_INT2 };  // INT1/INT2  (left)
Motor motorB = { CH_INT3, CH_INT4 };  // INT3/INT4  (right)

inline void writeMotor(const Motor &m, uint32_t dutyA, uint32_t dutyB) {
  ledcWrite(m.inA_ch, dutyA);
  ledcWrite(m.inB_ch, dutyB);
}

// speed: -255..255 (negative = reverse)
void driveMotorSigned(const Motor &m, int speed) {
  int s = constrain(speed, -255, 255);
  uint32_t duty = (uint32_t)abs(s);
  if      (s > 0) writeMotor(m, duty, 0);
  else if (s < 0) writeMotor(m, 0, duty);
  else            writeMotor(m, 0, 0);
}

void stopAll() {
  driveMotorSigned(motorA, 0);
  driveMotorSigned(motorB, 0);
}

// ---- Watchdog: auto-stop motors if no command in 1 s ---------------------
int lastL = 0, lastR = 0;
unsigned long lastMotorCmd = 0;
const unsigned long MOTOR_TIMEOUT_MS = 1000;

// ---- HTTP server (serves only the page) + WebSocket server ---------------
WebServer server(80);
WebSocketsServer ws(81);

// Broadcast current servo state to every connected client.
void broadcastState() {
  JsonDocument doc;
  doc["type"] = "state";
  doc["s1"]   = getServoAngle(0);
  doc["s2"]   = getServoAngle(1);
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
}

// ---- WebSocket event handler ---------------------------------------------
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      // Client gone: ensure motors stop so a dropped phone doesn't leave
      // the robot driving into a wall.
      stopAll();
      lastL = 0; lastR = 0;
      break;

    case WStype_CONNECTED:
      // Send current state to the newly connected client.
      broadcastState();
      break;

    case WStype_TEXT: {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload, length);
      if (err) break;
      const char* cmd = doc["cmd"];
      if (!cmd) break;

      if (strcmp(cmd, "motor") == 0) {
        int l = doc["l"] | 0;
        int r = doc["r"] | 0;
        driveMotorSigned(motorA, l);
        driveMotorSigned(motorB, r);
        lastL = l; lastR = r;
        lastMotorCmd = millis();
      } else if (strcmp(cmd, "servo") == 0) {
        int id = doc["id"] | 0;
        id = constrain(id, 0, 1);
        cycleServo(id);
        broadcastState();    // push new angle to all clients immediately
      } else if (strcmp(cmd, "stop") == 0) {
        stopAll();
        lastL = 0; lastR = 0;
      } else if (strcmp(cmd, "ping") == 0) {
        ws.sendTXT(num, "{\"type\":\"pong\"}");
      }
      break;
    }

    case WStype_ERROR:
      break;

    default:
      break;
  }
}

// ---- HTML page -----------------------------------------------------------
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>ESP32-C3 Robot</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    background: linear-gradient(180deg, #1a1a1a, #0f0f0f);
    color: #fff;
    min-height: 100vh;
    display: flex; flex-direction: column; align-items: center;
    padding: 20px;
    user-select: none; -webkit-user-select: none;
    touch-action: manipulation;
  }
  h1 { font-size: 1.15rem; font-weight: 600; letter-spacing: 0.5px; }
  .status {
    display: flex; align-items: center;
    font-size: 0.8rem; color: #888; margin-top: 4px;
  }
  .status .dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: #555; margin-right: 6px;
    transition: background 0.3s, box-shadow 0.3s;
  }
  .status.online .dot { background: #4ade80; box-shadow: 0 0 6px #4ade80; }
  .status.offline .dot { background: #ef4444; }
  .ip { font-size: 0.7rem; color: #555; margin-top: 2px; }

  .joystick-wrap { margin: 28px 0 16px; }
  .joystick {
    width: 240px; height: 240px; border-radius: 50%;
    background: radial-gradient(circle at 50% 50%, #2a2a2a 0%, #1a1a1a 70%);
    border: 2px solid #333;
    position: relative;
    touch-action: none;
    box-shadow: inset 0 4px 16px rgba(0,0,0,0.5);
  }
  .joystick::before {
    content: ''; position: absolute; inset: 25%;
    border: 1px dashed #333; border-radius: 50%;
  }
  .joystick::after {
    content: ''; position: absolute;
    top: 50%; left: 50%;
    width: 4px; height: 4px; margin: -2px 0 0 -2px;
    background: #444; border-radius: 50%;
  }
  .stick {
    position: absolute;
    top: 50%; left: 50%;
    width: 80px; height: 80px; margin: -40px 0 0 -40px;
    border-radius: 50%;
    background: radial-gradient(circle at 30% 30%, #5aa0ff, #2255cc);
    box-shadow: 0 6px 16px rgba(0,0,0,0.6), inset 0 -3px 6px rgba(0,0,0,0.3);
    pointer-events: none;
    transition: transform 0.04s linear;
  }

  .servo-row { display: flex; gap: 16px; margin-top: 8px; }
  .servo-btn {
    background: #252525;
    border: 2px solid #3a3a3a;
    color: #fff;
    padding: 14px 28px;
    border-radius: 14px;
    font-size: 0.85rem;
    cursor: pointer;
    min-width: 130px;
    text-align: center;
    transition: background 0.15s, transform 0.1s;
    font-family: inherit;
  }
  .servo-btn:active { background: #353535; transform: scale(0.96); }
  .servo-btn .angle {
    display: block; font-size: 1.5rem; font-weight: 700;
    color: #5aa0ff; margin-top: 4px;
  }

  .stop-btn {
    margin-top: 22px;
    background: #dc2626; color: #fff; border: none;
    padding: 14px 56px; border-radius: 14px;
    font-size: 1rem; font-weight: 700; letter-spacing: 1px;
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(220,38,38,0.4);
    transition: background 0.15s, transform 0.1s;
    font-family: inherit;
  }
  .stop-btn:active { background: #b91c1c; transform: scale(0.96); }

  .hint { font-size: 0.7rem; color: #555; margin-top: 20px;
          text-align: center; line-height: 1.5; }
</style>
</head>
<body>
  <h1>ESP32-C3 Robot Control</h1>
  <div class="status" id="status"><span class="dot"></span><span class="text">Connecting...</span></div>
  <div class="ip">192.168.4.1</div>

  <div class="joystick-wrap">
    <div class="joystick" id="joystick">
      <div class="stick" id="stick"></div>
    </div>
  </div>

  <div class="servo-row">
    <button class="servo-btn" data-id="0">Servo 1<span class="angle" id="ang0">90&deg;</span></button>
    <button class="servo-btn" data-id="1">Servo 2<span class="angle" id="ang1">90&deg;</span></button>
  </div>

  <button class="stop-btn" id="stop">STOP</button>

  <div class="hint">Joystick: drive &bull; Buttons: cycle servo position<br>Motors auto-stop 1 s after release.</div>

<script>
(function() {
  var joy = document.getElementById('joystick');
  var stick = document.getElementById('stick');
  var dragging = false;
  var radius = 100;

  // ---- WebSocket setup --------------------------------------------------
  var ws = null;
  var statusEl = document.getElementById('status');
  var statusText = statusEl.querySelector('.text');
  var everConnected = false;

  function markOnline() {
    statusEl.classList.add('online');
    statusEl.classList.remove('offline');
    statusText.textContent = 'Connected';
  }
  function markOffline() {
    statusEl.classList.add('offline');
    statusEl.classList.remove('online');
    statusText.textContent = everConnected ? 'Reconnecting...' : 'Connecting...';
  }

  function connectWS() {
    var proto = (location.protocol === 'https:') ? 'wss:' : 'ws:';
    var url = proto + '//' + location.hostname + ':81/';
    ws = new WebSocket(url);

    ws.onopen = function() {
      everConnected = true;
      markOnline();
      // Ask for current state in case we reconnected mid-session
      send({cmd:'ping'});
    };
    ws.onclose = function() { markOffline(); setTimeout(connectWS, 800); };
    ws.onerror = function() { try { ws.close(); } catch(e){} };
    ws.onmessage = function(ev) {
      var msg;
      try { msg = JSON.parse(ev.data); } catch(e) { return; }
      if (msg.type === 'state') {
        if (typeof msg.s1 === 'number') document.getElementById('ang0').textContent = msg.s1 + '\u00B0';
        if (typeof msg.s2 === 'number') document.getElementById('ang1').textContent = msg.s2 + '\u00B0';
      }
      // pong messages just keep the connection alive; no UI action needed
    };
  }

  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  // ---- Joystick ---------------------------------------------------------
  var lastSend = 0;

  function updateRadius() { radius = joy.offsetWidth / 2 - 20; }

  function handleMove(e) {
    if (!dragging) return;
    e.preventDefault();
    var rect = joy.getBoundingClientRect();
    var cx = rect.left + rect.width / 2;
    var cy = rect.top + rect.height / 2;
    var dx = e.clientX - cx;
    var dy = e.clientY - cy;
    var dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > radius) {
      dx = dx * radius / dist;
      dy = dy * radius / dist;
    }
    stick.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
    var nx = dx / radius;
    var ny = -dy / radius;
    // Differential drive: motorA(left) = y + x, motorB(right) = y - x
    var l = Math.round(Math.max(-1, Math.min(1, ny + nx)) * 255);
    var r = Math.round(Math.max(-1, Math.min(1, ny - nx)) * 255);
    var now = Date.now();
    if (now - lastSend > 60) {        // ~16 updates/sec max
      lastSend = now;
      send({cmd:'motor', l:l, r:r});
    }
  }

  function endDrag() {
    if (!dragging) return;
    dragging = false;
    stick.style.transform = 'translate(0,0)';
    send({cmd:'motor', l:0, r:0});
  }

  joy.addEventListener('pointerdown', function(e) {
    dragging = true;
    updateRadius();
    joy.setPointerCapture(e.pointerId);
    handleMove(e);
  });
  joy.addEventListener('pointermove', handleMove);
  joy.addEventListener('pointerup', endDrag);
  joy.addEventListener('pointercancel', endDrag);
  window.addEventListener('resize', updateRadius);
  updateRadius();

  // ---- Servo buttons ----------------------------------------------------
  var servoBtns = document.querySelectorAll('.servo-btn');
  for (var i = 0; i < servoBtns.length; i++) {
    (function(btn) {
      btn.addEventListener('click', function() {
        send({cmd:'servo', id:parseInt(btn.getAttribute('data-id'),10), cycle:1});
      });
    })(servoBtns[i]);
  }

  // ---- Stop button ------------------------------------------------------
  document.getElementById('stop').addEventListener('click', function() {
    endDrag();
    send({cmd:'stop'});
  });

  // ---- Heartbeat (detect silent disconnects) ----------------------------
  setInterval(function() {
    if (ws && ws.readyState === WebSocket.OPEN) send({cmd:'ping'});
  }, 5000);

  connectWS();
})();
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

// ---- Arduino setup / loop -----------------------------------------------
void setup() {
  // No Serial.begin() - GPIO20/21 must stay free for motor PWM.

  // Motor PWM (channels 2-5, leave 0-1 free for ESP32Servo library).
  ledcSetup(CH_INT1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(CH_INT2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(CH_INT3, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(CH_INT4, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(PIN_INT1, CH_INT1);
  ledcAttachPin(PIN_INT2, CH_INT2);
  ledcAttachPin(PIN_INT3, CH_INT3);
  ledcAttachPin(PIN_INT4, CH_INT4);
  stopAll();

  // Servos via ESP32Servo library - auto-allocates LEDC channels 0 and 1,
  // uses 14-bit timer resolution (ESP32-C3 max), 50 Hz, ~544..2400 us pulse.
  ESP32PWM::allocateTimer(0);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(PIN_SERVO1, 544, 2400);
  servo2.attach(PIN_SERVO2, 544, 2400);
  applyServo(0);
  applyServo(1);

  // WiFi
#if USE_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
  }
#endif

  // mDNS for http://robot.local
  MDNS.begin("robot");
  MDNS.addService("http", "tcp", 80);

  // HTTP server: just serves the page. Everything else is WebSocket on :81.
  server.on("/", handleRoot);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();

  // WebSocket server on port 81
  ws.begin();
  ws.onEvent(onWebSocketEvent);
}

void loop() {
  server.handleClient();
  ws.loop();

  // Watchdog: stop motors if no command received within timeout
  if ((lastL != 0 || lastR != 0) && (millis() - lastMotorCmd > MOTOR_TIMEOUT_MS)) {
    stopAll();
    lastL = 0; lastR = 0;
  }
}
