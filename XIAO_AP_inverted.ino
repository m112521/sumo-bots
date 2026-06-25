#include <WiFi.h>
#include <WebServer.h>

// ========== FIXES FOR ESP32-C3 ==========
#define CONFIG_LWIP_TCP_MSS 1460
#define CONFIG_LWIP_TCP_WND 29200

// ========== WI-FI (USE AP MODE FOR RELIABILITY) ==========
const char* ap_ssid = "XIAO_Robot";
const char* ap_password = "12345678";

// ========== MOTOR PINS ==========
const int leftForwardPin = D0;
const int leftBackwardPin = D1;
const int rightForwardPin = D2;
const int rightBackwardPin = D3;

// ========== LEDC PWM ==========
#define LEDC_TIMER_13_BIT  8
#define LEDC_BASE_FREQ     5000

#define CH_LEFT_FORWARD    0
#define CH_LEFT_BACKWARD   1
#define CH_RIGHT_FORWARD   2
#define CH_RIGHT_BACKWARD  3

WebServer server(80);
unsigned long lastClientCheck = 0;
bool wifiConnected = false;

// --- NEW: Individual Inversion States ---
bool isInvertedLeft = false; 
bool isInvertedRight = false; 

// HTML with Individual Invert Buttons
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SUMOBOT</title>
    <style>
        * {
            user-select: none;
            touch-action: manipulation;
            -webkit-tap-highlight-color: transparent;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        .controller {
            max-width: 500px;
            width: 100%;
            margin: 0 auto;
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 40px;
            padding: 20px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
            border: 1px solid rgba(255,255,255,0.2);
        }
        
        h1 {
            text-align: center;
            color: #fff;
            font-size: 1.5em;
            margin: 0 0 5px 0;
            letter-spacing: 2px;
        }
        
        .subtitle {
            text-align: center;
            color: #88d4ff;
            font-size: 0.8em;
            margin-bottom: 25px;
        }
        
        .joystick-area {
            background: rgba(0,0,0,0.3);
            border-radius: 30px;
            padding: 20px;
            margin-bottom: 20px;
        }
        
        .dpad {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            grid-template-rows: auto;
            gap: 12px;
            max-width: 300px;
            margin: 0 auto;
            justify-items: center;
            align-items: center;
        }
        
        .control-btn {
            width: 80px;
            height: 80px;
            border-radius: 50%;
            border: none;
            font-size: 1.8em;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.1s ease;
            box-shadow: 0 6px 0 rgba(0,0,0,0.3);
            background: #2c3e66;
            color: white;
            touch-action: manipulation;
        }
        
        .control-btn:active {
            transform: translateY(4px);
            box-shadow: 0 2px 0 rgba(0,0,0,0.3);
        }
        
        .btn-up { grid-column: 2; grid-row: 1; background: #e74c3c; box-shadow: 0 6px 0 #c0392b; }
        .btn-left { grid-column: 1; grid-row: 2; background: #3498db; box-shadow: 0 6px 0 #2980b9; }
        .btn-stop { grid-column: 2; grid-row: 2; background: #f39c12; color: #fff; font-size: 1.2em; box-shadow: 0 6px 0 #e67e22; }
        .btn-right { grid-column: 3; grid-row: 2; background: #3498db; box-shadow: 0 6px 0 #2980b9; }
        .btn-down { grid-column: 2; grid-row: 3; background: #2ecc71; box-shadow: 0 6px 0 #27ae60; }
        
        .invert-controls {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        
        .invert-btn {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 15px;
            font-size: 1em;
            font-weight: bold;
            cursor: pointer;
            color: white;
            transition: all 0.1s ease;
        }
        
        .invert-btn:active {
            transform: translateY(4px);
            box-shadow: 0 1px 0 rgba(0,0,0,0.3);
        }
        
        .invert-m1 { background: #9b59b6; box-shadow: 0 5px 0 #8e44ad; }
        .invert-m2 { background: #16a085; box-shadow: 0 5px 0 #1abc9c; }
        .inverted-state { background: #e74c3c !important; box-shadow: 0 5px 0 #c0392b !important; }
        
        .status-panel {
            background: rgba(0,0,0,0.5);
            border-radius: 20px;
            padding: 15px;
            margin-top: 20px;
            color:#fff;
        }
        
        .status-label { color: #fff; font-size: 0.8em; text-align: center; margin-bottom: 5px; }
        .command-status { color: #fff; font-size: 1.2em; text-align: center; font-weight: bold; word-break: break-word; }
        .ip-info { text-align: center; color: #fff; font-size: 0.7em; margin-top: 15px; }
        
        @media (max-width: 480px) {
            .control-btn { width: 70px; height: 70px; font-size: 1.5em; }
            .controller { padding: 15px; }
        }
        
        @media (max-width: 380px) {
            .control-btn { width: 60px; height: 60px; font-size: 1.3em; }
        }
        
        button { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        .connection-badge { text-align: center; color: #fff; font-size: 0.7em; margin-top: 10px; }
        .speed-slider { width:100%; }
        .slider-text { color: #fff; margin-bottom:1rem; }
    </style>
</head>
<body>
<div class="controller">
    <h1>🤖 SUMO BOT</h1>
    <div class="subtitle">XIAO ESP32-C3 | Remote Control</div>

    <input type="range" id="speed" class="speed-slider" min="0" max="255" value="200">
    <div class="slider-text">Motor 1 speed: <span id="speedVal">200</span></div>

    <input type="range" id="speed2" class="speed-slider" min="0" max="255" value="200">
    <div class="slider-text">Motor 2 speed: <span id="speedVal2">200</span></div>

    <div class="joystick-area">
      <div class="dpad">
        <div></div>
        <button class="control-btn btn-up" ontouchstart="send('forward')" ontouchend="send('stop')">▲</button>
        <div></div>
        <button class="control-btn btn-left" ontouchstart="send('left')" ontouchend="send('stop')">◀</button>
        <button class="control-btn btn-stop" ontouchstart="send('stop')">■</button>
        <button class="control-btn btn-right" ontouchstart="send('right')" ontouchend="send('stop')">▶</button>
        <div></div>
        <button class="control-btn btn-down" ontouchstart="send('backward')" ontouchend="send('stop')">▼</button>
        <div></div>
      </div>
      
      <div class="invert-controls">
        <button class="invert-btn invert-m1" id="invertBtn1" ontouchstart="toggleInvert(1)">INVERT M1</button>
        <button class="invert-btn invert-m2" id="invertBtn2" ontouchstart="toggleInvert(2)">INVERT M2</button>
      </div>
    </div>
    
    <div class="status-panel" id="status">⚡ Ready</div>
</div>

    <script>
        let spd = 200;
        let spd2 = 200;
        
        document.getElementById('speed').oninput = function() {
            spd = this.value;
            document.getElementById('speedVal').innerHTML = spd;
            fetch('/speed/' + spd);
        };
        
        document.getElementById('speed2').oninput = function() {
            spd2 = this.value;
            document.getElementById('speedVal2').innerHTML = spd2;
            fetch('/speed2/' + spd2);
        };
        
        function send(cmd) {
            fetch('/cmd?dir=' + cmd + '&speed=' + spd + '&speed2=' + spd2);
            document.getElementById('status').innerHTML = cmd.toUpperCase();
            navigator.vibrate?.(30);
        }; 
        
        function toggleInvert(motor) {
            fetch('/invert?motor=' + motor)
                .then(response => response.text())
                .then(state => {
                    let btn = document.getElementById('invertBtn' + motor);
                    if(state === "1") {
                        btn.innerHTML = "M" + motor + " INVERTED";
                        btn.classList.add('inverted-state');
                    } else {
                        btn.innerHTML = "INVERT M" + motor;
                        btn.classList.remove('inverted-state');
                    }
                    navigator.vibrate?.(50);
                });
        }
    </script>
</body>
</html>
)rawliteral";

// ========== MOTOR CONTROL ==========
void initLEDC() {
    ledcSetup(CH_LEFT_FORWARD, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcSetup(CH_LEFT_BACKWARD, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcSetup(CH_RIGHT_FORWARD, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcSetup(CH_RIGHT_BACKWARD, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    
    ledcAttachPin(leftForwardPin, CH_LEFT_FORWARD);
    ledcAttachPin(leftBackwardPin, CH_LEFT_BACKWARD);
    ledcAttachPin(rightForwardPin, CH_RIGHT_FORWARD);
    ledcAttachPin(rightBackwardPin, CH_RIGHT_BACKWARD);
}

void stopMotors() {
    ledcWrite(CH_LEFT_FORWARD, 0); ledcWrite(CH_LEFT_BACKWARD, 0);
    ledcWrite(CH_RIGHT_FORWARD, 0); ledcWrite(CH_RIGHT_BACKWARD, 0);
}

// --- NEW: Helper functions that respect inversion ---
void setLeftMotor(int speed, bool forward) {
    speed = constrain(speed, 0, 255);
    if (isInvertedLeft) forward = !forward; // Flip direction if inverted
    
    if (forward) {
        ledcWrite(CH_LEFT_FORWARD, speed); ledcWrite(CH_LEFT_BACKWARD, 0);
    } else {
        ledcWrite(CH_LEFT_FORWARD, 0); ledcWrite(CH_LEFT_BACKWARD, speed);
    }
}

void setRightMotor(int speed, bool forward) {
    speed = constrain(speed, 0, 255);
    if (isInvertedRight) forward = !forward; // Flip direction if inverted
    
    if (forward) {
        ledcWrite(CH_RIGHT_FORWARD, speed); ledcWrite(CH_RIGHT_BACKWARD, 0);
    } else {
        ledcWrite(CH_RIGHT_FORWARD, 0); ledcWrite(CH_RIGHT_BACKWARD, speed);
    }
}

// Movement functions now use the helpers
void moveForward(int s, int s2) {
    setLeftMotor(s, true);
    setRightMotor(s2, true);
}
void moveBackward(int s, int s2) {
    setLeftMotor(s, false);
    setRightMotor(s2, false);
}
void moveLeft(int s) {
    setLeftMotor(s, false); // Left backward
    setRightMotor(s, true); // Right forward
}
void moveRight(int s) {
    setLeftMotor(s, true);  // Left forward
    setRightMotor(s, false);// Right backward
}

// ========== WEB HANDLERS ==========
void handleRoot() { server.send(200, "text/html", index_html); }
void handleSpeed() { server.send(200, "text/plain", "OK"); }

// --- NEW: Handler to toggle individual motors ---
void handleInvert() {
    if (server.hasArg("motor")) {
        int motor = server.arg("motor").toInt();
        
        if (motor == 1) {
            isInvertedLeft = !isInvertedLeft;
            server.send(200, "text/plain", isInvertedLeft ? "1" : "0");
            Serial.printf("Motor 1 Inverted: %d\n", isInvertedLeft);
        } 
        else if (motor == 2) {
            isInvertedRight = !isInvertedRight;
            server.send(200, "text/plain", isInvertedRight ? "1" : "0");
            Serial.printf("Motor 2 Inverted: %d\n", isInvertedRight);
        } 
        else {
            server.send(400, "text/plain", "Invalid Motor");
        }
    } else {
        server.send(400, "text/plain", "Missing Arguments");
    }
}

void handleCommand() {
    if (server.hasArg("dir") && server.hasArg("speed")) {
        
        String cmd = server.arg("dir");
        int spd = server.arg("speed").toInt();
        int spd2 = server.arg("speed2").toInt();

        if (cmd == "forward") moveForward(spd, spd2);
        else if (cmd == "backward") moveBackward(spd, spd2);
        else if (cmd == "left") moveLeft(spd);
        else if (cmd == "right") moveRight(spd);
        else if (cmd == "stop") stopMotors();
        
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Missing Arguments");
    }
}

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    initLEDC();
    stopMotors();
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    
    Serial.println("\n=== XIAO Robot ===");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("SSID: "); Serial.println(ap_ssid);
    Serial.println("Password: 12345678");
    
    server.on("/", handleRoot);
    server.on("/cmd", handleCommand);
    server.on("/invert", handleInvert); // Register new invert route
    server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
    
    server.begin();
    WiFi.setSleep(false);
    
    Serial.println("Server started!");
}

void loop() {
    server.handleClient();
    
    if (server.client() && !server.client().connected()) {
        server.client().stop();
    }
    
    delay(10);
}
