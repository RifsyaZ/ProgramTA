/**
 * ESP32-C3 SWERVE MASTER + REAL-TIME MONITOR
 * - Kirim perintah ke slave STM32 via RS485
 * - Terima feedback OK:ID:Aangle.00:Rrpm.00
 * - Tampilkan semua sudut (0-360°) real-time di web
 * - Animasi steering dan putaran roda
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ==================== PIN ====================
#define RS485_TX 6
#define RS485_RX 7
#define BAUD 115200
#define SLAVE_ID 1

// ==================== WEB & AP ====================
const char* ap_ssid = "Swerve_Master";
const char* ap_password = "12345678";
WebServer server(80);

// ==================== DATA FEEDBACK ====================
struct FeedbackData {
  float angle;
  float rpm;
  String raw;
  unsigned long time;
  bool valid;
};

FeedbackData currentFeedback = {0, 0, "", 0, false};

// Variabel kontrol manual
float cmdAngle = 0;
float cmdRPM = 0;

unsigned long lastReceiveTime = 0;
int packetCount = 0;

// ==================== FUNGSI RS485 ====================
void sendCommand(float angle, float rpm) {
  String cmd = String(SLAVE_ID) + ":" + String(angle) + ":" + String(rpm);
  Serial1.println(cmd);
  
  Serial.print("📤 SEND: ");
  Serial.println(cmd);
}

// Parse feedback dari slave
void parseFeedback(String data) {
  data.trim();
  if (data.length() == 0) return;
  
  Serial.print("📥 RAW: ");
  Serial.println(data);
  
  // Format: OK:1:A45.00:R30.00
  if (data.startsWith("OK:")) {
    int firstColon = data.indexOf(':', 3);
    int secondColon = data.indexOf(':', firstColon + 1);
    int thirdColon = data.indexOf(':', secondColon + 1);
    
    if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
      String angleStr = data.substring(firstColon + 2, secondColon);
      String rpmStr = data.substring(secondColon + 2, thirdColon);
      
      currentFeedback.angle = angleStr.toFloat();
      currentFeedback.rpm = rpmStr.toFloat();
      currentFeedback.raw = data;
      currentFeedback.time = millis();
      currentFeedback.valid = true;
      lastReceiveTime = millis();
      packetCount++;
      
      Serial.print("✅ PARSED: Angle=");
      Serial.print(currentFeedback.angle);
      Serial.print("°, RPM=");
      Serial.println(currentFeedback.rpm);
    }
  }
}

// ==================== HALAMAN WEB ====================
String getHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no, viewport-fit=cover">
    <title>SWERVE REAL-TIME MONITOR</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            user-select: none;
        }
        
        body {
            background: linear-gradient(135deg, #0a0a0f 0%, #0f0f1a 100%);
            font-family: 'Segoe UI', 'Arial', sans-serif;
            width: 100vw;
            height: 100vh;
            overflow: hidden;
            position: fixed;
        }
        
        .warning {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: #0a0a0f;
            z-index: 9999;
            display: none;
            justify-content: center;
            align-items: center;
            flex-direction: column;
            color: white;
            text-align: center;
        }
        
        .landscape {
            width: 100%;
            height: 100%;
            display: flex;
            flex-direction: column;
            padding: 10px 15px;
        }
        
        .header {
            text-align: center;
            margin-bottom: 5px;
            flex-shrink: 0;
        }
        
        .header h1 {
            color: #e94560;
            font-size: 1.2rem;
        }
        
        .status-led {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #00ff00;
            animation: blink 1s infinite;
            margin-left: 8px;
        }
        
        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }
        
        .main-content {
            flex: 1;
            display: flex;
            flex-direction: row;
            gap: 15px;
            min-height: 0;
        }
        
        .left-column, .right-column {
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 12px;
            max-width: 300px;
        }
        
        .center-column {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 15px;
            flex-shrink: 0;
            background: rgba(10,10,15,0.6);
            border-radius: 30px;
            padding: 15px 25px;
        }
        
        .wheel-card {
            background: rgba(10,10,15,0.9);
            border-radius: 20px;
            padding: 12px;
            text-align: center;
            border: 1px solid rgba(233,69,96,0.3);
            transition: all 0.2s;
        }
        
        .wheel-card:hover {
            border-color: #e94560;
            transform: scale(1.01);
        }
        
        .wheel-name {
            font-size: 1.2rem;
            font-weight: bold;
        }
        .wheel-name.fl { color: #4ecdc4; }
        .wheel-name.fr { color: #ff6b6b; }
        .wheel-name.rl { color: #a8e6cf; }
        .wheel-name.rr { color: #ffe66d; }
        
        .steering-container {
            position: relative;
            width: 100px;
            height: 100px;
            margin: 10px auto;
        }
        
        .steering-circle {
            width: 100px;
            height: 100px;
            background: #0a0a0f;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            box-shadow: inset 0 0 20px rgba(0,0,0,0.5), 0 5px 15px rgba(0,0,0,0.3);
            border: 2px solid #e94560;
        }
        
        .steering-arrow {
            width: 70px;
            height: 70px;
            transition: transform 0.05s linear;
        }
        
        .steering-arrow svg {
            width: 100%;
            height: 100%;
            filter: drop-shadow(0 0 3px #e94560);
        }
        
        .angle-text {
            font-size: 1rem;
            font-weight: bold;
            color: #e94560;
            margin-top: 5px;
            font-family: monospace;
        }
        
        .wheel-spin-container {
            margin: 10px auto;
        }
        
        .wheel-spin {
            width: 70px;
            height: 70px;
            background: linear-gradient(145deg, #1a1a2a, #0f0f1a);
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            margin: 0 auto;
            box-shadow: 0 0 0 3px #e94560, inset 0 0 20px rgba(0,0,0,0.5);
        }
        
        .wheel-spin svg {
            width: 55px;
            height: 55px;
        }
        
        @keyframes spin-cw {
            from { transform: rotate(0deg); }
            to { transform: rotate(360deg); }
        }
        
        @keyframes spin-ccw {
            from { transform: rotate(0deg); }
            to { transform: rotate(-360deg); }
        }
        
        .rpm-text {
            font-size: 1.1rem;
            font-weight: bold;
            font-family: monospace;
            margin-top: 5px;
        }
        .rpm-positive { color: #4ecdc4; }
        .rpm-negative { color: #ff6b6b; }
        .rpm-zero { color: #666; }
        
        .direction-text {
            font-size: 0.65rem;
            color: #888;
            margin-top: 3px;
        }
        
        .control-title {
            color: #e94560;
            font-size: 0.8rem;
            margin-bottom: 5px;
            text-align: center;
        }
        
        .dpad {
            display: grid;
            grid-template-columns: 55px 55px 55px;
            grid-template-rows: 55px 55px 55px;
            gap: 5px;
            background: #0a0a0f;
            padding: 10px;
            border-radius: 30px;
        }
        
        .dpad-btn {
            background: #2a2a3a;
            border: none;
            border-radius: 15px;
            font-size: 22px;
            cursor: pointer;
            color: white;
            box-shadow: 0 4px 0 #0a0a12;
            transition: 0.03s linear;
        }
        
        .dpad-btn:active {
            transform: translateY(2px);
            box-shadow: 0 1px 0 #0a0a12;
        }
        
        .dpad-btn.up { grid-column: 2; grid-row: 1; background: #e94560; }
        .dpad-btn.left { grid-column: 1; grid-row: 2; background: #4ecdc4; }
        .dpad-btn.center { grid-column: 2; grid-row: 2; background: #3a3a4a; font-size: 16px; }
        .dpad-btn.right { grid-column: 3; grid-row: 2; background: #ff6b6b; }
        .dpad-btn.down { grid-column: 2; grid-row: 3; background: #ffe66d; color: #333; }
        
        .slider-container {
            text-align: center;
            margin-top: 10px;
            width: 100%;
        }
        
        input[type="range"] {
            width: 100%;
            height: 6px;
            -webkit-appearance: none;
            background: linear-gradient(90deg, #4ecdc4, #e94560);
            border-radius: 5px;
        }
        
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #e94560;
            cursor: pointer;
            box-shadow: 0 0 8px #e94560;
        }
        
        .rpm-slider-value {
            font-size: 1.1rem;
            font-weight: bold;
            color: #e94560;
            margin-top: 5px;
        }
        
        .btn-stop {
            background: #f44336;
            border: none;
            padding: 8px 20px;
            border-radius: 30px;
            color: white;
            font-weight: bold;
            margin-top: 10px;
            cursor: pointer;
            width: 100%;
        }
        
        .depan, .belakang {
            background: #e94560;
            padding: 4px 15px;
            border-radius: 30px;
            font-weight: bold;
            color: white;
            font-size: 0.7rem;
            text-align: center;
        }
        .belakang { background: #533483; }
        
        .side-labels {
            display: flex;
            gap: 30px;
            font-size: 0.7rem;
        }
        .side-left { color: #4ecdc4; }
        .side-right { color: #ff6b6b; }
        
        .status-panel {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin-top: 8px;
            padding: 8px;
            background: rgba(10,10,15,0.6);
            border-radius: 15px;
            flex-shrink: 0;
            flex-wrap: wrap;
        }
        
        .status-item {
            text-align: center;
        }
        .status-label {
            font-size: 0.6rem;
            color: #666;
        }
        .status-value {
            font-size: 0.8rem;
            font-weight: bold;
            color: #e94560;
        }
        
        .footer {
            text-align: center;
            font-size: 0.5rem;
            color: #333;
            margin-top: 5px;
        }
        
        @media (max-width: 700px) {
            .steering-container { width: 75px; height: 75px; }
            .steering-circle { width: 75px; height: 75px; }
            .steering-arrow { width: 52px; height: 52px; }
            .wheel-spin { width: 55px; height: 55px; }
            .wheel-spin svg { width: 42px; height: 42px; }
            .wheel-name { font-size: 1rem; }
            .angle-text { font-size: 0.8rem; }
            .rpm-text { font-size: 0.9rem; }
            .dpad { grid-template-columns: 45px 45px 45px; grid-template-rows: 45px 45px 45px; }
            .dpad-btn { font-size: 18px; }
        }
        
        @media (max-width: 550px) {
            .steering-container { width: 65px; height: 65px; }
            .steering-circle { width: 65px; height: 65px; }
            .steering-arrow { width: 45px; height: 45px; }
            .wheel-spin { width: 48px; height: 48px; }
            .wheel-spin svg { width: 36px; height: 36px; }
        }
    </style>
</head>
<body>
    <div class="warning" id="warning">
        <div class="rotate-icon">📱 ➡️ 📱</div>
        <h2>Rotate to Landscape</h2>
        <p>Please rotate your phone to horizontal mode</p>
    </div>
    
    <div class="landscape" id="mainContent">
        <div class="header">
            <h1>⚙️ SWERVE REAL-TIME MONITOR <span class="status-led"></span></h1>
        </div>
        
        <div class="main-content">
            <div class="left-column">
                <div class="wheel-card">
                    <div class="wheel-name fl">🟢 FL (KIRI DEPAN)</div>
                    <div class="steering-container">
                        <div class="steering-circle">
                            <div class="steering-arrow" id="steerFL">
                                <svg viewBox="0 0 100 100">
                                    <line x1="50" y1="15" x2="50" y2="85" stroke="#4ecdc4" stroke-width="4" stroke-linecap="round"/>
                                    <polygon points="50,10 40,32 60,32" fill="#4ecdc4"/>
                                    <circle cx="50" cy="50" r="4" fill="#4ecdc4" opacity="0.5"/>
                                </svg>
                            </div>
                        </div>
                    </div>
                    <div class="angle-text" id="angleFL">0.0°</div>
                    <div class="wheel-spin-container">
                        <div class="wheel-spin" id="spinFL">
                            <svg viewBox="0 0 60 60">
                                <circle cx="30" cy="30" r="25" fill="none" stroke="#4ecdc4" stroke-width="3"/>
                                <line x1="30" y1="5" x2="30" y2="55" stroke="#4ecdc4" stroke-width="2"/>
                                <line x1="5" y1="30" x2="55" y2="30" stroke="#4ecdc4" stroke-width="2"/>
                                <line x1="12" y1="12" x2="48" y2="48" stroke="#4ecdc4" stroke-width="2"/>
                                <line x1="48" y1="12" x2="12" y2="48" stroke="#4ecdc4" stroke-width="2"/>
                            </svg>
                        </div>
                    </div>
                    <div class="rpm-text" id="rpmFL">0 RPM</div>
                    <div class="direction-text" id="dirFL">● STOP</div>
                </div>
                
                <div class="wheel-card">
                    <div class="wheel-name rl">🔵 RL (KIRI BELAKANG)</div>
                    <div class="steering-container">
                        <div class="steering-circle">
                            <div class="steering-arrow" id="steerRL">
                                <svg viewBox="0 0 100 100">
                                    <line x1="50" y1="15" x2="50" y2="85" stroke="#a8e6cf" stroke-width="4" stroke-linecap="round"/>
                                    <polygon points="50,10 40,32 60,32" fill="#a8e6cf"/>
                                    <circle cx="50" cy="50" r="4" fill="#a8e6cf" opacity="0.5"/>
                                </svg>
                            </div>
                        </div>
                    </div>
                    <div class="angle-text" id="angleRL">0.0°</div>
                    <div class="wheel-spin-container">
                        <div class="wheel-spin" id="spinRL">
                            <svg viewBox="0 0 60 60">
                                <circle cx="30" cy="30" r="25" fill="none" stroke="#a8e6cf" stroke-width="3"/>
                                <line x1="30" y1="5" x2="30" y2="55" stroke="#a8e6cf" stroke-width="2"/>
                                <line x1="5" y1="30" x2="55" y2="30" stroke="#a8e6cf" stroke-width="2"/>
                                <line x1="12" y1="12" x2="48" y2="48" stroke="#a8e6cf" stroke-width="2"/>
                                <line x1="48" y1="12" x2="12" y2="48" stroke="#a8e6cf" stroke-width="2"/>
                            </svg>
                        </div>
                    </div>
                    <div class="rpm-text" id="rpmRL">0 RPM</div>
                    <div class="direction-text" id="dirRL">● STOP</div>
                </div>
            </div>
            
            <div class="center-column">
                <div class="depan">⬆️ DEPAN ⬆️</div>
                <div class="control-title">🎮 JOYSTICK CONTROL</div>
                <div class="dpad">
                    <button class="dpad-btn up" id="btnUp">▲</button>
                    <button class="dpad-btn left" id="btnLeft">◀</button>
                    <button class="dpad-btn center" id="btnCenter">●</button>
                    <button class="dpad-btn right" id="btnRight">▶</button>
                    <button class="dpad-btn down" id="btnDown">▼</button>
                </div>
                <div class="slider-container">
                    <input type="range" id="rpmSlider" min="0" max="100" value="0" step="5">
                    <div class="rpm-slider-value" id="rpmSliderValue">0 RPM</div>
                </div>
                <button class="btn-stop" id="btnStop">🛑 EMERGENCY STOP</button>
                <div class="side-labels">
                    <span class="side-left">⬅️ KIRI</span>
                    <span class="side-right">KANAN ➡️</span>
                </div>
                <div class="belakang">⬇️ BELAKANG ⬇️</div>
            </div>
            
            <div class="right-column">
                <div class="wheel-card">
                    <div class="wheel-name fr">🔴 FR (KANAN DEPAN)</div>
                    <div class="steering-container">
                        <div class="steering-circle">
                            <div class="steering-arrow" id="steerFR">
                                <svg viewBox="0 0 100 100">
                                    <line x1="50" y1="15" x2="50" y2="85" stroke="#ff6b6b" stroke-width="4" stroke-linecap="round"/>
                                    <polygon points="50,10 40,32 60,32" fill="#ff6b6b"/>
                                    <circle cx="50" cy="50" r="4" fill="#ff6b6b" opacity="0.5"/>
                                </svg>
                            </div>
                        </div>
                    </div>
                    <div class="angle-text" id="angleFR">0.0°</div>
                    <div class="wheel-spin-container">
                        <div class="wheel-spin" id="spinFR">
                            <svg viewBox="0 0 60 60">
                                <circle cx="30" cy="30" r="25" fill="none" stroke="#ff6b6b" stroke-width="3"/>
                                <line x1="30" y1="5" x2="30" y2="55" stroke="#ff6b6b" stroke-width="2"/>
                                <line x1="5" y1="30" x2="55" y2="30" stroke="#ff6b6b" stroke-width="2"/>
                                <line x1="12" y1="12" x2="48" y2="48" stroke="#ff6b6b" stroke-width="2"/>
                                <line x1="48" y1="12" x2="12" y2="48" stroke="#ff6b6b" stroke-width="2"/>
                            </svg>
                        </div>
                    </div>
                    <div class="rpm-text" id="rpmFR">0 RPM</div>
                    <div class="direction-text" id="dirFR">● STOP</div>
                </div>
                
                <div class="wheel-card">
                    <div class="wheel-name rr">🟡 RR (KANAN BELAKANG)</div>
                    <div class="steering-container">
                        <div class="steering-circle">
                            <div class="steering-arrow" id="steerRR">
                                <svg viewBox="0 0 100 100">
                                    <line x1="50" y1="15" x2="50" y2="85" stroke="#ffe66d" stroke-width="4" stroke-linecap="round"/>
                                    <polygon points="50,10 40,32 60,32" fill="#ffe66d"/>
                                    <circle cx="50" cy="50" r="4" fill="#ffe66d" opacity="0.5"/>
                                </svg>
                            </div>
                        </div>
                    </div>
                    <div class="angle-text" id="angleRR">0.0°</div>
                    <div class="wheel-spin-container">
                        <div class="wheel-spin" id="spinRR">
                            <svg viewBox="0 0 60 60">
                                <circle cx="30" cy="30" r="25" fill="none" stroke="#ffe66d" stroke-width="3"/>
                                <line x1="30" y1="5" x2="30" y2="55" stroke="#ffe66d" stroke-width="2"/>
                                <line x1="5" y1="30" x2="55" y2="30" stroke="#ffe66d" stroke-width="2"/>
                                <line x1="12" y1="12" x2="48" y2="48" stroke="#ffe66d" stroke-width="2"/>
                                <line x1="48" y1="12" x2="12" y2="48" stroke="#ffe66d" stroke-width="2"/>
                            </svg>
                        </div>
                    </div>
                    <div class="rpm-text" id="rpmRR">0 RPM</div>
                    <div class="direction-text" id="dirRR">● STOP</div>
                </div>
            </div>
        </div>
        
        <div class="status-panel">
            <div class="status-item">
                <div class="status-label">📡 FEEDBACK</div>
                <div class="status-value" id="fbStatus">WAITING...</div>
            </div>
            <div class="status-item">
                <div class="status-label">🎯 REAL ANGLE</div>
                <div class="status-value" id="realAngle">0.0°</div>
            </div>
            <div class="status-item">
                <div class="status-label">⚡ REAL RPM</div>
                <div class="status-value" id="realRPM">0</div>
            </div>
            <div class="status-item">
                <div class="status-label">📦 PACKETS</div>
                <div class="status-value" id="packetCount">0</div>
            </div>
        </div>
        
        <div class="footer">
            ▲=0° ◀=90° ▶=270° ▼=180° | Slider 0-100 | Feedback: OK:ID:Aangle:Rrpm
        </div>
    </div>
    
    <script>
        let cmdAngle = 0;
        let cmdRPM = 0;
        let lastUpdateTime = Date.now();
        
        function checkOrientation() {
            if (window.matchMedia("(orientation: portrait)").matches) {
                document.getElementById('warning').style.display = 'flex';
                document.getElementById('mainContent').style.display = 'none';
            } else {
                document.getElementById('warning').style.display = 'none';
                document.getElementById('mainContent').style.display = 'flex';
            }
        }
        
        window.addEventListener('resize', checkOrientation);
        window.addEventListener('load', checkOrientation);
        
        function updateAllWheels(angle, rpm) {
            const wheels = ['FL', 'FR', 'RL', 'RR'];
            
            for (let wheel of wheels) {
                const steer = document.getElementById(`steer${wheel}`);
                if (steer) {
                    steer.style.transform = `rotate(${angle}deg)`;
                }
                document.getElementById(`angle${wheel}`).innerHTML = angle.toFixed(1) + "°";
                
                const spin = document.getElementById(`spin${wheel}`);
                const rpmDiv = document.getElementById(`rpm${wheel}`);
                const dirDiv = document.getElementById(`dir${wheel}`);
                
                if (rpm === 0) {
                    spin.style.animation = "none";
                    rpmDiv.innerHTML = "0 RPM";
                    rpmDiv.className = "rpm-text rpm-zero";
                    dirDiv.innerHTML = "● STOP";
                } else {
                    let duration = Math.max(0.15, 2 / (Math.abs(rpm) / 15));
                    if (rpm > 0) {
                        spin.style.animation = `spin-cw ${duration}s linear infinite`;
                        rpmDiv.innerHTML = `+${rpm} RPM 🔄`;
                        rpmDiv.className = "rpm-text rpm-positive";
                    } else {
                        spin.style.animation = `spin-ccw ${duration}s linear infinite`;
                        rpmDiv.innerHTML = `${rpm} RPM 🔄`;
                        rpmDiv.className = "rpm-text rpm-negative";
                    }
                    
                    if (angle > 0 && angle < 180) dirDiv.innerHTML = "↗ KE KANAN";
                    else if (angle > 180 && angle < 360) dirDiv.innerHTML = "↖ KE KIRI";
                    else if (angle === 0 || angle === 360) dirDiv.innerHTML = "↑ MAJU";
                    else if (angle === 180) dirDiv.innerHTML = "↓ MUNDUR";
                    else dirDiv.innerHTML = "● RUN";
                    
                    if (rpm > 0) dirDiv.innerHTML += " +";
                    else if (rpm < 0) dirDiv.innerHTML += " -";
                }
            }
        }
        
        async function fetchFeedback() {
            try {
                const res = await fetch('/feedback');
                const data = await res.json();
                
                if (data.valid) {
                    updateAllWheels(data.angle, data.rpm);
                    document.getElementById('realAngle').innerHTML = data.angle.toFixed(1) + "°";
                    document.getElementById('realRPM').innerHTML = data.rpm;
                    document.getElementById('packetCount').innerHTML = data.count;
                    document.getElementById('fbStatus').innerHTML = "● ONLINE";
                    document.getElementById('fbStatus').style.color = "#4ecdc4";
                    lastUpdateTime = Date.now();
                } else {
                    let elapsed = Math.floor((Date.now() - lastUpdateTime) / 1000);
                    if (elapsed > 2) {
                        document.getElementById('fbStatus').innerHTML = "⚠️ NO DATA";
                        document.getElementById('fbStatus').style.color = "#ff6b6b";
                    }
                }
            } catch(e) {
                console.log("Fetch error:", e);
                document.getElementById('fbStatus').innerHTML = "⚠️ OFFLINE";
                document.getElementById('fbStatus').style.color = "#ff6b6b";
            }
        }
        
        function sendCommand() {
            fetch('/control?angle=' + cmdAngle + '&rpm=' + cmdRPM)
                .catch(err => console.log(err));
        }
        
        function setAngle(angle) {
            cmdAngle = angle;
            sendCommand();
        }
        
        function setRPM(rpm) {
            cmdRPM = rpm;
            document.getElementById('rpmSliderValue').innerHTML = rpm + " RPM";
            sendCommand();
        }
        
        function stopMotor() {
            cmdRPM = 0;
            document.getElementById('rpmSlider').value = 0;
            document.getElementById('rpmSliderValue').innerHTML = "0 RPM";
            sendCommand();
        }
        
        document.getElementById('btnUp').onclick = () => setAngle(0);
        document.getElementById('btnDown').onclick = () => setAngle(180);
        document.getElementById('btnLeft').onclick = () => setAngle(90);
        document.getElementById('btnRight').onclick = () => setAngle(270);
        document.getElementById('btnCenter').onclick = () => setAngle(0);
        document.getElementById('btnStop').onclick = () => stopMotor();
        
        const rpmSlider = document.getElementById('rpmSlider');
        rpmSlider.oninput = (e) => setRPM(parseInt(e.target.value));
        
        setAngle(0);
        setRPM(0);
        
        setInterval(fetchFeedback, 100);
    </script>
</body>
</html>
)rawliteral";
  return html;
}

// ==================== WEB HANDLER ====================
void handleRoot() {
  server.send(200, "text/html", getHTML());
}

void handleControl() {
  if (server.hasArg("angle") && server.hasArg("rpm")) {
    float angle = server.arg("angle").toFloat();
    float rpm = server.arg("rpm").toFloat();
    sendCommand(angle, rpm);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleFeedback() {
  StaticJsonDocument<256> doc;
  doc["angle"] = currentFeedback.angle;
  doc["rpm"] = currentFeedback.rpm;
  doc["valid"] = currentFeedback.valid;
  doc["count"] = packetCount;
  doc["raw"] = currentFeedback.raw;
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/feedback", handleFeedback);
  server.begin();
  
  delay(500);
  sendCommand(0, 0);
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║     SWERVE MASTER - REAL-TIME MONITOR     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("📡 WiFi AP: ");
  Serial.println(ap_ssid);
  Serial.print("🔗 IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("✅ System ready!");
}

// ==================== LOOP ====================
void loop() {
  while (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    if (data.length() > 0) {
      parseFeedback(data);
    }
  }
  
  server.handleClient();
  delay(10);
}