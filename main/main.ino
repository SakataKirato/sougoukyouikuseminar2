#include "config.h"
#include "motor.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

uint8_t readLineSensors();
int calculateLineError(uint8_t pattern, int previousError);
void lineTraceControl();
void resetLineTraceController();
void printLineSensorValues();
void setupWiFi();
void setupWebServer();
void handleRootRequest();
void handleUpdateRequest();
void handleNotFound();
String buildControlPage();
void applyLineParameterBounds();

/* =========================
 * グローバル変数
 * ========================= */
RobotState currentState = STOP;
RobotState previousState = STOP;
Motor motor(LINE_BASE_SPEED); // 初期速度をベーススピードに合わせる
float lineIntegral = 0.0f;
int linePreviousError = 0;
unsigned long lastLineUpdate = 0;
unsigned long lastLineSensorLog = 0;
int lineBaseSpeed = LINE_BASE_SPEED;
int lineMaxSpeed = LINE_MAX_SPEED;
float lineKp = LINE_KP;
float lineKi = LINE_KI;
float lineKd = LINE_KD;
float lineIntegralLimit = LINE_INTEGRAL_LIMIT;
WebServer server(80);

/* =========================
 * setup
 * ========================= */
void setup() {
  // ===== モータピン設定 =====
  pinMode(Motor_L1_F_PIN, OUTPUT);
  pinMode(Motor_L1_B_PIN, OUTPUT);
  pinMode(Motor_L2_F_PIN, OUTPUT);
  pinMode(Motor_L2_B_PIN, OUTPUT);
  pinMode(Motor_R1_F_PIN, OUTPUT);
  pinMode(Motor_R1_B_PIN, OUTPUT);
  pinMode(Motor_R2_F_PIN, OUTPUT);
  pinMode(Motor_R2_B_PIN, OUTPUT);

  // ===== PWM 初期化（ESP32 LEDC）=====
  int motorPins[] = {Motor_L1_F_PIN, Motor_L1_B_PIN, Motor_L2_F_PIN,
                     Motor_L2_B_PIN, Motor_R1_F_PIN, Motor_R1_B_PIN,
                     Motor_R2_F_PIN, Motor_R2_B_PIN};

  for (int i = 0; i < 8; i++) {
    ledcAttach(motorPins[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(motorPins[i], 0);
  }

  // ===== STBY ピン有効化 =====
  pinMode(CONTROL_PIN_2, OUTPUT);
  pinMode(CONTROL_PIN_15, OUTPUT);
  digitalWrite(CONTROL_PIN_2, HIGH);
  digitalWrite(CONTROL_PIN_15, HIGH);

  // ===== フォトリフレクタ =====
  for (int i = 0; i < 8; i++) {
    pinMode(Photoreflector_PIN[i], INPUT);
  }

  // ===== Serial =====
  Serial.begin(115200);
  Serial.println("ESP32 4WD Robot Control");
  Serial.println("Commands:");
  Serial.println("0/s: STOP");
  Serial.println("1/f: FORWARD");
  Serial.println("2/b: BACKWARD");
  Serial.println("3/l: LEFT");
  Serial.println("4/r: RIGHT");
  Serial.println("5/L: TURN LEFT");
  Serial.println("6/R: TURN RIGHT");
  Serial.println("7/t: LINE TRACE");
  Serial.println("8/p: PRINT LINE SENSORS");
  Serial.print("Connect to Wi-Fi SSID: ");
  Serial.print(WIFI_AP_SSID);
  Serial.println(" for GUI");

  setupWiFi();
  setupWebServer();

  // 初期停止
  motor.stop();
}

/* =========================
 * loop
 * ========================= */
void loop() {

  server.handleClient();

  // ===== コマンド受付 =====
  if (Serial.available() > 0) {
    char command = Serial.read();

    // 改行は無視
    if (command == '\r' || command == '\n') {
      return;
    }

    switch (command) {
    case '0':
    case 's':
      currentState = STOP;
      break;

    case '1':
    case 'f':
      currentState = FORWARD;
      break;

    case '2':
    case 'b':
      currentState = BACKWARD;
      break;

    case '3':
    case 'l':
      currentState = LEFT;
      break;

    case '4':
    case 'r':
      currentState = RIGHT;
      break;

    case '5':
    case 'L':
      currentState = TURN_LEFT;
      break;

    case '6':
    case 'R':
      currentState = TURN_RIGHT;
      break;

    case '7':
    case 't':
    case 'T':
      currentState = LINE_TRACE;
      break;

    case '8':
    case 'p':
    case 'P':
      printLineSensorValues();
      return;

    default:
      Serial.println("Invalid command");
      return;
    }
  }

  if (previousState != currentState) {
    if (currentState == LINE_TRACE) {
      resetLineTraceController();
    } else if (previousState == LINE_TRACE) {
      motor.stop();
    }
    previousState = currentState;
  }

  // ===== 状態に応じた動作 =====
  switch (currentState) {
  case STOP:
    motor.stop();
    break;

  case FORWARD:
    motor.forward();
    break;

  case BACKWARD:
    motor.back();
    break;

  case LEFT:
    motor.left();
    break;

  case RIGHT:
    motor.right();
    break;

  case TURN_LEFT:
    motor.turnLeft();
    break;

  case TURN_RIGHT:
    motor.turnRight();
    break;

  case LINE_TRACE:
    lineTraceControl();
    break;
  }
}

uint8_t readLineSensors() {
  uint8_t pattern = 0;
  for (int i = 0; i < 8; i++) {
    if (!LINE_SENSOR_ENABLED[i]) {
      continue;
    }
    int raw = digitalRead(Photoreflector_PIN[i]);
    bool active = LINE_SENSOR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
    if (active) {
      pattern |= (1 << i);
    }
  }
  return pattern;
}

int calculateLineError(uint8_t pattern, int previousError) {
  int weightedSum = 0;
  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    if (!LINE_SENSOR_ENABLED[i]) {
      continue;
    }
    if (pattern & (1 << i)) {
      weightedSum += LINE_SENSOR_WEIGHTS[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    return previousError;
  }

  return weightedSum / activeCount;
}

void resetLineTraceController() {
  lineIntegral = 0.0f;
  linePreviousError = 0;
  lastLineUpdate = 0;
  lastLineSensorLog = 0;
}

void lineTraceControl() {
  unsigned long now = millis();
  if (lastLineUpdate != 0 &&
      (now - lastLineUpdate) < LINE_CONTROL_INTERVAL_MS) {
    return;
  }

  float dt = (lastLineUpdate == 0) ? (LINE_CONTROL_INTERVAL_MS / 1000.0f)
                                   : (now - lastLineUpdate) / 1000.0f;
  lastLineUpdate = now;

  uint8_t pattern = readLineSensors();
  int error = calculateLineError(pattern, linePreviousError);

  float derivative =
      dt > 0 ? static_cast<float>(error - linePreviousError) / dt : 0.0f;
  lineIntegral += static_cast<float>(error) * dt;
  lineIntegral = constrain(lineIntegral, -lineIntegralLimit, lineIntegralLimit);

  float correction =
      (lineKp * error) + (lineKi * lineIntegral) + (lineKd * derivative);
  int correctionValue = static_cast<int>(correction);

  int leftSpeed = lineBaseSpeed - correctionValue;
  int rightSpeed = lineBaseSpeed + correctionValue;
  leftSpeed = constrain(leftSpeed, -lineMaxSpeed, lineMaxSpeed);
  rightSpeed = constrain(rightSpeed, -lineMaxSpeed, lineMaxSpeed);

  motor.drive(leftSpeed, rightSpeed);
  linePreviousError = error;

  if (lastLineSensorLog == 0 ||
      (now - lastLineSensorLog) >= LINE_SENSOR_LOG_INTERVAL_MS) {
    printLineSensorValues();
    lastLineSensorLog = now;
  }
}

void printLineSensorValues() {
  int rawValues[8];
  for (int i = 0; i < 8; i++) {
    rawValues[i] = digitalRead(Photoreflector_PIN[i]);
  }

  Serial.print("Photoreflector RAW: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(rawValues[i]);
    if (i < 7) {
      Serial.print(' ');
    }
  }
  Serial.print(" | Active:");
  for (int i = 0; i < 8; i++) {
    bool active = LINE_SENSOR_ENABLED[i] &&
                  (LINE_SENSOR_ACTIVE_LOW ? (rawValues[i] == LOW)
                                          : (rawValues[i] == HIGH));
    Serial.print(' ');
    Serial.print(active ? 1 : 0);
  }
  Serial.println();
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  if (apStarted) {
    Serial.println("SoftAP started");
    Serial.print("SSID: ");
    Serial.println(WIFI_AP_SSID);
    Serial.print("Password: ");
    Serial.println(WIFI_AP_PASSWORD);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("SoftAP start failed");
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRootRequest);
  server.on("/update", HTTP_POST, handleUpdateRequest);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.print("GUI URL: http://");
  Serial.println(WiFi.softAPIP());
}

void handleRootRequest() { server.send(200, "text/html", buildControlPage()); }

void handleUpdateRequest() {
  if (server.hasArg("baseSpeed")) {
    lineBaseSpeed = server.arg("baseSpeed").toInt();
  }
  if (server.hasArg("maxSpeed")) {
    lineMaxSpeed = server.arg("maxSpeed").toInt();
  }
  if (server.hasArg("kp")) {
    lineKp = server.arg("kp").toFloat();
  }
  if (server.hasArg("ki")) {
    lineKi = server.arg("ki").toFloat();
  }
  if (server.hasArg("kd")) {
    lineKd = server.arg("kd").toFloat();
  }
  if (server.hasArg("integralLimit")) {
    lineIntegralLimit = server.arg("integralLimit").toFloat();
  }

  applyLineParameterBounds();
  motor.setSpeed(lineBaseSpeed);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNotFound() { server.send(404, "text/plain", "Not found"); }

String buildControlPage() {
  String page = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Line "
                "Trace Control</title>";
  page += "<style>body{font-family:sans-serif;margin:20px;}label{display:"
          "block;margin-top:10px;}input{width:100%;max-width:300px;padding:4px;"
          "}</style></head><body>";
  page += "<h2>ラインコントローラ設定</h2>";
  page += "<form method='POST' action='/update'>";
  page += "<label>BASE SPEED<input type='number' name='baseSpeed' min='0' "
          "max='255' value='" +
          String(lineBaseSpeed) + "'></label>";
  page += "<label>MAX SPEED<input type='number' name='maxSpeed' min='0' "
          "max='255' value='" +
          String(lineMaxSpeed) + "'></label>";
  page += "<label>Kp<input type='number' step='0.1' name='kp' value='" +
          String(lineKp, 2) + "'></label>";
  page += "<label>Ki<input type='number' step='0.1' name='ki' value='" +
          String(lineKi, 2) + "'></label>";
  page += "<label>Kd<input type='number' step='0.1' name='kd' value='" +
          String(lineKd, 2) + "'></label>";
  page +=
      "<label>Integral Limit<input type='number' step='1' name='integralLimit' "
      "value='" +
      String(lineIntegralLimit, 2) + "'></label>";
  page += "<button type='submit'>更新</button></form>";
  page += "<p>現在のPID出力は実行中のライン制御に即座に反映されます。</p>";
  page += "</body></html>";
  return page;
}

void applyLineParameterBounds() {
  lineMaxSpeed = constrain(lineMaxSpeed, 0, 255);
  lineBaseSpeed = constrain(lineBaseSpeed, 0, lineMaxSpeed);
  if (lineKp < 0.0f) {
    lineKp = 0.0f;
  }
  if (lineKi < 0.0f) {
    lineKi = 0.0f;
  }
  if (lineKd < 0.0f) {
    lineKd = 0.0f;
  }
  if (lineIntegralLimit < 0.0f) {
    lineIntegralLimit = 0.0f;
  }
}