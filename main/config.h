#ifndef CONFIG_H
#define CONFIG_H

/* =========================
 * モータードライバ（DRV8833）
 * ========================= */

// 左側モーター
constexpr int Motor_L1_F_PIN = 12;
constexpr int Motor_L1_B_PIN = 13;
constexpr int Motor_L2_F_PIN = 14;
constexpr int Motor_L2_B_PIN = 27;

// 右側モーター
constexpr int Motor_R1_F_PIN = 32;
constexpr int Motor_R1_B_PIN = 33;
constexpr int Motor_R2_F_PIN = 26;
constexpr int Motor_R2_B_PIN = 25;

// STBY
constexpr int CONTROL_PIN_15 = 15;
constexpr int CONTROL_PIN_2 = 2;

/* =========================
 * フォトリフレクタ
 * ========================= */
constexpr int PR_1_PIN = 23;
constexpr int PR_2_PIN = 22;
constexpr int PR_3_PIN = 21;
constexpr int PR_4_PIN = 19;
constexpr int PR_5_PIN = 18;
constexpr int PR_6_PIN = 5;
constexpr int PR_7_PIN = 17;
constexpr int PR_8_PIN = 16;

constexpr int Photoreflector_PIN[8] = {PR_1_PIN, PR_2_PIN, PR_3_PIN, PR_4_PIN,
                                       PR_5_PIN, PR_6_PIN, PR_7_PIN, PR_8_PIN};
constexpr bool LINE_SENSOR_ENABLED[8] = {true,  true, true, true,
                                         false, true, true, true};

/* =========================
 * PWM 設定
 * ========================= */
constexpr int PWM_FREQ = 20000;   // 20kHz
constexpr int PWM_RESOLUTION = 8; // 8bit
constexpr int MIN_DUTY = 180;
constexpr int MAX_DUTY = 252;

/* =========================
 * ロボット状態
 * ========================= */
enum RobotState {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  LINE_TRACE
};

/* =========================
 * ライントレース設定
 * ========================= */
constexpr int LINE_SENSOR_WEIGHTS[8] = {-3, -2, -1, 0, 0, 1, 2, 3};
constexpr bool LINE_SENSOR_ACTIVE_LOW = false;
constexpr int LINE_BASE_SPEED = 255;
constexpr int LINE_MAX_SPEED = 255;
constexpr float LINE_KP = 1000.0f;
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 0.0f;
constexpr float LINE_INTEGRAL_LIMIT = 200.0f;
constexpr unsigned long LINE_CONTROL_INTERVAL_MS = 10;
constexpr unsigned long LINE_SENSOR_LOG_INTERVAL_MS = 100;

/* =========================
 * Wi-Fi (SoftAP) 設定
 * ========================= */
constexpr const char *WIFI_AP_SSID = "LineTraceSetup";
constexpr const char *WIFI_AP_PASSWORD = "12345678";

#endif // CONFIG_H