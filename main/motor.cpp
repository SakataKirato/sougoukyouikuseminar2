#include "motor.h"

// ===== コンストラクタ =====
Motor::Motor(int speed) { speed_ = speed; }

// ===== 内部関数 =====
void Motor::setMotorSpeed(int pinF, int pinB, int speedValue) {
  if (speedValue == 0) {
    ledcWrite(pinF, 0);
    ledcWrite(pinB, 0);
    return;
  }

  int absSpeed = constrain(abs(speedValue), 0, 255);
  int duty = MIN_DUTY + (absSpeed * (MAX_DUTY - MIN_DUTY)) / 255;
  duty = constrain(duty, MIN_DUTY, MAX_DUTY);

  if (speedValue > 0) {
    ledcWrite(pinF, duty);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinF, 0);
    ledcWrite(pinB, duty);
  }
}

// ===== 公開API =====
void Motor::stop() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
}

void Motor::forward() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -speed_);
}

void Motor::back() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, speed_);
}

void Motor::left() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -speed_);
}

void Motor::right() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, speed_);
}

void Motor::turnLeft() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, speed_);
}

void Motor::turnRight() {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, speed_);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, speed_);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -speed_);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -speed_);
}

void Motor::setSpeed(int speed) { speed_ = constrain(speed, 0, 255); }

void Motor::drive(int leftSpeed, int rightSpeed) {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -leftSpeed);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -leftSpeed);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -rightSpeed);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -rightSpeed);
}