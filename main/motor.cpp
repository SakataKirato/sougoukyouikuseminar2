#include "motor.h"

// ===== コンストラクタ =====
Motor::Motor(int speed)
    : speed_(speed), minDuty_(MIN_DUTY), maxDuty_(MAX_DUTY) {}

// ===== 内部関数 =====
void Motor::setMotorSpeed(int pinF, int pinB, int speedValue) {
  if (speedValue == 0) {
    ledcWrite(pinF, 0);
    ledcWrite(pinB, 0);
    return;
  }

  int absSpeed = constrain(abs(speedValue), 0, 255);
  int duty = minDuty_ + (absSpeed * (maxDuty_ - minDuty_)) / 255;
  duty = constrain(duty, minDuty_, maxDuty_);

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

void Motor::setDutyRange(int minDuty, int maxDuty) {
  minDuty_ = constrain(minDuty, 0, 255);
  maxDuty_ = constrain(maxDuty, 0, 255);
  if (maxDuty_ < minDuty_) {
    maxDuty_ = minDuty_;
  }
}

void Motor::drive(int leftSpeed, int rightSpeed) {
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -leftSpeed);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -leftSpeed);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -rightSpeed);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -rightSpeed);
}

void Motor::driveMecanum(int FL, int FR, int BL, int backRight) {
  // メカナムホイール個別制御
  // FL: 前左, FR: 前右, BL: 後左, backRight: 後右
  setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -FL);
  setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -FR);
  setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -BL);
  setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -backRight);
}
