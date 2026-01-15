#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"
#include <Arduino.h>

class Motor {
public:
  // コンストラクタ
  Motor(int speed = 150);

  // 動作
  void stop();
  void forward();
  void back();
  void left();
  void right();
  void turnLeft();
  void turnRight();
  void drive(int leftSpeed, int rightSpeed);
  void driveMecanum(int FL, int FR, int BL, int backRight);  // メカナムホイール個別制御

  // 速度変更
  void setSpeed(int speed);
  void setDutyRange(int minDuty, int maxDuty);

private:
  int speed_;
  int minDuty_;
  int maxDuty_;

  // 内部用：1モータ制御
  void setMotorSpeed(int pinF, int pinB, int speedValue);
};

#endif // MOTOR_H
