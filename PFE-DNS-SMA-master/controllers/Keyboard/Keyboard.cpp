#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  DistanceSensor *ds[8];
  
  char dsNames[8][4] = {"ps0", "ps1", "ps2", "ps3","ps4", "ps5", "ps6", "ps7"};

  for (int i = 0; i < 8; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }

  double max_speed = 6.2;
  Motor *leftMotor;
  leftMotor = robot->getMotor("left wheel motor");
  leftMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  Motor *rightMotor;
  rightMotor = robot->getMotor("right wheel motor");
  rightMotor->setPosition(INFINITY);
  rightMotor->setVelocity(0.0);
  
  kb.enable(TIME_STEP);
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    if (key==315){
    leftSpeed = 1.0;
    rightSpeed = 1.0;
    } else if (key==317){
    leftSpeed = -1.0;
    rightSpeed = -1.0;
    }else if (key==316){
    leftSpeed = 0.5;
    rightSpeed = -0.5;
    }else if (key==314){
    leftSpeed = -0.5;
    rightSpeed = 0.5;
    }else {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    }
    leftMotor->setVelocity(max_speed*leftSpeed);
    rightMotor->setVelocity(max_speed*rightSpeed);
   
  }
  delete robot;
  return 0;
}