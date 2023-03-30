/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftFront            motor         5               
// LeftBack             motor         1               
// RightFront           motor         19              
// RightBack            motor         20              
// FourBar              motor         10              
// BackLift             motor         2               
// Claw                 motor         15              
// Intake               motor         4               
// Inertial             inertial      12              
// Potentiometer        pot           A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

float arcMeasure(float inches, float radius) {
  return (inches/(2*radius*M_PI))*360;
}

float arcLength(float degrees, float radius) {
  return (degrees/360)*(2*radius*M_PI);
}

int auton = 4;
//1 = left WP; 2 = right; 3 = left; >10 = skills
bool driveDirection = true;
bool fourBarMode = true;
bool clawMode = true;
int intake = 0;
float wheelRadius = 2.0625;
float botWidth = 15.25;
//bool moved = false;
int x;
int y;
bool driverInfo = false;
//bool hookMode = true;
/*bool X_old = false;
bool Y_old = false;
bool A_old = false;
bool B_old = false;
//bool ButtonL_old = false;
bool ButtonR_old = false;
bool L1_old = false;
bool L2_old = false;
bool R1_old = false;*/
//int count = 0;

void resetEncoders() {
  LeftFront.setPosition(0,degrees);
  LeftBack.setPosition(0,degrees);
  RightFront.setPosition(0,degrees);
  RightBack.setPosition(0,degrees);
}

void move(float inches,float leftSpeed, float rightSpeed, bool blocking = true) {
  float spinDistance = arcMeasure(inches,wheelRadius);
  LeftFront.spinFor(forward,spinDistance,deg,leftSpeed,velocityUnits::pct,false);
  LeftBack.spinFor(forward,spinDistance,deg,leftSpeed,velocityUnits::pct,false);
  RightFront.spinFor(forward,spinDistance,deg,rightSpeed,velocityUnits::pct,false);
  RightBack.spinFor(forward,spinDistance,deg,rightSpeed,velocityUnits::pct,blocking);
}

void move(float inches,float speed) {
  move(inches, speed, speed, true);
}

void stopDrive() {
  LeftFront.stop();
  LeftBack.stop();
  RightFront.stop();
  RightBack.stop();
}

void moveInertial(float inches, float speed = 100, bool stopping = true, float kP = 5) {
  resetEncoders();
  while (Inertial.isCalibrating()) {}
  Inertial.resetRotation();
  float leftSpeed = speed;
  float rightSpeed = speed;
  float sensorValue;
  while (inches>0?(LeftFront.position(degrees)<arcMeasure(inches,wheelRadius)):(LeftFront.position(degrees)>arcMeasure(inches,wheelRadius))) {
    sensorValue = Inertial.rotation(degrees);
    if (sensorValue > 0) {
      leftSpeed = inches>0?speed - (sensorValue*kP):speed + (sensorValue*kP);
      rightSpeed = speed;
    } else if (sensorValue < 0) {
      rightSpeed = inches>0?speed + (sensorValue*kP):speed - (sensorValue*kP);
      leftSpeed = speed;
    } else {
      leftSpeed = speed;
      rightSpeed = speed;
    }
    LeftFront.spin(inches>0?forward:reverse,leftSpeed,pct);
    LeftBack.spin(inches>0?forward:reverse,leftSpeed,pct);
    RightFront.spin(inches>0?forward:reverse,rightSpeed,pct);
    RightBack.spin(inches>0?forward:reverse,rightSpeed,pct);
  }
  //WARNING: Setting stopping to false will never stop, include separate stopDrive() later
  if (stopping) {
    stopDrive();
  } else {
    leftSpeed = speed;
    rightSpeed = speed;
  }
}
 
void rotate(float degrees, char turnDirection, float speed) {
  float spinDistance = arcMeasure(arcLength(degrees,botWidth/2),wheelRadius);
  RightFront.spinFor((turnDirection=='L'?forward:reverse),spinDistance,deg,speed,velocityUnits::pct,false);
  RightBack.spinFor((turnDirection=='L'?forward:reverse),spinDistance,deg,speed,velocityUnits::pct,false);
  LeftFront.spinFor((turnDirection=='R'?forward:reverse),spinDistance,deg,speed,velocityUnits::pct,false);
  LeftBack.spinFor((turnDirection=='R'?forward:reverse),spinDistance,deg,speed,velocityUnits::pct);
}

void rotateInertial(float target, char turnDirection, float kP = 0.8, float kD = 0.12, float tolerance = 0.4) {
  while (Inertial.isCalibrating()) {}
  Inertial.resetRotation();
  float sensorValue = 0;
  float error = target;
  float speed;
  float previousError = 0;
  float derivative;
  float minimum = 1;
  wait(10,msec);
  while (std::abs(error) - tolerance > 0) {
    sensorValue = std::abs(Inertial.rotation(degrees));
    error = target - sensorValue;
    derivative = error - previousError;
    speed = (error * kP) + (derivative * kD);
    if (speed > 0 && speed < minimum) {
      speed = minimum;
    } else if (speed < 0 && speed > -minimum) {
      speed = -minimum;
    }
    /*Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Sensor: %.2f",sensorValue);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Speed: %.2f",speed);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("Error: %.2f",std::abs(error));*/
    LeftFront.spin(turnDirection=='R'?forward:reverse,speed,pct);
    LeftBack.spin(turnDirection=='R'?forward:reverse,speed,pct);
    RightFront.spin(turnDirection=='L'?forward:reverse,speed,pct);
    RightBack.spin(turnDirection=='L'?forward:reverse,speed,pct);
    previousError = error;
    wait(0.2,sec);
  }
  stopDrive();
}

void rotateInertial(float target, char turnDirection, bool hasGoal) {
  rotateInertial(target, turnDirection, 0.8, hasGoal?0.15:0.12);
}

void arcTurn(float target, char turnDirection, char side, float kP, float kD) {
  //Controller1.Screen.clearScreen();
  while (Inertial.isCalibrating()) {}
  Inertial.resetRotation();
  float sensorValue = 0;
  float error = target;
  float speed;
  float previousError = 0;
  float derivative;
  float minimum = 2;
  while (std::abs(error) - 1 > 0) {
    sensorValue = std::abs(Inertial.rotation(degrees));
    error = target - sensorValue;
    derivative = error - previousError;
    speed = (error * kP) + (derivative * kD);
    if (speed > 0 && speed < minimum) {
      speed = minimum;
    } else if (speed < 0 && speed > -minimum) {
      speed = -minimum;
    }
    /*Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Sensor: %.2f",sensorValue);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("Speed: %.2f",speed);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("Error: %.2f",std::abs(error));*/
    if (turnDirection == 'L' && side == 'L') {
      
    } else if (turnDirection == 'L' && side == 'R') {
      RightFront.spin(forward, speed, pct);
      RightBack.spin(forward, speed, pct);
    } else if (turnDirection == 'R' && side == 'L') {
      LeftFront.spin(forward, speed, pct);
      LeftBack.spin(forward, speed, pct);
    }
    previousError = error;
    wait(0.2,sec);
  }
  stopDrive();
}

void moveBackLift(bool blocking, bool potentiometer) {
  if(potentiometer) {
    float sensorValue = std::abs(Potentiometer.value(degrees)-250);
    bool direction = sensorValue<84;
    while (direction?(sensorValue<137):(sensorValue>83)) {
      sensorValue = std::abs(Potentiometer.value(degrees)-250);
      BackLift.spin(direction?reverse:forward,100,pct);
      task::sleep(20);
    }
  } else {
    BackLift.rotateTo(BackLift.position(degrees)<-720?-390:-840,degrees,100,velocityUnits::pct,blocking);
  }
  BackLift.stop();
}

void moveBackLiftDown() {
  float sensorValue = std::abs(Potentiometer.value(degrees)-250);
  while (sensorValue<135) {
    sensorValue = std::abs(Potentiometer.value(degrees)-250);
    BackLift.spin(reverse,100,pct);
  }
  BackLift.stop();
}

void resetBackLift() {
  float sensorValue = std::abs(Potentiometer.value(degrees)-250);
  while (sensorValue>10) {
    sensorValue = std::abs(Potentiometer.value(degrees)-250);
    BackLift.spin(forward,100,pct);
  }
  BackLift.stop();
}

int moveBackLiftInt() {
  moveBackLift(true, true);
  return 1;
}

void moveBackLift() {
  task BackLiftTask(moveBackLiftInt);
}

//input negative angle for hook to go down, positive to go up
/*void moveBackLift(float angle, float speed) {
  BackLift.rotateTo(angle,deg,speed,velocityUnits::pct);
}*/

void moveFourBar(bool blocking) {
  if (fourBarMode) {
    FourBar.rotateTo(FourBar.position(degrees)>10?0:545,degrees,100,velocityUnits::pct,blocking);
    /*if(Claw.position(degrees)<500) {
      Claw.rotateTo(FourBar.position(degrees)>10?60:50,degrees);
    }*/
  }
}

void moveFourBar() {
  moveFourBar(false);
}

void moveClaw(float speed, bool blocking) {
  Claw.rotateTo(Claw.position(degrees)>-380?-440:-290,degrees,speed,velocityUnits::pct,blocking);
}

void moveClaw() {
  moveClaw(100, false);
}

void changeDriveDirection() {
  driveDirection = !driveDirection;
  Controller1.rumble(".");
}
void changeFourBarMode() {
  fourBarMode = !fourBarMode;
  Controller1.rumble(rumbleLong);
}

void changeClawMode() {
  clawMode = !clawMode;
  Controller1.rumble(rumbleLong);
}

float driveCurve(float x) {
  //Drive curve found using Desmos exponential regression
  float a = -112.5;//0.00761072;
  float b = 0.978267;//1.09948;
  float c = -a;
  return ((a * pow(b,std::abs(x))) + c)*(std::abs(x)/x);
}

void changeIntake() {
  if (intake == 2) {
    intake = 0;
  } else {
    intake++;
  }
  Controller1.rumble("-");
}

void setLiftStopping(brakeType type) {
  FourBar.setStopping(type);
  BackLift.setStopping(type);
  Claw.setStopping(type);
}

void setDriveStopping(brakeType type) {
  LeftFront.setStopping(type);
  LeftBack.setStopping(type);
  RightFront.setStopping(type);
  RightBack.setStopping(type);
}

void setDriveTimeout(float seconds) {
  LeftFront.setTimeout(seconds, sec);
  LeftBack.setTimeout(seconds, sec);
  RightFront.setTimeout(seconds, sec);
  RightBack.setTimeout(seconds, sec);
}

void auton1() {
  auton = 1;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Left WP");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 1 selected");
}

void auton2() {
  auton = 2;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Right neutral");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 2 selected");
}

void auton3() {
  auton = 3;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Right WP");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 3 selected");
}

void auton4() {
  auton = 4;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Full WP");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 4 selected");
}

void auton5() {
  auton = 5;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton 5");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 5 selected");
}

void auton6() {
  auton = 6;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton 6");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 6 selected");
}

void auton7() {
  auton = 7;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton 7");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 7 selected");
}

void auton8() {
  auton = 8;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton 8");
  Brain.Screen.setCursor(10,2);
  Brain.Screen.clearLine();
  Brain.Screen.print("Auton 8 selected");
}

void screenControls() {
  driverInfo = false;
  Brain.Screen.clearScreen();
  Brain.Screen.setPenWidth(1);
  for (int i = 0;i < 8; i++) {
    Brain.Screen.drawRectangle(i*60+5,20,50,40);
    Brain.Screen.setCursor(2,i*6+2);
    Brain.Screen.print(i+1);
  }
  /*Brain.Screen.drawRectangle(5,20,50,40);
  Brain.Screen.setCursor(2,2);
  Brain.Screen.print("1");
  Brain.Screen.drawRectangle(65,20,50,40);
  Brain.Screen.setCursor(2,8);
  Brain.Screen.print("2");
  Brain.Screen.drawRectangle(125,20,50,40);
  Brain.Screen.setCursor(2,14);
  Brain.Screen.print("3");
  Brain.Screen.drawRectangle(185,20,50,40);
  Brain.Screen.setCursor(2,20);
  Brain.Screen.print("4");
  Brain.Screen.drawRectangle(245,20,50,40);
  Brain.Screen.setCursor(2,26);
  Brain.Screen.print("5");*/
  Brain.Screen.drawRectangle(5,130,80,40);
  Brain.Screen.setCursor(8,2);
  Brain.Screen.print("Confirm");
  do {
    x = Brain.Screen.xPosition();
    y = Brain.Screen.yPosition();
    if (x >= 5 && x < 55 && y >= 20 && y < 60) {
      auton1();
    } else if (x >= 65 && x < 115 && y >= 20 && y < 60) {
      auton2();
    } else if (x >= 125 && x < 175 && y >= 20 && y < 60) {
      auton3();
    } else if (x >= 185 && x < 235 && y >= 20 && y < 60) {
      auton4();
    } else if (x >= 245 && x < 295 && y >= 20 && y < 60) {
      auton5();
    } else if (x >= 305 && x < 355 && y >= 20 && y < 60) {
      auton6();
    } else if (x >= 365 && x < 415 && y >= 20 && y < 60) {
      auton7();
    } else if (x >= 425 && x < 475 && y >= 20 && y < 60) {
      auton8();
    }
  } while (!(x >= 5 && x < 85 && y >= 130 && y < 170));
  Brain.Screen.clearScreen();
  driverInfo = true;
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Inertial.startCalibration();
  Potentiometer.value(degrees);
  Brain.Screen.pressed(screenControls);
  /*Controller1.ButtonUp.pressed(auton1);
  Controller1.ButtonRight.pressed(auton2);
  Controller1.ButtonDown.pressed(auton3);
  Controller1.ButtonLeft.pressed(auton4);*/
}

void autonomous(void) {
  setLiftStopping(hold);
  setDriveStopping(hold);
  while (Inertial.isCalibrating()) {}
  switch(auton) {
    //Left side win point
    case 1:
    /*move(3,15);
    moveClaw(20,true);
    wait(0.5,seconds);
    move(-8,30);
    moveClaw(50,true);*/
    moveBackLift();
    move(-14,25);
    moveBackLift();
    move(10,50);
    Intake.spin(fwd,100,pct);
    moveFourBar();
    move(20,25);
    break;

    //Right side mogo, middle, win point (a2)
    case 2:
    resetEncoders();
    Claw.rotateTo(-290, degrees, 100, velocityUnits::pct, false);
    wait(10,msec);
    moveInertial(38,100,false);
    moveClaw();
    moveInertial(4,100);
    stopDrive();
    moveInertial(-18.5,100);
    moveClaw();
    wait(0.1,sec);
    moveInertial(-11.5,100);
    rotateInertial(34.8,'L',0.7,0.05);
    resetEncoders();
    moveInertial(48,100,false, 4);
    moveClaw(100,false);
    moveInertial(6,100);
    stopDrive();
    FourBar.rotateTo(100, degrees, 100, velocityUnits::pct);
    moveInertial(-44,100);
    rotateInertial(50,'L',0.7,0.4);
    moveBackLift();
    moveInertial(6,100);
    wait(1,sec);
    moveFourBar();
    moveInertial(4,100);
    setDriveTimeout(2);
    moveInertial(-20,50);
    moveBackLift(true,true);
    moveFourBar();
    moveFourBar();
    Intake.spin(forward,100,pct);
    //moveFourBar();
    moveInertial(20,100);
    //rotateInertial(30,'R',0.8,0.2);
    //moveInertial(20,100);
    //moveInertial(40,60);
    break;

    //Left side neutral + win point (a3)
    case 3:
    resetEncoders();
    Claw.rotateTo(-290, degrees, 100, velocityUnits::pct, false);
    wait(20, msec);
    moveInertial(43,100,false);
    moveClaw();
    moveInertial(3,100);
    stopDrive();
    moveInertial(-10,100);
    moveFourBar();
    wait(10,msec);
    rotateInertial(15,'R',0.8,0.13);
    moveInertial(-32,100);
    wait(0.1,sec);
    rotateInertial(88,'L',0.7,0.14);
    moveInertial(10,40);
    moveBackLift(true,true);
    //wait(0.1,sec);
    moveInertial(-20,30);
    moveBackLift(true,true);
    Intake.spin(forward, 100, pct);
    moveInertial(5, 50);
    rotateInertial(60,'L',0.7,0.15,3);
    moveInertial(15,20);
    moveInertial(-15,50);
    break;

    //secret right side auton (a4)
    case 4:
    resetEncoders();
    Claw.rotateTo(-290, degrees, 100, velocityUnits::pct, false);
    moveInertial(38,100,false);
    moveClaw(100,false);
    moveInertial(6,100);
    stopDrive();
    moveInertial(-22,100);
    FourBar.rotateTo(200, degrees, 100, velocityUnits::pct);
    moveBackLift();
    rotateInertial(135,'R', 0.4, 0);
    moveInertial(-20,100);
    BackLift.rotateFor(300, degrees, 100, velocityUnits::pct,false);
    moveInertial(-16,100);
    moveInertial(20,60);
    rotateInertial(160, 'L', 0.4, 0);
    moveInertial(-17,100);
    moveBackLiftDown();
    moveInertial(-5,100);
    moveInertial(18,100);
    rotateInertial(35,'L');
    moveInertial(-15,100);
    moveInertial(-5,35);
    moveBackLift();
    Intake.spin(forward, 100, velocityUnits::pct);
    break;

    case 5:
    //juke auton
    Claw.rotateTo(-290, degrees, 100, velocityUnits::pct, false);
    moveInertial(21,100);
    rotateInertial(47,'L',false);
    moveInertial(40,100,false);
    moveClaw();
    moveInertial(6,50);
    stopDrive();
    moveInertial(-50,100);
    //moveInertial()
    //moveInertial(-)

    //Testing
    case 10:
    rotateInertial(90, 'R'); //0.8 0.15 with mogo, 0.8 0.12 without
    break;
    
    //Skills (s1)
    case 11:
    Inertial.startCalibration();
    while (Inertial.isCalibrating()) {}
    /*moveBackLift(true,true);
    move(-12,30);
    moveFourBar();
    moveBackLift(true,true);
    Intake.spin(forward, 100, pct);
    wait(2,sec);
    moveInertial(10,35);
    move(-10,35);
    moveInertial(10,35);*/
    moveInertial(102);
    //moveBackLift(true, true);
    moveInertial(-12);
    resetBackLift();
    rotateInertial(90, 'L');
    setDriveTimeout(2);
    move(-6,100);
    setDriveTimeout(30);
    moveInertial(20);
    rotateInertial(90,'L');
    moveInertial(50);
    moveInertial(-4,100);
    wait(10,msec);
    rotateInertial(90,'R');
    moveInertial(24,100);
    rotateInertial(90,'R');
    //Go for middle
    setDriveTimeout(5);
    moveInertial(25,100);
    moveInertial(36,100);
    moveInertial(-4);
    rotateInertial(90,'L');
    moveInertial(26,100);
    rotateInertial(88,'L');
    //Go for last neutral
    moveInertial(18,50);
    moveClaw();
    moveInertial(31,100);
    rotateInertial(40, 'R', false);
    moveInertial(30,100);
    setDriveTimeout(5);
    move(-120.,100);
    moveInertial(4);
    rotateInertial(90, 'R');
    setDriveTimeout(2);
    moveInertial(30);
    setDriveTimeout(30);
    moveInertial(-4);
    rotateInertial(90, 'L');
    moveInertial(150);
    /*Drop neutral in corner
    moveClaw();
    move(-10,100);
    //Turn toward alliance goal
    rotateInertial(110, 'L');
    moveInertial(12, 40);
    //Grab alliance goal
    moveClaw(100,true);
    move(-12,100);
    rotateInertial(110,'R');
    moveInertial(-15,100);
    rotateInertial(40, 'L');
    //Back up to wall
    move(-70,100);
    move(-10,50);
    moveInertial(27,100);
    rotateInertial(90, 'L');
    move(10,50);
    moveBackLift();
    move(-15,50);
    moveBackLift();
    moveInertial(30,100);
    moveFourBar();
    Intake.spin(forward,100,pct);
    moveInertial(45,50);
    FourBar.rotateFor(-100,deg,20,velocityUnits::pct);
    moveClaw();
    FourBar.rotateFor(100,deg,20,velocityUnits::pct);
    moveInertial(-12,100);
    moveFourBar();*/
    break;

    //Skills 2 (with rings) (s2)
    case 12:
    moveBackLift(true,true);
    move(-12,30);
    moveFourBar();
    moveBackLift(true,true);
    Intake.spin(forward, 100, pct);
    wait(2,sec);
    moveInertial(10,35);
    move(-10,35);
    moveInertial(10,35);
    moveInertial(-110,35);
    moveBackLift(true, true);
    moveInertial(12, 35);
    moveBackLift(true,true);
    break;

    //Skills 3 (s3)
    case 13:
    Inertial.startCalibration();
    while (Inertial.isCalibrating()) {}
    moveBackLift(true,true);
    moveInertial(-8,25);
    moveFourBar();
    moveBackLift();
    moveInertial(-8, 15);
    Intake.spin(forward, 100, pct);
    moveInertial(14, 15);
    moveFourBar();
    Intake.stop();
    moveInertial(-10, 100);
    rotateInertial(90, 'L', true);
    moveInertial(-20, 100);
    rotateInertial(90, 'L', true);
    //Get right side neutral
    moveInertial(20, 100);
    moveClaw();
    moveInertial(30, 100);
    moveFourBar();
    Intake.spin(forward, 100, pct);
    rotateInertial(90, 'L', true);
    moveInertial(24, 100);
    break;

    //Skills 4 (s4)
    case 14:
    moveBackLift();
    wait(10,msec);
    moveInertial(-18);
    rotateInertial(90, 'R');
    moveInertial(-10);
    moveBackLift();
    moveInertial(-6, 25);
    moveInertial(13, 100);
    rotateInertial(90, 'R', true);
    moveInertial(24, 100);
    moveClaw(100,true);
    moveInertial(8);
    Intake.spin(forward,100,pct);
    moveFourBar(true);
    rotateInertial(45, 'L', 0.8, 0.2);
    moveInertial(34);
    rotateInertial(20, 'R', 0.8, 0.2);
    moveInertial(5, 20);
    FourBar.rotateTo(520, degrees, 100, velocityUnits::pct);
    wait(1,sec);
    moveClaw(100,true);
    FourBar.rotateTo(545, degrees, 100, velocityUnits::pct);
    moveInertial(-5);
    rotateInertial(45, 'L', true);
    moveBackLift(true,true);
    moveInertial(54);
    moveInertial(6, 20);
    moveClaw(100,true);
    FourBar.rotateTo(100, degrees, 100, velocityUnits::pct);
    rotateInertial(180, 'R');
    moveInertial(6);
    moveFourBar();
    moveInertial(-4,50);
    resetBackLift();
    rotateInertial(90, 'L');
    moveInertial(4, 30);
    FourBar.rotateTo(520, degrees, 100, velocityUnits::pct);
    moveClaw();
    FourBar.rotateTo(545, degrees, 100, velocityUnits::pct);
    moveInertial(-4);
    rotateInertial(90, 'R');
    moveInertial(-59);
    moveInertial(-4, 30);
    break;
  }
}

void usercontrol(void) {
    //Controller1.Screen.clearScreen();
    Brain.Screen.clearScreen();
    //Driving speed multiplier, accepts decimal values 0 to 1
    //float driveSpeed = 1;
    setDriveStopping(brake);
    Controller1.ButtonL2.pressed(moveBackLift);
    Controller1.ButtonL1.pressed(moveClaw);
    Controller1.ButtonR1.pressed(moveFourBar);
    //Controller1.ButtonY.pressed(changeDriveDirection);
    Controller1.ButtonRight.pressed(changeFourBarMode);
    //Controller1.ButtonLeft.pressed(changeClawMode);
    Controller1.ButtonB.pressed(changeIntake);
    //Controller1.ButtonA.pressed(screenControls);
    setLiftStopping(hold);
    float J2;
    float J3;

  while (1) {
    //int J1 = Controller1.Axis1.position();
    //int J4 = Controller1.Axis4.position();
    J2 = Controller1.Axis2.position();
    J3 = Controller1.Axis3.position();

    //Reduces turn speed to less than driving speed
      /*if((J3>0 && J2<0) || (J3<0 && J2>0) || (abs(J2-J3)>10)) {
        driveSpeed = 0.6;
      }
      else{
        driveSpeed = 1;
      }*/

    //Tank drive with reversible direction (driveDirection) and adjustable speed (driveSpeed)
    LeftFront.spin(driveDirection?forward:reverse,driveCurve(driveDirection?J3:J2),pct);
    LeftBack.spin(driveDirection?forward:reverse,driveCurve(driveDirection?J3:J2),pct);
    RightFront.spin(driveDirection?forward:reverse,driveCurve(driveDirection?J2:J3),pct);
    RightBack.spin(driveDirection?forward:reverse,driveCurve(driveDirection?J2:J3),pct);

    //Intake
    if (intake == 1 && FourBar.position(degrees) > 10) {
      Intake.spin(forward,100,pct);
    } else if (intake == 2 && FourBar.position(degrees) > 10) {
      Intake.spin(reverse,50,pct);
    } else {
      Intake.stop();
    }

    //Manual 4-bar lift
    if(!fourBarMode) { 
      if (Controller1.ButtonR1.pressing()) {
        FourBar.spin(forward,100,pct);
      } else if (Controller1.ButtonR2.pressing()) {
        FourBar.spin(reverse,100,pct);
      } else {
        FourBar.stop();
      }
    }

    //Manual back lift
    /*if (Controller1.ButtonL1.pressing()) {
      BackLift.spin(forward,50,pct);
    } else if (Controller1.ButtonL2.pressing()) {
      BackLift.spin(reverse,50,pct);
    } else {
      BackLift.stop();
    }*/

    //Manual claw
    /*if(!clawMode) {
      if (Controller1.ButtonUp.pressing()) {
        Claw.spin(forward,10,pct);
      } else if (Controller1.ButtonDown.pressing()) {
        Claw.spin(reverse,10,pct);
      } else {
        Claw.stop();
      }
    }*/

    if (Claw.torque(Nm)>1.5 && Claw.direction() == reverse) {
      Claw.stop();
      Controller1.rumble(rumbleShort);
    }

    /*Rear hook lift
    if(L2 && !L2_old) {
    BackLift.rotateTo(BackLift.position(degrees)>10?0:180,degrees,false);
    }

    //Front 4-bar lift main
    //Automatic
    if(fourBarMode && R1 && !R1_old) {
      FourBar.rotateTo(FourBar.position(degrees)>10?0:830,degrees,false);
    }

    //Front 4-bar lift clamp
    if(L1 && !L1_old) {
      FourBarClamp.rotateTo(FourBarClamp.position(degrees)>10?0:270,degrees,false);
    }

    //Change drive and lift modes when toggle buttons are pressed
    if(X && !X_old) {
      driveDirection = !driveDirection;
      //Controller1.rumble(".");
      count++;
    }
    
    if(ButtonR && !ButtonR_old) {
      fourBarMode = !fourBarMode;
      Controller1.rumble(rumblePulse);
    }
    */

    if (driverInfo) {
      //Prints position or torque of lifts to controller
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Claw: %.2f",Claw.position(degrees));
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print("Torque: %.2f",Claw.torque());
      /*if(!Inertial.isCalibrating()) {
      Controller1.Screen.print("Rotation: %.2f",Inertial.rotation(degrees));
      }*/
      Controller1.Screen.setCursor(3,1);
      Controller1.Screen.print("Potentiometer: %.2f",std::abs(Potentiometer.value(degrees)-250));

      //Prints motor temperatures to brain
      Brain.Screen.setCursor(2,2);
      Brain.Screen.print("Temperatures");
      Brain.Screen.setCursor(3,2);
      Brain.Screen.print("Left front: %.2f",LeftFront.temperature(fahrenheit));
      Brain.Screen.setCursor(4,2);
      Brain.Screen.print("Left back: %.2f",LeftBack.temperature(fahrenheit));
      Brain.Screen.setCursor(5,2);
      Brain.Screen.print("Right front: %.2f",RightFront.temperature(fahrenheit));
      Brain.Screen.setCursor(6,2);
      Brain.Screen.print("Right back: %.2f",RightBack.temperature(fahrenheit));
      Brain.Screen.setCursor(7,2);
      Brain.Screen.print("4-bar: %.2f",FourBar.temperature(fahrenheit));
      Brain.Screen.setCursor(8,2);
      Brain.Screen.print("Back lift: %.2f",BackLift.temperature(fahrenheit));
      Brain.Screen.setCursor(9,2);
      Brain.Screen.print("Claw: %.2f",Claw.temperature(fahrenheit));
      Brain.Screen.setCursor(10,2);
      Brain.Screen.print("Intake: %.2f",Intake.temperature(fahrenheit));
      //Controller1.Screen.print(count);
    }

    /*X_old = X;
    L1_old = L1;
    L2_old = L2;
    ButtonR_old = ButtonR;
    //ButtonL_old = ButtonL;*/
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}