#include "vex.h"
#include "robot-config.h"
#include "basic-functions.h"
#include "parameters.h"
#include "PID.h"
#include "my-timer.h"
#include "ezize.h"
#include "iostream"
#include "roundRobinQueue.h"
#include <deque>
#include <queue>
using namespace std;

bool autonChecker = false;

void autonFlipper(bool _checker) { autonChecker = _checker; }
float abbs(float x) { return x >= 0 ? x : -x; }
float deg2rad(float deg) { return deg / 180.0 * PI; }
float rad2deg(float rad) { return rad / PI * 180.0; }
float sqrf(float x) { return x * x; }
int sign(float _input) {
  if (_input > 0) return 1;
  else if (_input < 0) return -1;
  else return 0;
}
float deg2range(float _degree){
  int _cir = int(_degree / 360);
  _degree -= _cir * 360;
  if (_degree > 180.0) _degree -= 360;
  if (_degree < -180.0) _degree += 360;
  return _degree;
}

void moveLeft(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseLB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveLeftVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockLeft(void) {
  // locks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
}

void unlockLeft(void) {
  // unlocks all motors on left side of base
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
}

void moveRight(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRM.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
  Motor_BaseRB.spin(directionType::fwd, (int)130 * _input, voltageUnits::mV);
}

void moveRightVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int) _input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

void lockRight(void) {
  // locks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
}

void unlockRight(void) {
  // unlocks all motors on right side of base
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
}

void moveForward(float _input) {
  // move forward with _input% power
  moveLeft(_input);
  moveRight(_input);
}

void moveForwardVel(float _input) {
  // move forward with _input% speed
  moveLeftVel(_input);
  moveRightVel(_input);
}

void moveClockwise(float _input) {
  // rotate clockwise with _input% power
  moveLeft(_input);
  moveRight(-_input);
}
void lockBase(void) {
  // lock the base
  lockLeft();
  lockRight();
}

void unlockBase(void) {
  // unlock the base
  unlockLeft();
  unlockRight();
}

float getCrtVel(){
  return (Motor_BaseLM.velocity(pct) + Motor_BaseRM.velocity(pct)) / 2;
}

static volatile float _leftPosLast = 0, _rightPosLast = 0;

float getLeftPos() {
  // return the position of left side of base (mm from last reset position) according to encoder value
  return (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360) - _leftPosLast; // return mm
}

float getRightPos() {
  // return the position of right side of base (mm from last reset position) according to encoder value
  return (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360) - _rightPosLast; // return mm
  //return 0;
}

float getForwardPos() {
  // return the vertical position of base (mm from last reset position) according to encoder value
  return (getLeftPos() + getRightPos()) / 2;
}

float getForwardOdoXPos() {
 return Motor_OdoX.position(deg) * (ODO_WHEEL_DIAMETER * 25.4 * 3.1416) / 360.0;
}

float getForwardOdoYPos() {
 return Motor_OdoY.position(deg) * (ODO_WHEEL_DIAMETER * 25.4 * 3.1416) / 360.0;
}

void resetLeftPos() {
  // reset encoder on the left side of the base
  _leftPosLast = (Motor_BaseLF.position(deg)+Motor_BaseLM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360);
}

void resetRightPos() {
  // reset encoder on the right side of the base
  _rightPosLast = (Motor_BaseRF.position(deg)+Motor_BaseRM.position(deg)) * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4 * 3.1416) / (2.0*360);
}

void resetForwardPos() {
  // reset encoders on both side of the base
  resetLeftPos();
  resetRightPos();
}

static float headingOffset = 0;

float getHeading() {
  // return the heading angle of robot in deg (+360 after a full clockwise turn, -360 after a counter-clockwise turn) 
  return IMU.rotation() + headingOffset;
}

void resetHeading() {
  IMU.resetRotation();
  headingOffset = 0;
}

void resetHeading(float _offset) {
  // reset current heading angle of robot 
  IMU.resetRotation();
  headingOffset = _offset;
}

float getPitch() {
  // return the pitch angle of robot in deg
  return IMU.pitch();
}

// ------------------------------------------OTHER FUNCTIONS-------------------------------------------------
float intakeSpeed = 0;

//threaded
void intake() {
  while (true) { 
    Motor_Intake.spin(directionType::fwd, (int)130 * intakeSpeed, voltageUnits::mV);
    this_thread::sleep_for(1);
  } 
}

void setIntakeSpeed(float _input){
  intakeSpeed = _input;
}

float armSpeed = 10; //20
void arm() {
  while (true) {
    Motor_Arm.spin(directionType::fwd, (int) 130*armSpeed, voltageUnits::mV);
    this_thread::sleep_for(1);
  } 
}

void setArmSpeed(float _input){
  armSpeed = _input;
}

void ArmPID(float _target, float p = 0.8, float i = 0.01, float d = 0.03) {
  float actual_target = _target*3;
  auto pid = PID();
  pid.setCoefficient(p, i, d);
  pid.setTarget(actual_target);
  pid.setIMax(30);
  pid.setIRange(10);
  pid.setErrorTolerance(10);
  pid.setDTolerance(25);
  pid.setJumpTime(20);
  while (Motor_Arm.position(deg) < _target-30){ //} && myTimer.getTime() < 1000 + abbs(target * 10)) {
    pid.update(Motor_Arm.position(deg));
    setArmSpeed(100);
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print("Set Point: %.1f                           ", getForwardPos());
    this_thread::sleep_for(5);
  }
  while (!pid.targetArrived()){ //} && myTimer.getTime() < 1000 + abbs(target * 10)) {
    pid.update(Motor_Arm.position(deg));
    setArmSpeed(pid.getOutput());
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print("Set Point: %.1f                           ", getForwardPos());
    this_thread::sleep_for(5);
  }
}

void WallStake() {
  ArmPID(-37);
  setArmSpeed(-5);
}

void setArm(bool _input) {
  if (_input) {
    setArmSpeed(-100);
    this_thread::sleep_for(250);
  } else {
  setArmSpeed(80); //15
  this_thread::sleep_for(400);
  setArmSpeed(10);

  }
}

void setIntake(bool _input) {
  if (_input)  IntakePiston.off();
  else IntakePiston.on();
}

void setClamp(bool _input) {
  if (_input)  ClampPiston.off();
  else ClampPiston.on();
}

void setClimb(bool _input) {
  if (_input)  ClimbPiston.off();
  else ClimbPiston.on();
}

//---------------------------------------------------CONVEYOR-----------------------------------------------
bool alliance = 0; // 0 - red alliance; 1 - blue alliance
//VARIABLES
float conveyorSpeed;
int HookPeriod = 360*68 / 6.0 / 4.0; //1020
int conveyorPeriod = HookPeriod*4; 
int conveyorTolerance = 450;

void conveyor() {
  while (true) { 
    Motor_Conveyor.spin(directionType::fwd, (int) 130*conveyorSpeed, voltageUnits::mV);
    this_thread::sleep_for(1);
  } 
}
void setConveyorSpeed(float _input) {
  conveyorSpeed = _input;
}
