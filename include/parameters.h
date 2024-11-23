#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

//static int CRTCOLOR = 1; // 1 is red, 0 is blue

const float Movement_LowerLimit = 5;
const float Joystick_LowerDeadzone = 5;
volatile static float gpsPosX, gpsPosY, gpsHeading;

static const float PI = 3.1416;

#ifdef ROBOT1
  static const float WHEEL_DIAMETER = 2.75;
  static const float ODO_WHEEL_DIAMETER = 2.0;
  static const float CHASSIS_GEAR_RATIO = 0.8;
  static const float CHASSISRADIUS = 142; //what is 
  static const float kGyro = 1800.0 / 1800.0;
  static const color kRed = color(225, 127, 0);
  static const color kBlue = color(0, 0, 255);
#endif
#endif