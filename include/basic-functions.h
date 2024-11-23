#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "robot-config.h"
#include "parameters.h"
#include "vex.h"

// using namespace vex;
//autonchecker
void autonFlipper(bool);
// Calculation
float abbs(float);
float deg2rad(float);
float rad2deg(float);
float sqrf(float);
int sign(float);
float deg2range(float);

// Output functions
void moveLeft(float);
void moveLeftVel(float);
void lockLeft(void);
void unlockLeft(void);
void moveRight(float);
void moveRightVel(float);
void lockRight(void);
void unlockRight(void);
void moveForward(float);
void moveForwardVel(float);
void moveClockwise(float);
void lockBase(void);
void unlockBase(void);
float getCrtVel();

// Input functions

float getLeftPos();
float getRightPos();
float getForwardPos();
void resetLeftPos();
void resetRightPos();
void resetForwardPos();
float getForwardOdoXPos();
float getForwardOdoYPos();

float getHeading();
void resetHeading();
void resetHeading(float);
float getPitch();
//-----------------------------------------
//INTAKE
void intake();
void setIntakeSpeed(float);

//CONVEYOR
void conveyor();


float getConveyorPos();
void resetConveyorPos();
void setConveyorSpeed(float);


//Arm
void arm();
void setArmSpeed(float);
void setArm(bool);
void ArmPID(float, float, float, float);
void WallStake();

void setIntake(bool);
void setClamp(bool);
void setClimb(bool);



#endif
