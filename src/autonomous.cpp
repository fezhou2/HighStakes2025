#include "autonomous.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"
#include <iostream>



const int pistonInterval = 400;

void autonInit(void) {

  resetHeading();
  resetForwardPos();
  MyGps.resetForwardPosGps();
}

void auton_negative_red(){
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(100);
  setConveyorSpeed(100);
  MyGps.gpsPIDMove(-20, 800, 1);
  posForwardRelWithHeading(30, -420, 33); // move to ring
  posForwardRelWithHeading(-30, 100, 70); // move back
  this_thread::sleep_for(300);
  setConveyorSpeed(0);
  posForwardRelWithHeading(30, -450, 75); // move to ring
  MyGps.gpsPIDMove(780, 920, -1); // move to goal 

  setClamp(true);
  setConveyorSpeed(100);

  MyGps.gpsPIDMove(250, 900, 1);

  MyGps.gpsPIDMove(200, 100, 1); //other ring

  // PIDPosForwardRel(-120);
  // PIDAngleRotateAbs(225);
  MyGps.gpsPIDMove(0, 0, 1);
  setIntakeSpeed(-100);
  posForwardRelWithHeading(40, 200, 225); // go into the corner rings
  setIntakeSpeed(100);

  PIDPosForwardRel(-150); // move back

  // qual (touch hang)
  // PIDPosForwardRel(100);
  // this_thread::sleep_for(600);
  // MyGps.gpsPIDMove(1300, 950, -1); // move back

  //elim (stake)
  PIDAngleRotateAbs(110);
  setConveyorSpeed(0);
  setClamp(false);
  setIntakeSpeed(-100);

  MyGps.gpsPIDMove(1370, -170, 1); // move to stake
  setIntakeSpeed(100);
  PIDAngleRotateAbs(0);
  posForwardRelWithHeading(-50, -130, 0);
  setConveyorSpeed(100);
  this_thread::sleep_for(100);
  WallStake();
  this_thread::sleep_for(400);

  Brain.Screen.setCursor(11, 1);
  std::cout << autotimer.getTime() << std::endl;
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_AWP_red(){
  
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(-100); //reverse to prop up ring
  //MyGps.gpsPIDMove(0, 182);
  posForwardAbsWithHeading(40, 170, 0);

  setIntakeSpeed(100); //intake
  setConveyorSpeed(100);
  this_thread::sleep_for(100);
  setConveyorSpeed(0);

  MyGps.gpsPIDMove(0, -950, -1); //go to goal
  posForwardAbsWithHeading(40, -1120, 0);
  this_thread::sleep_for(100);
  setClamp(true); 
  this_thread::sleep_for(100);
  setConveyorSpeed(100);

  MyGps.gpsPIDMove(300, -760, 1); //go to ring2

  MyGps.gpsPIDMove(-400, -950, 1); //move to ring pos //-400, 950
  setIntake(true); 
  posForwardRelWithHeading(30, 600, -100); //intake it moving in
  setIntake(false);

  posForwardRelWithHeading(-30, 150, -102); //back to intake
  this_thread::sleep_for(200);
  
  setIntakeSpeed(-100); //to repel blue ring.
  posForwardRelWithHeading(60, 1100, -135); 
  PIDAngleRotateAbs(-210);

  setConveyorSpeed(0);
  setClamp(false);

  setIntakeSpeed(100);
  PIDPosForwardRel(600); // go to ring
  
  MyGps.gpsPIDMove(-650, -1900, -1); // go to goal
  setClamp(true);
  setConveyorSpeed(100);
  this_thread::sleep_for(1000);
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_positive_red(){
  
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(100); 
  MyGps.gpsPIDMove(15, -800, -1);
  MyGps.gpsPIDMove(220, -1250, -1); // move to goal
  this_thread::sleep_for(100);
  setClamp(true);

  setConveyorSpeed(100);

  MyGps.gpsPIDMove(250, -600, 1); // go to ring
  this_thread::sleep_for(200);

  // qual path (2 rings, 1 on each mobile goal; touches hang) very consistent
  setConveyorSpeed(0);
  setIntakeSpeed(-100);
  PIDPosForwardRel(200);
  MyGps.gpsPIDMove(0, 0, -1); // drop off the goal in the corner
  setClamp(false);
  this_thread::sleep_for(200);
  MyGps.gpsPIDMove(850, -600, -1); // go to goal
  setClamp(true);
  setConveyorSpeed(100);
  this_thread::sleep_for(300);
  MyGps.gpsPIDMove(1150, -850, -1);

  // elim path (3 rings, 1 on one mobile goal, 2 on the other; doesn't touch hang) not as consistent
  // PIDAngleRotateAbs(-120);
  // setClamp(false); 
  // PIDPosForwardRel(-900); // turn outward
  // MyGps.gpsPIDMove(150, 200, 1); // go to rings
  // setIntakeSpeed(-100);
  // posForwardRelWithHeading(50, -400, -40);
  // setIntakeSpeed(100);
  // this_thread::sleep_for(300);
  // posForwardRelWithHeading(-50, -400, -10);
  // setConveyorSpeed(0);
  // MyGps.gpsPIDMove(650, -550, -1); // go to goal
  // setClamp(true); 
  // setConveyorSpeed(100);
  // this_thread::sleep_for(2000);
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_positive_stake_red() {
  MyTimer autotimer;
  autotimer.reset();

  setConveyorSpeed(100);
  setIntakeSpeed(-100);
  PIDPosForwardAbs(450); // fling away rings 
  PIDAngleRotateAbs(90);
  setIntakeSpeed(100);
  PIDPosForwardAbs(500); 
  this_thread::sleep_for(500);
  setConveyorSpeed(0);
  PIDAngleRotateAbs(140);
  PIDPosForwardAbs(100); // move to ring

  MyGps.gpsPIDMove(30, 530, -1); // move to stake
  posForwardRelWithHeading(-45, -230, 0);
  PIDAngleRotateAbs(90);
  setConveyorSpeed(100);
  this_thread::sleep_for(200);
  WallStake();
  this_thread::sleep_for(1500);
  setArmSpeed(20);

  MyGps.gpsPIDMove(150, -100, 1); // move to preload ring
  setConveyorSpeed(0);

  MyGps.gpsPIDMove(700, -120, -1); // move to goal
  setClamp(true); 
  setConveyorSpeed(100);
  PIDAngleRotateAbs(180);
  PIDPosForwardAbs(650); 
  this_thread::sleep_for(1500);

  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_positive_stake_blue() {
  MyTimer autotimer;
  autotimer.reset();

  setConveyorSpeed(100);
  setIntakeSpeed(-100);
  PIDPosForwardAbs(450); // fling away rings 
  PIDAngleRotateAbs(-90);
  setIntakeSpeed(100);
  PIDPosForwardAbs(500); 
  this_thread::sleep_for(500);
  setConveyorSpeed(0);
  PIDAngleRotateAbs(-140);
  PIDPosForwardAbs(100); // move to ring

  MyGps.gpsPIDMove(-30, 530, -1); // move to stake
  posForwardRelWithHeading(-45, -230, 0);
  PIDAngleRotateAbs(-90);
  setConveyorSpeed(100);
  this_thread::sleep_for(200);
  WallStake();
  this_thread::sleep_for(1500);
  setArmSpeed(20);

  MyGps.gpsPIDMove(-150, -100, 1); // move to preload ring
  setConveyorSpeed(0);

  MyGps.gpsPIDMove(700, -120, -1); // move to goal
  setClamp(true); 
  setConveyorSpeed(100);
  PIDAngleRotateAbs(-180);
  PIDPosForwardAbs(650); 
  this_thread::sleep_for(1500);

  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}


void auton_negative_blue(){
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(100);
  setConveyorSpeed(100);
  MyGps.gpsPIDMove(20, 800, 1);
  posForwardRelWithHeading(30, -420, -33); // move to ring
  posForwardRelWithHeading(-30, 100, -70); // move back
  this_thread::sleep_for(300);
  setConveyorSpeed(0);
  posForwardRelWithHeading(30, -450, -75); // move to ring
  MyGps.gpsPIDMove(-780, 920, -1); // move to goal 

  setClamp(true);
  setConveyorSpeed(100);

  MyGps.gpsPIDMove(-250, 900, 1);

  MyGps.gpsPIDMove(-200, 100, 1); //other ring

  // PIDPosForwardRel(-120);
  // PIDAngleRotateAbs(225);
  MyGps.gpsPIDMove(0, 0, 1);
  setIntakeSpeed(-100);
  posForwardRelWithHeading(40, 200, -225); // go into the corner rings
  setIntakeSpeed(100);

  PIDPosForwardRel(-150); // move back

  // qual (touch hang)
  // PIDPosForwardRel(100);
  // this_thread::sleep_for(600);
  // MyGps.gpsPIDMove(-1300, 950, -1); // move back

  //elim (stake)
  PIDAngleRotateAbs(-110);
  setConveyorSpeed(0);
  setClamp(false);
  setIntakeSpeed(-100);

  MyGps.gpsPIDMove(-1370, -170, 1); // move to stake
  setIntakeSpeed(100);
  PIDAngleRotateAbs(0);
  posForwardRelWithHeading(-50, -130, 0);
  setConveyorSpeed(100);
  this_thread::sleep_for(100);
  WallStake();
  this_thread::sleep_for(400);

  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_AWP_blue(){
  
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(-100); //reverse to prop up ring
  //MyGps.gpsPIDMove(0, 182);
  posForwardAbsWithHeading(40, 170, 0);

  setIntakeSpeed(100); //intake
  setConveyorSpeed(100);
  this_thread::sleep_for(100);
  setConveyorSpeed(0);

  MyGps.gpsPIDMove(0, -950, -1); //go to goal
  posForwardAbsWithHeading(40, -1120, 0);
  this_thread::sleep_for(100);
  setClamp(true); 
  this_thread::sleep_for(100);
  setConveyorSpeed(100);

  MyGps.gpsPIDMove(-300, -760, 1); //go to ring2

  MyGps.gpsPIDMove(400, -950, 1); //move to ring pos //-400, 950
  setIntake(true); 
  posForwardRelWithHeading(30, 600, 100); //intake it moving in
  setIntake(false);

  posForwardRelWithHeading(-30, 150, 102); //back to intake
  this_thread::sleep_for(200);
  
  setIntakeSpeed(-100); //to repel blue ring.
  posForwardRelWithHeading(60, 1100, 135); 
  PIDAngleRotateAbs(210);

  setConveyorSpeed(0);
  setClamp(false);

  setIntakeSpeed(100);
  PIDPosForwardRel(600); // go to ring
  
  MyGps.gpsPIDMove(650, -1900, -1); // go to goal
  setClamp(true);
  setConveyorSpeed(100);
  this_thread::sleep_for(1000);
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

void auton_positive_blue(){
  
  MyTimer autotimer;
  autotimer.reset();

  setIntakeSpeed(100); 
  MyGps.gpsPIDMove(-15, -800, -1);
  MyGps.gpsPIDMove(-220, -1250, -1); // move to goal
  this_thread::sleep_for(100);
  setClamp(true);

  setConveyorSpeed(100);

  MyGps.gpsPIDMove(-250, -600, 1); // go to ring
  this_thread::sleep_for(200);

  // qual path (2 rings, 1 on each mobile goal; touches hang) very consistent
  setConveyorSpeed(0);
  setIntakeSpeed(-100);
  PIDPosForwardRel(200);
  MyGps.gpsPIDMove(0, 0, -1); // drop off the goal in the corner
  setClamp(false);
  this_thread::sleep_for(200);
  MyGps.gpsPIDMove(-850, -600, -1); // go to goal
  setClamp(true);
  setConveyorSpeed(100);
  this_thread::sleep_for(300);
  MyGps.gpsPIDMove(-1150, -850, -1);

  // elim path (3 rings, 1 on one mobile goal, 2 on the other; doesn't touch hang) not as consistent
  // PIDAngleRotateAbs(120);
  // setClamp(false); 
  // PIDPosForwardRel(-900); // turn outward
  // MyGps.gpsPIDMove(-150, 200, 1); // go to rings
  // setIntakeSpeed(-100);
  // posForwardRelWithHeading(50, -400, 40);
  // setIntakeSpeed(100);
  // this_thread::sleep_for(300);
  // posForwardRelWithHeading(-50, -400, 10);
  // setConveyorSpeed(0);
  // MyGps.gpsPIDMove(-650, -550, -1); // go to goal
  // setClamp(true); 
  // setConveyorSpeed(100);
  // this_thread::sleep_for(2000);
  
  Brain.Screen.setCursor(11, 1);
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
}

// #ifdef ROBOT1
void runAuton(int auton_choose) {
  
  setAutonMode();
  autonFlipper(true);
  autonInit();

  if (auton_choose == 1) auton_AWP_red();  // solo AWP red (doesn't actually give AWP)
  else if (auton_choose == 2) auton_negative_red(); // negative corner red
  else if (auton_choose == 3) auton_positive_red();  // positive corner red
  else if (auton_choose == 4) auton_positive_stake_red(); // positive corner stake red
  else if (auton_choose == 5) auton_AWP_blue();  // solo AWP blue (doesn't actually give AWP)
  else if (auton_choose == 6) auton_negative_blue(); // negative corner blue
  else if (auton_choose == 7) auton_positive_blue(); // positive corner blue
  else if (auton_choose == 8) auton_positive_stake_blue(); // positive corner stake blue
}
// #endif
