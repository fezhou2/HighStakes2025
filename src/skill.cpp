#include "skill.h"
#include "parameters.h"
#include "my-timer.h"
#include "robot-config.h"
#include "GPS.h"
#include "iostream"
using namespace std;


void skillInit() {
 
  MyGps.resetForwardPosGps();
}

void skill(){
  MyTimer autotimer;
  autotimer.reset();

  //Adjust Wall-Stake
  WallStake();
  setConveyorSpeed(100);
  this_thread::sleep_for(1000);
  setConveyorSpeed(0);
  setArmSpeed(12); //Keep Arm Down

  //MOBILE GOAL
  PIDPosForwardAbs(400);
  MyGps.gpsPIDMove(-400, 400, -1);
  setClamp(true);

  //Ring 1/6
  setIntakeSpeed(100);
  MyGps.gpsPIDMove(-450, 1300, 1);
  setConveyorSpeed(100);
  this_thread::sleep_for(250);
  setConveyorSpeed(0);

  //Ring 2/6
  MyGps.gpsPIDMove(150, 1900, 1);

  //Center --> Corner
  MyGps.gpsPIDMove(-450, 1100, 1);
  setConveyorSpeed(100);

  //Ring 3/6
  MyGps.gpsPIDMove(-1000, 1100, 1);

  //Ring 4-5/6
  MyGps.gpsPIDMove(-1000, 600, 1);
  this_thread::sleep_for(150);
  posForwardRelWithHeading(40, 300, -180);

  MyGps.gpsPIDMove(-800, 600, -1);

  //Ring 6/6
  MyGps.gpsPIDMove(-1250,450,1);

  //Release + Corner
  MyGps.gpsPIDMove(-1400,100, -1);
  setClamp(false);
  setConveyorSpeed(-100);
  this_thread::sleep_for(100);
  setConveyorSpeed(0);
  

  //WALL STAKE
  //Ring 1/2 
  MyGps.gpsPIDMove(-1400,1700,1);
  this_thread::sleep_for(200);
  setConveyorSpeed(100);
  this_thread::sleep_for(300);
  setConveyorSpeed(0);

  //Ring 2/2
  MyGps.gpsPIDMove(-1250, 2350, 1);
  this_thread::sleep_for(150);
  setConveyorSpeed(100);
  this_thread::sleep_for(300);
  setConveyorSpeed(0);

  //Score On Wall Stake
  MyGps.gpsPIDMove(-1400,1750,-1);
  PIDAngleRotateAbs(87);
  posForwardRelWithHeading(-30, 200, 87);

  setArm(true);
  //setConveyorSpeed(100);
  this_thread::sleep_for(1000);
  setConveyorSpeed(0);
  setArm(false);

  //Reset Position
  MyGps.resetForwardPosGps();


  

  








  


  




  





  


  


  
  Brain.Screen.setCursor(8, 1);
  std::cout << "AutonTimer: %d                            ", autotimer.getTime();
  Brain.Screen.print("AutonTimer: %d                            ", autotimer.getTime());
  
}

void runSkill(){
    skillInit();
    skill();
}
