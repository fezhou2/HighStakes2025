#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

#ifdef ROBOT1
  motor Motor_BaseLF = motor(PORT1, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT4, ratio6_1, false);
  motor Motor_BaseLB = motor(PORT3, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT10, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT9, ratio6_1, true);
  motor Motor_BaseRB = motor(PORT8, ratio6_1, false);
  motor Motor_Intake = motor(PORT2, ratio6_1, false);
  motor Motor_Conveyor = motor(PORT13, ratio6_1, false);
  motor Motor_Arm = motor(PORT11, ratio36_1, false);
  led IntakePiston = led(Brain.ThreeWirePort.H);
  led ClampPiston = led(Brain.ThreeWirePort.D);
  led ClimbPiston = led(Brain.ThreeWirePort.F);
  // motor Motor_Intake2 = motor(PORT10, ratio6_1, false);
  inertial IMU = inertial(PORT7);
  optical colorSensor1 = optical(PORT20); 
  optical colorSensor2 = optical(PORT19);
  motor Motor_OdoX = motor(PORT6);
  motor Motor_OdoY = motor(PORT16);
#endif

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
