using namespace vex;
using signature = vision::signature;
extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Motor_BaseLF;
extern motor Motor_BaseLM;
extern motor Motor_BaseLB;
extern motor Motor_BaseRF;
extern motor Motor_BaseRM;
extern motor Motor_BaseRB;
extern motor Motor_Intake;
extern motor Motor_Conveyor;
extern motor Motor_Arm;
extern motor Motor_OdoX;
extern motor Motor_OdoY;

extern led IntakePiston;
extern led ClampPiston;
extern led ClimbPiston;
extern inertial IMU;
extern optical colorSensor1;
extern optical colorSensor2;
// VEXcode devices
// Andew: as a code practice - can you modify this into an array of signatures?   
// eg array<signature, 5> signatures;






/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
