using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFront;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;
extern motor FourBar;
extern motor BackLift;
extern motor Claw;
extern motor Intake;
extern inertial Inertial;
extern pot Potentiometer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );