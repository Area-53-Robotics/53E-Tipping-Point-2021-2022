#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT5, ratio18_1, true);
motor LeftBack = motor(PORT1, ratio18_1, true);
motor RightFront = motor(PORT19, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
motor FourBar = motor(PORT10, ratio36_1, true);
motor BackLift = motor(PORT2, ratio36_1, false);
motor Claw = motor(PORT15, ratio36_1, false);
motor Intake = motor(PORT4, ratio18_1, true);
inertial Inertial = inertial(PORT12);
pot Potentiometer = pot(Brain.ThreeWirePort.A);

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