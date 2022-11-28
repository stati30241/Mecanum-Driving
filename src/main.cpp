/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\stati                                            */
/*    Created:      Thu Nov 17 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "MecDrivetrain.h"

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Intializes the controller and mecanum drivetrain
  vex::controller controller1;
  MecDrivetrain mecDrivetrain{ 0, 1, 2, 3, 319, 175, 260 };

  // Main loop
  while (true) {
    // Gets the positions of the drive control axis (axis 3 and 4)
    int32_t x = -controller1.Axis4.position(vex::percentUnits::pct);
    int32_t y = controller1.Axis3.position(vex::percentUnits::pct);

    // Calculates the magnitude and angle from the x and y components
    double mag = std::sqrt(x * x + y * y) * 3.0 / 4.0; // Multiplied by 3/4 to reduce speed (for now)
    double theta = std::atan2(y, x) - M_PI_2; // Offset so that front is 0 rad

    // Gets the position of the rotation control axis (axis 1)
    int32_t rot = controller1.Axis1.position(vex::percentUnits::pct) * 3.0 / 4.0; // Multiplied by 3/4 to reduce speed (for now)

    // Drives and rotates the robot based on the input
    if (rot) mecDrivetrain.rotate(rot, vex::velocityUnits::pct);
    else mecDrivetrain.drive(mag, vex::velocityUnits::pct, theta);

    // Prints the info on the controller screen
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(1, 1);
    controller1.Screen.print("Magnitude: %lf", mag);
    controller1.Screen.setCursor(2, 1);
    controller1.Screen.print("Angle: %lf", theta * 180.0 / M_PI);

    vex::task::sleep(50);
  }
  
  return 0;
}
