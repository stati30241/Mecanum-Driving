# Mecanum-Driving

Private repo for the code for the mecanum drivetrain.

## Issues to fix
- The measurements are not working accurately: when rotating, it overturns (might just be the error in measuring)
- Implement a system to convert between velocity units (pct to rpm)
- Implementing torque
- Shoving all the controller code into a InputHandler class
- Better way to control speed (we dont robot to be too fast)

### Also we need encoders for odometry

## Will get to later
### Making the robot follow a path (for autonomous)
- What kind of path type (straight lines, bezier curves, other spline types)?
- An interpreter to make the robot follow the path
- A script for the user to edit the path (made in python maybe? I do wanna make it gui)
