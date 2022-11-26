#include "MecDrivetrain.h"


// Constructor
MecDrivetrain::MecDrivetrain(int32_t a, int32_t b, int32_t c, int32_t d, double wheelCircumference, double trackWidth, double wheelBase)
  : m_a{ a }, m_b{ b }, m_c{ c }, m_d{ d },
    m_wheelCircumference{ wheelCircumference },
    m_trackWidth{ trackWidth },
    m_wheelBase{ wheelBase } {
    initializeMotors();
}


// Initializes the motors
void MecDrivetrain::initializeMotors() {
  // Sets the motors on the right as reversed
  m_b.setReversed(true);
  m_d.setReversed(true);

  // Sets the default max velocity values
  // Set as default to 50% (50 pct)
  m_driveVelocity = 50;
  m_turnVelocity = 50;
  m_driveVelocityUnits = vex::velocityUnits::pct;
  m_turnVelocityUnits = vex::velocityUnits::pct;

  // ...
}


// Drives the drivetrain either forward or backward
void MecDrivetrain::drive(const vex::directionType& dir) {
  m_a.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_b.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_c.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_d.spin(dir, m_driveVelocity, m_driveVelocityUnits);
}


// Drives the drivetrain either forward or backward for a specific amount of time
void MecDrivetrain::drive(const vex::directionType& dir, uint32_t time, const vex::timeUnits& tUnits) {
  // Converts the time from seconds to milliseconds
  if (tUnits == vex::timeUnits::sec) time *= 1000;
  
  // Spins all the motors
  m_a.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_b.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_c.spin(dir, m_driveVelocity, m_driveVelocityUnits);
  m_d.spin(dir, m_driveVelocity, m_driveVelocityUnits);

  // Wait for the time duration
  vex::task::sleep(time);

  // Stops all the motors
  m_a.stop();
  m_b.stop();
  m_c.stop();
  m_d.stop();
}


// Drives the drivetrain in the given direction at the given velocity
// Angle inputted in radians
void MecDrivetrain::drive(double vel, const vex::velocityUnits& vUnits, double angle) {
  // Gets the x and y components of the velocity relative to the mecanum wheels
  double velX = vel * std::sin(angle + M_PI_4);
  double velY = vel * std::cos(angle + M_PI_4);

  // Spins the motors according to their velocities
  m_a.spin(vex::directionType::fwd, velX, vUnits);
  m_d.spin(vex::directionType::fwd, velX, vUnits);
  m_b.spin(vex::directionType::fwd, velY, vUnits);
  m_c.spin(vex::directionType::fwd, velY, vUnits);
}


// Rotates the drivetrain based on the velocity
void MecDrivetrain::rotate(double vel, const vex::velocityUnits& vUnits) {
  m_a.spin(vex::directionType::fwd, vel, vUnits);
  m_b.spin(vex::directionType::rev, vel, vUnits);
  m_c.spin(vex::directionType::fwd, vel, vUnits);
  m_d.spin(vex::directionType::rev, vel, vUnits);
}


// Rotates the drivetrain for the given amount of rotation
void MecDrivetrain::rotate(double rot, const vex::rotationUnits& rUnits) {
  // Gets the rotation angle
  double theta = 0.0;
  switch (rUnits) {
  case vex::rotationUnits::deg:
    theta = rot * M_PI / 180.0;
    break;
  case vex::rotationUnits::rev:
    theta = rot * 2 * M_PI;
    break;
  default:
    break;
  }

  // Calculates the length of the arc needed to travel
  double radius = std::sqrt(std::pow(m_trackWidth / 2.0, 2.0) + std::pow(m_wheelBase / 2.0, 2.0));
  //double circumference = 2 * M_PI * radius;
  //double arc = circumference * (theta / 2 * M_PI);
  double arc = theta * radius;

  // Spins the motors for the arc distance calculated
  // NEEDS THE VELOCITY TO BE IN RPM FOR THIS TO WORK
  // TODO: Find a way to convert from pct to rpm
  double vel = (m_turnVelocity * m_wheelCircumference) / 60000.0;
  uint32_t spinTime = std::round(std::abs(arc / vel));
  m_a.spin(vex::directionType::rev, m_turnVelocity, m_turnVelocityUnits);
  m_b.spin(vex::directionType::fwd, m_turnVelocity, m_turnVelocityUnits);
  m_c.spin(vex::directionType::rev, m_turnVelocity, m_turnVelocityUnits);
  m_d.spin(vex::directionType::fwd, m_turnVelocity, m_turnVelocityUnits);

  // Waits for the time duration
  vex::task::sleep(spinTime);

  // Stops all the motors
  m_a.stop();
  m_b.stop();
  m_c.stop();
  m_d.stop();
}


// Rotates the drivetrain for the given amount of time
void MecDrivetrain::rotate(bool dir, uint32_t time, const vex::timeUnits& tUnits) {
  // Converts the time from seconds to milliseconds
  if (tUnits == vex::timeUnits::sec) time *= 1000;

  // Spins all the motors
  m_a.spin(dir ? vex::directionType::rev : vex::directionType::fwd, m_driveVelocity, m_driveVelocityUnits);
  m_b.spin(dir ? vex::directionType::fwd : vex::directionType::rev, m_driveVelocity, m_driveVelocityUnits);
  m_c.spin(dir ? vex::directionType::rev : vex::directionType::fwd, m_driveVelocity, m_driveVelocityUnits);
  m_d.spin(dir ? vex::directionType::fwd : vex::directionType::rev, m_driveVelocity, m_driveVelocityUnits);

  // Wait for the time duration
  vex::task::sleep(time);

  // Stops all the motors
  m_a.stop();
  m_b.stop();
  m_c.stop();
  m_d.stop();
}


// Stops all the motors on the robot
void MecDrivetrain::stop(const vex::brakeType& mode) {
  m_a.stop(mode);
  m_b.stop(mode);
  m_c.stop(mode);
  m_d.stop(mode);
}
