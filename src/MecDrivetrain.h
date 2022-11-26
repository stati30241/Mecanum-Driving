#ifndef MEC_DRIVETRAIN_H
#define MEC_DRIVETRAIN_H


#include "vex.h"

#include <cmath>


// Class to control a robot with a Mecanum Wheel Drivetrain
class MecDrivetrain {
private:
  // The motors
  vex::motor m_a;
  vex::motor m_b;
  vex::motor m_c;
  vex::motor m_d;

  // The dimentions of the drivetrain
  double m_wheelCircumference;
  double m_trackWidth;
  double m_wheelBase;

  // Velocities
  double m_driveVelocity;
  vex::velocityUnits m_driveVelocityUnits;
  double m_turnVelocity;
  vex::velocityUnits m_turnVelocityUnits;

private:
  // Initializes the motors
  void initializeMotors();

public:
  // Constructor
  MecDrivetrain(int32_t a, int32_t b, int32_t c, int32_t d, double wheelCircumference, double trackWidth, double wheelBase);

  // Getters
  double getDriveVelocity() const { return m_driveVelocity; } // Maybe add velocity units as a parameter? How tho?
  double getTurnVelocity() const { return m_turnVelocity; }
  vex::velocityUnits getDriveVelUnits() const { return m_driveVelocityUnits; };
  vex::velocityUnits getTurnVelUnits() const {return m_turnVelocityUnits; }

  // Setters
  void setDriveVelocity(double driveVelocity, const vex::velocityUnits& dvUnits) {
    m_driveVelocity = driveVelocity;
    m_driveVelocityUnits = dvUnits;
  }
  void setTurnVelocity(double turnVelocity, const vex::velocityUnits& tvUnits) {
    m_turnVelocity = turnVelocity;
    m_turnVelocityUnits = tvUnits;
  }

  // Makes the drivetrain move in the specified direction
  void drive(const vex::directionType& dir);
  void drive(const vex::directionType& dir, uint32_t time, const vex::timeUnits& tUnits = vex::timeUnits::msec);
  void drive(double vel, const vex::velocityUnits& vUnits, double angle);

  // Rotates the drivetrain
  void rotate(double vel, const vex::velocityUnits& vUnits);
  void rotate(double rot, const vex::rotationUnits& rUnits);
  void rotate(bool dir, uint32_t time, const vex::timeUnits& tUnits = vex::timeUnits::msec);

  // Stops all the motors
  void stop(const vex::brakeType& mode);
};


#endif // !MEC_DRIVETRAIN_H