/* Code for controlling drivetrain */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <AccelStepper.h>

// for accelstepper library
#define HALFSTEP 4

// drive motor constants
#define MAX_SPEED 1000.0
#define ACCELERATION 500.0 

// motor identifiers
enum MotorID { LEFT_MOTOR, RIGHT_MOTOR };

void setup_motors();                            // setup motor constants
AccelStepper* get_motor(MotorID id);            // get motor object by id
void drive_motor(MotorID id, long position);    // drive motor by position amount

#endif // MOTOR_CONTROL_H