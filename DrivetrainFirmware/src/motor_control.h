/* Code for controlling drivetrain */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <AccelStepper.h>

// for accelstepper library
#define HALFSTEP 4

// drive motor constants
#define MAX_SPEED 1000.0
#define ACCELERATION 500.0 

// wheel math constants
#define WHEEL_DIAMETER_MM 75.0
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159)
#define STEPS_PER_REVOLUTION 2048.0
#define STEPS_PER_MM (STEPS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_MM)
#define WHEEL_CENTER_DISTANCE_MM 210.0

// motor identifiers
enum MotorID { LEFT_MOTOR, RIGHT_MOTOR };                   // motor identifiers
AccelStepper* get_motor(MotorID id);                        // get motor object by id

// administrative / misc functions
void setup_motors();                                        // setup motor constants
long mm_to_steps(long mm);                                  // convert mm to steps
void calculate_turn_steps(long turn_angle_deg,              // calculate steps for a turn in place
    long* left_steps, long* right_steps); 

// motor control functions
void drive_motor(MotorID id, long position);                // drive motor by position amount
void drive_forward(long position);                          // drive both motors forward by position amount
void mix_drive(long left_position, long right_position);    // drive motors independently

#endif // MOTOR_CONTROL_H