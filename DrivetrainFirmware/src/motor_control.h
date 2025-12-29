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
#define STEPS_PER_REVOLUTION 400.0
#define STEPS_PER_MM (STEPS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_MM)
#define WHEEL_CENTER_DISTANCE_MM 210.0

// motor identifiers
enum MotorID { LEFT_MOTOR, RIGHT_MOTOR };                   // motor identifiers
AccelStepper* get_motor(MotorID id);                        // get motor object by id

// administrative / misc functions
void setup_motors();                                        // setup motor constants
int mm_to_steps(int mm);                                    // convert mm to steps
void calculate_turn_steps(int turn_angle_deg,               // calculate steps for a turn in place, >0 -> right turn
    int* left_steps, int* right_steps); 

// motor control functions
void drive_motor(MotorID id, int position);                 // drive motor by position amount
void drive_forward(int position);                           // drive both motors forward by position amount
void mix_drive(int left_position, int right_position);      // drive motors independently
void turn_in_place(int turn_angle_deg);                     // turn robot in place by angle, >0 -> right turn
void drive_forward_distance_mm(int distance_mm);            // drive both motors forward by distance in mm

#endif // MOTOR_CONTROL_H