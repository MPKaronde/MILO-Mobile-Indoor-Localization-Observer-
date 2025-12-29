/* Code for communicating with coordinator */
#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>

#define BAUDRATE 9600                   // serial communication baudrate

enum CommandID  {                       // command identifiers
    CMD_START           =   0xDEAD,     // starts a command, 57005
    CMD_END             =   0xBEEF,     // ends a command, 48879
    CMD_EXTEND          =   0xFFFF,     // allows command to be split
    CMD_ERROR           =   0xC0DE,     // indicates error in command
    CMD_PING            =   1,
    DRIVE_MOTOR         =   2,          // drive single motor (motor_id, position)
    DRIVE_POSTION       =   3,          // drive both motors to position (position)
    MIX_DRIVE           =   4,          // drive motors independently (left_position, right_position)
    TURN_IN_PLACE       =   5,          // turn robot in place (turn_angle_deg)
    DRIVE_DISTANCE      =   6           // drive both motors forward by distance in mm (distance_mm)
};

void setup_comms();                     // setup communication parameters
void parse_command(int* cmd_id,         // parse incoming command
    int* params, 
    int* num_params);              
void ping();                            // respond to ping command

#endif // COMMS_H