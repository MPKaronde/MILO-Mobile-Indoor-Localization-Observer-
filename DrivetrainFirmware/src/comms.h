/* Code for communicating with coordinator */
#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>

#define BAUDRATE 9600                   // serial communication baudrate

enum CommandID  {                       // command identifiers
    CMD_START =         0xDEAD,     // starts a command, 57005
    CMD_END =           0xBEEF,     // ends a command, 48879
    CMD_EXTEND =        0xFFFF,     // allows command to be split
    CMD_PING =          1,
    CMD_DRIVE_MOTOR =   2
};

void setup_comms();                     // setup communication parameters
void parse_command(int* cmd_id,         // parse incoming command
    int* params, 
    int* num_params);              
void ping();                            // respond to ping command

#endif // COMMS_H