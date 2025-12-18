/* Code for communicating with coordinator */
#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>

#define BAUDRATE 9600           // serial communication baudrate

enum CommandID  {               // command identifiers
    CMD_START =     0xDEAD,
    CMD_END =       0xBEEF,
    CMD_PING =      1,
};

void setup_comms();             // setup communication parameters
void parse_command();           // parse incoming command
void ping();                    // respond to ping command

#endif // COMMS_H