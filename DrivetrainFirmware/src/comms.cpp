#include <comms.h>
#include <Arduino.h>

// setup communication parameters
void setup_comms(){
    Serial.begin(BAUDRATE);
}

/*
read and parse incoming command
Expect commands of structure:
[START_BYTE][CMD_ID][NUM_PARAMS][PARAM_1]...[PARAM_N][END_BYTE]
*/
void parse_command(int* cmd_id, int* params, int* num_params){
    // wait for start byte
    while(Serial.available() > 0){
        int byte_in = Serial.read();
        Serial.println("Byte received:");
        Serial.println(byte_in, HEX);
        if(byte_in == (CMD_START & 0xFF)){ // check for start byte
            // read command id
            while(Serial.available() == 0);
            *cmd_id = Serial.read();
            
            // read number of parameters
            while(Serial.available() == 0);
            *num_params = Serial.read();
            
            // read parameters
            for(int i = 0; i < *num_params; i++){
                while(Serial.available() == 0);
                params[i] = Serial.read();
            }
            
            // wait for end byte
            while(Serial.available() == 0);
            int end_byte = Serial.read();
            if(end_byte == (CMD_END & 0xFF)){
                // valid command received
                return;
            } else {
                // invalid command, reset cmd_id and num_params
                *cmd_id = -1;
                *num_params = 0;
                return;
            }
        }
    }
}

// respond to ping command
void ping(){
    for(int i = 0; i < 5; i++){
        Serial.println("PONG - Drivetrain Firmware");
    }
}