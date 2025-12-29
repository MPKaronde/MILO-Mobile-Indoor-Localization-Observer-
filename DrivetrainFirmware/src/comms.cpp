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
    byte start_byte;
    while(true){
        if(Serial.available() >= 1){
            Serial.readBytes(&start_byte, 1);
            if(start_byte == CMD_START){
                break;
            }
        }
    }

    Serial.println("Start byte received");

    // read command ID
    while(Serial.available() < sizeof(int)) {}
    Serial.readBytes((char*)cmd_id, sizeof(int));

    Serial.print("Command ID: ");
    Serial.println(*cmd_id);

    // read number of parameters
    while(Serial.available() < sizeof(int)) {}
    Serial.readBytes((char*)num_params, sizeof(int));

    Serial.print("Number of parameters: ");
    Serial.println(*num_params);

    // read parameters
    for(int i = 0; i < *num_params; i++){
        while(Serial.available() < sizeof(int)) {}
        Serial.readBytes((char*)&params[i], sizeof(int));
    }

    Serial.println("Parameters received");
    for(int i = 0; i < *num_params; i++){
        Serial.print("Param ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(params[i]);
    }

    // read end byte
    int end_byte;
    while(Serial.available() < sizeof(int)) {}
    Serial.readBytes((char*)&end_byte, sizeof(int));

    if(end_byte != CMD_END){
        *cmd_id = CMD_ERROR;
        Serial.println("Error: Invalid end byte");
    }
    Serial.println("End byte received");
}

// respond to ping command
void ping(){
    for(int i = 0; i < 5; i++){
        Serial.println("PONG - Drivetrain Firmware");
    }
}