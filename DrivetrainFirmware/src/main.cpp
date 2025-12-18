#include <Arduino.h>
#include <motor_control.h>
#include <comms.h>

// max number of parameters expected
#define PARAM_BUFFER_SIZE 10

void setup() {
  setup_motors();
  setup_comms();
}

// calls the appropriate function based on command ID
void execute_command(int cmd_id, int* params, int num_params) {
    switch(cmd_id) {
        case CMD_PING:
            ping();
            break;
        case CMD_DRIVE_MOTOR:
            if(num_params >= 2) {
                MotorID motor_id = static_cast<MotorID>(params[0]);
                long position = static_cast<long>(params[1]);
                drive_motor(motor_id, position);
            }
            break;
        default:
            // Unknown command
            break;
    }
}

// main loop
void loop() {
    int cmd_id = -1;
    int params[PARAM_BUFFER_SIZE];  // buffer for parameters
    int* overflow_params = nullptr; // in case of overflow
    int num_params = 0;

    // parse incoming command
    parse_command(&cmd_id, params, &num_params);

    // TODO: handle overflow on num params with more packets if needed

    // execute command if valid
    if(cmd_id != -1) {
        execute_command(cmd_id, params, num_params);
    }
}
