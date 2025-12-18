#include <Arduino.h>
#include <motor_control.h>
#include <comms.h>

void setup() {
  setup_motors();
  setup_comms();
}

void execute_command(int cmd_id, int* params, int num_params) {
    switch(cmd_id) {
        case CMD_PING:
            ping();
            break;
        default:
            // Unknown command
            break;
    }
}

void loop() {
}
