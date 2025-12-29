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
        case DRIVE_MOTOR:
            if(num_params >= 2) {
                MotorID motor_id = static_cast<MotorID>(params[0]);
                long position = static_cast<long>(params[1]);
                drive_motor(motor_id, position);
            }
            break;
        case DRIVE_POSTION:
            if(num_params >= 2) {
                long position = static_cast<long>(params[0]);
                drive_forward(position); // assuming both motors get same position
            }
            break;
        case MIX_DRIVE:
            if(num_params >= 1) {
                long left_position = static_cast<long>(params[0]);
                long right_position = static_cast<long>(params[1]);
                mix_drive(left_position, right_position);
            }
            break;
        case TURN_IN_PLACE:
            if(num_params >= 1) {
                long turn_angle_deg = static_cast<long>(params[0]);
                turn_in_place(turn_angle_deg);
            }
            break;
        case DRIVE_DISTANCE:
            if(num_params >= 1) {
                long distance_mm = static_cast<long>(params[0]);
                drive_forward_distance_mm(distance_mm);
            }
            break;
        default:
            // Unknown command
            break;
    }
}

// main loop
void loop() {
    static bool printed = false;
    if(!printed) {
        Serial.println("Drivetrain Firmware Initialized");
        printed = true;
    }

    int cmd_id = -1;
    int params[PARAM_BUFFER_SIZE];  // buffer for parameters
    int num_params = 0;

    // parse incoming command
    parse_command(&cmd_id, params, &num_params);

    // TODO: handle overflow on num params with more packets if needed

    // execute command if valid
    if(cmd_id != -1) {
        execute_command(cmd_id, params, num_params);
    }
}   

