#include <motor_control.h>

// motor objects
AccelStepper Left_Motor(HALFSTEP, 8, 9, 10, 11);
AccelStepper Right_Motor(HALFSTEP, 2, 3, 4, 5); 

// setup motor constants
void setup_motors(){
    Left_Motor.setMaxSpeed(MAX_SPEED);
    Left_Motor.setAcceleration(ACCELERATION);
    Right_Motor.setMaxSpeed(MAX_SPEED);
    Right_Motor.setAcceleration(ACCELERATION);
}

// get motor object by id
AccelStepper* get_motor(MotorID id){
    // sanity check input
    if(id != LEFT_MOTOR && id != RIGHT_MOTOR){ return nullptr; }

    // return motor object
    if(id == LEFT_MOTOR) {return &Left_Motor;} 
    else {return &Right_Motor;}
}

// drive motor by position amount
void drive_motor(MotorID id, long position){
    AccelStepper* motor = get_motor(id);
    if(motor != nullptr){
        motor->setCurrentPosition(0);
        motor->moveTo(position);
        while(motor->distanceToGo() != 0){
            motor->run();
        }
    }
}   