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
void drive_motor(MotorID id, int position){
    AccelStepper* motor = get_motor(id);
    if(motor != nullptr){
        motor->setCurrentPosition(0);
        motor->moveTo(position);
        while(motor->distanceToGo() != 0){
            motor->run();
        }
    }
}

// drive both motors forward by position amount
void drive_forward(int position){
    Left_Motor.setCurrentPosition(0);
    Right_Motor.setCurrentPosition(0);
    Left_Motor.moveTo(position);
    Right_Motor.moveTo(position);
    while(Left_Motor.distanceToGo() != 0 || Right_Motor.distanceToGo() != 0){
        Left_Motor.run();
        Right_Motor.run();
    }
}   

// drive motors independently
void mix_drive(int left_position, int right_position){
    Left_Motor.setCurrentPosition(0);
    Right_Motor.setCurrentPosition(0);
    Left_Motor.moveTo(left_position);
    Right_Motor.moveTo(right_position);
    while(Left_Motor.distanceToGo() != 0 || Right_Motor.distanceToGo() != 0){
        Left_Motor.run();
        Right_Motor.run();
    }   
}

// convert mm to steps
int mm_to_steps(int mm){
    return static_cast<int>(mm * STEPS_PER_MM);
}

// calculate steps for a turn in place
// direction: + -> turn right, - -> turn left
void calculate_turn_steps(int turn_angle_deg,
    int* left_steps, int* right_steps){
    // calculate the arc length each wheel needs to travel
    double turn_circumference_mm = WHEEL_CENTER_DISTANCE_MM * 3.14159; // assuming 200mm between wheels
    double arc_length_mm = (turn_angle_deg / 360.0) * turn_circumference_mm;    
    // convert to steps
    int steps = mm_to_steps(static_cast<int>(arc_length_mm));
    // set left and right steps (one forward, one backward)
    *left_steps = steps;
    *right_steps = -steps;
}

// turn robot in place by angle
// direction: + -> turn right, - -> turn left
void turn_in_place(int turn_angle_deg){
    int left_steps = 0;
    int right_steps = 0;
    calculate_turn_steps(turn_angle_deg, &left_steps, &right_steps);
    mix_drive(left_steps, right_steps);
}

// drive both motors forward by distance in mm
void drive_forward_distance_mm(int distance_mm){
    int steps = mm_to_steps(distance_mm);
    drive_forward(steps);
}