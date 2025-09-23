#include "globals.hh"
#include "rev/rev.hh"
#include "HybridOdometry.hh"
//#include "campbell_turn_copy.hh"

std::shared_ptr<rev::AsyncRunner> odom_runner;
std::shared_ptr<rev::AsyncRunner> reckless_runner;
std::shared_ptr<rev::AsyncRunner> turn_runner;

std::shared_ptr<rev::HybridOdometry> odom;

std::shared_ptr<rev::Reckless> reckless;

// motor ports
/* rev::Motor_Group right_motor_group ({rev::Motor(6), rev::Motor(7), rev::Motor(8)});
rev::Motor_Group left_motor_group ({rev::Motor(-3), rev::Motor(-4), rev::Motor(-5)});
rev::Motor_Group intake_arm ({rev::Motor(17), rev::Motor(-14)});
rev::MotorGroup conveyor ({rev::Motor(15)});
rev::MotorGroup rollers ({rev::Motor(20)}); */
rev::MotorGroup slides ({rev::Motor(-13),rev::Motor(-18)});
rev::MotorGroup winch ({rev::Motor(7)});
/* std::shared_ptr<rev::SkidSteerChassis> 	chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group); */

rev::Motor_Group left_motor_group ({rev::Motor(9), rev::Motor(-13), rev::Motor(-17), rev::Motor(-20)});
rev::Motor_Group right_motor_group ({rev::Motor(2), rev::Motor(-3), rev::Motor(6), rev::Motor(14)});
// rev::Motor_Group intake_arm ({rev::Motor(-5), rev::Motor(8)});
rev::Motor_Group intake_arm ({rev::Motor(5)});
rev::MotorGroup conveyor ({rev::Motor(7)});
rev::MotorGroup rollers ({rev::Motor(10)});
std::shared_ptr<rev::SkidSteerChassis>     chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group);

// Right motors: +2, -3, +6, +14
// Left motors: +9, -13, -17, -20
// Wall Stake Scoring: 7
// Wall Stake Scoring Sensor : 1
// Color Sensor : 4
// Robot Orientation: 12, 19
// Conveyor: 5
// Intake Rollers: 10
// Odom: 15, 16



// sensor inputs
pros::IMU imu(IMU_PORT);
pros::Rotation fwd(FWD_PORT);
pros::Rotation lat(LAT_PORT);
pros::Optical color_sensor(COLOR_SENSOR_1_PORT);
pros::ADIDigitalIn beam_break(BEAM_BREAK_PORT);


//hydraulics
pros::ADIDigitalOut mogoClamp(MOGO_CLAMP);




//intake

void roller_conveyor(int roller_time, int conveyor_time){ //roller time is the time to run the rollers, conveyor time is the time to run the conveyor, takes beam break into account
    auto start_time = pros::millis();
    move_roller(FORWARD_FAST);
    while(pros::millis() - start_time < roller_time){
        continue;
    }
    move_roller(STOP);
    pros::delay(100);

    move_conveyor(REVERSE_FAST);
    pros::delay(100);
    auto conveyor_start_time = pros::millis();
    move_conveyor(FORWARD_FAST);
    while(pros::millis() - conveyor_start_time < conveyor_time){
        continue;
    }
    move_conveyor(STOP);
    
}
void move_roller(SPEED speed){
    if (speed != STOP) {
        intake_arm.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    } else {
        intake_arm.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    }
    rollers.move_voltage((int) speed);
}

void intake(int min_time, int timeout_millis, bool stop_first) {
    move_roller(FORWARD_FAST);
    
    auto start_time = pros::millis();
    while ((beam_break.get_value() != BEAM_BROKEN || pros::millis() - start_time < min_time) && (pros::millis() - start_time < timeout_millis)) {
        pros::delay(20);
    }
    if (stop_first)
        move_roller(STOP);
    //move_conveyor(FORWARD_MID);
    //pros::delay(200);
    //move_conveyor(REVERSE_MID);
    //pros::delay(200);
    move_conveyor(FORWARD_FAST);
    pros::delay(1500);
    move_conveyor(STOP);
}

void intake_variable_time(int min_time, int timeout_millis, bool stop_first, int conveyor_amount) {
    move_roller(FORWARD_FAST);
    
    auto start_time = pros::millis();
    while ((beam_break.get_value() != BEAM_BROKEN || pros::millis() - start_time < min_time) && (pros::millis() - start_time < timeout_millis)) {
        pros::delay(20);
    }
    if (stop_first)
        move_roller(STOP);
    //move_conveyor(FORWARD_MID);
    //pros::delay(200);
    //move_conveyor(REVERSE_MID);
    //pros::delay(200);
    move_conveyor(FORWARD_FAST);
    pros::delay(conveyor_amount);
    move_conveyor(STOP);
}

//conveyer belt
void move_conveyor(SPEED speed){
    conveyor.move_voltage((int) -speed);
}

void clamp(MODE mode){
    mogoClamp.set_value((int) mode);
}

bool run_until(unsigned int millis) {
    auto start_time = pros::millis();

    while (!reckless->is_completed() && (pros::millis() - start_time < millis)) {
        print_position();
        pros::delay(20);
    }

#ifdef STEP_MODE
pros::Controller controller(pros::E_CONTROLLER_MASTER);
while (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
#endif

    reckless->breakout();

    return reckless->is_completed();
    pros::delay(3000);
}
