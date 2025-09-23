#ifndef GLOBALS_H  // these are header guards: https://www.geeksforgeeks.org/header-guard-in-c/#
#define GLOBALS_H

#include "rev/rev.hh"
#include "HybridOdometry.hh"

#define IMU_PORT 12
#define FWD_PORT 16, false
#define LAT_PORT 15, false

//const int ROLLERS_IN = 12000;

//#define STEP_MODE

//sensor ports
#define COLOR_SENSOR_1_PORT 4
#define BEAM_BREAK_PORT 'A'
#define MOGO_CLAMP 'C'


#define BEAM_BROKEN 0


// Odometry Parameters
#define WHEEL_DIAMETER 63.89_mm        // Diameter of forward and sideways wheel
#define FORWARD_WHEEL_OFFSET -1.125_in // How far to the right of the center of the robot the forward wheel is
#define LATERAL_WHEEL_OFFSET -1_in     // How far to the rear of the robot the lateral wheel is from the center


// turning parameters
#define TURN_MAX_POWER 0.7
#define COAST_POWER 0.3
#define HARSH_COEFF 0.1
#define COAST_COEFF 0.2
#define BRAKE_TIME 0.2_s

//pilons parameters
#define FAST_POWER 0.9
#define FAST_CORRECTION_COEFF 2
#define FAST_CORRECTION_DIST 0.8_in
#define FAST_HARSH_THRESHOLD 0.05_s
#define FAST_COAST_THRESHOLD 0.2_s
#define FAST_COAST_POWER 0.3

#define MID_POWER 0.5
#define MID_CORRECTION_COEFF 2
#define MID_CORRECTION_DIST 0.2_in
#define MID_HARSH_THRESHOLD 0.04_s
#define MID_COAST_THRESHOLD 0.2_s
#define MID_COAST_POWER 0.3

#define SLOW_POWER 0.3
#define SLOW_CORRECTION_COEFF 2
#define SLOW_CORRECTION_DIST 0.8_in
#define SLOW_HARSH_THRESHOLD 0.05_s
#define SLOW_COAST_THRESHOLD 0.2_s
#define SLOW_COAST_POWER 0.3

// background threads
// https://pros.cs.purdue.edu/v5/tutorials/topical/multitasking.html
extern std::shared_ptr<rev::AsyncRunner> odom_runner;     // calculates robot's position in the background
extern std::shared_ptr<rev::AsyncRunner> reckless_runner; // controls the chassis in the background
extern std::shared_ptr<rev::AsyncRunner> turn_runner;     // does point turns in the background


// controllers
extern std::shared_ptr<rev::HybridOdometry> odom; // tracks global position/velocity/angle
extern std::shared_ptr<rev::SkidSteerChassis> chassis;         // controls the motors
extern std::shared_ptr<rev::Reckless> reckless;                // drives the robot to points on the field
//extern std::shared_ptr<rev::> turn;                // point turns


// motor groups
extern rev::Motor_Group left_motor_group;
extern rev::Motor_Group right_motor_group;
extern rev::Motor_Group intake_arm;
extern rev::Motor_Group conveyor;
extern rev::Motor_Group rollers;
extern rev::Motor_Group slides;
extern rev::Motor_Group winch;


// sensor inputs 
extern pros::IMU imu;
extern pros::Rotation fwd;
extern pros::Rotation lat;


// Beam Break
extern pros::ADIDigitalIn beam_break;  


//color sensor
extern pros::Optical color_sensor;

// hydraulics
extern pros::ADIDigitalOut mogoClamp;

enum MODE{
    RELEASE = 1, CLAMP = 0
};

//clamp mogo
extern void clamp(MODE mode);

//Speeds
enum SPEED{
    FORWARD_FAST = 12000, FORWARD_MID = 9000, FORWARD_SLOW = 3000, STOP = 0, REVERSE_FAST = -12000, REVERSE_MID = -9000, REVERSE_SLOW = -6000
};

/**
 * Runs reckless code until it either completes or <param>millis</param> has passed.
 * returns whether reckless actually completed or it timed out. 
 */
extern bool run_until(unsigned int millis);

//intake function
extern void move_roller(SPEED speed);

//conveyer belt
extern void move_conveyor(SPEED speed);

extern void print_position();

extern void intake(int, int, bool=false);
extern void intake_variable_time(int min_time, int timeout_millis, bool stop_first, int conveyor_amount);
extern void roller_conveyor(int roller_time, int conveyor_time);

#endif // GLOBALS_H